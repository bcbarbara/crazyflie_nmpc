#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

// standard
#include <stdio.h>
#include <stdlib.h>

// acados
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// TODO(oj) remove, when setters for Cyt,idxb available
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// example specific
#include "crazyflie_pos_model.h"
#include "acados_solver_crazyflie_pos.h"

// ** global data **
ocp_nlp_in * nlp_in;
ocp_nlp_out * nlp_out;
ocp_nlp_solver * nlp_solver;
void * nlp_opts;
ocp_nlp_plan * nlp_solver_plan;
ocp_nlp_config * nlp_config;
ocp_nlp_dims * nlp_dims;

external_function_casadi * forw_vde_casadi;


using std::cout;
using std::endl;

// acados dim defines
#define N 	40 	/* Number of intervals in the horizon. */
#define NX 	6 	/* Number of differential state variables.  */
#define NU 	4	/* Number of control inputs. */
#define NY 	10	/* Number of measurements/references on nodes 0..N-1. */
#define NYN 	6	/* Number of measurements/references on node N. */


double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}


class NMPC
{
public:

    NMPC(
        const std::string& worldFrame,
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_worldFrame(worldFrame)
        , m_frame(frame)
	, m_state(Idle)
        , m_pubNav()
	, m_listener()
	, m_thrust(0)
	, m_startZ(0)
        , m_serviceTakeoff()
        , m_serviceLand()
    {
	int status = 0;
	status = acados_create();

	if (status){
	  cout << "acados_create() returned status " << status << ". Exiting." << endl;
	  exit(1);
	}

        ros::NodeHandle nh;
	m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
        m_pubNav 	 = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	m_imu_sub 	 = nh.subscribe("/crazyflie/imu", 1000, &NMPC::imuCallback, this);
        m_serviceTakeoff = nh.advertiseService("takeoff", &NMPC::takeoff, this);
        m_serviceLand    = nh.advertiseService("land", &NMPC::land, this);
	
	vx = 0.0;
	vy = 0.0;
	vz = 0.0;
    
	x_samples.resize(5);
	y_samples.resize(5);
	z_samples.resize(5);
	for (unsigned int i = 0; i <= 4; i++) x_samples[i] = 0.0;
	for (unsigned int i = 0; i <= 4; i++) y_samples[i] = 0.0;
	for (unsigned int i = 0; i <= 4; i++) z_samples[i] = 0.0;
	
	vx_filter_samples.resize(5);
	vy_filter_samples.resize(5);
	vz_filter_samples.resize(5);
	for (unsigned int i = 0; i <= 4; i++) vx_filter_samples[i] = 0.0;
	for (unsigned int i = 0; i <= 4; i++) vy_filter_samples[i] = 0.0;
	for (unsigned int i = 0; i <= 4; i++) vz_filter_samples[i] = 0.0;

    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &NMPC::iteration, this);
        ros::spin();
    }

private:

    enum task{
	  Idle = 0,
	  Tracking = 1,
	  TakingOff = 2,
	  Landing = 3,
    };

    enum controlInputs{
	  cf_thrust=0,
	  cf_roll=1,
	  cf_pitch=2,
	  cf_yawr=3
    };

    enum systemStates{
	   X=0,
	   Y=1,
	   Z=2,
	   DX=3,
	   DY=4,
	   DZ=5
    };


    struct solver_output{
      double status, KKT_res, cpu_time;
      double u0[NU];
      double x1[NX];
    };

    struct solver_input{
      double x0[NX];
      double yref[(NX+NU)*N];
      double yref_e[NYN];
    };

    int acados_status;


    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

	actual_roll  = msg->angular_velocity.x;
	actual_pitch = msg->angular_velocity.y;
	actual_yaw   = msg->angular_velocity.z;
    }

    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;

        tf::StampedTransform transform;
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
	m_startZ = transform.getOrigin().z();

        return true;
    }

    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }

    void nmpcReset()
    {
        acados_free();
    }
    
    /* Estimate the velocity using Euler initially and after a low-pass filter */
    double linear_velocity(std::vector <double> q_samples, double Ts, double elapsed_time, std::vector <double> dq_samples) {

	double dq = 0;

	if (elapsed_time > 0.1) {
		dq = 1.573*dq_samples[4] - 0.6188*dq_samples[3] + 22.65*q_samples[4] - 22.65*q_samples[3];
		//dq = ((25. / 12.)*q_samples[4] - 4 * q_samples[3] + 3 * q_samples[2] - (4. / 3.)*q_samples[1] + 0.25*q_samples[0]) / Ts; 4th order Gear Formula
	}
	else {
		dq = (q_samples[4] - q_samples[3]) / Ts;
	}
	
	return dq;
}


    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();
	
	delta_t += dt;

	switch(m_state){
	      case Idle:
		{
		    ROS_INFO("Idle mode.");
		    geometry_msgs::Twist msg;

		    msg.linear.x = 0; // pitch
		    msg.linear.y = 0; // roll
		    msg.linear.z = 0; // thrust
		    msg.angular.z = 0; // yaw rate
		    m_pubNav.publish(msg);
		}
		break;


	      case TakingOff:
		{
		    ROS_INFO("Taking off...");
		    tf::StampedTransform transform;
		    m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
		    if (transform.getOrigin().z() > m_startZ + 0.05 || m_thrust > 50000)
		    {
			m_state = Tracking;
			m_thrust = 0;
		    }
		    else
		    {
			m_thrust += 10000 * dt;
			geometry_msgs::Twist msg;
			msg.linear.z = m_thrust;
			m_pubNav.publish(msg);
		    }

		}
		break;


	      case Landing:
		{
		    ROS_INFO("Landing...");
		    tf::StampedTransform transform;
		    m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
		    if (transform.getOrigin().z() <= m_startZ + 0.05){
			m_state = Idle;
			geometry_msgs::Twist msg;
			m_pubNav.publish(msg);
		    }
		} // intentional fall-thru
		break;


	      case Tracking:
		{
		    ROS_INFO("NMPC tracker initiated.");

		    geometry_msgs::Twist msg;
		    solver_input acados_in;
		    solver_output acados_out;
		    
		    // update reference
		    yref_sign[0] = 0.0;
		    yref_sign[1] = 0.0;
		    yref_sign[2] = 0.4; // hovering at 40 cm high
		    yref_sign[3] = 0.0;
		    yref_sign[4] = 0.0;
		    yref_sign[5] = 0.0;
		    
		    tf::StampedTransform currentDrone;
		    m_listener.lookupTransform(m_frame, m_worldFrame, ros::Time(0), currentDrone);
		    

		    // get position from the mocap
		    x0_sign[X] = currentDrone.getOrigin().x();
		    x0_sign[Y] = currentDrone.getOrigin().y();
		    x0_sign[Z] = currentDrone.getOrigin().z();
		    
		    //estimate the velocity
		    x_samples[0] = x_samples[1];
		    x_samples[1] = x_samples[2];
		    x_samples[2] = x_samples[3];
		    x_samples[3] = x_samples[4];
		    x_samples[4] = x0_sign[X];
		    y_samples[0] = y_samples[1];
		    y_samples[1] = y_samples[2];
		    y_samples[2] = y_samples[3];
		    y_samples[3] = y_samples[4];
		    y_samples[4] = x0_sign[Y];
		    z_samples[0] = z_samples[1];
		    z_samples[1] = z_samples[2];
		    z_samples[2] = z_samples[3];
		    z_samples[3] = z_samples[4];
		    z_samples[4] = x0_sign[Z];

		    vx = linear_velocity(x_samples, dt, delta_t, vx_filter_samples);
		    vy = linear_velocity(y_samples, dt, delta_t, vy_filter_samples);
		    vz = linear_velocity(z_samples, dt, delta_t, vz_filter_samples);
		    // passing to the state vector
		    x0_sign[DX] = vx;
		    x0_sign[DY] = vy;
		    x0_sign[DZ] = vz;
		    vx_filter_samples[0] = vx_filter_samples[1];
		    vx_filter_samples[1] = vx_filter_samples[2];
		    vx_filter_samples[2] = vx_filter_samples[3];
		    vx_filter_samples[3] = vx_filter_samples[4];
		    vx_filter_samples[4] = vx;
		    vy_filter_samples[0] = vy_filter_samples[1];
		    vy_filter_samples[1] = vy_filter_samples[2];
		    vy_filter_samples[2] = vy_filter_samples[3];
		    vy_filter_samples[3] = vy_filter_samples[4];
		    vy_filter_samples[4] = vy;
		    vz_filter_samples[0] = vz_filter_samples[1];
		    vz_filter_samples[1] = vz_filter_samples[2];
		    vz_filter_samples[2] = vz_filter_samples[3];
		    vz_filter_samples[3] = vz_filter_samples[4];
		    vz_filter_samples[4] = vz;

		    // copy signals into local buffers
		    for (unsigned int i = 0; i < NX; i++) acados_in.x0[i] = x0_sign[i];

		    for (unsigned int i = 0; i < N; i++){
		      for(unsigned int j = 0; i < NY; j++){
			acados_in.yref[i*NY+j] = yref_sign[i*NY+j];
		      }
		    }

		    for (unsigned int i = 0; i < NYN; i++) acados_in.yref_e[i] =  yref_sign[N*NY+i];

		    // set initial condition
		    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_in.x0);
		    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_in.x0);

		    // update reference
		    for (unsigned int i = 0; i < N; i++) {
			ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", acados_in.yref + i*10);
		    }
		    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", acados_in.yref_e);

		    // call solver
		    acados_status = acados_solve();

		    // assign output signals
		    acados_out.status = acados_status;
		    acados_out.KKT_res = (double)nlp_out->inf_norm_res;
		    acados_out.cpu_time = (double)nlp_out->total_time;


		    // get solution
		    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void *)acados_out.u0);

		    // get next state
		    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", (void *)acados_out.x1);

		    msg.linear.x = acados_out.u0[cf_pitch]; 	// pitch
		    msg.linear.y = acados_out.u0[cf_roll]; 	// roll
		    msg.linear.z = acados_out.u0[cf_thrust]; 	// thrust
		    msg.angular.z = acados_out.u0[cf_yawr]; 	// yaw rate
		    m_pubNav.publish(msg);

		    ROS_INFO_STREAM("thrust: "   << acados_out.u0[cf_thrust] << " " <<
				    "roll: "     << acados_out.u0[cf_roll]   << " " <<
				    "pitch: "    << acados_out.u0[cf_pitch]  << " " <<
				    "yaw rate: " << acados_out.u0[cf_yawr]   << endl);
		}
	}
    }

private:

    std::string m_worldFrame;
    std::string m_frame;

    ros::Publisher m_pubNav;
    ros::Subscriber m_joy_sub;
    ros::Subscriber m_task_sub;
    ros::Subscriber m_imu_sub;
    
    tf::TransformListener m_listener;

    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    
    // Flag to choose the task to perform
    task m_state;
    
    double vx,vy,vz;
    std::vector<double> x_samples;
    std::vector<double> y_samples;
    std::vector<double> z_samples;
    std::vector<double> vx_filter_samples;
    std::vector<double> vy_filter_samples;
    std::vector<double> vz_filter_samples;
    float delta_t;

    // Variables of the nmpc control process
    float x0_sign[NX];
    float yref_sign[NY];

    // Variables for take-off and landing services
    float m_thrust;
    float m_startZ;

    // Variables for reading the IMU data
    float actual_roll;
    float actual_pitch;
    float actual_yaw;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "acados_nmpc");

  // Read parameters
  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "/world");
  std::string crazyflieFrame;
  n.getParam("frame", crazyflieFrame);
  double frequency;
  n.param("frequency", frequency, 50.0);

  NMPC nmpc(worldFrame, crazyflieFrame, n);
  nmpc.run(frequency);

  return 0;
}
