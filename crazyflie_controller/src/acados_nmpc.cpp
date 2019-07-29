#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/QuaternionStamped.h>


// Crazyflie services
#include "crazyflie_driver/Takeoff.h"
#include "crazyflie_driver/Land.h"

// Matrices and vectors
#include <eigen3/Eigen/Dense>

// standard
#include <stdio.h>
#include <stdlib.h>

// acados
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// crazyflie specific
#include "crazyflie_model.h"
#include "acados_solver_crazyflie.h"

// ** global data **
ocp_nlp_in * nlp_in;
ocp_nlp_out * nlp_out;
ocp_nlp_solver * nlp_solver;
void * nlp_opts;
ocp_nlp_plan * nlp_solver_plan;
ocp_nlp_config * nlp_config;
ocp_nlp_dims * nlp_dims;

external_function_casadi * forw_vde_casadi;

using namespace Eigen;
using std::cout;
using std::endl;

// acados dim defines
#define N 	50 	/* Number of intervals in the horizon. */
#define NX 	13 	/* Number of differential state variables.  */
#define NU 	4	/* Number of control inputs. */
#define NY 	17	/* Number of measurements/references on nodes 0..N-1. */
#define NYN 	13	/* Number of measurements/references on node N. */


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

    NMPC(const ros::NodeHandle& n)
        : m_state(Idle)
        , m_pubNav()
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
        m_pubNav   = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	m_eRaptor_sub = nh.subscribe("/crazyflie/external_position", 1000, &NMPC::eRaptorCallback, this);
	m_imu_sub  = nh.subscribe("/crazyflie/imu", 1000, &NMPC::imuCallback, this);
	m_quat_sub = nh.subscribe("/crazyflie/kf_quaternion", 1000, &NMPC::quaternionCallback, this);
	m_serviceTakeoff = nh.advertiseService("takeoff", &NMPC::takeoff, this);
	m_serviceLand = nh.advertiseService("land", &NMPC::land, this);
	
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

    enum systemStates{
	  xq = 0,
	  yq = 1,
	  zq = 2,
	  q1 = 3,
	  q2 = 4,
	  q3 = 5,
	  q4 = 6,
	  vbx = 7,
	  vby = 8,
	  vbz = 9,
	  wx = 10,
	  wy = 11,
	  wz = 12
    };
    
    enum controlInputs{
	  w1 = 0,
	  w2 = 1,
	  w3 = 2,
	  w4 = 3
    };

    struct euler{
      double phi;
      double theta;
      double psi;
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
    
    struct cf_cmd_vel{
	double thrust;
	double roll;
	double pitch;
 	double yawr;
    };

    int acados_status;
    
    void eRaptorCallback(const geometry_msgs::PointStampedConstPtr& msg){
      
        // Position of crazyflie marker
	  actual_x = msg->point.x;
	  actual_y = msg->point.y;
	  actual_z = msg->point.z;
	  
	  /*ROS_INFO_STREAM("x: "   << actual_x << " " <<
			  "y: "   << actual_y << " " <<
			  "z: "   << actual_z << std::endl);*/
    }
    
    // Estimate the velocity using Euler initially and after a low-pass filter 
    double linearVelocity(std::vector <double> q_samples, double Ts, double elapsed_time, std::vector <double> dq_samples) {

	double dq = 0;
	if (elapsed_time > 0.1) dq = 1.573*dq_samples[4] - 0.6188*dq_samples[3] + 22.65*q_samples[4] - 22.65*q_samples[3];
	else dq = (q_samples[4] - q_samples[3]) / Ts;
	
	return dq;
    }
    
    euler quatern2euler(float* q){
	
	euler angle;
      
	double R11 = 2*q[0]*q[0]-1+2*q[1]*q[1];
	double R21 = 2*(q[1]*q[2]-q[0]*q[3]);
	double R31 = 2*(q[1]*q[3]+q[0]*q[2]);
	double R32 = 2*(q[2]*q[3]-q[0]*q[1]);
	double R33 = 2*q[0]*q[0]-1+2*q[3]*q[3];
	
	double phi = atan2(R32, R33);
	double theta = -atan(R31/sqrt(1-R31*R31));
	double psi = atan2(R21, R11);
	
	angle.phi = phi;
	angle.theta = theta;
	angle.psi = psi;
	
	return angle;
	
    }
    
    void estimateWordLinearVelocities(float dt, float delta){
	  //estimate the velocity
	  x_samples[0] = x_samples[1];
	  x_samples[1] = x_samples[2];
	  x_samples[2] = x_samples[3];
	  x_samples[3] = x_samples[4];
	  x_samples[4] = actual_x;
	  y_samples[0] = y_samples[1];
	  y_samples[1] = y_samples[2];
	  y_samples[2] = y_samples[3];
	  y_samples[3] = y_samples[4];
	  y_samples[4] = actual_y;
	  z_samples[0] = z_samples[1];
	  z_samples[1] = z_samples[2];
	  z_samples[2] = z_samples[3];
	  z_samples[3] = z_samples[4];
	  z_samples[4] = actual_z;

	  vx = linearVelocity(x_samples, dt, delta, vx_filter_samples);
	  vy = linearVelocity(y_samples, dt, delta, vy_filter_samples);
	  vz = linearVelocity(z_samples, dt, delta, vz_filter_samples);
	  
	  // passing to the state vector
	  x0_sign[vbx] = vx;
	  x0_sign[vby] = vy;
	  x0_sign[vbz] = vz;
	  
	  /*ROS_INFO_STREAM("vx: "   << x0_sign[vbx] << " " <<
			  "vy: "   << x0_sign[vby] << " " <<
			  "vz: "   << x0_sign[vbz] << std::endl);*/

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
    }
    
    void rotateLinearVeloE2B(float* q){
      
	 /* This is the convertion between 
	    quaternion orientation to rotation matrix 
	    from Earth to Body */
	 
	 double S11 = 2*(q[0]*q[0]+q[1]*q[1])-1;
	 double S12 = 2*(q[1]*q[2]+q[0]*q[3]);
	 double S13 = 2*(q[1]*q[3]-q[0]*q[2]);
	 double S21 = 2*(q[1]*q[2]-q[0]*q[3]);
	 double S22 = 2*(q[0]*q[0]+q[2]*q[2])-1;
	 double S23 = 2*(q[2]*q[3]+q[0]*q[1]);
	 double S31 = 2*(q[1]*q[3]+q[0]*q[2]);
	 double S32 = 2*(q[2]*q[3]-q[0]*q[1]);
	 double S33 = 2*(q[0]*q[0]+q[3]*q[3])-1;
	 
	 Sq << S11,S12,S13,
	       S21,S22,S23,
	       S31,S32,S33;
	       
	 vi << x0_sign[vbx],x0_sign[vby],x0_sign[vbz];
	 
	 vb = Sq*vi;
	 
	 // overwriting the values
	  x0_sign[vbx] = vb[0];
	  x0_sign[vby] = vb[1];
	  x0_sign[vbz] = vb[2]; 
    }
    
    bool takeoff(
        crazyflie_driver::Takeoff::Request& req,
	crazyflie_driver::Takeoff::Response& res)
    {
        ROS_INFO("Take off requested!");
        m_state = TakingOff;
	
	req.groupMask = 0;
	req.height = 0.5;
	req.duration =  ros::Duration(20.0);

        return true;
    }

    bool land(
        crazyflie_driver::Land::Request& req,
	crazyflie_driver::Land::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }
    
    void quaternionCallback(const geometry_msgs::QuaternionStampedPtr& msg){
      
      	// Quaternion
	actual_q1 = msg->quaternion.w;
	actual_q2 = msg->quaternion.x;
	actual_q3 = msg->quaternion.y;
	actual_q4 = msg->quaternion.z;	
    }


    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

        // Angular rates w.r.t. body frame in rad/s
	actual_wx = msg->angular_velocity.x;
	actual_wy = msg->angular_velocity.y;
	actual_wz = msg->angular_velocity.z;
    }

    void nmpcReset()
    {
        acados_free();
    }


    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();
	
	t0 += dt;

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
		  m_state = Tracking;
		    /*ROS_INFO("Taking off...");

		    if (actual_z > m_startZ + 0.5 || m_thrust > 50000)
		    {
			m_state = Tracking;
			//m_thrust = 0;
		    }
		    else
		    {
			m_thrust += 20000 * dt;
			geometry_msgs::Twist msg;
			msg.linear.z = m_thrust;
			m_pubNav.publish(msg);
		    }*/

		}
		break;


	      case Landing:
		{
		    /*ROS_INFO("Landing...");
		    
		    if (actual_z <= m_startZ + 0.5){
			m_state = Idle;
			geometry_msgs::Twist msg;
			m_pubNav.publish(msg);
		    }*/
		} // intentional fall-thru
		break;


	      case Tracking:
		{
		    ROS_INFO("NMPC tracker initiated.");
		    
		    // cmd_vel message
		    geometry_msgs::Twist msg;
		    
		    solver_input acados_in;
		    solver_output acados_out;
		    
		    cf_cmd_vel cmd_vel;
		    
		    // update reference
		    yref_sign[0] = 0.0;
		    yref_sign[1] = 0.0;
		    yref_sign[2] = 0.5; // hovering at 50 cm high
		    yref_sign[3] = 1.0;
		    yref_sign[4] = 0.0;
		    yref_sign[5] = 0.0;
		    yref_sign[6] = 0.0;
		    yref_sign[7] = 0.0;
		    yref_sign[8] = 0.0;
		    yref_sign[9] = 0.0;
		    yref_sign[10] = 0.0;
		    yref_sign[11] = 0.0;
		    yref_sign[12] = 0.0;

		    // get position from the mocap
		    x0_sign[xq] = actual_x;
		    x0_sign[yq] = actual_y;
		    x0_sign[zq] = actual_z;
		    
		    // get the quaternion from onboard kalman
		    x0_sign[q1] = actual_q1;
		    x0_sign[q2] = actual_q2;
		    x0_sign[q3] = actual_q3;
		    x0_sign[q4] = actual_q4;
		    
		    // estimate the velocity w.r.t. Earth
		    estimateWordLinearVelocities(dt,t0);
		    
		    // rotate velocities to Body
		    q[0] = x0_sign[q1];
		    q[1] = x0_sign[q2];
		    q[2] = x0_sign[q3];
		    q[3] = x0_sign[q4];
		    
		    rotateLinearVeloE2B(q);
		    
		    // get body angular rates
		    x0_sign[wx] = actual_wx;
		    x0_sign[wy] = actual_wy;
		    x0_sign[wz] = actual_wz;
		    
		    // up to this point we already stored the 13 states required for the NMPC

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
		    
		    // Select the optimal state to calculate the cf control inputs
		    q[0] = acados_out.x1[q1];
		    q[1] = acados_out.x1[q2];
		    q[2] = acados_out.x1[q3];
		    q[3] = acados_out.x1[q4];
		    
		    // Convert quaternion into euler angles
		    eu = quatern2euler(q);
		    
		    // assignment of cf control inputs
		    cmd_vel.roll  = eu.phi;
		    cmd_vel.pitch = eu.theta;
		    cmd_vel.yawr  = eu.psi;
		    cmd_vel.thrust = (acados_out.u0[w1]+acados_out.u0[w2]+acados_out.u0[w3]+acados_out.u0[w4])/4;
		    
		    // Populate the cmd_vel publisher with the correspondent struct
		    msg.linear.x  = cmd_vel.pitch; 	// pitch
		    msg.linear.y  = cmd_vel.roll; 	// roll
		    msg.linear.z  = cmd_vel.thrust; 	// thrust
		    msg.angular.z = cmd_vel.yawr; 	// yaw rate
		    
		    m_pubNav.publish(msg);

		    ROS_INFO_STREAM("thrust: "   << cmd_vel.thrust << " " <<
				    "roll: "     << cmd_vel.roll   << " " <<
				    "pitch: "    << cmd_vel.pitch  << " " <<
				    "yaw rate: " << cmd_vel.yawr   << endl);
		}
	}
    }

private:

    std::string m_worldFrame;
    std::string m_frame;

    ros::Publisher m_pubNav;
    
    ros::Subscriber m_imu_sub;
    ros::Subscriber m_eRaptor_sub;
    ros::Subscriber m_quat_sub;
    
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
    float t0;

    // Variables of the nmpc control process
    float x0_sign[NX];
    float yref_sign[NY];
    
    // Rotation matrix 
    Matrix3d Sq;
    Vector3d vi,vb;
    
    // Variables to be used in convertions
    float q[4];
    euler eu;
    
    // Variables for take-off and landing services
    float m_thrust;
    float m_startZ;

    // Variables for reading the IMU data
    float actual_wx;
    float actual_wy;
    float actual_wz;
    float actual_q1;
    float actual_q2;
    float actual_q3;
    float actual_q4;
  
    // Variables for eRaptor data
    float actual_x;
    float actual_y;
    float actual_z;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "acados_nmpc");

  // Read parameters
  ros::NodeHandle n("~");
  double frequency;
  n.param("frequency", frequency, 50.0);

  NMPC nmpc(n);
  nmpc.run(frequency);

  return 0;
}
