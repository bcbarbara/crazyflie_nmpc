#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
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
        , m_pubNav()
        , m_listener()
        , m_thrust(0)
	, actual_x(0)
	, actual_y(0)
	, actual_z(0)
	, goal_x(0)
	, goal_y(0)
	, goal_z(0)
    {
	int status = 0;
	status = acados_create();
	
	if (status){ 
	  std::cout << "acados_create() returned status " << status << ". Exiting." << std::endl; 
	  exit(1);   
	}
    
        ros::NodeHandle nh;
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	m_joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &NMPC::joyCallback, this);
	m_imu_sub = nh.subscribe("/crazyflie/imu", 1000, &NMPC::imuCallback, this);
	m_viconpos_sub = nh.subscribe("/vicon/pos", 1000, &NMPC::posViconCallback, this);
	m_service_takeoff = nh.advertiseService("takeoff", &NMPC::takeoff, this);
        m_service_land = nh.advertiseService("land", &NMPC::land, this);
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
    
    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res){
       
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;

        tf::StampedTransform transform;
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
        m_startZ = transform.getOrigin().z();

        return true;
    }

    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res){
      
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }
    
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){  
      //Routine to manage joystick buttons as inputs
      int i;
      int j;

      if(num==0){
	      for(i=0;i<14;i++) m_hover[i]=0;
      }

      if (remainder(2*num+1,2)==1){
	for(i=0;i<14;i++){
		      last_hover_state[i]=m_hover[i];
		      m_hover[i]=joy->buttons[i];
		      std::cout << m_hover[i] << " " << last_hover_state[i]<< std::endl;
	      }
	      std::cout << " joy callback. " << std::endl;
	      last_m_state = joy_state;
	      for(i=0;i<14;i++){
		      if (last_hover_state[i]!=m_hover[i]){
			      joy_state=i;
			      if (last_m_state>joy_state) i=15;
		      }
	}
      }
      num++;
    }
    
    void posViconCallback( const geometry_msgs::Vector3::ConstPtr& msg){
      
	actual_x = msg->x;
	actual_y = msg->y;
	actual_z = msg->z;
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
      
	actual_roll = msg->angular_velocity.x*3.141592654/180;
	actual_pitch = msg->angular_velocity.y*3.141592654/180;
	actual_yaw = msg->angular_velocity.z*3.141592654/180;
    }
    
    
    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
    }

    void nmpcReset()
    {
        acados_free();
    }

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();
	
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
		    ROS_INFO("Taking off.");
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
		    ROS_INFO("Landing.");
		    geometry_msgs::Twist msg;
		    
		    tf::StampedTransform transform;
		    m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
		    if (transform.getOrigin().z() <= m_startZ + 0.05) {
			m_state = Idle;
			geometry_msgs::Twist msg;
			m_pubNav.publish(msg);
		    }
		}
		break;
		
	      case Tracking:
		{
		    ROS_INFO("NMPC tracker.");
		    tf::StampedTransform transform;
		    m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

		    geometry_msgs::PoseStamped targetWorld;
		    targetWorld.header.stamp = transform.stamp_;
		    targetWorld.header.frame_id = m_worldFrame;
		    targetWorld.pose = m_goal.pose;

		    geometry_msgs::PoseStamped targetDrone;
		    m_listener.transformPose(m_frame, targetWorld, targetDrone);

		    tfScalar roll, pitch, yaw;
		    tf::Matrix3x3(
			tf::Quaternion(
			    targetDrone.pose.orientation.x,
			    targetDrone.pose.orientation.y,
			    targetDrone.pose.orientation.z,
			    targetDrone.pose.orientation.w
			)).getRPY(roll, pitch, yaw);

		    geometry_msgs::Twist msg;
		    solver_input acados_in;
		    solver_output acados_out;
		    
		    // get position and velocity from mocap
		    x0_sign[X] = actual_x;
		    x0_sign[Y] = actual_y;
		    x0_sign[Z] = actual_z;
		    x0_sign[DX] = actual_x/dt;
		    x0_sign[DY] = actual_y/dt;
		    x0_sign[DZ] = actual_z/dt;
		    		    
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
		}
		

	}
    }

private:
  
    std::string m_worldFrame;
    std::string m_frame;
    geometry_msgs::PoseStamped m_goal;
    ros::Publisher m_pubNav;
    tf::TransformListener m_listener;
    ros::Subscriber m_joy_sub;
    ros::Subscriber m_imu_sub;
    ros::Subscriber m_viconpos_sub;
     ros::ServiceServer m_service_takeoff;
    ros::ServiceServer m_service_land;
    task m_state;
    
    //Variables for joystick controller management
    int m_hover[14];
    int last_hover_state[14];
    int joy_state;
    int last_m_state;
    double num;
    
    //Variables of the control process
    float x0_sign[NX];
    float yref_sign[NY];
    float goal_x;
    float goal_y;
    float goal_z;

    float m_thrust;
    float m_startZ;
    float actual_x;
    float actual_y;
    float actual_z;
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
  std::string frame;
  n.getParam("frame", frame);
  double frequency;
  n.param("frequency", frequency, 50.0);

  NMPC nmpc(worldFrame, frame, n);
  nmpc.run(frequency);

  return 0;
}
