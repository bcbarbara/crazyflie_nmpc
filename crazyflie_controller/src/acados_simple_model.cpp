#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

// Generic log data
#include "crazyflie_controller/GenericLogData.h"

// Dynamic reconfirgure
#include <dynamic_reconfigure/server.h>
#include <crazyflie_controller/crazyflie_paramsConfig.h>
#include <boost/thread.hpp>
#include "boost/thread/mutex.hpp"

// Matrices and vectors
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// standard
#include <iostream>
#include <sstream>
#include <fstream>
#include <ios>

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
#include "crazyflie_simple_model/c_generated_code/crazyflie_simple_model_model/crazyflie_simple_model_model.h"
#include "crazyflie_simple_model/c_generated_code/acados_solver_crazyflie_simple_model.h"
// #include "crazyflie_simple_model.h"
// #include "acados_solver_crazyflie_simple_model.h"

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
using std::ofstream;
using std::cout;
using std::endl;
using std::fixed;
using std::showpos;


// acados dim defines
#define N 	50 	/* Number of intervals in the horizon. */
#define NX 	9 	/* Number of differential state variables.  */
#define NU 	4	/* Number of control inputs. */
#define NY 	13	/* Number of measurements/references on nodes 0..N-1. */
#define NYN 	9	/* Number of measurements/references on node N. */

#define pi      3.14159265358979323846
#define g0	9.80665

#define REGULATION_POINT 1

// delay compensation
#define FIXED_U0 1 // possible values 0, 1

class NMPC{
public:

    NMPC(const ros::NodeHandle& n){

	int status = 0;
	status = acados_create();

	if (status){
	  ROS_INFO_STREAM("acados_create() returned status " << status << ". Exiting." << endl);
	  exit(1);
	}

	ros::NodeHandle nh;
	// publisher for the real robot inputs (thrust, roll, pitch, yaw rate)
	m_pubNav		= nh.advertise<geometry_msgs::Twist>("/crazyflie/cmd_vel", 1);
	// subscriber for the motion capture system position
	m_eRaptor_sub  	 	= nh.subscribe("/crazyflie/external_position", 5, &NMPC::eRaptorCallback, this);
	// subscriber for the IMU linear acceleration and angular velocities from acc and gyro
	m_imu_sub		= nh.subscribe("/crazyflie/imu", 5, &NMPC::imuCallback, this);
	// subscriber for the IMU stabilizer euler angles
	m_euler_sub		= nh.subscribe("/crazyflie/euler_angles", 5, &NMPC::eulerCallback, this);
	// publisher for acados z output for the 1st & N shooting node + z from the mocap
	m_acados_position       = nh.advertise<geometry_msgs::Vector3>("/crazyflie/acados_traj",1);
	// publisher for the current value of the linear velocities
	m_cf_lvb 		= nh.advertise<geometry_msgs::Vector3>("/crazyflie/linear_velo",1);

	// Set initial value of the linear velocities to zero
	vx = 0.0;
	vy = 0.0;
	vz = 0.0;

	// Set size of buffer
	x_samples.resize(5);
	y_samples.resize(5);
	z_samples.resize(5);
	
	// Set initial value of the buffer to zero
	for (unsigned int i = 0; i <= 4; i++) x_samples[i] = 0.0;
	for (unsigned int i = 0; i <= 4; i++) y_samples[i] = 0.0;
	for (unsigned int i = 0; i <= 4; i++) z_samples[i] = 0.0;

	// Set size of buffer
	vx_filter_samples.resize(5);
	vy_filter_samples.resize(5);
	vz_filter_samples.resize(5);

	for (unsigned int i = 0; i <= 4; i++) vx_filter_samples[i] = 0.0;
	for (unsigned int i = 0; i <= 4; i++) vy_filter_samples[i] = 0.0;
	for (unsigned int i = 0; i <= 4; i++) vz_filter_samples[i] = 0.0;

	// Initializing control inputs
	for(unsigned int i=0; i < NU; i++) acados_out.u1[i] = 0.0;
	
	
	// Set elapsed time to zero initially
	t0 = 0.0;
	
	Ct = 3.25e-4/1e+6;	// [N/rpm^2]
	
	// Steady-state control input value
	mq = 33e-3; 	  	// [Kg]
	uss = mq*g0; 	
    }

    void run(double frequency){
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &NMPC::iteration, this);
	
	
	ROS_DEBUG("Setting up the dynamic reconfigure panel and server");
	dynamic_reconfigure::Server<crazyflie_controller::crazyflie_paramsConfig> server;
	dynamic_reconfigure::Server<crazyflie_controller::crazyflie_paramsConfig>::CallbackType f;
	f = boost::bind(&NMPC::callback_dynamic_reconfigure, this, _1, _2);
	server.setCallback(f);
  
        ros::spin();
    }
    
    void callback_dynamic_reconfigure(crazyflie_controller::crazyflie_paramsConfig &config, uint32_t level){
      
      if(REGULATION_POINT){
	ROS_INFO("Changing the desired regulation point!");
	xq_des 		= config.xq_des;
	yq_des 		= config.yq_des;
	zq_des 		= config.zq_des;
	
	ROS_INFO_STREAM("Current regulation point: " << xq_des << ", " << yq_des << ", " << zq_des << endl);
      }
    }

    enum systemStates{
	  xq = 0,
	  yq = 1,
	  zq = 2,
	  dxq = 3,
	  dyq = 4,
	  dzq = 5,
	  roll = 6,
	  pitch = 7,
	  yaw = 8
    };

    enum controlInputs{
	  thrust = 0,
	  droll = 1,
	  dpitch = 2,
	  dyaw = 3
    };

    struct euler{
      double phi;
      double theta;
      double psi;
    };

    struct solver_output{
      double status, KKT_res, cpu_time;
      double u0[NU];
      double u1[NU];
      double x1[NX];
      double x2[NX];
      double xAllStages[NX];
    };

    struct solver_input{
      double x0[NX];
      double yref[(NY*N)+NY];
      double yref_e[NYN];
      double W[NY*NY];
      double WN[NX*NX];
    };
    
    struct crazyflie_control_inputs{
      double roll;
      double pitch;
      double yaw_rate;
      double thrust;
    };
   
    double linearVelocity(std::vector <double> q_samples, std::vector <double> dq_samples, double Ts, double elapsed_time) {

      double dq = 0;
      if (elapsed_time > 1.0) dq = 0.1814*dq_samples[4] - 0.00823*dq_samples[3] + 26.13*q_samples[4] - 26.13*q_samples[3];
      else dq = (q_samples[4] - q_samples[3]) / Ts;
      return dq;
    }

    Vector3d estimateWordLinearVelocities(float dt, float delta){
      
	  Vector3d vearth;
      
	  //estimte the velocity
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

	  vearth[0] = linearVelocity(x_samples, vx_filter_samples, dt, delta);
	  vearth[1] = linearVelocity(y_samples, vy_filter_samples, dt, delta);
	  vearth[2] = linearVelocity(z_samples, vz_filter_samples, dt, delta);

	  vx_filter_samples[0] = vx_filter_samples[1];
	  vx_filter_samples[1] = vx_filter_samples[2];
	  vx_filter_samples[2] = vx_filter_samples[3];
	  vx_filter_samples[3] = vx_filter_samples[4];
	  vx_filter_samples[4] = vearth[0];
	  vy_filter_samples[0] = vy_filter_samples[1];
	  vy_filter_samples[1] = vy_filter_samples[2];
	  vy_filter_samples[2] = vy_filter_samples[3];
	  vy_filter_samples[3] = vy_filter_samples[4];
	  vy_filter_samples[4] = vearth[1];
	  vz_filter_samples[0] = vz_filter_samples[1];
	  vz_filter_samples[1] = vz_filter_samples[2];
	  vz_filter_samples[2] = vz_filter_samples[3];
	  vz_filter_samples[3] = vz_filter_samples[4];
	  vz_filter_samples[4] = vearth[2];
	  
	  return vearth;
    }

    double deg2Rad(double deg) {
	  return deg / 180.0 * pi;
    }

    double rad2Deg(double rad) {
	  return rad * 180.0 / pi;
    }

    void eRaptorCallback(const geometry_msgs::PointStampedConstPtr& msg){

	// Position of crazyflie marker
	actual_x = msg->point.x;
	actual_y = msg->point.y;
	actual_z = msg->point.z;
    }

    void eulerCallback(const geometry_msgs::Vector3StampedPtr& msg){

      	// Euler angles
	actual_roll  = msg->vector.x; 
	actual_pitch = -msg->vector.y; // the pitch coming from the IMU seems to have an inverted sign (for no reason)
	actual_yaw   = msg->vector.z;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

        // Angular rates w.r.t. body frame in rad/s
	actual_wx = msg->angular_velocity.x;
	actual_wy = msg->angular_velocity.y;
	actual_wz = msg->angular_velocity.z;
    }

    void nmpcReset(){
      
        acados_free();
    }
    

    int rpm2pwm(double rpm){

      int pwm = (rpm-4070.3)/0.2685;
      return pwm;
    }


    void iteration(const ros::TimerEvent& e){
	
      if(e.last_real.isZero()) {
	t0 = e.current_real.toSec();
      }
      
      double dt = e.current_real.toSec() - e.last_real.toSec();    
          
      try{
	// Update reference
	for (k = 0; k < N+1; k++) {
	      yref_sign[k * NY + 0] = xq_des; 	// xq
	      yref_sign[k * NY + 1] = yq_des;	// yq
	      yref_sign[k * NY + 2] = zq_des;	// zq
	      yref_sign[k * NY + 3] = 0.00;	// dxq
	      yref_sign[k * NY + 4] = 0.00;	// dyq
	      yref_sign[k * NY + 5] = 0.00;	// dzq
	      yref_sign[k * NY + 6] = 0.00;	// roll
	      yref_sign[k * NY + 7] = 0.00;	// pitch
	      yref_sign[k * NY + 8] = 0.00;	// yaw
	      yref_sign[k * NY + 9] = uss;	// thrust
	      yref_sign[k * NY + 10] = 0.00;	// droll
	      yref_sign[k * NY + 11] = 0.00;	// dpitch
	      yref_sign[k * NY + 12] = 0.00;	// dyaw
	}

	// Storing inertial positions in state vector
	x0_sign[xq] = actual_x;
	x0_sign[yq] = actual_y;
	x0_sign[zq] = actual_z;
	
	// Storing world linear velocities
	Vector3d vi_mat;
	vi_mat = estimateWordLinearVelocities(dt,t0);

	x0_sign[dxq] = vi_mat[0];
	x0_sign[dyq] = vi_mat[1];
	x0_sign[dzq] = vi_mat[2];

	// Get the euler angles from the onboard stabilizer
	euler eu;
	eu.phi   = deg2Rad(actual_roll);
	eu.theta = deg2Rad(actual_pitch);
	eu.psi   = deg2Rad(actual_yaw);
	
	x0_sign[roll] 	= eu.phi;
	x0_sign[pitch] 	= eu.theta;
	x0_sign[yaw] 	= actual_wz*dt;
	
		
	//---------------------------------------//
	//				         //
	//		acados NMPC 	         //
	//				         //
	//---------------------------------------//


	// copy signals into local buffers
	for (i = 0; i < NX; i++){
	  acados_in.x0[i] = x0_sign[i];
	  //cout << "x0: " << acados_in.x0[i] << endl;
	}

	for (i = 0; i < N; i++) {
	    for (j = 0; j < NY; ++j) {
		    acados_in.yref[i*NY + j] = yref_sign[i*NY + j];
	    }
	    //cout << i << " " << "yref: " << acados_in.yref[i] << endl;
	}

	for (i = 0; i < NYN; i++) {
	    acados_in.yref_e[i] = yref_sign[N*NY + i];
	}

	// set initial condition
	ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_in.x0);
	ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_in.x0);
	
	if (FIXED_U0 == 1) {
	  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbu", acados_out.u1);
	  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubu", acados_out.u1);
	}

	// update reference
	for (ii = 0; ii < N; ii++) {
	    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "yref", acados_in.yref + ii*NY);
	}

	ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", acados_in.yref_e);

	// call solver
	acados_status = acados_solve();
	
	// assign output signals
	acados_out.status = acados_status;
	acados_out.KKT_res = (double)nlp_out->inf_norm_res;
	acados_out.cpu_time = (double)nlp_out->total_time;

	// Get solution
	ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void *)acados_out.u0);
	
	// Get solution at stage 1
	ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "u", (void *)acados_out.u1);
	
	// Get next stage
	ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", (void *)acados_out.x1);
	  
	// Get stage 2 which compensates 15 ms for the delay
	ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 2, "x", (void *)acados_out.x2);	
		
	// Select control inputs
	crazyflie_control_inputs cf_control_inputs;

	if (FIXED_U0 == 1) {
	  cf_control_inputs.thrust =  acados_out.u1[thrust];
	  cf_control_inputs.roll   =  acados_out.x2[roll];
	  cf_control_inputs.pitch  =  acados_out.x2[pitch];
	  cf_control_inputs.yaw_rate =  acados_out.u1[dyaw];
	} else {
	  cf_control_inputs.thrust =  acados_out.u0[thrust];
	  cf_control_inputs.roll   =  acados_out.x1[roll];
	  cf_control_inputs.pitch  =  acados_out.x1[pitch];
	  cf_control_inputs.yaw_rate =  acados_out.u0[dyaw];
	}
	
 //	ROS_INFO_STREAM("T: " <<  cf_control_inputs.thrust << " " << "roll: " << cf_control_inputs.roll << " " << "pitch: " << cf_control_inputs.pitch<< " " << "yaw rate: " << cf_control_inputs.yaw_rate << endl);

	// converting thrust to base motor speed (thrust -> rpm)
	double RPM = sqrt(cf_control_inputs.thrust/(4*Ct));
	// converting base motors rpm to pwm (rpm -> pwm)
	double PWM = rpm2pwm(RPM);
	
	// Publish real control inputs	
	geometry_msgs::Twist msg;
	msg.linear.x  = -rad2Deg(cf_control_inputs.pitch); //linear_x -> pitch
	msg.linear.y  = rad2Deg(cf_control_inputs.roll);   // linear_y -> roll
	msg.linear.z  = PWM;
	msg.angular.z = rad2Deg(cf_control_inputs.yaw_rate);
	
	ROS_INFO_STREAM("RPM: " << RPM << " " <<"PWM: " << msg.linear.z << " " << "roll: " << msg.linear.y << " " << "pitch: " << msg.linear.x << " " << "yaw rate: " << msg.angular.z << endl);

	m_pubNav.publish(msg);
/*
	for(ii=0; ii< N; ii++){
	  
	   ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", (void *)(acados_out.xAllStages));
		  
 	  // Log open-loop trajectory
	  ofstream trajLog("traj_openloop.txt", std::ios_base::app | std::ios_base::out);

	  if (trajLog.is_open()){
	    trajLog << acados_out.xAllStages[xq] << " ";
	    trajLog << acados_out.xAllStages[yq] << " ";
	    trajLog << acados_out.xAllStages[zq] << " ";
	    trajLog << acados_out.xAllStages[dxq] << " ";
	    trajLog << acados_out.xAllStages[dyq] << " ";
	    trajLog << acados_out.xAllStages[dzq] << " ";
	    trajLog << acados_out.xAllStages[yaw] << " ";
	    trajLog << endl;
	    
	    trajLog.close();
	  }      
	}
 	
 	// Log current state x0 and acados next state
	ofstream motorsLog("full_log.txt", std::ios_base::app | std::ios_base::out);

	if (motorsLog.is_open()){
	  motorsLog << xq_des << " ";
	  motorsLog << yq_des << " ";
	  motorsLog << zq_des << " ";
	  motorsLog << actual_roll << " ";
	  motorsLog << -actual_pitch << " ";
	  motorsLog << actual_yaw  << " ";
	  motorsLog << msg.linear.y << " ";
	  motorsLog << msg.linear.x << " ";
	  motorsLog << msg.angular.z << " ";
	  motorsLog << actual_x << " ";
	  motorsLog << actual_y << " ";
	  motorsLog << actual_z << " ";
	  motorsLog << x0_sign[dxq] << " ";
	  motorsLog << x0_sign[dyq] << " ";
	  motorsLog << x0_sign[dzq] << " ";
	  motorsLog << x0_sign[yaw] << " ";
	  motorsLog << acados_out.x2[xq] << " ";
	  motorsLog << acados_out.x2[yq] << " ";
	  motorsLog << acados_out.x2[zq] << " ";
	  motorsLog << acados_out.x2[dxq] << " ";
	  motorsLog << acados_out.x2[dyq] << " ";
	  motorsLog << acados_out.x2[dzq] << " ";
	  motorsLog << acados_out.x2[yaw] << " ";
	  motorsLog << acados_out.u1[thrust] << " ";
	  motorsLog << acados_out.u1[roll] << " ";
	  motorsLog << acados_out.u1[pitch] << " ";
	  motorsLog << acados_out.u1[yaw_rate] << " ";
	  motorsLog << msg.linear.z<< " ";
	  motorsLog << endl;
	  
	  motorsLog.close();
	}*/
      }
      catch (int acados_status){
	cout << "An exception occurred. Exception Nr. " << acados_status << '\n';
      }
    }

private:
    ros::Publisher m_cf_lvb;
    ros::Publisher m_acados_position;
    ros::Publisher m_motvel_pub;
    ros::Publisher m_pubNav;
   
    ros::Subscriber m_imu_sub;
    ros::Subscriber m_eRaptor_sub;
    ros::Subscriber m_euler_sub;

    unsigned int k,i,j,ii;

    double uss,mq, Ct;

    double vx,vy,vz;
    std::vector<double> x_samples;
    std::vector<double> y_samples;
    std::vector<double> z_samples;
    std::vector<double> vx_filter_samples;
    std::vector<double> vy_filter_samples;
    std::vector<double> vz_filter_samples;
    double t0;

    // Variables of the nmpc control process
    double x0_sign[NX];
    double yref_sign[(NY*N)+NY];

    // Variables for dynamic reconfigure
    double xq_des,yq_des,zq_des;
    
    // acados struct
    solver_input acados_in;
    solver_output acados_out;
    
    int acados_status;

    // Variables for reading the IMU data
    float actual_wx;
    float actual_wy;
    float actual_wz;
    float actual_roll;
    float actual_pitch;
    float actual_yaw;

    // Variables for eRaptor data
    float actual_x;
    float actual_y;
    float actual_z;  
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "acados_debug");

  // Read parameters
  ros::NodeHandle n("~");
  double frequency;
  n.param("frequency", frequency, 50.0);

  NMPC nmpc(n);
  nmpc.run(frequency);

  return 0;
}
