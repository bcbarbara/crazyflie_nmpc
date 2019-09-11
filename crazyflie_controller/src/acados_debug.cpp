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
#include <typeinfo>

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
#include "crazyflie_full_model/c_generated_code/crazyflie_model/crazyflie_model.h"
#include "crazyflie_full_model/c_generated_code/acados_solver_crazyflie.h"
// #include "crazyflie_model.h"
// #include "acados_solver_crazyflie.h"

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
#define NX 	13 	/* Number of differential state variables.  */
#define NU 	4	/* Number of control inputs. */
#define NY 	17	/* Number of measurements/references on nodes 0..N-1. */
#define NYN 	13	/* Number of measurements/references on node N. */

#define pi      3.14159265358979323846
#define g0	9.80665

#define CONTROLLER 1

// delay compensation
#define FIXED_U0 1 // possible values 0, 1

class NMPC{
public:

    NMPC(const ros::NodeHandle& n, const std::string& ref_traj){

	// print log header
	ofstream trajLog("openloop_traj.txt", std::ios_base::trunc | std::ios_base::out);
	if (trajLog.is_open()) trajLog << "trajLog" << endl;
	trajLog.close();
	ofstream motorsLog("full_log.txt", std::ios_base::trunc | std::ios_base::out);
	if (trajLog.is_open()) motorsLog << "motorsLog" << endl;
	motorsLog.close();

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
	// publisher for the control inputs of acados (motor speeds to be applied)
	m_motvel_pub 		= nh.advertise<geometry_msgs::Quaternion>("/crazyflie/acados_motvel",1);
	// publisher for acados z output for the 1st & N shooting node + z from the mocap
	m_acados_position       = nh.advertise<geometry_msgs::Vector3>("/crazyflie/acados_traj",1);
	// publisher for the current value of the quaternion
	m_cf_quat 		= nh.advertise<geometry_msgs::Quaternion>("/crazyflie/quat",1);
	// publisher for the current value of the linear velocities
	m_cf_lvb 		= nh.advertise<geometry_msgs::Vector3>("/crazyflie/linear_velo",1);
	// publisher for the current value of the angular velocities
	m_cf_avb 		= nh.advertise<geometry_msgs::Vector3>("/crazyflie/angular_velo",1);
	// subscriber fro the motors rpm
	//m_motors 		= nh.subscribe("/crazyflie/log1", 5, &NMPC::motorsCallback, this);

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

	// Steady-state control input value
	mq = 33e-3; 	  		// [Kg]
	Ct = 3.25e-4;			// [N/Krpm^2]
	uss = sqrt((mq*g0)/(4*Ct));
	
	const char * c = ref_traj.c_str();
	
	// Pre-load the trajectory
	N_STEPS = readDataFromFile(c, precomputed_traj);
	if (N_STEPS == 0){
		ROS_WARN("Cannot load CasADi optimal trajectory!");
	}
	else{
		ROS_INFO_STREAM("Number of steps: " << N_STEPS << endl);
	}
	
	// Set number of trajectory iterations to zero initially
	iter = 0;
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

      if (level && CONTROLLER){
	if(config.enable_traj_tracking){
	  config.enable_regulation = false;
	  crazyflie_state = Tracking;
	}
	if(config.enable_regulation){
	  config.enable_traj_tracking = false;
	  xq_des = config.xq_des;
	  yq_des = config.yq_des;
	  zq_des = config.zq_des;
	  crazyflie_state = Regulation;
	}	
      }

      ROS_INFO_STREAM(fixed << showpos << "Quad status" << endl
				       << "NMPC for regulation: " <<  (config.enable_regulation?"ON":"off") << endl
				       << "NMPC trajectory tracker: " <<  (config.enable_traj_tracking?"ON":"off") << endl
				       << "Current regulation point: " << xq_des << ", " << yq_des << ", " << zq_des << endl);
    }

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
    
    enum cf_state{
        Regulation = 0,
        Tracking = 1,
	Position_Hold = 2
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
  
    int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data){
	std::ifstream file(fileName);
	std::string line;
	int num_of_steps = 0;

	if (file.is_open()){
		while(getline(file, line)){
			++num_of_steps;
			std::istringstream linestream( line );
			std::vector<double> linedata;
			double number;

			while( linestream >> number ){
				linedata.push_back( number );
			}
			data.push_back( linedata );
		}

		file.close();
		cout << num_of_steps << endl;
	}
	else
		return 0;

	return num_of_steps;
}

    euler quatern2euler(Quaterniond* q){

	euler angle;

	double R11 = q->w()*q->w()+q->x()*q->x()-q->y()*q->y()-q->z()*q->z();
	double R21 = 2*(q->x()*q->y()+q->w()*q->z());
	double R31 = 2*(q->x()*q->z()-q->w()*q->y());
	double R32 = 2*(q->y()*q->z()+q->w()*q->x());
	double R33 = q->w()*q->w()-q->x()*q->x()-q->y()*q->y()+q->z()*q->z();

	double phi   = atan2(R32, R33);
	double theta = asin(-R31);
	double psi   = atan2(R21, R11);

	angle.phi   = phi;
	angle.theta = theta;
	angle.psi   = psi;

	return angle;
    }

    Quaterniond euler2quatern(euler angle){

    Quaterniond q;

	  double cosPhi = cos(angle.phi*0.5);
	  double sinPhi = sin(angle.phi*0.5);

	  double cosTheta = cos(angle.theta*0.5);
	  double sinTheta = sin(angle.theta*0.5);

	  double cosPsi = cos(angle.psi*0.5);
	  double sinPsi = sin(angle.psi*0.5);

	  // Convention according the firmware of the crazyflie
	  q.w() = cosPsi*cosTheta*cosPhi + sinPsi*sinTheta*sinPhi;
	  q.x() = -(cosPsi*cosTheta*sinPhi - sinPsi*sinTheta*cosPhi);
	  q.y() = -(cosPsi*sinTheta*cosPhi + sinPsi*cosTheta*sinPhi);
	  q.z() = -(sinPsi*cosTheta*cosPhi - cosPsi*sinTheta*sinPhi);

	  if(q.w() < 0){
	    q.w() = -q.w();
	    q.vec() = -q.vec();
	  }

	  return q;
    }

    double linearVelocity(std::vector <double> q_samples, std::vector <double> dq_samples, double Ts, double elapsed_time) {
      
      // digital low-pass filter considering Ts = 15 ms
      double dq = 0;
      if (elapsed_time > 1.0) dq = 0.3306*dq_samples[4] - 0.02732*dq_samples[3] + 35.7*q_samples[4] - 35.7*q_samples[3];
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

    Vector3d rotateLinearVeloE2B(Quaterniond* q, Vector3d v_inertial){

         // This is the convertion between
	 // quaternion orientation to rotation matrix
	 // from EARTH to BODY (considering ZYX euler sequence)
 	 Matrix3d Sq;
 	 Vector3d vb;

	 double S11 = 2*(q->w()*q->w()+q->x()*q->x())-1;
	 double S12 = 2*(q->x()*q->y()+q->w()*q->z());
	 double S13 = 2*(q->x()*q->z()-q->w()*q->y());
	 double S21 = 2*(q->x()*q->y()-q->w()*q->z());
	 double S22 = 2*(q->w()*q->w()+q->y()*q->y())-1;
	 double S23 = 2*(q->y()*q->z()+q->w()*q->x());
	 double S31 = 2*(q->x()*q->z()+q->w()*q->y());
	 double S32 = 2*(q->y()*q->z()-q->w()*q->x());
	 double S33 = 2*(q->w()*q->w()+q->z()*q->z())-1;


	 Sq << S11,S12,S13,
	       S21,S22,S23,
	       S31,S32,S33;

	 vb = Sq*v_inertial;

	 return vb;
    }

    double deg2Rad(double deg) {
	  return deg / 180.0 * pi;
    }

    double rad2Deg(double rad) {
	  return rad * 180.0 / pi;
    }
    
    void motorsCallback(const crazyflie_controller::GenericLogDataConstPtr& msg){
    
	// motors rpm
	actual_m1 = msg->values[0];
	actual_m2 = msg->values[1];
	actual_m3 = msg->values[2];
	actual_m4 = msg->values[3];
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

    int krpm2pwm(double Krpm){

      int pwm = ((Krpm*1000)-4070.3)/0.2685;
      return pwm;
    }


    void iteration(const ros::TimerEvent& e){

      if(e.last_real.isZero()) {
	t0 = e.current_real.toSec();
      }

      double dt = e.current_real.toSec() - e.last_real.toSec();

      try{
	switch(crazyflie_state){
	
	  case Regulation: 
	  {
	    // Update regulation point
	    for (k = 0; k < N+1; k++){
		  yref_sign[k * NY + 0] = xq_des; 	// xq
		  yref_sign[k * NY + 1] = yq_des;	// yq
		  yref_sign[k * NY + 2] = zq_des;	// zq
		  yref_sign[k * NY + 3] = 1.00;		// q1
		  yref_sign[k * NY + 4] = 0.00;		// q2
		  yref_sign[k * NY + 5] = 0.00;		// q3
		  yref_sign[k * NY + 6] = 0.00;		// q4
		  yref_sign[k * NY + 7] = 0.00;		// vbx
		  yref_sign[k * NY + 8] = 0.00;		// vby
		  yref_sign[k * NY + 9] = 0.00;		// vbz
		  yref_sign[k * NY + 10] = 0.00;	// wx
		  yref_sign[k * NY + 11] = 0.00;	// wy
		  yref_sign[k * NY + 12] = 0.00;	// wz
		  yref_sign[k * NY + 13] = uss;		// w1
		  yref_sign[k * NY + 14] = uss;		// w2
		  yref_sign[k * NY + 15] = uss;		// w3
		  yref_sign[k * NY + 16] = uss;		// w4
	    }
	    break;	    
	  }
	  
	  case Tracking:
	  {
	    if(iter < N_STEPS-N){
		// Update reference
 		for (k = 0; k < N+1; k++) {
 		      yref_sign[k * NY + 0] = precomputed_traj[iter + k][xq];
		      yref_sign[k * NY + 1] = precomputed_traj[iter + k][yq];
		      yref_sign[k * NY + 2] = precomputed_traj[iter + k][zq];
		      yref_sign[k * NY + 3] = precomputed_traj[iter + k][q1];
		      yref_sign[k * NY + 4] = precomputed_traj[iter + k][q2];
		      yref_sign[k * NY + 5] = precomputed_traj[iter + k][q3];
		      yref_sign[k * NY + 6] = precomputed_traj[iter + k][q4];
		      yref_sign[k * NY + 7] = precomputed_traj[iter + k][vbx];
		      yref_sign[k * NY + 8] = precomputed_traj[iter + k][vby];
		      yref_sign[k * NY + 9] = precomputed_traj[iter + k][vbz];
		      yref_sign[k * NY + 10] = precomputed_traj[iter + k][wx];
		      yref_sign[k * NY + 11] = precomputed_traj[iter + k][wy];
		      yref_sign[k * NY + 12] = precomputed_traj[iter + k][wz];
		      yref_sign[k * NY + 13] = precomputed_traj[iter + k][13];
		      yref_sign[k * NY + 14] = precomputed_traj[iter + k][14];
		      yref_sign[k * NY + 15] = precomputed_traj[iter + k][15];
		      yref_sign[k * NY + 16] = precomputed_traj[iter + k][16];
 		}
		++iter;
		//cout << iter << endl;
	    }
 	    else crazyflie_state = Position_Hold;
	    break;
	  }
	  
	  case Position_Hold:
	  {
	    ROS_INFO("Holding last position of the trajectory.");
	    // Get last point of tracketory and hold 
	    for (k = 0; k < N+1; k++){
		    yref_sign[k * NY + 0] = precomputed_traj[N_STEPS-1][xq]; 
		    yref_sign[k * NY + 1] = precomputed_traj[N_STEPS-1][yq]; 
		    yref_sign[k * NY + 2] = precomputed_traj[N_STEPS-1][zq]; 
		    yref_sign[k * NY + 3] = 1.00;
		    yref_sign[k * NY + 4] = 0.00;
		    yref_sign[k * NY + 5] = 0.00;
		    yref_sign[k * NY + 6] = 0.00;
		    yref_sign[k * NY + 7] = 0.00;
		    yref_sign[k * NY + 8] = 0.00;
		    yref_sign[k * NY + 9] = 0.00;
		    yref_sign[k * NY + 10] = 0.00;
		    yref_sign[k * NY + 11] = 0.00;
		    yref_sign[k * NY + 12] = 0.00;
		    yref_sign[k * NY + 13] = uss;
		    yref_sign[k * NY + 14] = uss;
		    yref_sign[k * NY + 15] = uss;
		    yref_sign[k * NY + 16] = uss;
	      }
	  }
	  break;
	}
	
	//ROS_INFO_STREAM(crazyflie_state << endl);
	// Storing inertial positions in state vector
	x0_sign[xq] = actual_x;
	x0_sign[yq] = actual_y;
	x0_sign[zq] = actual_z;

	// Get the euler angles from the onboard stabilizer
	euler eu;
	eu.phi   = deg2Rad(actual_roll);
	eu.theta = deg2Rad(actual_pitch);
	eu.psi   = deg2Rad(actual_yaw);

	// Convert IMU euler angles to quaternion
	Quaterniond q_imu = euler2quatern(eu);
	q_imu.normalize();

	x0_sign[q1] = q_imu.w();
	x0_sign[q2] = q_imu.x();
	x0_sign[q3] = q_imu.y();
	x0_sign[q4] = q_imu.z();

	Vector3d vi_mat;
	vi_mat = estimateWordLinearVelocities(dt,t0);

	Vector3d vb_mat;
	vb_mat = rotateLinearVeloE2B(&q_imu,vi_mat);

	// overwriting linear velocities in the body frame in state vector
	x0_sign[vbx] = vb_mat[0];
	x0_sign[vby] = vb_mat[1];
	x0_sign[vbz] = vb_mat[2];

	// Storing body angular velocities in state vector
	x0_sign[wx] = actual_wx;
	x0_sign[wy] = actual_wy;
	x0_sign[wz] = actual_wz;

	// Up to this point we already stored the 13 states required for the NMPC. So advertise them!
	geometry_msgs::Quaternion cf_st_quat; // publisher for quaternion
	geometry_msgs::Vector3    cf_st_lvb;  // publisher for the linear velocities w.r.t. the body frame
	geometry_msgs::Vector3    cf_st_avb;  // publisher for the angular velocities w.r.t. the body frame

	cf_st_quat.w = x0_sign[q1];
	cf_st_quat.x = x0_sign[q2];
	cf_st_quat.y = x0_sign[q3];
	cf_st_quat.z = x0_sign[q4];
	cf_st_lvb.x  = x0_sign[vbx];
	cf_st_lvb.y  = x0_sign[vby];
	cf_st_lvb.z  = x0_sign[vbz];
	cf_st_avb.x  = x0_sign[wx];
	cf_st_avb.y  = x0_sign[wy];
	cf_st_avb.z  = x0_sign[wz];

	m_cf_quat.publish(cf_st_quat);
	m_cf_lvb.publish(cf_st_lvb);
	m_cf_avb.publish(cf_st_avb);

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
		    //cout <<  "yref: " << acados_in.yref[i] << endl;
	    }
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

	// Publish acados output
	geometry_msgs::Quaternion _acadosOut;
	if (FIXED_U0 == 1) {
	  _acadosOut.w =  acados_out.u1[w1];
	  _acadosOut.x =  acados_out.u1[w2];
	  _acadosOut.y =  acados_out.u1[w3];
	  _acadosOut.z =  acados_out.u1[w4];
	} else {
	  _acadosOut.w =  acados_out.u0[w1];
	  _acadosOut.x =  acados_out.u0[w2];
	  _acadosOut.y =  acados_out.u0[w3];
	  _acadosOut.z =  acados_out.u0[w4];
	}
	m_motvel_pub.publish(_acadosOut);

	// Select the set of optimal states to calculate the real cf control inputs
	Quaterniond q_acados_out;
	q_acados_out.w() = acados_out.x2[q1];
	q_acados_out.x() = acados_out.x2[q2];
	q_acados_out.y() = acados_out.x2[q3];
	q_acados_out.z() = acados_out.x2[q4];
	q_acados_out.normalize();

	// Convert acados output quaternion to desired euler angles
	euler eu_imu;
	eu_imu = quatern2euler(&q_acados_out);

	// Publish real control inputs
	geometry_msgs::Twist msg;
	msg.linear.x  = -rad2Deg(eu_imu.theta); //linear_x -> pitch
	msg.linear.y  = rad2Deg(eu_imu.phi);  // linear_y -> roll
	msg.linear.z  = krpm2pwm((acados_out.u1[w1]+acados_out.u1[w2]+acados_out.u1[w3]+acados_out.u1[w4])/4);
	msg.angular.z  = rad2Deg(acados_out.x1[wz]);

	m_pubNav.publish(msg);

	for(ii=0; ii< N; ii++){

	   ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", (void *)(acados_out.xAllStages));

 	  // Log open-loop trajectory
	  ofstream trajLog("openloop_traj.txt", std::ios_base::app | std::ios_base::out);

	  if (trajLog.is_open()){
	    trajLog << acados_out.xAllStages[xq] << " ";
	    trajLog << acados_out.xAllStages[yq] << " ";
	    trajLog << acados_out.xAllStages[zq] << " ";
// 	    trajLog << acados_out.xAllStages[q1] << " ";
// 	    trajLog << acados_out.xAllStages[q2] << " ";
// 	    trajLog << acados_out.xAllStages[q3] << " ";
// 	    trajLog << acados_out.xAllStages[q4] << " ";
// 	    trajLog << acados_out.xAllStages[vbx] << " ";
// 	    trajLog << acados_out.xAllStages[vby] << " ";
// 	    trajLog << acados_out.xAllStages[vbz] << " ";
// 	    trajLog << acados_out.xAllStages[wx] << " ";
// 	    trajLog << acados_out.xAllStages[wy] << " ";
// 	    trajLog << acados_out.xAllStages[wz] << " ";
	    trajLog << endl;

	    trajLog.close();
	  }
	}
	
 	// Log current state x0 and acados output x1 and x2
	ofstream motorsLog("full_log.txt", std::ios_base::app | std::ios_base::out);

	if (motorsLog.is_open()){
	  motorsLog << actual_roll << " ";
	  motorsLog << -actual_pitch << " ";
	  motorsLog << actual_yaw  << " ";
	  motorsLog << msg.linear.y << " ";
	  motorsLog << msg.linear.x << " ";
	  motorsLog << msg.angular.z << " ";
	  motorsLog << msg.linear.z<< " ";
	  motorsLog << actual_x << " ";
	  motorsLog << actual_y << " ";
	  motorsLog << actual_z << " ";
	  motorsLog << x0_sign[q1] << " ";
	  motorsLog << x0_sign[q2] << " ";
	  motorsLog << x0_sign[q3] << " ";
	  motorsLog << x0_sign[q4] << " ";
	  motorsLog << x0_sign[vbx] << " ";
	  motorsLog << x0_sign[vby] << " ";
	  motorsLog << x0_sign[vbz] << " ";
	  motorsLog << x0_sign[wx] << " ";
	  motorsLog << x0_sign[wy] << " ";
	  motorsLog << x0_sign[wz] << " ";
	  motorsLog << acados_out.x2[xq] << " ";
	  motorsLog << acados_out.x2[yq] << " ";
	  motorsLog << acados_out.x2[zq] << " ";
	  motorsLog << acados_out.x2[q1] << " ";
	  motorsLog << acados_out.x2[q2] << " ";
	  motorsLog << acados_out.x2[q3] << " ";
	  motorsLog << acados_out.x2[q4] << " ";
	  motorsLog << acados_out.x2[vbx] << " ";
	  motorsLog << acados_out.x2[vby] << " ";
	  motorsLog << acados_out.x2[vbz] << " ";
	  motorsLog << acados_out.x2[wx] << " ";
	  motorsLog << acados_out.x2[wy] << " ";
	  motorsLog << acados_out.x2[wz] << " ";
	  motorsLog << acados_out.u1[w1] << " ";
	  motorsLog << acados_out.u1[w2] << " ";
	  motorsLog << acados_out.u1[w3] << " ";
	  motorsLog << acados_out.u1[w4] << " ";
	  motorsLog << precomputed_traj[iter][xq] << " ";
	  motorsLog << precomputed_traj[iter][yq] << " ";
	  motorsLog << precomputed_traj[iter][zq] << " ";
	  motorsLog << precomputed_traj[iter][q1] << " ";
	  motorsLog << precomputed_traj[iter][q2] << " ";
	  motorsLog << precomputed_traj[iter][q3] << " ";
	  motorsLog << precomputed_traj[iter][q4] << " ";
	  motorsLog << precomputed_traj[iter][vbx] << " ";
	  motorsLog << precomputed_traj[iter][vby] << " ";
	  motorsLog << precomputed_traj[iter][vbz] << " ";
	  motorsLog << precomputed_traj[iter][wx] << " ";
	  motorsLog << precomputed_traj[iter][wy] << " ";
	  motorsLog << precomputed_traj[iter][wz] << " ";
	  motorsLog << precomputed_traj[iter][13] << " ";
	  motorsLog << precomputed_traj[iter][14] << " ";
	  motorsLog << precomputed_traj[iter][15] << " ";
	  motorsLog << precomputed_traj[iter][16] << " ";
	  motorsLog << acados_out.cpu_time << " ";
	  motorsLog << endl;

	  motorsLog.close();
	}
      }
      catch (int acados_status){
	cout << "An exception occurred. Exception Nr. " << acados_status << '\n';
      }
    }

private:
    ros::Publisher m_cf_quat;
    ros::Publisher m_cf_lvb;
    ros::Publisher m_cf_avb;
    ros::Publisher m_acados_position;
    ros::Publisher m_motvel_pub;
    ros::Publisher m_pubNav;

    // Variables for joy callback
    double joy_roll,joy_pitch,joy_yaw;
    double joy_thrust;

    int32_t actual_m1,actual_m2,actual_m3,actual_m4;

    ros::Subscriber m_imu_sub;
    ros::Subscriber m_eRaptor_sub;
    ros::Subscriber m_euler_sub;
    ros::Subscriber m_motors;

    unsigned int k,i,j,ii;

    float uss,Ct,mq;

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
    
    // For dynamic reconfigure
    cf_state crazyflie_state;
    
    // Variable for storing he optimal trajectory
    std::vector<std::vector<double>> precomputed_traj;
    int N_STEPS,iter;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "acados_debug");

  // Read parameters
  ros::NodeHandle n("~");
  double frequency;
  n.param("frequency", frequency, 65.0);
  
  std::string ref_traj;
  n.getParam("ref_traj", ref_traj);
  

  NMPC nmpc(n,ref_traj);
  nmpc.run(frequency);

  return 0;
}
