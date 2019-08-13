#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>


// Dynamic reconfirgure
#include <dynamic_reconfigure/server.h>
#include <crazyflie_controller/WeightMatricesConfig.h>
#include <boost/thread.hpp>
#include "boost/thread/mutex.hpp"

// Matrices and vectors
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// standard
#include <iostream>

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
#define WEIGHT_MATRICES 1


class NMPC
{
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
	// publisher for the motion capture system position
	m_eRaptor_sub  	 	= nh.subscribe("/crazyflie/external_position", 5, &NMPC::eRaptorCallback, this);
	// publisher for the IMU linear acceleration and angular velocities from acc and gyro
	m_imu_sub		= nh.subscribe("/crazyflie/imu", 5, &NMPC::imuCallback, this);
	// publisher for the IMU stabilizer euler angles
	m_euler_sub		= nh.subscribe("/crazyflie/euler_angles", 5, &NMPC::eulerCallback, this);
	// publisher for the control inputs of acados (motor speeds to be applied)
	m_motvel_pub 		= nh.advertise<geometry_msgs::Quaternion>("/crazyflie/acados_motvel",1);
	// publisher for acados z output for the 1st & N shooting node + z from the mocap
	m_acados_position       = nh.advertise<geometry_msgs::Vector3>("/crazyflie/acados_position",1);
	// publisher for the current value of the quaternion
	m_cf_quat 		= nh.advertise<geometry_msgs::Quaternion>("/crazyflie/quat",1);
	// publisher for the current value of the linear velocities
	m_cf_lvb 		= nh.advertise<geometry_msgs::Vector3>("/crazyflie/linear_velo",1);
	// publisher for the current value of the angular velocities
	m_cf_avb 		= nh.advertise<geometry_msgs::Vector3>("/crazyflie/angular_velo",1);

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

	// Set elapsed time to zero initially
	t0 = 0.0;
	
	// Steady-state control input value
	mq = 33e-3; 	  		// [Kg]
	Ct = 1e+6*3.1582e-10;  		// [N/Krpm^2]
	uss = sqrt((mq*g0)/(4*Ct)); 	// 16000
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &NMPC::iteration, this);
	
	
	ROS_DEBUG("Setting up the dynamic reconfigure panel and server");
	dynamic_reconfigure::Server<crazyflie_controller::WeightMatricesConfig> server;
	dynamic_reconfigure::Server<crazyflie_controller::WeightMatricesConfig>::CallbackType f;
	f = boost::bind(&NMPC::callback_dynamic_reconfigure, this, _1, _2);
	server.setCallback(f);
  
        ros::spin();
    }
    
    void callback_dynamic_reconfigure(crazyflie_controller::WeightMatricesConfig &config, uint32_t level){
      
      if (level & WEIGHT_MATRICES){
	ROS_INFO("Changing the weight of NMPC matrices!");
	Wdiag_xq 	= config.Wdiag_xq;
	Wdiag_yq 	= config.Wdiag_yq;
	Wdiag_zq 	= config.Wdiag_zq;
	Wdiag_q1 	= config.Wdiag_q1;
	Wdiag_q2 	= config.Wdiag_q2;
	Wdiag_q3 	= config.Wdiag_q3;
	Wdiag_q4 	= config.Wdiag_q4;
	Wdiag_vbx 	= config.Wdiag_vbx;
	Wdiag_vby 	= config.Wdiag_vby;
	Wdiag_vbz 	= config.Wdiag_vbz;
	Wdiag_wx 	= config.Wdiag_wx;
	Wdiag_wy 	= config.Wdiag_wy;
	Wdiag_wz 	= config.Wdiag_wz;
	Wdiag_w1 	= config.Wdiag_w1;
	Wdiag_w2 	= config.Wdiag_w2;
	Wdiag_w3 	= config.Wdiag_w3;
	Wdiag_w4 	= config.Wdiag_w4;
      }
    }

private:

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
      double xN[NX];
    };

    struct solver_input{
      double x0[NX];
      double yref[(NY*N)+NY];
      double yref_e[NYN];
      double W[(NX+NU)*(NX+NU)];
      double WN[NX*NX];
    };

    struct cf_cmd_vel{
	int thrust;
	double roll;
	double pitch;
 	double yawr;
    };

    int acados_status;

    // Estimate the velocity using Euler initially and after a low-pass filter
    double linearVelocity(std::vector <double> q_samples, double Ts, double elapsed_time, std::vector <double> dq_samples) {

	double dq = 0;
// 	if (elapsed_time > 0.01) dq = 1.573*dq_samples[4] - 0.6188*dq_samples[3] + 22.65*q_samples[4] - 22.65*q_samples[3];
// 	else dq = (q_samples[4] - q_samples[3]) / Ts;
	if (elapsed_time > 0.1) dq = 1.573*dq_samples[4] - 0.6188*dq_samples[3] + 22.65*q_samples[4] - 22.65*q_samples[3];
	else dq = 0.0;

	return dq;
    }
    

    euler quatern2euler(Quaterniond* q){

	euler angle;

// 	double R11 = 2*q->w()*q->w()-1+2*q->x()*q->x();
// 	double R21 = 2*(q->x()*q->y()-q->w()*q->z());
// 	double R31 = 2*(q->x()*q->z()+q->w()*q->y());
// 	double R32 = 2*(q->y()*q->z()-q->w()*q->x());
// 	double R33 = 2*q->w()*q->w()-1+2*q->z()*q->z();
	
	// According to the rotation matrix found in the firmware of the crazyflie
	double R11 = 2*q->w()*q->w()-1+2*q->x()*q->x();
	double R21 = 2*(q->x()*q->y()+q->w()*q->z());
	double R31 = 2*(q->x()*q->z()-q->w()*q->y());
	double R32 = 2*(q->y()*q->z()+q->w()*q->x());
	double R33 = 2*q->w()*q->w()-1+2*q->z()*q->z();

	double phi   = atan2(R32, R33);
	double theta = -atan(R31/sqrt(1-R31*R31));
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
	  q.x() = cosPsi*cosTheta*sinPhi - sinPsi*sinTheta*cosPhi;
	  q.y() = cosPsi*sinTheta*cosPhi + sinPsi*cosTheta*sinPhi;
	  q.z() = sinPsi*cosTheta*cosPhi - cosPsi*sinTheta*sinPhi;

	  return q;
    }

    void estimateWordLinearVelocities(float dt, float delta){
      
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

	  vx = linearVelocity(x_samples, dt, delta, vx_filter_samples);
	  vy = linearVelocity(y_samples, dt, delta, vy_filter_samples);
	  vz = linearVelocity(z_samples, dt, delta, vz_filter_samples);

	  // passing to the state vector
	  x0_sign[vbx] = vx;
	  x0_sign[vby] = vy;
	  x0_sign[vbz] = vz;

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

    Vector3d rotateLinearVeloE2B(Quaterniond* q){

	 // This is the convertion between
	 // quaternion orientation to rotation matrix
	 // from BODY to EARTH according the crazyflie firmware
// 	 Matrix3d Sq;
// 	 Vector3d vi,vb;
// 
// 	 double S11 = 2*(q->w()*q->w()+q->x()*q->x())-1;;
// 	 double S12 = 2*(q->x()*q->y()-q->w()*q->z());
// 	 double S13 = 2*(q->x()*q->z()+q->w()*q->y());
// 	 double S21 = 2*(q->x()*q->y()+q->w()*q->z());
// 	 double S22 = 2*(q->w()*q->w()+q->y()*q->y())-1;
// 	 double S23 = 2*(q->y()*q->z()-q->w()*q->x());
// 	 double S31 = 2*(q->x()*q->z()-q->w()*q->y());
// 	 double S32 = 2*(q->y()*q->z()+q->w()*q->x());
// 	 double S33 = 2*(q->w()*q->w()+q->z()*q->z())-1;
      
         // This is the convertion between
	 // quaternion orientation to rotation matrix
	 // from EARTH to BODY (considering ZYX euler sequence)
	 Matrix3d Sq;
	 Vector3d vi,vb;

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

	 vi << x0_sign[vbx],x0_sign[vby],x0_sign[vbz];

	 vb = Sq*vi;
	 return vb;
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
	actual_pitch = -msg->vector.y; // the pitch coming from the IMU sees to have inverted sign
	actual_yaw   = msg->vector.z;
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

    int krpm2pwm(double Krpm){

      int pwm = ((Krpm*1000)-4070.3)/0.2685;
      return pwm;
    }


    void iteration(const ros::TimerEvent& e){

#if 0

#endif
#if 1
      if(e.last_real.isZero()) {
		t0 = e.current_real.toSec();
      }

      double dt = e.current_real.toSec() - e.last_real.toSec();
      

		// Update reference
	   	for (k = 0; k < N+1; k++) {
		      yref_sign[k * NY + 0] = 0.0; 	// xq
		      yref_sign[k * NY + 1] = 0.0;	// yq
		      yref_sign[k * NY + 2] = 0.0;	// zq
		      yref_sign[k * NY + 3] = 1.0;	// q1
		      yref_sign[k * NY + 4] = 0.0;	// q2
		      yref_sign[k * NY + 5] = 0.0;	// q3
		      yref_sign[k * NY + 6] = 0.0;	// q4
		      yref_sign[k * NY + 7] = 0.0;	// vbx
		      yref_sign[k * NY + 8] = 0.0;	// vby
		      yref_sign[k * NY + 9] = 0.0;	// vbz
		      yref_sign[k * NY + 10] = 0.0;	// wx
		      yref_sign[k * NY + 11] = 0.0;	// wy
		      yref_sign[k * NY + 12] = 0.0;	// wz
		      yref_sign[k * NY + 13] = uss;	// w1
		      yref_sign[k * NY + 14] = uss;	// w2
		      yref_sign[k * NY + 15] = uss;	// w3
		      yref_sign[k * NY + 16] = uss;	// w4
		}

// 		for (ii = 0; ii < ((NU+NX)*(NU*NX)); ii++) {
// 		      acados_in.W[ii] = 0.0;
// 		}
// 		for (ii = 0; ii < ((NX)*(NX)); ii++) {
// 		      acados_in.WN[ii] = 0.0;
// 		}
// 
// 		acados_in.W[0+0*(NU+NX)]   = Wdiag_xq;
// 		acados_in.W[1+1*(NU+NX)]   = Wdiag_yq;
// 		acados_in.W[2+2*(NU+NX)]   = Wdiag_zq;
// 		acados_in.W[3+3*(NU+NX)]   = Wdiag_q1;
// 		acados_in.W[4+4*(NU+NX)]   = Wdiag_q2;
// 		acados_in.W[5+5*(NU+NX)]   = Wdiag_q3;
// 		acados_in.W[6+6*(NU+NX)]   = Wdiag_q4;
// 		acados_in.W[7+7*(NU+NX)]   = Wdiag_vbx;
// 		acados_in.W[8+8*(NU+NX)]   = Wdiag_vby;
// 		acados_in.W[9+9*(NU+NX)]   = Wdiag_vbz;
// 		acados_in.W[10+10*(NU+NX)] = Wdiag_wx;
// 		acados_in.W[11+11*(NU+NX)] = Wdiag_wy;
// 		acados_in.W[12+12*(NU+NX)] = Wdiag_wz;
// 		acados_in.W[13+13*(NU+NX)] = Wdiag_w1;
// 		acados_in.W[14+14*(NU+NX)] = Wdiag_w2;
// 		acados_in.W[15+15*(NU+NX)] = Wdiag_w3;
// 		acados_in.W[16+16*(NU+NX)] = Wdiag_w4;
// 
// 		acados_in.WN[0+0*(NX)]   = Wdiag_xq;
// 		acados_in.WN[1+1*(NX)]   = Wdiag_yq;
// 		acados_in.WN[2+2*(NX)]   = Wdiag_zq;
// 		acados_in.WN[3+3*(NX)]   = Wdiag_q1;
// 		acados_in.WN[4+4*(NX)]   = Wdiag_q2;
// 		acados_in.WN[5+5*(NX)]   = Wdiag_q3;
// 		acados_in.WN[6+6*(NX)]   = Wdiag_q4;
// 		acados_in.WN[7+7*(NX)]   = Wdiag_vbx;
// 		acados_in.WN[8+8*(NX)]   = Wdiag_vby;
// 		acados_in.WN[9+9*(NX)]   = Wdiag_vbz;
// 		acados_in.WN[10+10*(NX)] = Wdiag_wx;
// 		acados_in.WN[11+11*(NX)] = Wdiag_wy;
// 		acados_in.WN[12+12*(NX)] = Wdiag_wz;

		// Storing inertial positions in state vector
		x0_sign[xq] = actual_x;
		x0_sign[yq] = actual_y;
		x0_sign[zq] = actual_z;

		// Get the euler angles from the onboard stabilizer
		eu.phi   = deg2Rad(actual_roll);
		eu.theta = deg2Rad(actual_pitch);
		eu.psi   = deg2Rad(actual_yaw);

		// Convert IMU euler angles to quaternion
		Quaterniond q_imu = euler2quatern(eu);

		x0_sign[q1] = q_imu.w();
		x0_sign[q2] = q_imu.x();
		x0_sign[q3] = q_imu.y();
		x0_sign[q4] = q_imu.z();

		estimateWordLinearVelocities(dt,t0);

		Vector3d vb_mat;
		vb_mat = rotateLinearVeloE2B(&q_imu);

     	        // overwriting linear velocities in the body frame in state vector
		x0_sign[vbx] = vb_mat[0];
		x0_sign[vby] = vb_mat[1];
		x0_sign[vbz] = vb_mat[2];

		// Storing body angular velocities in state vector
		x0_sign[wx] = actual_wx;
		x0_sign[wy] = actual_wy;
		x0_sign[wz] = actual_wz;

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

		// update reference
		for (ii = 0; ii < N; ii++) {
			    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "yref", acados_in.yref + ii*NY);
			    // weights
			    //ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "W", acados_in.W);
		}

		//ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "WN", acados_in.WN);

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

		// Select the set of optimal state to calculate the cf control inputs
		Quaterniond q_acados_out;
		q_acados_out.w() = acados_out.x1[q1];
		q_acados_out.x() = acados_out.x1[q2];
		q_acados_out.y() = acados_out.x1[q3];
		q_acados_out.z() = acados_out.x1[q4];


		euler eu_imu;
		eu_imu = quatern2euler(&q_acados_out);

		geometry_msgs::Twist msg;
		msg.linear.x  = rad2Deg(eu_imu.theta);
		msg.linear.y  = rad2Deg(eu_imu.phi);
		msg.linear.z  = krpm2pwm((acados_out.u0[w1]+acados_out.u0[w2]+acados_out.u0[w3]+acados_out.u0[w4])/4);
		msg.angular.z  = rad2Deg(acados_out.x1[wz]);

		m_pubNav.publish(msg);

    }

 #endif

private:
    ros::Publisher m_cf_quat;
    ros::Publisher m_cf_lvb;
    ros::Publisher m_cf_avb;
    ros::Publisher m_acados_position;
    ros::Publisher m_motvel_pub;
    ros::Publisher m_pubNav;
    ros::Publisher m_cfStates_pub;
    ros::Publisher m_acadosOut_pub;

    ros::Subscriber m_imu_sub;
    ros::Subscriber m_eRaptor_sub;
    ros::Subscriber m_euler_sub;

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
    double Wdiag_xq,Wdiag_yq,Wdiag_zq;
    double Wdiag_q1,Wdiag_q2,Wdiag_q3,Wdiag_q4;
    double Wdiag_vbx,Wdiag_vby,Wdiag_vbz;
    double Wdiag_wx,Wdiag_wy,Wdiag_wz;
    double Wdiag_w1,Wdiag_w2,Wdiag_w3,Wdiag_w4;
    
    // acados struct
    solver_input acados_in;
    solver_output acados_out;

    // Variables to be used in convertions
    Quaterniond q,q_rot;
    euler eu;

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
