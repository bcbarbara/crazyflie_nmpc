#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>


#include "crazyflie_controller/GenericLogData.h"

// standard
#include <iostream>
#include <sstream>
#include <fstream>
#include <ios>

// Matrices and vectors
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


// acados dim defines
#define N 	50 	/* Number of intervals in the horizon. */
#define NX 	13 	/* Number of differential state variables.  */
#define NU 	4	/* Number of control inputs. */
#define NY 	17	/* Number of measurements/references on nodes 0..N-1. */
#define NYN 	13	/* Number of measurements/references on node N. */

#define pi      3.14159265358979323846
#define g0	9.80665


using namespace Eigen;
using std::ofstream;
using std::cout;
using std::endl;
using std::fixed;
using std::showpos;


class meas_vec
{
public:

    meas_vec(const ros::NodeHandle& n){

        ros::NodeHandle nh;
	// subscriber for the motion capture system position
	m_eRaptor_sub  	 	= nh.subscribe("/crazyflie/external_position", 5, &meas_vec::eRaptorCallback, this);
	// publisher for the IMU stabilizer euler angles
	m_euler_sub		= nh.subscribe("/crazyflie/euler_angles", 5, &meas_vec::eulerCallback, this);
	// subscriber for the IMU linear acceleration and angular velocities from acc and gyro
	m_imu_sub		= nh.subscribe("/crazyflie/imu", 5, &meas_vec::imuCallback, this);
	// publisher for the current value of the quaternion
	m_quat_pub 		= nh.advertise<geometry_msgs::Quaternion>("/crazyflie/quat",1);
	// publisher for the current value of the linear velocities
	m_lvb_pub 		= nh.advertise<geometry_msgs::Vector3>("/crazyflie/linear_velo",1);
	// publisher for the current value of the angular velocities
	m_avb_pub 		= nh.advertise<geometry_msgs::Vector3>("/crazyflie/angular_velo",1);
	// subscriber fro the motors rpm
	m_motors 		= nh.subscribe("/crazyflie/log1", 5, &meas_vec::motorsCallback, this);
	
	
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

    }
    
    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &meas_vec::iteration, this);
	
        ros::spin();
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
	actual_pitch = -msg->vector.y;
	actual_yaw   = msg->vector.z;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
      
	// Gyro angular rates
	actual_wx  = msg->angular_velocity.x;
	actual_wy  = msg->angular_velocity.y;
	actual_wz  = msg->angular_velocity.z;
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

    Vector3d rotateLinearVeloE2B(Quaterniond* q, Vector3d v_inertial){

	 // This is the convertion between
	 // quaternion orientation to rotation matrix
	 // from BODY to EARTH according the crazyflie firmware
// 	 Matrix3d Sq;
// 	 Vector3d vb;
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
    
    void iteration(const ros::TimerEvent& e){
	  
      if(e.last_real.isZero()) {
	t0 = e.current_real.toSec();
	//return;
      }
	
      double dt = e.current_real.toSec() - e.last_real.toSec();   
        
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
      
      m_quat_pub.publish(cf_st_quat);
      m_lvb_pub.publish(cf_st_lvb);
      m_avb_pub.publish(cf_st_avb);
      
      
      ofstream statesLog("states_controls.txt", std::ios_base::app | std::ios_base::out);

      if (statesLog.is_open()){

// 	statesLog << eu.phi      << " ";
// 	statesLog << eu.theta    << " ";
// 	statesLog << eu.psi      << " ";
// 	statesLog << vi_mat[0]   << " ";
// 	statesLog << vi_mat[1]   << " ";
// 	statesLog << vi_mat[2]   << " ";
	statesLog << x0_sign[xq] << " ";
	statesLog << x0_sign[yq] << " ";
	statesLog << x0_sign[zq] << " ";
	statesLog << x0_sign[q1] << " ";
	statesLog << x0_sign[q2] << " ";
	statesLog << x0_sign[q3] << " ";
	statesLog << x0_sign[q4] << " ";
	statesLog << x0_sign[vbx] << " ";
	statesLog << x0_sign[vby] << " ";
	statesLog << x0_sign[vbz] << " ";
	statesLog << x0_sign[wx] << " ";
	statesLog << x0_sign[wy] << " ";
	statesLog << x0_sign[wz] << " ";
	statesLog << actual_m1 << " ";
	statesLog << actual_m2 << " ";
	statesLog << actual_m3 << " ";
	statesLog << actual_m4 << " ";	
	statesLog << endl;
	
	statesLog.close();
      }	
    }

private:

    ros::Subscriber m_imu_sub;
    ros::Subscriber m_euler_sub;
    ros::Subscriber m_eRaptor_sub;
    ros::Subscriber m_motors;
    
    ros::Publisher m_quat_pub;
    ros::Publisher m_lvb_pub;
    ros::Publisher m_avb_pub;

    // Variables for reading the IMU data
    double actual_wx,actual_wy,actual_wz;
    double actual_roll,actual_pitch,actual_yaw;
    int32_t actual_m1,actual_m2,actual_m3,actual_m4;
    
    // Variables for the mocap callback
    double actual_x,actual_y,actual_z;
    
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
  
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "measurement_vector");

  ros::NodeHandle n("~");
  double frequency;
  n.param("frequency", frequency, 50.0);

  meas_vec meas_vec(n);
  meas_vec.run(frequency);

  return 0;
}
