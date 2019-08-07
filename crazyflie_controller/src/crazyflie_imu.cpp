#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// standard
#include <stdio.h>
#include <stdlib.h>


using std::cout;
using std::endl;
using std::fixed;
using std::showpos;



class NMPC
{
public:

    NMPC(const ros::NodeHandle& n){

        ros::NodeHandle nh;
	m_imu_sub 	 = nh.subscribe("/crazyflie/imu", 1000, &NMPC::imuCallback, this);

    }

private:



    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

	p  = msg->angular_velocity.x;
	q  = msg->angular_velocity.y;
	r  = msg->angular_velocity.z;

	abx = msg->linear_acceleration.x;
	aby = msg->linear_acceleration.y;
	abz = msg->linear_acceleration.z;

	ROS_INFO_STREAM(fixed << showpos << "\nQuad flight data at time [" << msg->header.stamp << "s "<< "]" << endl
		        << "Gyro [p,q,r] = [" << p << ", " << q << ", " << r << "]" << endl
		        << "Acc [abx,aby,abz] = [" << abx << ", " << aby << ", " << abz << "]" << endl);
    }

private:

    ros::Subscriber m_imu_sub;

    // Variables for reading the IMU data
    float q1,q2,q3,q4;
    float p;
    float q;
    float r;
    float abx,aby,abz;
    float t;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_imu");

  ros::NodeHandle n("~");

  NMPC nmpc(n);
  ros::spin();

  return 0;
}
