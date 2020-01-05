/*
* Copyright 2020 Barbara Barros Carlos, Tommaso Sartor
*
* This file is part of crazyflie_nmpc.
*
* The MIT License (MIT)
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/


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
