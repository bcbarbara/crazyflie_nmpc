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


// standard
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "geometry_msgs/PointStamped.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

class Estimation
{
public:

    Estimation(const ros::NodeHandle& n){
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

    	ros::NodeHandle nh;
    	m_eRaptor_sub = nh.subscribe("/crazyflie/external_position", 1000, &Estimation::eRaptorCallback, this);
    }


    /* Estimate the velocity using Euler initially and after a low-pass filter */
    double linear_velocity(std::vector <double> q_samples, double Ts, double elapsed_time, std::vector <double> dq_samples) {

      double dq = 0;
      if (elapsed_time > 0.1) dq = 1.573*dq_samples[4] - 0.6188*dq_samples[3] + 22.65*q_samples[4] - 22.65*q_samples[3];
      else dq = (q_samples[4] - q_samples[3]) / Ts;

      return dq;
    }


    void eRaptorCallback(const geometry_msgs::PointStampedConstPtr& msg){
  	  float current_time = ros::Time::now().toSec();
  	  float dt = current_time - last_time;

  	  delta_t += dt;

  	  // Position of crazyflie marker
  	  actual_x = msg->point.x;
  	  actual_y = msg->point.y;
  	  actual_z = msg->point.z;

  	  ROS_INFO_STREAM("x: "   << actual_x << " " <<
  			  "y: "   << actual_y << " " <<
  			  "z: "   << actual_z << std::endl);

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

  	  vx = linear_velocity(x_samples, dt, delta_t, vx_filter_samples);
  	  vy = linear_velocity(y_samples, dt, delta_t, vy_filter_samples);
  	  vz = linear_velocity(z_samples, dt, delta_t, vz_filter_samples);
  	  // passing to the state vector

  	  ROS_INFO_STREAM("vx: "   << vx << " " <<
  			  "vy: "   << vy << " " <<
  			  "vz: "   << vz << std::endl);

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

  	  last_time = current_time;
    }

    // Variables for eRaptor data
    float actual_x;
    float actual_y;
    float actual_z;

    ros::Subscriber m_eRaptor_sub;

    double vx,vy,vz;
    std::vector<double> x_samples;
    std::vector<double> y_samples;
    std::vector<double> z_samples;
    std::vector<double> vx_filter_samples;
    std::vector<double> vy_filter_samples;
    std::vector<double> vz_filter_samples;

    float delta_t;
    float current_time = 0.0;
    float last_time = 0.0;
};


int main(int argc, char **argv){
  ros::init(argc, argv, "lv_estimate");

  ros::NodeHandle n("~");

  Estimation estimation(n);
  ros::spin();

  return 0;
}
