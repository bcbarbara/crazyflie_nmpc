crazyflie_nmpc
=============
![featured](https://user-images.githubusercontent.com/50755258/93484971-d65ea880-f902-11ea-8fca-97c12f13730d.png)

ROS stack containing an efficient real-time NMPC for the [Crazyflie](http://www.bitcraze.se/) and with the following features:

* Support for the Crazyflie 2.1
* Compatible with ROS Kinetic
* NMPC controller for regulation and trajectory tracking (if motion capture system is available)
* Publishes the onboard sensors data and the controller variables in ROS standard message formats

## Citing this work
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

If you use this project, we would appreciate it if you give us the credit by citing this work

```
@inproceedings{barroscarlos2020,
  author     = {Barros Carlos, B\'arbara and
                Sartor, Tommaso and
                Zanelli, Andrea and
                Frison, Gianluca and
                Burgard, Wolfram and
                Diehl, Moritz and
                Oriolo, Giuseppe},
  title      = {An Efficient Real-Time {NMPC} for Quadrotor Position Control under Communication Time-Delay},
  booktitle  = {Proc. 16th International Conference on Control, Automation, Robotics and Vision},
  year       = {2020},
  publisher  = {IEEE}
}

```
Watch our YouTube video showing the experiments done in this work:

[![video](https://user-images.githubusercontent.com/50755258/96165708-40c43200-0f1d-11eb-84fa-159b6a37f391.png)](https://youtu.be/xZLVQ7BdUHA)


## Installation

Check the [wiki](https://github.com/bcbarbara/crazyflie_nmpc/wiki) for more information, including the installation instructions for [ROS Kinetic](https://github.com/bcbarbara/crazyflie_nmpc/wiki/Install-on-Ubuntu-16.04-LTS-with-ROS-Kinetic).

## Packages

There are seven packages in this stack: `acados`, `crazyflie_cpp`, `crazyflie_driver`, `crazyflie_tools`, `crazyflie_description`, `crazyflie_controller`, and `crazyflie_demo`.

#### acados

A submodule with fast and embedded solvers for nonlinear optimal control.

- Documentation can be found on [docs.acados.org](https://docs.acados.org/).
- Forum: in case of any acados-related question  [discourse.acados.org](https://discourse.acados.org/).



#### crazyflie_cpp

This submodule contains a cpp library for the Crazyradio and the Crazyflie. It can be used independently of ROS. It supports the logging subsystem for streaming data in real-time and the parameter subsystem for adjusting parameters.

#### crazyflie_driver

It contains a ROS wrapper around `crazyflie_cpp`. The logging and parameter subsystems are mapped into ROS messages and ROS parameters respectively.

#### crazyflie_tools

This package is capable of:
* Finding any Crazyflie in range
* Listing the available logging variables and parameters

#### crazyflie_description

This package contains a 3D model of the Crazyflie 1.0 for visualization purposes in rviz.

#### crazyflie_controller

This package contains an efficient and modular implementation of a nonlinear model predictive controller (NMPC) tailored to the Crazyflie with time-delay compensation. The NMPC is implementend using the high-performance software package acados, which solves optimal control problems and implements a real-time iteration (RTI) variant of a sequential quadratic programming (SQP) scheme with Gauss-Newton Hessian approximation. The quadratic subproblems (QP) arising in the SQP scheme are solved with HPIPM, an interior-point method solver, built on top of the hardware-tailored linear algebra library BLASFEO. We use a recently proposed Hessian condesing algorithm, particularly suitable for partial condensing, to further speed-up solution times.  

The package also contains a PID controller for waypoint navigation developed by Wolfgang Honig.

Both implementations rely on the knownledge of the global position of the Crazyflie (e.g., though a motion capture system).


#### crazyflie_demo

Contains sample scripts and launch files for teleoperation, hovering, and waypoint navigation for both single and multi Crazyflie cases.

  For more detailed information about each   package, we refer to the `crazyflie_ros` stack [documentation](http://act.usc.edu/publications/Hoenig_Springer_ROS2017.pdf).

## ROS Features

### Parameters

The launch file supports the following arguments:
* uri: Specifier for the crazyflie (e.g., radio://0/80/2M)
* tf_prefix: tf prefix for the crazyflie frame
* roll_trim: Trim in degrees (e.g., negative if the Crazyflie drifts to the left)
* pitch_trim: Trim in degrees (e.g., negative if the Crazyflie drifts forward)

See how to obtain good trim values [here](http://wiki.bitcraze.se/projects:crazyflie:userguide:tips_and_tricks).

### Subscribers

#### cmd_vel

The following fields are used:
* linear.y: roll (e.g., -30 to 30 degrees)
* linear.x: pitch (e.g., -30 to 30 degrees)
* angular.z: yawrate (e.g., -200 to 200 degrees/second)
* linear.z: thrust (10000 to 60000 -- mapped to PWM output)

>**Note** See how to tune these parameters for your joystick in `crazyflie_controller/launch/logitech.launch`

### Publishers

#### euler_angles
* geometry_msgs/Vector3Stamped
* contains the euler angles from the onboard stabilizer
* update: 10 ms (time between Crazyflie and ROS not synchronized!)

#### motor_speeds
* crazyflie_driver/GenericLogData
* contains the individual motor speeds (PWM value)
* update: 10 ms (time between Crazyflie and ROS not synchronized!)

#### imu
* sensor_msgs/IMU
* contains the sensor readings of gyroscope and accelerometer
* update: 10 ms (time between Crazyflie and ROS not synchronized!)
* can be viewed in rviz
