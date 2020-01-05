crazyflie_nmpc
=============
![version](https://img.shields.io/badge/version-1.0.0-blue)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)


ROS stack for [Bitcraze Crazyflie](http://www.bitcraze.se/), with the following features:

* Support for Crazyflie 2.1
* Compatible with ROS Kinetic
* Publishes onboard sensors data in ROS standard message formats
* Includes NMPC controller for regulation and trajectory tracking (if motion capture system is available)

Check the [wiki](https://github.com/bcbarbara/crazyflie_nmpc/wiki) for more information, including the installation instructions for ROS Kinetic which can be found [here](https://github.com/bcbarbara/crazyflie_nmpc/wiki/Install-on-Ubuntu-16.04-LTS-with-ROS-Kinetic).

## Packages

There are seven packages included: `acados`, `crazyflie_cpp`, `crazyflie_driver`, `crazyflie_tools`, `crazyflie_description`, `crazyflie_controller`, and `crazyflie_demo`.

#### acados

A submodule with fast and embedded solvers for nonlinear optimal control.

- Documentation can be found on [docs.acados.org](https://docs.acados.org/).
- Forum: in case of any acados-related question  [discourse.acados.org](https://discourse.acados.org/).



#### crazyflie_cpp

This submodule contains a cpp library for the Crazyradio and Crazyflie. It can be used independently of ROS. It supports the logging framework streaming data in real-time and the parameter framework adjusting parameters, such as PID
gains.

#### crazyflie_driver

It contains a ROS wrapper around `crazyflie_cpp`. The logging subsystem is mapped to ROS messages and parameters are mapped to ROS parameters as well.

#### crazyflie_tools

This package has some features such as: find any Crazyflie in range and list the available logging variables and parameters.

#### crazyflie_description

This package contains a 3D model of the Crazyflie 1.0. This is for visualization purposes in rviz.

#### crazyflie_controller

This package contains an efficient and modular implementation of a nonlinear model predictive controller (NMPC) tailored for the Crazyflie's online trajectory generation and tracking problem. The real-time iteration (RTI) scheme, through a sequential quadratic programming (SQP) algorithm, is used in order to solve the nonlinear program (NLP).

The package also contains a PID position controller for hovering or waypoint navigation, which has been developed by Wolfgang Honig.

Both implementations rely on the knownledge of the Crazyflie's position (e.g. though a motion capture system).


#### crazyflie_demo

Contains sample scripts and launch files for teleoperation, hovering, and waypoint navigation for both single and multi Crazyflie cases.

For more detailed information regarding each individual package, we refere to the `crazyflie_ros` stack [documentation](http://act.usc.edu/publications/Hoenig_Springer_ROS2017.pdf).

## ROS Features

### Parameters

The launch file supports the following arguments:
* uri: Specifier for the crazyflie, e.g. radio://0/80/2M
* tf_prefix: tf prefix for the crazyflie frame(s)
* roll_trim: Trim in degrees, e.g. negative if flie drifts to the left
* pitch_trim: Trim in degrees, e.g. negative if flie drifts forward

See how to obtain good trim values [here](http://wiki.bitcraze.se/projects:crazyflie:userguide:tips_and_tricks) for details on.

### Subscribers

#### cmd_vel

The following fields are used:
* linear.y: roll [e.g. -30 to 30 degrees]
* linear.x: pitch [e.g. -30 to 30 degrees]
* angular.z: yawrate [e.g. -200 to 200 degrees/second]
* linear.z: thrust [10000 to 60000 (mapped to PWM output)]

>**Note** See how to tune these parameters for your joystick in `crazyflie_controller/launch/logitech.launch`

### Publishers

#### euler_angles
* geometry_msgs/Vector3Stamped
* contains the euler angles from the onboard stabilizer
* update: 100 ms (time between crazyflie and ROS not synchronized!)

#### motor_speeds
* crazyflie_driver/GenericLogData
* contains the individual motor speeds (PWM value)
* update: 100 ms (time between crazyflie and ROS not synchronized!)

#### imu
* sensor_msgs/IMU
* contains the sensor readings of gyroscope and accelerometer
* The covariance matrices are set to unknown
* orientation is not set (this could be done by the magnetometer readings in the future.)
* update: 10 ms (time between crazyflie and ROS not synchronized!)
* can be viewed in rviz

#### temperature
* sensor_msgs/Temperature
* From Barometer (10DOF version only) in degree Celcius (Sensor readings might be higher than expected because the PCB warms up; see [here](http://www.bitcraze.se/2014/02/logging-and-parameter-frameworks-turtorial/))
* update: 100 ms (time between crazyflie and ROS not synchronized!)

#### magnetic_field
* sensor_msgs/MagneticField
* update: 100 ms (time between crazyflie and ROS not synchronized!)

#### pressure
* Float32
* hPa (or mbar)
* update: 100 ms (time between crazyflie and ROS not synchronized!)

#### battery
* Float32
* Volts
* update: 100 ms (time between crazyflie and ROS not synchronized!)
