crazyflie_nmpc
=============
![gem](https://img.shields.io/badge/ver -1.0.0-blue)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)


ROS stack for [Bitcraze Crazyflie](http://www.bitcraze.se/), with the following features:

* Support for Crazyflie 2.1
* Compatible with ROS Kinetic
* Publishes onboard sensors data in ROS standard message formats
* Includes NMPC controller for regulation and trajectory tracking (if motion capture system is available)

Check the [wiki]() for more informations. Installation instructions for ROS Kinetic can be found [here]().

## Packages

There are seven packages included: acados, crazyflie_cpp, crazyflie_driver, crazyflie_tools, crazyflie_description, crazyflie_controller, and crazyflie_demo.

#### Crazyflie_Cpp

This package contains a cpp library for the Crazyradio and Crazyflie. It can be used independently of ROS.

#### Crazyflie_driver

This package contains the driver. In order to support multiple Crazyflies with a single Crazyradio, there is crazyflie_server (communicating with all Crazyflies) and crazyflie_add to dynamically add Crazyflies.
The server does not communicate to any Crazyflie initially, hence crazyflie_add needs to be used.

#### Crazyflie_tools

This package contains tools which are helpful, but not required for normal operation. So far, it just support one tool for scanning for a Crazyflie.

You can find connected Crazyflies using:
```
rosrun crazyflie_tools scan
```

#### Crazyflie_description

This package contains a 3D model of the Crazyflie (1.0). This is for visualization purposes in rviz.

#### Crazyflie_controller

This package contains an efficient and modular implementation of a Nonlinear Model Predictive Control (NMPC) tailored for the Crazyflie's online trajectory generation and tracking problem. A Real-Time Iteration (RTI) scheme through a Sequential Quadratic Programming (SQP) online algorithm is used in order to solve the Nonlinear Program (NLP).

The package also contains a simple PID controller for hovering or waypoint navigation, which has been develop by Wolfgang Honig, and can be used with external motion capture systems, such as VICON.


#### Crazyflie_demo

For teleoperation using a joystick, use:
```
roslaunch crazyflie_demo teleop_logitech.launch
```
where the uri specifies the uri of your Crazyflie. You can find valid uris using the scan command in the `crazyflie_tools` package.
> **Note** By default the services for take off and landing in teleoperation mode are disable, while the emergency service is always enable in any type of mode. The button mapping for each service can be found in `scripts/controller.py`


For launching the node of the NMPC together with a motion capture system (MOCAP) use:
```
roslaunch crazyflie_demo mocap_acados.launch
```

For retrieving the data from the IMU use:
```
roslaunch crazyflie_demo imu_feedback.launch uri:=radio://0/80/2M/E7E7E7E7E7
```
> **Note** Acceleration: Be careful with acceleration data. Assuming that the Crazyflie is placed in its neutral right-side-up position on a flat surface you will:
  > *  Measure +9.81 meters per second squared for the Z axis.
  > *  If the sensor is rolled +90 degrees (left side up), the acceleration should be -9.81 meters per second squared for the Y axis.
  > *  If the sensor is pitched +90 degrees (front side down), it should read -9.81 meters per second squared for the X axis.

For checking the status of the 13 states which composes the NMPC state vector use:
```
roslaunch crazyflie_demo meas_vector.launch
```

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

>**Note** See how to tune these parameters for your joystick in `launch/logitech.launch`

### Publishers

#### euler angles
* geometry_msgs/Vector3Stamped
* contains the euler angles from the onboard stabilizer
* update: 100 ms (time between crazyflie and ROS not synchronized!)

#### motor speeds
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
* From Barometer (10DOF version only) in degree Celcius (Sensor readings might be higher than expected because the PCB warms up; see http://www.bitcraze.se/2014/02/logging-and-parameter-frameworks-turtorial/)
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
