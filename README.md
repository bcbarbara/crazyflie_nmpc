crazyflie_ros
=============

ROS stack for [Bitcraze Crazyflie](http://www.bitcraze.se/), with the following features:

* Support Crazyflie 2.1
* Publishes on-board sensors in ROS standard message formats
* Includes external controller for waypoint navigation (if motion capture system is available)
* No dependency to the Bitcraze SDK (Driver and Controller written in C++)

## Installation - Ubuntu 16.04 LTS with ROS Kinetic
1. Install ROS Kinetic (recommended: “Desktop-Full Install”) following [these instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu).
2. We work with Catkin Command Line Tools to build packages in the workspace. They can be installed with apt-get following [these instructions](https://catkin-tools.readthedocs.io/en/latest/installing.html#installing-on-ubuntu-with-apt-get).
> **Note** `catkin build​` instead of ​`catkin_mak​e`

3. Setup your catkin workspace:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws/
$ catkin build
$ source devel/setup.bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

4. Download [acados](https://github.com/acados/acados.git). You need to compile and install acados with `qpOASES` solver to be able to use its interface:
```
$ mkdir -p ~/acados/build
$ cd build
$ cmake -DACADOS_WITH_QPOASES=ON .. & make install
```

5. Clone the package into your catkin workspace:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/bcbarbara/crazyflie_ros.git
$ cd crazyflie_ros
$ git submodule init
$ git submodule update
```
6. Compile the shared library for the NMPC solver and place it in the system's path:
```
$ cd ~/catkin_ws/src/crazyflie_ros/crazyflie_controller/include/c_generated_code
$ make shared_lib
$ cp acados_solver_crazyflie_pos.so /usr/local/lib
```

7. Use `catkin build` on your workspace to compile.

## Usage

There are six packages included: crazyflie_cpp, crazyflie_driver, crazyflie_tools, crazyflie_description, crazyflie_controller, and crazyflie_demo.

### Crazyflie_Cpp

This package contains a cpp library for the Crazyradio and Crazyflie. It can be used independently of ROS.

### Crazyflie_driver

This package contains the driver. In order to support multiple Crazyflies with a single Crazyradio, there is crazyflie_server (communicating with all Crazyflies) and crazyflie_add to dynamically add Crazyflies.
The server does not communicate to any Crazyflie initially, hence crazyflie_add needs to be used.

### Crazyflie_tools

This package contains tools which are helpful, but not required for normal operation. So far, it just support one tool for scanning for a Crazyflie.

You can find connected Crazyflies using:
```
rosrun crazyflie_tools scan
```

### Crazyflie_description

This package contains a 3D model of the Crazyflie (1.0). This is for visualization purposes in rviz.

### Crazyflie_controller

This package contains an efficient and modular implementation of a Nonlinear Model Predictive Control (NMPC) tailored for the Crazyflie's online trajectory generation and tracking problem. A Real-Time Iteration (RTI) scheme through a Sequential Quadratic Programming (SQP) online algorithm is used in order to solve the Nonlinear Program (NLP).

The package also contains a simple PID controller for hovering or waypoint navigation, which has been develop by Wolfgang Honig, and can be used with external motion capture systems, such as VICON.



### Crazyflie_demo

For teleoperation using a joystick, use:
```
roslaunch crazyflie_demo teleop_logitech.launch uri:=radio://0/80/2M
```
where the uri specifies the uri of your Crazyflie. You can find valid uris using the scan command in the `crazyflie_tools` package.
> **Note** By default the services for take off and landing in teleoperation mode are disable, while the emergency service is always enable in any type of mode. The button mapping for each service can be found in `scripts/controller.py`


For launching the node of the NMPC together with a motion capture system (MOCAP), use:
```
roslaunch crazyflie_demo nmpc.launch uri:=radio://0/80/2M frame:=/cortex/crazyflie/crazyflie
```

For retrieving the data from the IMU, use:
```
roslaunch crazyflie_demo imu_feedback.launch uri:=radio://0/80/2M/E7E7E7E7E7
```
> **Note** Acceleration: Be careful with acceleration data. Assuming that the Crazyflie is placed in its neutral right-side-up position on a flat surface you will:
  > *  Measure +9.81 meters per second squared for the Z axis.
  > *  If the sensor is rolled +90 degrees (left side up), the acceleration should be -9.81 meters per second squared for the Y axis.
  > *  If the sensor is pitched +90 degrees (front side down), it should read -9.81 meters per second squared for the X axis.


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

#### imu
* sensor_msgs/IMU
* contains the sensor readings of gyroscope and accelerometer
* The covariance matrices are set to unknown
* orientation is not set (this could be done by the magnetometer readings in the future.)
* update: 10ms (time between crazyflie and ROS not synchronized!)
* can be viewed in rviz

#### temperature
* sensor_msgs/Temperature
* From Barometer (10DOF version only) in degree Celcius (Sensor readings might be higher than expected because the PCB warms up; see http://www.bitcraze.se/2014/02/logging-and-parameter-frameworks-turtorial/)
* update: 100ms (time between crazyflie and ROS not synchronized!)

#### magnetic_field
* sensor_msgs/MagneticField
* update: 100ms (time between crazyflie and ROS not synchronized!)

#### pressure
* Float32
* hPa (or mbar)
* update: 100ms (time between crazyflie and ROS not synchronized!)

#### battery
* Float32
* Volts
* update: 100ms (time between crazyflie and ROS not synchronized!)
