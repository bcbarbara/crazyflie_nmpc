Crazyflie NPMC
=============
ROS stack containing an efficient real-time NMPC for the [Crazyflie](http://www.bitcraze.se/) and with the following features:

* Support for the Crazyflie 2.1
* Compatible with ROS Melodic
* NMPC controller for regulation and trajectory tracking (if motion capture system is available)
* Publishes the onboard sensors data and the controller variables in ROS standard message formats


## Installation

1. Install ROS Noetic (recommended: “Desktop-Full Install”) following [these instructions](http://wiki.ros.org/noetic/Installation/Ubuntu).
2. We use Catkin Command Line Tools to build packages in the workspace. They can be installed with apt-get following [these instructions](https://catkin-tools.readthedocs.io/en/latest/installing.html#installing-on-ubuntu-with-apt-get).

3. Setup your catkin workspace:
```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ catkin_init_workspace
$ cd ~/catkin_ws/
$ catkin build
$ source devel/setup.bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

4. Clone the `acsi_crazyflie_nmpc` project into your catkin workspace:
```console
$ cd ~/catkin_ws/src/
$ git clone git@github.com:devanshdhrafani/acsi_crazyflie_nmpc.git
```

5. Initialize all submodules:
```console
$ cd acsi_crazyflie_nmpc/
$ git submodule update --init --recursive
```

6. Build and install `acados`:
```console
$ cd acados/
$ make static_library -j4
$ make -j4
```
> **Note** Set the appropriate `BLASFEO_TARGET` and `HPIPM_TARGET` via `<acados_root>/Makefile.local` file. For a list of supported targets, we refer to the [BLASFEO documentation](https://github.com/giaf/blasfeo/blob/master/README.md). It is also recommended to add the flag `ACADOS_WITH_QPOASES=1` to the `Makefile.local`.

7. Install `acados` Python interface with Python 3.8 
```console
$ cd ~/catkin_ws/src/acsi_crazyflie_nmpc/acados/interfaces/acados_template/
$ pip3 install .
```

8. In order to be able to successfully render C code templates, you need to download the `t_renderer` binaries for your platform from [here](https://github.com/acados/tera_renderer/releases/) and place them in `<acados_root>/bin`.
> **Note** Please strip the version and platform from the binaries (e.g. `t_renderer-v0.0.20 -> t_renderer`).

> **Note** Notice that you might need to make `t_renderer` executable. 

9. Run `export ACADOS_SOURCE_DIR=<acados_root>` such that the location of acados will be known to the Python package at run time.

10. Additionally, you will have to make sure that the environment variable `LD_LIBRARY_PATH` contains the path to `libacados.so` (default path is `<acados_root/lib>`).

11. Now you are able to generate the RTI instance tailored for the crazyflie NMPC in the `scripts` folder:
```console
$ cd ~/catkin_ws/src/acsi_crazyflie_nmpc/crazyflie_controller/scripts/crazyflie_full_model/
$ python3 generate_c_code.py
```

12. Then use the `catkin build` command to compile your workspace.


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

This package contains an efficient and modular implementation of a nonlinear model predictive controller (NMPC) tailored to the Crazyflie with time-delay compensation. The NMPC is implementend using the high-performance software package acados, which solves optimal control problems and implements a real-time iteration (RTI) variant of a sequential quadratic programming (SQP) scheme with Gauss-Newton Hessian approximation. The quadratic subproblems (QP) arising in the SQP scheme are solved with HPIPM, an interior-point method solver, built on top of the linear algebra library BLASFEO, finely tuned for multiple CPU architectures. We use a recently proposed Hessian condesing algorithm, particularly suitable for partial condensing, to further speed up solution times.  

The package also contains a PID controller for waypoint navigation developed by Wolfgang Honig.

Both implementations rely on the knownledge of the global position of the Crazyflie (e.g., though a motion capture system).


#### crazyflie_demo

Contains sample scripts and launch files for teleoperation, hovering, and waypoint navigation for both single and multi Crazyflie cases.

  For more detailed information about each   package, we refer to the `crazyflie_ros` stack [documentation](http://act.usc.edu/publications/Hoenig_Springer_ROS2017.pdf).-=

## Acknowledgements
This work is largely based on Barbara et al's paper titled An Efficient Real-Time NMPC for Quadrotor Position Control under Communication Time-Delay. Please refer the original paper [here](https://ieeexplore.ieee.org/document/9305513).
