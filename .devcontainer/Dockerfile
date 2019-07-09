FROM osrf/ros:kinetic-desktop-full

# Otherwise 'source' does not work
SHELL ["/bin/bash", "-c"]

# Install the catkin tools
RUN apt-get update
RUN apt-get install -y ros-kinetic-catkin-pip wget && pip install -U catkin_tools

# Download acados and casadi
RUN cd /root && \
    git clone https://github.com/acados/acados.git --recursive && \
    cd acados/external && \
    wget -q -nc https://github.com/casadi/casadi/releases/download/3.4.0/casadi-linux-py35-v3.4.0-64bit.tar.gz && \
    mkdir -p casadi-py35-v3.4.0-64bit && \
    tar -xf casadi-linux-py35-v3.4.0-64bit.tar.gz -C casadi-py35-v3.4.0-64bit && \
    cd ..

# Build and install acados
RUN cd /root/acados/ && \
    mkdir build && \
    cd build && \
    cmake -DACADOS_WITH_QPOASES=ON .. && \
    make -j4 && \
    make install

# Setup the catkin workspace in /root/catkin_ws
RUN source /opt/ros/kinetic/setup.bash && \
    mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws/src && \
    catkin_init_workspace && \
    cd .. && \
    catkin build && \
    source devel/setup.bash

# Make sure the joystick works
RUN apt-get install -y ros-kinetic-joy