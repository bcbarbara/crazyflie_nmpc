#!/bin/bash
set -e

# ------------------------------------------------------------
# Entrypoint for crazyflie_nmpc container
# Sources ROS2, workspace, and ACADOS automatically
# ------------------------------------------------------------

# Source ROS2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Source workspace
if [ -f /home/crazyflie/ros2_ws/install/setup.bash ]; then
    source /home/crazyflie/ros2_ws/install/setup.bash
fi

# ACADOS environment
export ACADOS_SOURCE_DIR=/opt/acados
export PATH=$ACADOS_SOURCE_DIR/bin:$PATH
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$ACADOS_SOURCE_DIR/interfaces/acados_template:$PYTHONPATH

# Move to workspace root
cd /home/crazyflie/ros2_ws

# If arguments are provided, execute them, else launch interactive bash
if [ $# -eq 0 ]; then
    exec /bin/bash
else
    exec "$@"
fi