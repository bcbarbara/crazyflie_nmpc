# ============================================================
# Dockerfile for crazyflie_nmpc
# ROS2 Jazzy + acados (latest from GitHub)
# ============================================================

FROM ros:jazzy-ros-base

SHELL ["/bin/sh", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# ------------------------------------------------------------
# 1. Install system and Python dependencies
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    python3 \
    python3-pip \
    python3-dev \
    python3-numpy \
    libblas-dev \
    liblapack-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# 2. Clone and build ACADOS (latest master)
# ------------------------------------------------------------
WORKDIR /opt
RUN git clone https://github.com/acados/acados.git && \
    cd acados && \
    git submodule update --init --recursive && \
    mkdir -p build && cd build && \
    cmake -DACADOS_WITH_QPOASES=ON .. && \
    make -j$(nproc) && \
    make install

# ------------------------------------------------------------
# 2.5 Install latest t_renderer binary for ACADOS
# ------------------------------------------------------------
RUN set -eux && \
    cd /opt/acados && \
    mkdir -p bin && \
    wget -qO bin/t_renderer-v0.2.0-linux-amd64 https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64 && \
    mv bin/t_renderer-v0.2.0-linux-amd64 bin/t_renderer && \
    chmod +x bin/t_renderer && \
    echo "Installed t_renderer to /opt/acados/bin"

# ------------------------------------------------------------
# 3. Configure ACADOS environment
# ------------------------------------------------------------
ENV ACADOS_SOURCE_DIR=/opt/acados
ENV PATH="${ACADOS_SOURCE_DIR}/bin:${PATH}"
# Use default empty if undefined to suppress warnings
ENV LD_LIBRARY_PATH="/opt/acados/lib:${LD_LIBRARY_PATH:-}"
ENV PYTHONPATH="/opt/acados/interfaces/acados_template:${PYTHONPATH:-}"

# ------------------------------------------------------------
# 4. Install ACADOS Python interface
# ------------------------------------------------------------
RUN cd /opt/acados/interfaces/acados_template && \
    pip install --no-cache-dir --break-system-packages .

# ------------------------------------------------------------
# 5. Copy the crazyflie_nmpc workspace and build all packages
# ------------------------------------------------------------
WORKDIR /root/ros2_ws
COPY . .

# Make ACADOS solver scripts executable
RUN chmod +x /root/ros2_ws/crazyflie_controller/acados_scripts/*.py

# Build all ROS2 packages
# RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install
# Build only the crazyflie_nmpc package
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select crazyflie_controller --symlink-install

# ------------------------------------------------------------
# 8. Default container shell
# ------------------------------------------------------------
CMD ["/bin/bash"]
