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
    sudo \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# 1.5 Create non-root user
# ------------------------------------------------------------
RUN useradd -ms /bin/bash crazyflie && \
    echo "crazyflie ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ENV HOME=/home/crazyflie

# ------------------------------------------------------------
# 2. Build ACADOS as root
# ------------------------------------------------------------
USER root

RUN git clone https://github.com/acados/acados.git /opt/acados && \
    cd /opt/acados && \
    git submodule update --init --recursive && \
    mkdir -p build && cd build && \
    cmake -DACADOS_WITH_QPOASES=ON .. && \
    make -j$(nproc) && make install

# Install latest t_renderer binary
RUN cd /opt/acados && \
    mkdir -p bin && \
    wget -qO bin/t_renderer-v0.2.0-linux-amd64 https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64 && \
    mv bin/t_renderer-v0.2.0-linux-amd64 bin/t_renderer && \
    chmod +x bin/t_renderer

# ------------------------------------------------------------
# 3. Configure ACADOS environment
# ------------------------------------------------------------
ENV ACADOS_SOURCE_DIR=/opt/acados
ENV PATH="${ACADOS_SOURCE_DIR}/bin:${PATH}"
ENV LD_LIBRARY_PATH="/opt/acados/lib:${LD_LIBRARY_PATH:-}"
ENV PYTHONPATH="/opt/acados/interfaces/acados_template:${PYTHONPATH:-}"

# ------------------------------------------------------------
# 4. Install ACADOS Python interface
# ------------------------------------------------------------
RUN cd /opt/acados/interfaces/acados_template && \
    pip install --no-cache-dir --break-system-packages .

# ------------------------------------------------------------
# 5. Switch to non-root user for workspace
# ------------------------------------------------------------
USER crazyflie
WORKDIR /home/crazyflie

# ------------------------------------------------------------
# 6. Setup ROS2 workspace
# ------------------------------------------------------------
RUN mkdir -p ros2_ws/src

WORKDIR /home/crazyflie/ros2_ws/src

# Copy the workspace (from build context) and set ownership
COPY --chown=crazyflie:crazyflie . .

# Ensure scripts are executable
RUN chmod +x crazyflie_controller/acados_scripts/*.py

# Fix ownership recursively for the entire workspace
RUN sudo chown -R crazyflie:crazyflie /home/crazyflie/ros2_ws

WORKDIR /home/crazyflie/ros2_ws

# Build ROS2 package as non-root user
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select crazyflie_controller --symlink-install

# ------------------------------------------------------------
# 7. Automatically source ROS2, workspace, and ACADOS in Bash
# ------------------------------------------------------------
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /home/crazyflie/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export ACADOS_SOURCE_DIR=/opt/acados" >> ~/.bashrc && \
    echo "export PATH=\$ACADOS_SOURCE_DIR/bin:\$PATH" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=\$ACADOS_SOURCE_DIR/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc && \
    echo "export PYTHONPATH=\$ACADOS_SOURCE_DIR/interfaces/acados_template:\$PYTHONPATH" >> ~/.bashrc

# ------------------------------------------------------------
# 8. Entrypoint
# ------------------------------------------------------------
COPY --chown=crazyflie:crazyflie entrypoint.sh /home/crazyflie/entrypoint.sh
RUN chmod +x /home/crazyflie/entrypoint.sh
ENTRYPOINT ["/home/crazyflie/entrypoint.sh"]

# ------------------------------------------------------------
# 9. Default shell
# ------------------------------------------------------------
CMD ["/bin/bash"]
