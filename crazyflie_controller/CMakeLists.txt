cmake_minimum_required(VERSION 3.8)
project(crazyflie_controller)

# ============================================================
# 1. Dependencies
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# ============================================================
# 2. ACADOS environment setup
# ============================================================
if(DEFINED ENV{ACADOS_SOURCE_DIR})
  set(ACADOS_SOURCE_DIR $ENV{ACADOS_SOURCE_DIR})
else()
  set(ACADOS_SOURCE_DIR "/opt/acados")
endif()

set(T_RENDERER_PATH "${ACADOS_SOURCE_DIR}/bin/t_renderer")
if(NOT EXISTS "${T_RENDERER_PATH}")
  message(FATAL_ERROR "t_renderer not found at ${T_RENDERER_PATH}")
else()
  message(STATUS "Found t_renderer at ${T_RENDERER_PATH}")
endif()

message(STATUS "Using ACADOS source dir: ${ACADOS_SOURCE_DIR}")

set(ACADOS_LIB_DIR "${ACADOS_SOURCE_DIR}/lib")
set(ACADOS_INCLUDE_DIR "${ACADOS_SOURCE_DIR}/include")

include_directories(
  ${ACADOS_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_directories(${ACADOS_LIB_DIR})

# ============================================================
# 3. NMPC solver code generation
# ============================================================
set(acados_scripts_path "${CMAKE_CURRENT_SOURCE_DIR}/acados_scripts")
set(nmpc_solver_generator_script "${acados_scripts_path}/main.py")
set(nmpc_solver_model_name "crazyflie_model")
set(nmpc_solver_generated_code_path "${CMAKE_CURRENT_BINARY_DIR}/${nmpc_solver_model_name}")

file(GLOB_RECURSE nmpc_solver_src_files
  LIST_DIRECTORIES false
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  ${acados_scripts_path}/*.py
)

file(MAKE_DIRECTORY ${nmpc_solver_generated_code_path})

# Generate solver code as a custom target
add_custom_target(
  acados_generate_solver ALL
  COMMAND ${Python3_EXECUTABLE} ${nmpc_solver_generator_script} ${nmpc_solver_generated_code_path}
  WORKING_DIRECTORY ${acados_scripts_path}
  COMMENT "Generating ACADOS NMPC solver code"
  VERBATIM
)

# Build the solver C code after code generation
add_custom_command(
  TARGET acados_generate_solver
  POST_BUILD
  COMMAND make -C ${nmpc_solver_generated_code_path}
  COMMENT "Building ACADOS solver shared library"
  VERBATIM
)

# ============================================================
# 4. IMU node
# ============================================================
add_executable(crazyflie_imu src/crazyflie_imu.cpp)
ament_target_dependencies(crazyflie_imu rclcpp sensor_msgs)

# ============================================================
# 5. Install targets
# ============================================================
# Node binaries
install(TARGETS crazyflie_imu
  DESTINATION lib/${PROJECT_NAME}
)

# ACADOS generated code
install(DIRECTORY ${nmpc_solver_generated_code_path}/
  DESTINATION share/${PROJECT_NAME}/generated
)

# Package metadata
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# 6. Finalize as a ROS2 ament package
# ============================================================
ament_package()
