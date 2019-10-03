cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_controller)

#find_package(PythonLibs REQUIRED)
#include_directories(${PYTHON_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  tf
  dynamic_reconfigure
  message_generation
 # pybind11_catkin
)

find_package(Eigen3 REQUIRED)

set(acados_include "/usr/local/include")

generate_dynamic_reconfigure_options(
  config/crazyflie_params.cfg
)

add_message_files(
  FILES
  GenericLogData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/c_generated_code/crazyflie_full_model/c_generated_code
  include/c_generated_code/crazyflie_simple_model/c_generated_code
  include/
  ${acados_include}
  ${acados_include}/blasfeo/include/
  ${acados_include}/hpipm/include/
  ${acados_include}/acados/
  ${acados_include}/qpOASES_e/
)

link_directories(${PROJECT_SOURCE_DIR}/include/crazyflie_full_model/c_generated_code)
link_directories(${PROJECT_SOURCE_DIR}/include/crazyflie_simple_model/c_generated_code)

# acados DEBUG for 13 state model
add_executable(acados_debug src/acados_debug.cpp)
  # Specify libraries to link a library or executable target against
target_link_libraries(acados_debug
  ${PROJECT_SOURCE_DIR}/include/crazyflie_full_model/c_generated_code/libacados_solver_crazyflie.so
  ${catkin_LIBRARIES}
)
add_dependencies(acados_debug ${PROJECT_NAME}_gencfg)

#simple model of the crazyflie with 7 states
add_executable(acados_simple_model src/acados_simple_model.cpp)
  # Specify libraries to link a library or executable target against
target_link_libraries(acados_simple_model
  ${PROJECT_SOURCE_DIR}/include/crazyflie_simple_model/c_generated_code/libacados_solver_crazyflie_simple_model.so
  ${catkin_LIBRARIES}
)

# measurement vector node
add_executable(measurements_vector src/measurements_vector.cpp)
target_link_libraries(measurements_vector
  ${catkin_LIBRARIES}
)
add_dependencies(measurements_vector measurements_vector_generate_messages_cpp)

# crazyflie imu node
add_executable(crazyflie_imu src/crazyflie_imu.cpp)
  # Specify libraries to link a library or executable target against
target_link_libraries(crazyflie_imu
  ${catkin_LIBRARIES}
)

# linear velocity estimation
add_executable(linvelo_estimation src/linvelo_estimation.cpp)
  # Specify libraries to link a library or executable target against
target_link_libraries(linvelo_estimation
  ${catkin_LIBRARIES}
)
