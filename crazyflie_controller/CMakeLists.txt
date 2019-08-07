cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_controller)

#find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  tf
 # pybind11_catkin
)

find_package(Eigen3 REQUIRED)

set(acados_include "/usr/local/include")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/c_generated_code/crazyflie_model
  include/c_generated_code
  include/
  ${acados_include} 
  ${acados_include}/blasfeo/include/
  ${acados_include}/acados/ 
  ${acados_include}/qpOASES_e/
  ${crazyflie_driver_INCLUDE_DIRS}
)

# acados DEBUG
add_executable(acados_debug src/acados_debug.cpp)
  # Specify libraries to link a library or executable target against
target_link_libraries(acados_debug
  ${PROJECT_SOURCE_DIR}/include/c_generated_code/libacados_solver_crazyflie.so
  ${catkin_LIBRARIES}
)
#add_dependencies(acados_debug acados_generate_messages_cpp)
#add_dependencies(acados_debug ${PROJECT_NAME}_gencfg)

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
