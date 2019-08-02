cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_controller)

#find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  tf
  message_generation
 # pybind11_catkin
)

find_package(Eigen3 REQUIRED)

set(acados_include "/usr/local/include")

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  acadosOutput.msg
  crazyflieStates.msg
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
  include/c_generated_code/crazyflie_model
  include/c_generated_code
  include/
  ${acados_include} 
  ${acados_include}/blasfeo/include/
  ${acados_include}/acados/ 
  ${acados_include}/qpOASES_e/
  ${crazyflie_driver_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(crazyflie_controller
#   src/${PROJECT_NAME}/crazyflie_controller.cpp
# )

# Declare a cpp executable
#add_executable(crazyflie_controller
#  src/controller.cpp)
  # Specify libraries to link a library or executable target against
#target_link_libraries(crazyflie_controller
#  ${catkin_LIBRARIES}
#)

# acados executable
#add_executable(acados_nmpc src/acados_nmpc.cpp)
  # Specify libraries to link a library or executable target against
#target_link_libraries(acados_nmpc
#  ${PROJECT_SOURCE_DIR}/include/c_generated_code/libacados_solver_crazyflie.so
#  ${catkin_LIBRARIES}
#)
#add_dependencies(acados_nmpc ${catkin_EXPORTED_TARGETS})

# acados DEBUG
add_executable(acados_debug src/acados_debug.cpp)
  # Specify libraries to link a library or executable target against
target_link_libraries(acados_debug
  ${PROJECT_SOURCE_DIR}/include/c_generated_code/libacados_solver_crazyflie.so
  ${catkin_LIBRARIES}
)
add_dependencies(acados_debug acados_generate_messages_cpp)

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

# RL trajectory generation test
#pybind_add_module(rl_trajectory_generation SHARED src/rl_trajectory_generation.cpp)

#add_executable(rl_test src/rl_test.cpp)
#target_link_libraries(rl_test ${PYTHON_LIBRARIES} rl_trajectory_generation)


# Add cmake target dependencies of the executable/library
# as an example, message headers may need to be generated before nodes
# add_dependencies(crazyflie_controller crazyflie_controller_generate_messages_cpp)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS crazyflie_controller crazyflie_controller_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_crazyflie_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
