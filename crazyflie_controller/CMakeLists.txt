cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_controller)

find_package(
	catkin REQUIRED COMPONENTS
	std_msgs
	tf
	dynamic_reconfigure
	message_generation
	crazyflie_demo
	rrt_planner	
)

find_package(
	Eigen3 REQUIRED
)

set(acados_include "${PROJECT_SOURCE_DIR}/../acados/include")
set(acados_lib "${PROJECT_SOURCE_DIR}/../acados/lib")

set(full_model_build ${PROJECT_SOURCE_DIR}/scripts/crazyflie_full_model/c_generated_code)

generate_dynamic_reconfigure_options(
	config/crazyflie_params.cfg
	config/crazyflie_estimator.cfg
)

add_message_files(
	FILES
	GenericLogData.msg
	CrazyflieState.msg
	PropellerSpeeds.msg
	PropellerSpeedsStamped.msg
	EulerAnglesStamped.msg
	CrazyflieStateStamped.msg
	CrazyflieOpenloopTraj.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	crazyflie_controller
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
	${full_model_build}
	${acados_include}
	${acados_include}/blasfeo/include/
	${acados_include}/hpipm/include/
	${acados_include}/acados/
	${acados_include}/qpOASES_e/
)

link_directories(${full_model_build})

# NMPC node
add_executable(acados_mpc src/acados_mpc.cpp)
target_link_libraries(acados_mpc
	${full_model_build}/libacados_solver_crazyflie.so
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)
add_dependencies(acados_mpc ${PROJECT_NAME}_gencfg)
add_dependencies(acados_mpc ${PROJECT_NAME}_gencpp)

# Estimator node
add_executable(acados_estimator src/acados_estimator.cpp)
target_link_libraries(acados_estimator
	${full_model_build}/libacados_solver_crazyflie.so
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)
add_dependencies(acados_estimator ${PROJECT_NAME}_gencfg)
add_dependencies(acados_estimator ${PROJECT_NAME}_gencpp)

# IMU node
add_executable(crazyflie_imu src/crazyflie_imu.cpp)
target_link_libraries(crazyflie_imu
  ${catkin_LIBRARIES}
)

add_executable(path_gen_client src/path_gen_client.cpp)
target_link_libraries(path_gen_client ${catkin_LIBRARIES})

# NMPC state vector node
add_executable(measurements_vector src/measurements_vector.cpp)
target_link_libraries(measurements_vector
  ${catkin_LIBRARIES}
)
add_dependencies(measurements_vector ${PROJECT_NAME}_gencfg)
add_dependencies(measurements_vector ${PROJECT_NAME}_gencpp)

catkin_install_python(PROGRAMS src/map_gen.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS src/rrt_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)