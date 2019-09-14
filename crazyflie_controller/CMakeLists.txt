cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_controller)

#find_package(PythonLibs REQUIRED)
#include_directories(${PYTHON_INCLUDE_DIRS})

find_package(
	catkin REQUIRED COMPONENTS
	std_msgs
	tf
	dynamic_reconfigure
	message_generation
)

find_package(
	Eigen3 REQUIRED
)

set(acados_include "${PROJECT_SOURCE_DIR}/../acados/include")
set(acados_lib "${PROJECT_SOURCE_DIR}/../acados/lib")
set(full_model_build ${PROJECT_SOURCE_DIR}/scripts/crazyflie_full_model/c_generated_code/)
set(simple_model_build ${PROJECT_SOURCE_DIR}/scripts/crazyflie_simple_model/c_generated_code/)

generate_dynamic_reconfigure_options(
	config/crazyflie_params.cfg
)

add_message_files(
	FILES
	GenericLogData.msg
	CrazyflieState.msg
	PropellerSpeeds.msg
	PropellerSpeedsStamped.msg
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

## Build

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
	${full_model_build}
	${full_model_build}
	${acados_include}
	${acados_include}/blasfeo/include/
	${acados_include}/hpipm/include/
	${acados_include}/acados/
	${acados_include}/qpOASES_e/
)

link_directories(${full_model_build})
link_directories(${simple_model_build})

add_executable(acados_mpc src/acados_mpc.cpp)
target_link_libraries(acados_mpc
	${full_model_build}libacados_solver_crazyflie.so
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)
add_dependencies(acados_mpc ${PROJECT_NAME}_gencfg)

add_executable(acados_estimator src/acados_estimator.cpp)
target_link_libraries(acados_estimator
	${full_model_build}libacados_solver_crazyflie.so
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)
add_dependencies(acados_estimator ${PROJECT_NAME}_gencfg)

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
