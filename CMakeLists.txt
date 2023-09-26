cmake_minimum_required(VERSION 3.5.0)
project(switch_controller VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

set(KORTEX_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/kortex_api")
set(KORTEX_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/kortex_api")

# Panda
find_package(Franka 0.9.0 REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)



include_directories(
  include/
  ${EIGEN3_INCLUDE_DIR}
	# include/kortex_api/
	# include/kortex_api/common/
	# include/kortex_api/client/
	# include/kortex_api/client_stubs/
	# include/kortex_api/messages/
	# include/kortex_api/cxxopts/
	# include/kortex_api/google/
)
link_libraries(Franka::Franka Threads::Threads)

#for kortex API
#for kortex API
include_directories(
  ${KORTEX_INCLUDE_DIR}
  ${KORTEX_INCLUDE_DIR}/client
  ${KORTEX_INCLUDE_DIR}/common
  ${KORTEX_INCLUDE_DIR}/messages
  ${KORTEX_INCLUDE_DIR}/client_stubs
  ${KORTEX_INCLUDE_DIR}/cxxopts/
  ${KORTEX_INCLUDE_DIR}/messages/
)
add_definitions(-D_OS_UNIX)


# link_libraries(${CMAKE_SOURCE_DIR}/lib/kortex_api/libKortexApiCpp.a)
#link_libraries(pthread)


set(
SRC_FILES 
	src/switch_controller.cpp
)
add_executable(switch_controller ${SRC_FILES})
target_link_libraries(Franka::Franka)
target_link_libraries(switch_controller ${KORTEX_LIB_DIR}/libKortexApiCpp.a)
#target_link_libraries(switch_controller ${CMAKE_SOURCE_DIR}/lib/kortex_api/libKortexApiCpp.a)
target_link_libraries(switch_controller pthread)