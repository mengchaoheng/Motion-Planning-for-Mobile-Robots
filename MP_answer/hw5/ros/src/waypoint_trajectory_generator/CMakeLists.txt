cmake_minimum_required(VERSION 3.0)
project(waypoint_trajectory_generator)

set(CMAKE_VERBOSE_MAKEFILE "false")
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native") # will be has died! process has died [pid 29211, exit code -11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  quadrotor_msgs
)
catkin_package()

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(include
    ${Eigen3_INCLUDE_DIRS}
  	${catkin_INCLUDE_DIRS}
  	 
)

add_executable(trajectory_generator_node 
	src/trajectory_generator_node.cpp
	src/trajectory_generator_waypoint.cpp
)
target_link_libraries(trajectory_generator_node OsqpEigen::OsqpEigen)

target_link_libraries(trajectory_generator_node
   ${catkin_LIBRARIES}
)
