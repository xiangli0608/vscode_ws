cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
#set (CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Debug)
# catkin_make -DCMAKE_BUILD_TYPE=Debug
#catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
#   Eigen3
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED QUIET)
# include(cmake/glog.cmake)

# find_package(gflags REQUIRED QUIET)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
 CATKIN_DEPENDS roscpp  tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(test_code src/test_code.cpp)
target_link_libraries(test_code 
${catkin_LIBRARIES} ${Boost_LIBRARIES}
${PCL_LIBRARY_DIRS}
)
add_dependencies(test_code ${catkin_EXPORTED_TARGETS})


add_executable(test_glog src/test_glog.cc)
target_link_libraries(test_glog 
${catkin_LIBRARIES} 
glog
)
add_dependencies(test_glog ${catkin_EXPORTED_TARGETS} )