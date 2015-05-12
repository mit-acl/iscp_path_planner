cmake_minimum_required(VERSION 2.8.12)
project(iscp_path_planner)

add_compile_options(-std=c++0x)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	nav_msgs
	actionlib_msgs
	acl_msgs
	)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES quad_control
	CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs acl_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

link_directories($ENV{HOME}/mosek/7/tools/platform/linux64x86/bin)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  $ENV{HOME}/mosek/7/tools/platform/linux64x86/h
)

add_executable(${PROJECT_NAME} src/main.cpp src/path_planner.cpp src/RosWrapper.cpp src/pathplannerarray.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} mosek64)
