
################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################

cmake_minimum_required(VERSION 3.0.2)
project(manta_dxl)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################


find_package(catkin REQUIRED COMPONENTS
geometry_msgs
roscpp
roslib
rospy
sensor_msgs
std_msgs
dynamixel_sdk
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

################################################################################
# Declare ROS messages, services and actions
################################################################################


################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES manta_dxl_lib
 CATKIN_DEPENDS geometry_msgs roscpp roslib rospy sensor_msgs std_msgs dynamixel_sdk
#  DEPENDS system_lib
)


################################################################################
# Build
################################################################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(manta_dxl_lib src/Parser.cpp src/trajectory.cpp)
add_dependencies(manta_dxl_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manta_dxl_lib ${catkin_LIBRARIES} dynamixel_sdk ${YAML_CPP_LIBRARIES})

add_executable(manta_dxl_node src/manta_dxl_node.cpp)
add_dependencies(manta_dxl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manta_dxl_node ${catkin_LIBRARIES} manta_dxl_lib )




# add_library(manta_dxl_lib src/trajectory.cpp src/dxl_func.cpp)
# add_dependencies(manta_dxl_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(manta_dxl_lib ${catkin_LIBRARIES} dynamixel_sdk ${YAML_CPP_LIBRARIES})


# add_executable(manta_dxl_node src/bulk_read_write_node.cpp)
# add_dependencies(manta_dxl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(manta_dxl_node ${catkin_LIBRARIES} manta_dxl_lib ${YAML_CPP_LIBRARIES})
