cmake_minimum_required(VERSION 2.8.3)
project(manta_mp3)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
  sensor_msgs
  geometry_msgs
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


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES manta_mp3_lib
  CATKIN_DEPENDS roscpp std_msgs roslib sensor_msgs geometry_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARIES}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(manta_mp3_lib src/Mp3.cpp)
add_dependencies(manta_mp3_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manta_mp3_lib ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/manta_mp3_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} manta_mp3_lib)

#install(TARGETS ${PROJECT_NAME}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)