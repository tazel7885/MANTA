cmake_minimum_required(VERSION 2.8.3)
project(manta_main)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES manta_led
  CATKIN_DEPENDS roscpp std_msgs roslib
  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/manta_led_node.cpp src/Led.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} pigpiod_if2)