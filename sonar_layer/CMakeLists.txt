cmake_minimum_required(VERSION 2.8.3)
project(sonar_layer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  sensor_msgs
  pluginlib)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/sonar_layer.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# TODO: install plugin.xml
