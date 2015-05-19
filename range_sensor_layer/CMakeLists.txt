cmake_minimum_required(VERSION 2.8.3)
project(range_sensor_layer)

find_package(catkin REQUIRED COMPONENTS
  angles
  roscpp
  costmap_2d
  sensor_msgs
  pluginlib)

generate_dynamic_reconfigure_options(cfg/RangeSensorLayer.cfg)

catkin_package(
CATKIN_DEPENDS sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/range_sensor_layer.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS range_sensor_layer
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES costmap_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

install(PROGRAMS scripts/send_message.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
