cmake_minimum_required(VERSION 3.0.2)
project(range_sensor_layer)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
    angles
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    pluginlib
    roscpp
    rospy
    sensor_msgs
    tf2_geometry_msgs
)

generate_dynamic_reconfigure_options(cfg/RangeSensorLayer.cfg)

catkin_package(
    CATKIN_DEPENDS
        angles
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        pluginlib
        roscpp
        rospy
        sensor_msgs
        tf2_geometry_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/range_sensor_layer.cpp)
add_dependencies(${PROJECT_NAME} ${range_sensor_layer_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES costmap_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/send_message.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
