cmake_minimum_required(VERSION 2.8.3)
project(social_navigation_layers)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  people_msgs
  pluginlib
  roscpp
)

generate_dynamic_reconfigure_options(
  cfg/ProxemicLayer.cfg
)

catkin_package(
    CATKIN_DEPENDS angles costmap_2d dynamic_reconfigure geometry_msgs people_msgs pluginlib roscpp
    INCLUDE_DIRS include
    LIBRARIES social_layers
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## add cpp library
add_library(social_layers
            src/social_layer.cpp
            src/proxemic_layer.cpp
            src/passing_layer.cpp
)
add_dependencies(social_layers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    social_layers ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(FILES costmap_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS social_layers
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
