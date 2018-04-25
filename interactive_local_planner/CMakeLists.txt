cmake_minimum_required(VERSION 2.8.3)
project(interactive_local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  dwa_local_planner
  cmake_modules
  costmap_2d
  dynamic_reconfigure
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf
)

# dynamic reconfigure
#generate_dynamic_reconfigure_options(
#    cfg/DWAPlanner.cfg
#)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

find_package(Boost REQUIRED COMPONENTS thread chrono)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES interactive_local_planner
    CATKIN_DEPENDS
        dwa_local_planner
        dynamic_reconfigure
        nav_msgs
        pluginlib
        roscpp
)

add_library(obstacle_classifier src/obstacle_classifier.cpp)
target_link_libraries(obstacle_classifier
  ${catkin_LIBRARIES}
)

add_library(global_planner_interaction src/interactive_local_planner/global_planner_interaction.cpp)
target_link_libraries(global_planner_interaction
  ${catkin_LIBRARIES}
)

add_library(interactive_local_planner src/interactive_local_planner.cpp)
add_dependencies(interactive_local_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  obstacle_classifier
  global_planner_interaction
)
target_link_libraries(interactive_local_planner
  ${catkin_LIBRARIES}
  obstacle_classifier
  global_planner_interaction
)

install(TARGETS interactive_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES interactive_local_planner_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
