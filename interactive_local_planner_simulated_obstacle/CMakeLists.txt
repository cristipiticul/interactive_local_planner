cmake_minimum_required(VERSION 2.8.3)
project(interactive_local_planner_simulated_obstacle)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_msgs
  rospy
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

