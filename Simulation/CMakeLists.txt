cmake_minimum_required(VERSION 3.0.2)
project(computer_vision_t3)

find_package(catkin REQUIRED
    gazebo_ros
    gazebo_ros_control
    controller_manager
    velocity_controllers
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)