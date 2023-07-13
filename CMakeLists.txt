cmake_minimum_required(VERSION 3.0.2)
project(dotpad_receiver)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

catkin_package(
 INCLUDE_DIRS 
    include 
    libs
 CATKIN_DEPENDS
    roscpp
    std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include 
    libs
)

add_executable(dotpad_receiver src/dotpad_receiver_node.cpp src/DotPadManager.cpp)
target_link_libraries(dotpad_receiver ${catkin_LIBRARIES})