cmake_minimum_required(VERSION 2.8.3)
project(aliengo_description)

find_package(catkin REQUIRED COMPONENTS
    genmsg
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf
)

catkin_package(
    CATKIN_DEPENDS
)

include_directories(
    # include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

add_executable(tf_robot_pose src/tf_robot_pose.cpp)
target_link_libraries(tf_robot_pose ${catkin_LIBRARIES} ${LIBS})
include_directories(include ${catkin_INCLUDE_DIRS} )

