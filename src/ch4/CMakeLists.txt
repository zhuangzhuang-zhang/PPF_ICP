cmake_minimum_required(VERSION 3.0.2)
project(ch4)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(sophus REQUIRED)
set(Sophus_LIBRARIES libSophus.so)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS
        sophus
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(hophus src/hophus.cpp)
target_link_libraries(hophus
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Sophus_LIBRARIES}
)





