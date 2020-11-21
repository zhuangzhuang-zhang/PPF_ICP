cmake_minimum_required(VERSION 3.0.2)
project(ch7)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

 list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
 ## Compile as C++11, supported in ROS Kinetic and newer
 set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

 find_package(Eigen3 REQUIRED)
 if(NOT EIGEN3_INCLUDE_DIRS)
 set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR};/usr/include/eigen3/Eigen/Eigen")
 endif()

 find_package(OpenCV REQUIRED)
 find_package(Ceres REQUIRED )
 find_package(g2o REQUIRED)
 find_package(CSparse REQUIRED)
 find_package(catkin REQUIRED COMPONENTS
   pcl_conversions
   pcl_ros
   sensor_msgs
   roscpp
   rospy
   std_msgs
 )

 catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs
 )

 include_directories(
  include
   ${catkin_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIR}
   ${OpenCV_INCLUDE_DIRS}
   ${CERES_INCLUDE_DIRS}
   ${G2O_INCLUDE_DIRS}
   ${CSPARSE_INCLUDE_DIR}
 )

 add_executable(pose_estimation3d3d src/pose_estimation3d3d.cpp)
 target_link_libraries(pose_estimation3d3d
     ${catkin_LIBRARIES}
     ${Boost_LIBRARIES}
     ${OpenCV_LIBS}
     ${CERES_LIBRARIES}
     g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
     ${CSPARSE_LIBRARY}
 )


