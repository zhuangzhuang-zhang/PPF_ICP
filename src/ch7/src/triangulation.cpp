#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>



int main ( int argc, char** argv )
{
  ros::init(argc, argv, "feature_extraction");

  //load image
  cv::Mat img_1 = cv::imread ("/home/zzz/pose_estimation/src/ch7/src/1.png", CV_LOAD_IMAGE_COLOR );
  cv::Mat img_2 = cv::imread ("/home/zzz/pose_estimation/src/ch7/src/2.png", CV_LOAD_IMAGE_COLOR );

  if (img_1.data == nullptr || img_2.data == nullptr)
  {
    std::cout << "can not find images" << std::endl;
    return 0;
  }



  return 0;
}













