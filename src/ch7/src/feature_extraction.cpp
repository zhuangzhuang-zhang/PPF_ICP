#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>


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

  //init
  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
  cv::Mat descriptors_1, descriptors_2;
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
  cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );

// first, detect the Oriented FAST Corner
  detector->detect (img_1, keypoints_1);
  detector->detect (img_2, keypoints_2);
// second, computer BRIEF descriptor
  descriptor->compute (img_1, keypoints_1, descriptors_1);
  descriptor->compute (img_2, keypoints_2, descriptors_2);

//  cv::Mat outimg1;
//  cv::drawKeypoints( img_1, keypoints_1, outimg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
//  cv::imshow("ORB key points",outimg1);

// third, match BRIEF descriptor using Hamming distance
  std::vector<cv::DMatch> matches;
  //BFMatcher matcher ( NORM_HAMMING );
  matcher->match ( descriptors_1, descriptors_2, matches);
//fourth, matches filter
  double min_dist=10000, max_dist=0;

  for (int i = 0; i < descriptors_1.rows; i++)
  {
    double dist = matches[i].distance;
    if ( dist < min_dist ) min_dist = dist;
    if ( dist > max_dist ) max_dist = dist;
  }

  printf ( "-- Max dist : %f \n", max_dist );
  printf ( "-- Min dist : %f \n", min_dist );

//choose the descriptor (Hamming < 2*min_dist), set low bound as 30
  std::vector< cv::DMatch > good_matches;
  for (int i = 0; i < descriptors_1.rows; i++)
  {
    if (matches[i].distance <= std::max(2*min_dist, 30.0))
    {
      good_matches.push_back (matches[i]);
    }
  }

//Draw the match result
  cv::Mat img_match;
  cv::Mat img_goodmatch;
  cv::drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
  cv::drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
  cv::imshow ( "all matches", img_match );
  cv::imshow ( "all matches after optimization", img_goodmatch );

  cv::waitKey(500000);

  return 0;
}













