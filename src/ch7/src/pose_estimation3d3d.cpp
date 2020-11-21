#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Eigen>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>

void find_feature_matches (const cv::Mat& img_1, const cv::Mat& img_2,
                            std::vector<cv::KeyPoint>& keypoints_1,
                            std::vector<cv::KeyPoint>& keypoints_2,
                           std::vector<cv::DMatch >& matches );

cv::Point2d pixel2cam ( const cv::Point2d& p, const cv::Mat& K );

void pose_estimation_3d3d (
    const std::vector<cv::Point3f>& pts1,
    const std::vector<cv::Point3f>& pts2,
    cv::Mat& R, cv::Mat& t);

//iterative closest point (ICP) 3D->3D
int main ( int argc, char** argv )
{
  ros::init(argc, argv, "feature_extraction");

  //load image
  cv::Mat img_1 = cv::imread ("/home/zzz/pose_estimation/src/ch7/src/1.png", CV_LOAD_IMAGE_COLOR );
  cv::Mat img_2 = cv::imread ("/home/zzz/pose_estimation/src/ch7/src/2.png", CV_LOAD_IMAGE_COLOR );

  if (img_1.data == nullptr || img_2.data == nullptr)
  {
    std::cout << "can not find color images" << std::endl;
    return 0;
  }

  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
  std::vector<cv::DMatch> matches;
  find_feature_matches (img_1, img_2, keypoints_1, keypoints_2, matches);
  std::cout<<"total: "<<matches.size() <<" "<<"matches"<<std::endl;

//build 3D points
  cv::Mat depth1 = cv::imread ("/home/zzz/pose_estimation/src/ch7/src/1_depth.png", CV_LOAD_IMAGE_UNCHANGED );
  cv::Mat depth2 = cv::imread ("/home/zzz/pose_estimation/src/ch7/src/2_depth.png", CV_LOAD_IMAGE_UNCHANGED );

  if (depth1.data == nullptr || depth2.data == nullptr)
  {
    std::cout << "can not find depth images" << std::endl;
    return 0;
  }

  cv::Mat K = (cv::Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
  std::vector<cv::Point3f> pts1, pts2;
  for (cv::DMatch m:matches )
  {
    ushort d1 = depth1.ptr<unsigned short> ( int ( keypoints_1[m.queryIdx].pt.y ) ) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
    ushort d2 = depth2.ptr<unsigned short> ( int ( keypoints_2[m.trainIdx].pt.y ) ) [ int ( keypoints_2[m.trainIdx].pt.x ) ];
    if ( d1==0 || d2==0 )   // bad depth
        continue;
    cv::Point2d p1 = pixel2cam ( keypoints_1[m.queryIdx].pt, K );
    cv::Point2d p2 = pixel2cam ( keypoints_2[m.trainIdx].pt, K );
    float dd1 = float ( d1 ) /5000.0;
    float dd2 = float ( d2 ) /5000.0;
    pts1.push_back (cv::Point3f (p1.x*dd1, p1.y*dd1, dd1 ));
    pts2.push_back (cv::Point3f (p2.x*dd2, p2.y*dd2, dd2 ));
  }
  std::cout<<"3d-3d pairs: "<<pts1.size() <<std::endl;
  cv::Mat R, t;
  pose_estimation_3d3d ( pts1, pts2, R, t );
  std::cout<<"ICP via SVD results: "<<std::endl;
  std::cout<<"R = "<<R<<std::endl;
  std::cout<<"t = "<<t<<std::endl;
  std::cout<<"R_inv = "<<R.t() <<std::endl;
  std::cout<<"t_inv = "<<-R.t() *t<<std::endl;

  return 0;
}


void find_feature_matches (const cv::Mat& img_1, const cv::Mat& img_2,
                            std::vector<cv::KeyPoint>& keypoints_1,
                            std::vector<cv::KeyPoint>& keypoints_2,
                            std::vector<cv::DMatch >& matches )
{
  //init
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

  // third, match BRIEF descriptor using Hamming distance
  std::vector<cv::DMatch> match;
  //BFMatcher matcher ( NORM_HAMMING );
  matcher->match ( descriptors_1, descriptors_2, match);

  //fourth, match filter
  double min_dist=10000, max_dist=0;

  for (int i = 0; i < descriptors_1.rows; i++)
  {
    double dist = match[i].distance;
    if ( dist < min_dist ) min_dist = dist;
    if ( dist > max_dist ) max_dist = dist;
  }

  printf ( "-- Max dist : %f \n", max_dist );
  printf ( "-- Min dist : %f \n", min_dist );

  for (int i = 0; i < descriptors_1.rows; i++)
  {
    if (match[i].distance <= std::max(2*min_dist, 30.0))
    {
      matches.push_back (match[i]);
    }
  }
}

cv::Point2d pixel2cam ( const cv::Point2d& p, const cv::Mat& K )
{
  return cv::Point2d
     (
       ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
       ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
     );
}

void pose_estimation_3d3d (
    const std::vector<cv::Point3f>& pts1,
    const std::vector<cv::Point3f>& pts2,
    cv::Mat& R, cv::Mat& t
)
{
  cv::Point3f p1, p2;     // center of mass
  int N = pts1.size();
  for ( int i=0; i<N; i++ )
  {
      p1 += pts1[i];
      p2 += pts2[i];
  }

  p1 = cv::Point3f(cv::Vec3f(p1) / N);
  p2 = cv::Point3f(cv::Vec3f(p2) / N);

  std::vector<cv::Point3f> q1(N), q2(N); // remove the center
  for (int i=0; i<N; i++)
  {
      q1[i] = pts1[i] - p1;
      q2[i] = pts2[i] - p2;
  }
  //compute q1*q2^T
  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();

  for (int i=0; i<N; i++)
  {
      W += Eigen::Vector3d (q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d (q2[i].x, q2[i].y, q2[i].z).transpose();
  }
  std::cout<<"W="<<W<<std::endl;

  //SVD on W
  Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  if (U.determinant() * V.determinant() < 0)
  {
    for (int x = 0; x < 3; ++x)
    {
        U(x, 2) *= -1;
    }
  }
  std::cout<<"U="<<U<<std::endl;
  std::cout<<"V="<<V<<std::endl;

  Eigen::Matrix3d R_ = U* (V.transpose());
  Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d (p2.x, p2.y, p2.z);
  // convert to cv::Mat
  R = (cv::Mat_<double> ( 3,3 ) <<
        R_ ( 0,0 ), R_ ( 0,1 ), R_ ( 0,2 ),
        R_ ( 1,0 ), R_ ( 1,1 ), R_ ( 1,2 ),
        R_ ( 2,0 ), R_ ( 2,1 ), R_ ( 2,2 )
      );
  t = (cv::Mat_<double> ( 3,1 ) << t_ ( 0,0 ), t_ ( 1,0 ), t_ ( 2,0 ) );
}


























































