#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>

#include <chrono>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "point_clouds");
  ros::Publisher pub;

/*
  cv::Mat image;
  image = cv::imread("/home/zzz/pose_estimation/src/pre_learning/src/ubuntu.png");
  if (image.data == nullptr)
  {
    std::cout<<"file"<<argv[1]<<"no file"<<std::endl;
    return 0;
  }

  cout<<"width"<<image.cols<<",height"<<image.rows<<",channels"<<image.channels()<<endl;
  //cv::imshow ( "image", image );
//  cv::waitKey(2000);

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  for (size_t y=0; y<image.rows; y++)
  {
    unsigned char* row_ptr = image.ptr<unsigned char> (y);
    for (size_t x=0; x<image.cols; x++)
    {
      unsigned char* data_ptr = &row_ptr[ x*image.channels()];
      for ( int c = 0; c != image.channels(); c++ )
      {
        unsigned char data = data_ptr[c];
      }
    }
  }
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
  std::cout<<"time cost："<<time_used.count()<<"s"<<std::endl;

  cv::Mat image_clone = image.clone();
  image_clone ( cv::Rect ( 0,0,100,100 ) ).setTo ( 255 );
  cv::imshow ( "image", image );
  cv::imshow ( "image_clone", image_clone );
  cv::waitKey (2000);

  cv::destroyAllWindows();
*/


  std::vector<cv::Mat> colorImgs, depthImgs;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;//camera pose

  std::ifstream fin("/home/zzz/pose_estimation/src/pre_learning/src/pose.txt");
  if (!fin)
  {
      std::cout<<"can not find pose.txt"<<std::endl;
      return 1;
  }

  for ( int i=0; i<5; i++ )
  {
    std::string color = cv::format("/home/zzz/pose_estimation/src/pre_learning/src/color/%d.png", i+1);
    std::string depth = cv::format("/home/zzz/pose_estimation/src/pre_learning/src/depth/%d.pgm", i+1);


//    boost::format fmt( "./%s/%d.%s" );
//    colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
//    depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 ));

//    colorImgs.push_back(cv::imread(color));
//    depthImgs.push_back(cv::imread(depth, -1)); //

    double data[7] = {0};
    for ( auto& d:data ){
      fin>>d;
    }
    Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
    Eigen::Isometry3d T(q);
    T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
    poses.push_back( T );
  }

////camera intrinsics, camera extrinsics
  double cx = 325.5;
  double cy = 253.5;
  double fx = 518.0;
  double fy = 519.0;
  double depthScale = 1000.0;

  cout<<"transform picture to point clouds..."<<endl;

  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloud;

  PointCloud::Ptr pointCloud( new PointCloud );

  for ( int i=0; i<5; i++ )
  {
      cout<<"transform picture: "<<i+1<<endl;
      cv::Mat color = colorImgs[i];
      cv::Mat depth = depthImgs[i];
      Eigen::Isometry3d T = poses[i];
      for ( int v=0; v<color.rows; v++ )
          for ( int u=0; u<color.cols; u++ )
          {
            unsigned int d = depth.ptr<unsigned short> (v)[u];
            if ( d==0 ) continue;
            Eigen::Vector3d point;
            point[2] = double(d)/depthScale;
            point[0] = (u-cx)*point[2]/fx;
            point[1] = (v-cy)*point[2]/fy;
            Eigen::Vector3d pointWorld = T*point;

            PointT p ;
            p.x = pointWorld[0];
            p.y = pointWorld[1];
            p.z = pointWorld[2];
//            p.b = color.data[ v*color.step+u*color.channels() ];
//            p.g = color.data[ v*color.step+u*color.channels()+1 ];
//            p.r = color.data[ v*color.step+u*color.channels()+2 ];

            p.b = color.at<cv::Vec3b>(v, u)[0];  //B
            p.g = color.at<cv::Vec3b>(v, u)[1];  //G
            p.r = color.at<cv::Vec3b>(v, u)[2];  //R

            pointCloud->points.push_back( p );
          }
  }

  pointCloud->is_dense = false;
  cout<<"number of point clouds: "<<pointCloud->size()<<endl;
  pcl::io::savePCDFileBinary("map.pcd", *pointCloud );
  pointCloud->points.clear();
  std::cout<<"Point cloud saved."<<std::endl;

  return 0;
}

















