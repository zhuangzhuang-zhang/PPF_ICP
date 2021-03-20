#pragma once
#include <opencv2/core.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/flann.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <iostream>
#include <fstream>
#include <vector>

#include "utils.hpp"
#include "hash_int.hpp"
#include "object6D.hpp"


namespace ppf
{
typedef pcl::PointXYZ PointT1;
typedef pcl::PointNormal PointT2;
typedef pcl::PointCloud<PointT1> CloudPointT1;
typedef pcl::PointCloud<PointT2> CloudPointT2;
typedef pcl::PointCloud<PointT1>::ConstPtr CloudPointT1Ptr;
typedef pcl::PointCloud<PointT2>::ConstPtr CloudPointT2Ptr;

typedef struct THash
{
    int id;
    int i, ppfInd;
} THash;


class PPF6DDetector
{

public:


  void clearTrainingModels();


  void computePPFFeatures(const cv::Vec3d& p1, const cv::Vec3d& n1,
                          const cv::Vec3d& p2, const cv::Vec3d& n2,
                          cv::Vec4d& f);

  virtual ~PPF6DDetector();
private:



};
}






