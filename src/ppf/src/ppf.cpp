#include <ppf.hpp>

namespace ppf
{




// compute per point PPF as in paper
void PPF6DDetector::computePPFFeatures(const cv::Vec3d& p1, const cv::Vec3d& n1,
                                       const cv::Vec3d& p2, const cv::Vec3d& n2,
                                       cv::Vec4d& f)
{
  cv::Vec3d d(p2 - p1);
  f[3] = cv::norm(d);
  if (f[3] <= EPS)
    return;
  d *= 1.0 / f[3];

  f[0] = TAngle3Normalized(n1, d);
  f[1] = TAngle3Normalized(n2, d);
  f[2] = TAngle3Normalized(n1, n2);
}



void PPF6DDetector::clearTrainingModels()
{

}

PPF6DDetector::~PPF6DDetector()
{
    clearTrainingModels();
}

}












