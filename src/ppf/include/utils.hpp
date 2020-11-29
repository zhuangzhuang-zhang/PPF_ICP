#pragma once
#include <opencv/cv.h>
#include <pcl/common/common_headers.h>
#include <cmath>
#include <cstdio>


static const float EPS = 1.192092896e-07F;        /* smallest such that 1.0+FLT_EPSILON != 1.0 */


static inline void TNormalize3(cv::Vec3d& v)
{
  double norm = cv::norm(v);
  if (norm > EPS)
  {
    v *= 1.0 / norm;
  }
}





/*
 *a normalized vector
 *b normalized vector
 *return angle between a and b vectors in radians
*/
static inline double TAngle3Normalized(const cv::Vec3d& a, const cv::Vec3d& b)
{
  /*
   angle = atan2(a dot b, |a x b|) # Bertram (accidental mistake)
   angle = atan2(|a x b|, a dot b) # Tolga Birdal (correction)
   angle = acos(a dot b)           # Hamdi Sahloul (simplification, a & b are normalized)
  */

  return acos(a.dot(b));
}






























































