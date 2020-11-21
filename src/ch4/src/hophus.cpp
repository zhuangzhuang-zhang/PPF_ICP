#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include "sophus/so3.h"
#include "sophus/se3.h"


int main ( int argc, char** argv )
{
  ros::init(argc, argv, "feature_extraction");

  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();

  Sophus::SO3 SO3_R(R);
  Sophus::SO3 SO3_v( 0, 0, M_PI/2 );
  Eigen::Quaterniond q(R);
  Sophus::SO3 SO3_q( q );

  std::cout<<"SO(3) from matrix: "<<SO3_R<<std::endl;
  std::cout<<"SO(3) from vector: "<<SO3_v<<std::endl;
  std::cout<<"SO(3) from quaternion :"<<SO3_q<<std::endl;

  Eigen::Vector3d so3 = SO3_R.log();
  std::cout<<"so3 = "<<so3.transpose()<<std::endl;
  std::cout<<"so3 hat=\n"<<Sophus::SO3::hat(so3)<<std::endl;

  std::cout<<"so3 hat vee= "<<Sophus::SO3::vee( Sophus::SO3::hat(so3) ).transpose()<<std::endl;

  Eigen::Vector3d update_so3(1e-4, 0, 0);
  Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3)*SO3_R;
  std::cout<<"SO3 updated = "<<SO3_updated<<std::endl;

/****************************************************************************/
  Eigen::Vector3d t(1,0,0);
  Sophus::SE3 SE3_Rt(R, t);
  Sophus::SE3 SE3_qt(q,t);
  std::cout<<SE3_Rt<<std::endl;
  std::cout<<SE3_qt<<std::endl;
//  std::cout<<"SE3 from q,t= "<std::endl<<SE3_qt<<std::endl;

  typedef Eigen::Matrix<double,6,1> Vector6d;
  Vector6d se3 = SE3_Rt.log();
  std::cout<<"se3 = "<<se3.transpose()<<std::endl;

  std::cout<<"se3 hat = "<<std::endl<<Sophus::SE3::hat(se3)<<std::endl;
  std::cout<<"se3 hat vee = "<<Sophus::SE3::vee( Sophus::SE3::hat(se3) ).transpose()<<std::endl;

  Vector6d update_se3;
  update_se3.setZero();
  update_se3(0,0) = 1e-4d;
  Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3)*SE3_Rt;
  std::cout<<"SE3 updated = "<<std::endl<<SE3_updated.matrix()<<std::endl;

  return 0;
}













