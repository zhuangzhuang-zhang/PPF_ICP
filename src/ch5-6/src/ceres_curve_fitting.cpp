#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

//cost function

struct CURVE_FITTING_COST
{
  CURVE_FITTING_COST ( double x, double y ) : _x ( x ), _y ( y ) {}
  // computer residual
  template <typename T>
  bool operator() (
      const T* const abc,     // template parameters，three dimensional
      T* residual ) const     // residual
  {
      residual[0] = T ( _y ) - ceres::exp ( abc[0]*T ( _x ) *T ( _x ) + abc[1]*T ( _x ) + abc[2] ); // y-exp(ax^2+bx+c)
      return true;
  }
  const double _x, _y;    // x,y data
};

int main ( int argc, char** argv )
{
  ros::init(argc, argv, "ceres");

  double a=1.0, b=2.0, c=1.0;         // true
  int N=100;                          // data number
  double w_sigma=1.0;                 // noise Sigma value
  cv::RNG rng;                        // OpenCV random number generator
  double abc[3] = {0,0,0};            // abc valuation

  std::vector<double> x_data, y_data;      // 数据

  std::cout<<"generating data: "<<std::endl;
  for ( int i=0; i<N; i++ )
  {
      double x = i/100.0;
      x_data.push_back (x);
      y_data.push_back (
          exp ( a*x*x + b*x + c ) + rng.gaussian ( w_sigma )
      );
      std::cout<<x_data[i]<<" "<<y_data[i]<<std::endl;
  }

  // Construct the least squares problem
  ceres::Problem problem;
  for ( int i=0; i<N; i++ )
  {
      problem.AddResidualBlock (
          new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3> (
              new CURVE_FITTING_COST ( x_data[i], y_data[i] )
          ),
          nullptr,            // Kernel function
          abc                 // need to estimate
      );
  }

  ceres::Solver::Options options;     //
  options.linear_solver_type = ceres::DENSE_QR;  //
  options.minimizer_progress_to_stdout = true;   //

  ceres::Solver::Summary summary;                //
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  ceres::Solve ( options, &problem, &summary );  //
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
  std::cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<std::endl;


  std::cout<<summary.BriefReport() <<std::endl;
  std::cout<<"estimated a,b,c = ";
  for ( auto a:abc ) std::cout<<a<<" ";
  std::cout<<std::endl;

  return 0;
}


































