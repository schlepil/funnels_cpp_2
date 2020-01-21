//
// Created by philipp on 1/16/20.
//

#include "heuristics/ex1_fixed.hh"

Eigen::Matrix2d getR(double alpha){
  double s_a = std::sin(alpha);
  double c_a = std::cos(alpha);
  Eigen::Matrix2d R;
  R(0,0) = c_a;
  R(1,1) = c_a;
  R(0,1) = -s_a;
  R(1,0) =  s_a;
//  R << c_a, -s_a, c_a, s_a;
  return R;
}