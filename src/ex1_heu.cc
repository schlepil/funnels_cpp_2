//
// Created by philipp on 13.01.20.
//

#include "heuristics/ex1_heu.hh"

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

double get_smallest_englobing(double r_src, double d_v_norm){
  assert(d_v_norm>=0.);
  return min_englobe_fac*(r_src+d_v_norm);
}