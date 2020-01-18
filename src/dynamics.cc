//
// Created by philipp on 17.01.20.
//
#include "funnels/dynamics.hh"

namespace dynamics {
//  constexpr size_t kinematic_2d_sys_t::dimx() {
//    return _dimx;
//  }
//  constexpr size_t kinematic_2d_sys_t::dimp() {
//    return _dimp;
//  }
//  constexpr size_t kinematic_2d_sys_t::dimv() {
//    return _dimv;
//  }
//  constexpr size_t kinematic_2d_sys_t::dimu() {
//    return _dimu;
//  }
  
  void kinematic_2d_sys_t::compute(matrix_t &x,
      const vector_u_t &u, const vector_t_t &t) {
    assert(x.cols() == t.size());
    assert(u.rows() == dimu && x.rows() == dimx && u.rows() == dimv);
    // State vector is [x,y,dx,dy]
    const Vector2d x0 = x.block<dimp, 1>(0, 0);
    size_t n = t.size();
    // Set velocity
    // For a kinematic sys, the velocity can change instantly
    x.block(dimp, 0, dimv, n).colwise() = u;
    // Compute state
    for (size_t i = 0; i < n; i++) {
      x.block<dimp, 1>(0, i) = x0 + u * t(i);
    }
    // Done
  }
}