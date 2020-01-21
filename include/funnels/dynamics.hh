//
// Created by philipp on 26.12.19.
//

#ifndef FUNNELS_CPP_DYNAMICS_HH
#define FUNNELS_CPP_DYNAMICS_HH

#include <Eigen/Core>
#include <cmath>
#include <cassert>
#include <memory>


using namespace Eigen;

namespace dynamics {
  
  class kinematic_2d_sys_t {
  public:
    constexpr static const size_t dimx = 4, dimp = 2, dimv = 2, dimu = 2;
  
    using matrix_t = Eigen::Matrix<double, dimx, Eigen::Dynamic>;
    using vector_x_t = Eigen::Matrix<double, dimx, 1>;
    using vector_u_t = Eigen::Matrix<double, dimu, 1>;
    using vector_t_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;
    using matrix_ptr_t = std::shared_ptr<matrix_t>;
    using vector_x_ptr_t = std::shared_ptr<vector_x_t>;
    using vector_u_ptr_t = std::shared_ptr<vector_u_t>;
    using vector_t_ptr_t = std::shared_ptr<vector_t_t>;
    
    static void
    compute(matrix_t &x, const vector_u_t &u, const vector_t_t &t);
  };
}
#endif //FUNNELS_CPP_DYNAMICS_HH
