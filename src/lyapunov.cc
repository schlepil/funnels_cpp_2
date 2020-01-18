//
// Created by philipp on 22.12.19.
//
#include <Eigen/Dense>

#include "funnels/lyapunov.hh"

namespace lyapunov{
  
  bool lyapunov_t::intersect(const lyap_zone_t &other) const {
    std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
    throw std::runtime_error("Virtual");
    return false;
  }
  
  bool lyapunov_t::covers(const lyap_zone_t &other) const {
    std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
    throw std::runtime_error("Virtual");
    return false;
  }
  
  bool lyapunov_t::is_covered(const lyap_zone_t &other) const {
    std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
    throw std::runtime_error("Virtual");
    return false;
  }
  
  double lyapunov_t::get_alpha() const {
    std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
    throw std::runtime_error("Virtual");
    return 0.;
  }
  
  fixed_ellipsoidal_zone_copied_t::fixed_ellipsoidal_zone_copied_t(
      const Eigen::VectorXd &x0, const Eigen::MatrixXd &C):
      _x0(x0), _C(C){}
  
  fixed_ellipsoidal_zone_ref_t::fixed_ellipsoidal_zone_ref_t(
      const Eigen::VectorXd &x0, const Eigen::MatrixXd &C):
      _x0(x0), _C(C){}
  
  
}