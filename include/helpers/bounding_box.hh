//
// Created by philipp on 1/16/20.
//

#ifndef FUNNELS_CPP_BOUNDING_BOX_HH
#define FUNNELS_CPP_BOUNDING_BOX_HH

#include <Eigen/Core>

namespace helper_geom {
  
  template <class LYAP>
  class aligned_bounding_box_t:public LYAP {
  public:
    
    using lyap_t = LYAP;
    using traj_t = typename lyap_t::traj_t;
    using dyn_t = typename lyap_t::dyn_t;
  
    using matrix_t = typename lyap_t::matrix_t;
    using vector_x_t = typename lyap_t::vector_x_t;
    using vector_u_t = typename lyap_t::vector_u_t;
    using vector_t_t = typename lyap_t::vector_t_t;
    using matrix_ptr_t = typename lyap_t::matrix_ptr_t;
    using vector_x_ptr_t = typename lyap_t::vector_x_ptr_t;
    using vector_u_ptr_t = typename lyap_t::vector_u_ptr_t;
    using vector_t_ptr_t = typename lyap_t::vector_t_ptr_t;
  
    using traj_t::dimx;
    using traj_t::dimp;
    using traj_t::dimv;
    using traj_t::dimu;
    using traj_t::x;
    
    // todo
    template<class ...LARGS>
    aligned_bounding_box_t(LARGS &&...largs):
    LYAP(largs...){}
  
    template <class ...CARGS>
    void compute(CARGS &&...cargs){
      lyap_t::compute(cargs...);
      create();
    }
    
    void create() {
      // First two are inner
      _corners.col(0) = traj_t::min_corner();
      _corners.col(1) = traj_t::max_corner();
      // The others are outer points
      _corners.col(2) = _corners.col(0) + lyap_t::min_corner();
      _corners.col(3) = _corners.col(1) + lyap_t::max_corner();
    }
    
    template<class DERIVED>
    const Eigen::MatrixBase<DERIVED> &inner_min_corner() const {
      return _corners.col(0);
    }
    template<class DERIVED>
    const Eigen::MatrixBase<DERIVED> &inner_max_corner() const {
      return _corners.col(1);
    }
    template<class DERIVED>
    const Eigen::MatrixBase<DERIVED> &outer_min_corner() const {
      return _corners.col(2);
    }
    template<class DERIVED>
    const Eigen::MatrixBase<DERIVED> &outer_max_corner() const {
      return _corners.col(3);
    }
  
    // Intersection of the outer BB
    bool intersect_out_out(const aligned_bounding_box_t &other) const{
      return (_corners.col(2).array() < other._corners.col(3).array()).all() &&
             (other._corners.col(2).array() < _corners.col(3).array()).all();
    }
    // Intersection of the outer BB of this and inner BB of other
    bool intersect_out_in(const aligned_bounding_box_t &other)const{
      return (_corners.col(2).array() < other._corners.col(1).array()).all() &&
             (other._corners.col(0).array() < _corners.col(3).array()).all();
    }
    
    // Intersection of the inner BB of this and outer BB of other
    bool intersect_in_out(const aligned_bounding_box_t &other)const{
      return (_corners.col(0).array() < other._corners.col(3).array()).all() &&
             (other._corners.col(2).array() < _corners.col(1).array()).all();
    }
    
    // Intersection of the inner BB
    bool intersect_in_in(const aligned_bounding_box_t &other)const{
      return (_corners.col(0).array() < other._corners.col(1).array()).all() &&
             (other._corners.col(0).array() < _corners.col(1).array()).all();
    }
  
  protected:
    Eigen::Matrix<double, dimx, 4> _corners; // First two cols inner points
  };

} // namespace
#endif //FUNNELS_CPP_BOUNDING_BOX_HH
