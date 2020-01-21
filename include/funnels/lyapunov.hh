//
// Created by philipp on 22.12.19.
//

#ifndef FUNNELS_CPP_LYAPUNOV_HH
#define FUNNELS_CPP_LYAPUNOV_HH

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <cmath>
#include <memory>

namespace lyapunov{
  
  
  template <class DERIVED>
  double projected_max_radius(const Eigen::MatrixBase<DERIVED> & C0,
                                     const Eigen::MatrixBase<DERIVED> & C1){
    // todo optimize for usage of triangular shape
    auto Cinv = C0.inverse();
    auto P1_prime = (Cinv.transpose()*(C1.transpose()*C1)*Cinv);
    double max_rad = 1./P1_prime.eigenvalues().real().array().minCoeff();
//    Eigen::MatrixXd Cinv = C0.inverse();
//    Eigen::MatrixXd P1_prime = (Cinv.transpose()*(C1.transpose()*C1)*Cinv);
//    double max_rad = 1./P1_prime.eigenvalues().real().array().minCoeff();
    return std::sqrt(max_rad);
  }
  
  struct lyap_zone_t{};
  
  class lyapunov_t{
  public:
    virtual bool intersect(const lyap_zone_t & other) const;
    virtual bool covers(const lyap_zone_t & other) const;
    virtual bool is_covered(const lyap_zone_t & other) const;
    virtual double get_alpha() const;
  };
  
  struct fixed_ellipsoidal_zone_t{
    virtual const Eigen::VectorXd & x0() const {
      std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
      throw std::runtime_error("Virtual");
      return Eigen::Vector2d::Zero();
    };
    virtual const Eigen::MatrixXd & C() const {
      std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
      throw std::runtime_error("Virtual");
      return Eigen::Matrix2d::Zero();
    };
  };
  
  struct fixed_ellipsoidal_zone_copied_t: public fixed_ellipsoidal_zone_t{
  public:
    fixed_ellipsoidal_zone_copied_t( const Eigen::VectorXd & x0,
        const Eigen::MatrixXd & C);
    const Eigen::VectorXd & x0() const {
      return _x0;
    }
    const Eigen::MatrixXd & C() const {
      return _C;
    }
    // (x-x0)^T.P.(x-x0) <= 1
    // ||C.(x-x0)||_2^2 <= 1
    Eigen::VectorXd _x0;
    Eigen::MatrixXd _C;
  };
  
  struct fixed_ellipsoidal_zone_ref_t: public fixed_ellipsoidal_zone_t{
  public:
    fixed_ellipsoidal_zone_ref_t( const Eigen::VectorXd & x0,
                                     const Eigen::MatrixXd & C);
    const Eigen::VectorXd & x0() const {
      return _x0;
    }
    const Eigen::MatrixXd & C() const {
      return _C;
    }
    // (x-x0)^T.P.(x-x0) <= 1
    // ||C.(x-x0)||_2^2 <= 1
    const Eigen::VectorXd &_x0;
    const Eigen::MatrixXd &_C;
  };
  
  // todo
//  // Does zone0 cover zone1
//  template<class DIST>
//  bool covers_helper(const fixed_ellipsoidal_zone_t &zone0,
//                     const fixed_ellipsoidal_zone_t &zone1,
//                     DIST &dist){
//
//
//    // First compute the projected distance
//    auto dy = zone0.C()*(dist.cp_vv(zone1.x0(), zone0.x0()));
//    double dy_norm = dy.norm(); //l2 norm
//    // center out of bounds
//    if (dy_norm>=1.){
//      return false;
//    }
//
//    // Compute radius
//    return dy_norm+projected_max_radius(zone0.C(), zone1.C())<=1.;
//  }
  
  template<class TRAJ, class DIST>
  class fixed_ellipsoidal_lyap_t: public TRAJ{
  public:
    using dist_t = DIST;
    using traj_t = TRAJ;
    using dyn_t = typename traj_t::dyn_t;
  
    using matrix_t = typename traj_t::matrix_t;
    using vector_x_t = typename traj_t::vector_x_t;
    using vector_u_t = typename traj_t::vector_u_t;
    using vector_t_t = typename traj_t::vector_t_t;
    using matrix_ptr_t = typename traj_t::matrix_ptr_t;
    using vector_x_ptr_t = typename traj_t::vector_x_ptr_t;
    using vector_u_ptr_t = typename traj_t::vector_u_ptr_t;
    using vector_t_ptr_t = typename traj_t::vector_t_ptr_t;
    
    using traj_t::dimx;
    using traj_t::dimp;
    using traj_t::dimv;
    using traj_t::dimu;
    
    using s_mat_t = Eigen::Matrix<double,dimx,dimx>;
    using s_vec_t = Eigen::Matrix<double,dimx,1>;
  
    using args = std::tuple<std::shared_ptr<s_mat_t>, double, DIST*>;
  
    template <class DERIVED>
    fixed_ellipsoidal_lyap_t(const Eigen::MatrixBase<DERIVED> &P,
        double gamma, DIST &dist):
        _gamma(gamma), _dist(dist){
      set_P(P);
    }
  
    template <class ...CARGS>
    void compute(CARGS &&...cargs){
      // Just forwarding
      traj_t::compute(cargs...);
    }
  
    template <class DERIVED>
    void set_P(const Eigen::MatrixBase<DERIVED>& P) {
      assert(P.rows() == dimx);
      assert(P.rows() == P.cols());
      
      // Compute cholesky
      Eigen::LLT<Eigen::Matrix<double, dimx, dimx>> llt_pre_comp;
      llt_pre_comp.compute(P);
      _C = llt_pre_comp.matrixU();
      
      // Compute the bounding box corner
      std::cout << P << std::endl;
      std::cout << _C << std::endl;
      for(size_t i=0; i<dimx; i++){
        _box_corner(i) = 1./_C.row(i).norm();
      }
      std::cout << _box_corner << std::endl;
    }
    s_mat_t get_P(){
      return _C.transpose()*_C;
    }
    
    const s_mat_t &C() const {
      return _C;
    }
    double gamma() const {
      return _gamma;
    }
    s_vec_t min_corner(){
      return -_box_corner;
    }
    const s_vec_t &max_corner(){
      return _box_corner;
    }
// todo
//    // Conservative intersect
//    // If true, the zone may intersect with this
//    // If false, the zone does definitively not intersect
//    template<class DERIVED>
//    bool intersect(const Eigen::MatrixBase<DERIVED> &x0,
//        const fixed_ellipsoidal_zone_t &zone) const {
//      // First compute the projected distance
//      auto dy = _C*(_dist.cp_vv(zone.x0(), x0));
//      double dy_norm = dy.norm(); //l2 norm
//
//      if (dy_norm <= 1.){
//        return true;
//      }
//
//      return dy_norm>=1.+projected_max_radius(_C, zone.C());
//    }
//
//    // Conservative cover
//    // If true, zone is definitively covered by this
//    // If false, it may not be covered
//    template<class DERIVED>
//    bool covers(const Eigen::MatrixBase<DERIVED> &x0,
//        const fixed_ellipsoidal_zone_t &zone) const {
//      fixed_ellipsoidal_zone_ref_t this_zone(x0, _C);
//      return covers_helper(this_zone, zone, _dist);
//    }
//
//    // Conservative cover
//    // If true, this is definitively covered by zone
//    // If false, it may not be covered
//    template<class DERIVED>
//    bool is_covered(const Eigen::MatrixBase<DERIVED> &x0,
//        const fixed_ellipsoidal_zone_t &zone) const {
//      fixed_ellipsoidal_zone_ref_t this_zone(x0, _C);
//      return covers_helper(zone, this_zone, _dist);
//    }
    
    double get_alpha(){
      return (double) 1./((_C.transpose()*_C)(0,0));
    }
    
    double get_gamma()const{
      return _gamma;
    }
    void set_gamma(double gamma){
      _gamma = gamma;
    }
    
    dist_t & dist() const {
      return _dist;
    }
    
    args get_args()const{
//      Eigen::MatrixXd P_scaled = _C.transpose()*_C;
      std::shared_ptr<s_mat_t> P = std::make_shared<s_mat_t>();
      *P = _C.transpose()*_C;
      return std::make_tuple(P, _gamma, &_dist);
    }
    
  protected:
    s_mat_t _C; // todo: Optimize to use triangular shape
    s_vec_t _box_corner;
    double _gamma;
    DIST &_dist;
  };
  
}

#endif //FUNNELS_CPP_LYAPUNOV_HH
