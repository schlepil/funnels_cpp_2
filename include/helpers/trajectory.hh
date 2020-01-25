//
// Created by philipp on 17.01.20.
//

#ifndef FUNNELS_CPP_TRAJECTORY_HH
#define FUNNELS_CPP_TRAJECTORY_HH

#include <Eigen/Core>
#include <memory>

namespace trajectories{
  
  template <class DYN>
  class timed_way_points_t: public DYN{
  public:
    
    using dyn_t = DYN;
    
//    using matrix_t = Eigen::MatrixXd;
//    using vector_t = Eigen::VectorXd;
//    using matrix_ptr_t = std::shared_ptr<matrix_t>;
//    using vector_ptr_t = std::shared_ptr<vector_t>;
    using matrix_t = typename dyn_t::matrix_t;
    using vector_x_t = typename dyn_t::vector_x_t;
    using vector_u_t = typename dyn_t::vector_u_t;
    using vector_t_t = typename dyn_t::vector_t_t;
    using matrix_ptr_t = typename dyn_t::matrix_ptr_t;
    using vector_x_ptr_t = typename dyn_t::vector_x_ptr_t;
    using vector_u_ptr_t = typename dyn_t::vector_u_ptr_t;
    using vector_t_ptr_t = typename dyn_t::vector_t_ptr_t;
    
    using dyn_t::dimx;
    using dyn_t::dimv;
    using dyn_t::dimu;

  
    matrix_ptr_t x_ptr()const{
      return _x;
    }
    vector_u_ptr_t u_ptr()const{
      return _u;
    }
    vector_t_ptr_t t_ptr()const{
      return _t;
    }
  
    matrix_t &x(){
      return *_x;
    }
    vector_x_t x0(){
      return _x->template leftCols<1>();
    }
    vector_u_t &u(){
      return *_u;
    }
    vector_t_t &t(){
      return *_t;
    }
    
    vector_x_t min_corner() const {
      return vector_x_t(_x->rowwise().minCoeff());
    }
    vector_x_t max_corner() const {
      return vector_x_t(_x->rowwise().maxCoeff());
    }
    
    void set_cyclic(bool is_cyclic){
      _is_cyclic=is_cyclic;
    }
    bool is_cyclic(){
      return _is_cyclic;
    }
    void set_size(size_t n){
      // Attention: invalidates x,t
      _n=n;
      _x = std::make_shared<matrix_t>(dimx, n);
      _t = std::make_shared<vector_t_t>(n);
    }
    size_t size(){
      return _n;
    }
    
    template <class T>
    void share_traj(const T &other){
      _x = other.x_ptr();
      _u = other.u_ptr();
      _t = other.t_ptr();
      _is_cyclic = other._is_cyclic;
    }
  
    void set_x(const matrix_ptr_t &x_ptr){
      assert((*x_ptr).rows()==dimx);
      assert((*x_ptr).cols()==size());
      _x =x_ptr;
    }
    void set_u(const vector_u_ptr_t &u_ptr){
      assert((*u_ptr).size()==dimu);
      _u =u_ptr;
    }
    void set_t(const vector_t_ptr_t &t_ptr){
      assert((*t_ptr).size()==size());
      _t =t_ptr;
    }
    void set_traj(const vector_t_ptr_t &t, const vector_u_ptr_t &u,
                  const matrix_ptr_t &x){
      assert(t->size() == x->cols());
      set_size(t->size());
      set_t(t);
      set_u(u);
      set_x(x);
    }
    
    bool check_cyclic(){
      // If start and end-point are close enough -> cyclic
      // Treshhold is relative to largest elem
      return (_x->template leftCols<1>() -
          _x->template rightCols<1>()).norm() < 1e-8*_x->array().abs().maxCoeff();
    }
    
    template<class DERIVED1, class DERIVED2>
    void compute(const Eigen::MatrixBase<DERIVED1> &x0, double t0, double t1,
                 const Eigen::MatrixBase<DERIVED2> &u_in){
      // Invalidates the pointers
      _x = matrix_ptr_t( new matrix_t(dimx, _n));
      x().col(0) = x0;
      _u = vector_u_ptr_t( new vector_u_t());
      u() = u_in;
      _t = vector_t_ptr_t( new vector_t_t(size()));
      t() = Eigen::VectorXd::LinSpaced(size(), t0, t1);
    
      // Compute the trajectory with the underlying dynamics
      DYN::compute(x(), u(), t());
    }
  
    
    // Member Vars
    size_t _n;
  
    matrix_ptr_t _x;
    vector_u_ptr_t _u;
    vector_t_ptr_t _t;
    bool _is_cyclic=false; // Note: WARNING, there is no verification if
    // this is actually feasible
    
  };

}

#endif //FUNNELS_CPP_TRAJECTORY_HH
