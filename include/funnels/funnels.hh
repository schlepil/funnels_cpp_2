//
// Created by philipp on 12/13/19.
//

#ifndef FUNNELS_CPP_FUNNELS_HH
#define FUNNELS_CPP_FUNNELS_HH

#include <vector>
#include <deque>
#include <unordered_map>
#include <memory>
#include <algorithm>

#include "Eigen/Dense"

#include "funnels/transitions.hh"
#include "funnels/funnel_computations.hh"
#include "aut/proc_ta.hh"

#include "funnels/transition_abstraction.hh"
#include "helpers/bounding_box.hh"

namespace funnels {
  
  template<class CVX_HULL>
  class funnel_t : public CVX_HULL {
  public:
    using cvx_hull_t = CVX_HULL;
    using lyap_t = typename cvx_hull_t::lyap_t;
    using traj_t = typename lyap_t::traj_t;
    using dyn_t = typename traj_t::dyn_t;
    
    using fun_t = funnel_t<cvx_hull_t>;
    using fun_ptr_t = std::shared_ptr<fun_t>;
    using fun_family_t = std::vector<std::set<fun_ptr_t>>;
    using fun_family_ptr_t = std::shared_ptr<fun_family_t>;
  
    using matrix_t = typename cvx_hull_t::matrix_t;
    using vector_x_t = typename cvx_hull_t::vector_x_t;
    using vector_u_t = typename cvx_hull_t::vector_u_t;
    using vector_t_t = typename cvx_hull_t::vector_t_t;
    using matrix_ptr_t = typename cvx_hull_t::matrix_ptr_t;
    using vector_x_ptr_t = typename cvx_hull_t::vector_x_ptr_t;
    using vector_u_ptr_t = typename cvx_hull_t::vector_u_ptr_t;
    using vector_t_ptr_t = typename cvx_hull_t::vector_t_ptr_t;
    
    using traj_t::x;
    using traj_t::x0;
    using traj_t::u;
    using traj_t::t;
    using traj_t::x_ptr;
    using traj_t::u_ptr;
    using traj_t::t_ptr;
    using traj_t::set_cyclic;
    using traj_t::is_cyclic;
    using traj_t::dimx;
    using traj_t::dimp;
    using traj_t::dimv;
    using traj_t::dimu;
    using traj_t::set_size;
    using traj_t::size;
    
    cvx_hull_t& cvx_hull(){
      return *dynamic_cast<cvx_hull_t*>(this);
    }
    lyap_t& lyap(){
      return *dynamic_cast<lyap_t *>(this);
    }
    traj_t& traj(){
      return *dynamic_cast<traj_t *>(this);
    }
    dyn_t& dyn(){
      return *dynamic_cast<dyn_t*>(this);
    }
  
    const cvx_hull_t& cvx_hull() const {
      return *dynamic_cast<const cvx_hull_t*>(this);
    }
    const lyap_t& lyap() const {
      return *dynamic_cast<const lyap_t *>(this);
    }
    const traj_t& traj() const {
      return *dynamic_cast<const traj_t *>(this);
    }
    const dyn_t& dyn() const {
      return *dynamic_cast<const dyn_t*>(this);
    }
    
  
    static void set_parent(fun_ptr_t &parent, fun_ptr_t &child){
      // todo : some asserts
      assert(parent->_fun_family_ptr != nullptr);
      size_t p_lvl = parent->get_lvl();
      
      if (parent->fun_family().size()<=p_lvl+1){
        parent->fun_family().emplace_back();
      }
      parent->fun_family()[p_lvl+1].emplace(child);
      child->_fun_family_ptr = parent->_fun_family_ptr;
      child->share_traj(parent);
    }
    
    template<class ...CVXARGS>
    funnel_t(size_t n, const std::string & name, const process_t &proc, CVXARGS &&...cvxargs):
        CVX_HULL(cvxargs...),
        _loc(utils_ext::loc_map.create_and_get(name, proc)){
      set_size(n);
    };
  
    void set_traj(const vector_t_ptr_t &t, const vector_u_ptr_t &u,
        const matrix_ptr_t &x){
      traj_t::set_traj(t,u,x);
      cvx_hull_t::create();
    }
    
    void share_traj(const fun_ptr_t &other){
      traj_t::share_traj(other->traj());
      cvx_hull_t::create();
    }
    
    
    template <class ...CARGS>
    void compute(CARGS &&...cargs){
      cvx_hull_t::compute(cargs...);
    }
    
    void share_family(const fun_ptr_t &other){
      assert(other->_fun_family_ptr != nullptr);
      _fun_family_ptr = other->_fun_family_ptr;
    }
    
    const location_t &loc() const {
      return _loc;
    }
    
    fun_ptr_t make_copy(const std::string &name, size_t n){
      // Specific todo generalise
      // using args = std::tuple<std::shared_ptr<Eigen::MatrixXd>, double,
      // DIST *>;
      typename lyap_t::args largs = lyap_t::get_args();
      
      fun_ptr_t  new_fun = std::make_shared<fun_t>(n, name, _loc.process(),
          *std::get<0>(largs), std::get<1>(largs), *std::get<2>(largs));
      
      return new_fun;
    }
    
    int get_lvl(){
      if (_fun_family_ptr == nullptr){
        return -1;
      }
      int i;
      for(i=0; i<_fun_family_ptr->size(); ++i){
        for(const auto &e : fun_family()[i]){
          if (this == e.get()){
            return i;
          }
        }
      }
      throw std::runtime_error("Not in own family?!");
    }
    
    bool operator==(const fun_t & other)const{
      return _loc == other._loc;
    }
  
    bool operator==(const fun_ptr_t & other)const{
      return _loc == other->_loc;
    }
    
    bool in_family(const fun_ptr_t &other)const{
      if (this->operator==(other)){
        return true;
      }
      for (const auto & fun_set : *_fun_family_ptr){
        if (fun_set.find(other) != fun_set.end()){
          return true;
        }
      }
      return false;
    }
    
    void gen_invariant(double t_step, const clock_ta_t & ctrl_clk){
      _loc.add_expr(expression_t(ctrl_clk, ">=", std::ceil((t())(0)*t_step)) );
      _loc.add_expr(expression_t(ctrl_clk, "<=", std::floor((t())(size()-1)*t_step)) );
    }
    
    void gen_internal_trans(double t_step, const clock_ta_t & ctrl_clk){
      if(is_cyclic()) {
        edge_t &cyc_trans = _edges.emplace_back(_loc, _loc,
                                                utils_ext::event_map["no_action"]);
  
        cyc_trans.guard().add_expr(ctrl_clk, "==",
                                   std::floor(t()(size() - 1) * t_step));
        cyc_trans.update().add_expr(ctrl_clk, "=",
                                    std::ceil(t()(0) * t_step));
      }
    }
    
    void clear_loc(){
      _loc.clear();
    }
    void clear_edges(){
      _edges.clear();
    }
    
    void start_family(fun_ptr_t &self){
      assert(_fun_family_ptr == nullptr);
      _fun_family_ptr = std::make_shared<fun_family_t>();
      fun_family().emplace_back();
      fun_family()[0].emplace(self);
    }
    
    fun_family_t &fun_family(){
      if(_fun_family_ptr == nullptr){
        throw std::runtime_error("No family yet");
      }
      return *_fun_family_ptr;
    }
    
  public:
    // List of edges with this funnel as src
    std::deque<edge_t> _edges;
  
    // Edge abstraction map
    // Abstractions can be adjusted for each funnel pair
    std::shared_ptr<trans_abs::trans_abstract_t> _trans_abs;
    
    // Whether this funnel is a "fake" funnel to handle collisions
    bool _is_colliding=false;
  
  protected:
    
    location_t &_loc;
    fun_family_ptr_t _fun_family_ptr= nullptr;
    
  };
  
}


#endif //FUNNELS_CPP_FUNNELS_HH
