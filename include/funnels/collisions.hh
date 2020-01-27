//
// Created by philipp on 1/15/20.
//

#ifndef FUNNELS_CPP_COLLISIONS_HH
#define FUNNELS_CPP_COLLISIONS_HH

#include "funnels/utils.hh"
#include "funnels/funnels.hh"
#include "funnels/funnel_computations.hh"
namespace collision {
  
  enum collision_abstract_t {
    BASE,
    MED
  };
  
  class avoid_collisions_t {
  public:
    ~avoid_collisions_t() {
      if (_sys_idx != nullptr)
        free(_sys_idx);
      if (_obs_idx != nullptr)
        free(_obs_idx);
    }
    
    template <class FUN_PTR_T>
    void base_abs(const FUN_PTR_T &sys, const FUN_PTR_T &obs,
        std::vector<FUN_PTR_T> &new_sys, std::pair<double, double> t_a_b,
        double t_step, const funnels::clock_ta_t &c_o,
        const funnels::label_t &col_lbl) {
      // Divide into three sub_states
      assert(new_sys.empty());
      
      const std::string sys_name = sys->loc().name();
      FUN_PTR_T l_a = sys->make_copy(sys_name + "_l_a", sys->size());
      FUN_PTR_T coll = sys->make_copy(sys_name + "_col", sys->size());
      FUN_PTR_T g_b = sys->make_copy(sys_name + "_l_b", sys->size());
      
      // They all share the same traj
      l_a->share_traj(sys);
      coll->share_traj(sys);
      g_b->share_traj(sys);
      // coll is committed and has no outgoing
      // transitions to avoid generation unnecessary states
      coll->_is_colliding=true;
      coll->loc().set_loc_typ(COMMITTED);
      coll->loc().add_label(col_lbl);
      
      // Add the invariants
      l_a->loc().add_expr(c_o, "<", std::floor(t_a_b.first*t_step));
      coll->loc().add_expr(c_o, ">", std::floor(t_a_b.first*t_step));
      coll->loc().add_expr(c_o, "<", std::ceil(t_a_b.second*t_step));
      g_b->loc().add_expr(c_o, ">", std::ceil(t_a_b.second*t_step));
      
      // Cyc
      l_a->set_cyclic(sys->is_cyclic());
      coll->set_cyclic(sys->is_cyclic());
      g_b->set_cyclic(sys->is_cyclic());
      
      // Add the new ones to the family without removing old ones
      // Set the ptr
      l_a->share_family(sys);
      coll->share_family(sys);
      g_b->share_family(sys);
      
      sys->fun_family()[sys->get_lvl()].emplace(l_a);
      sys->fun_family()[sys->get_lvl()].emplace(coll);
      sys->fun_family()[sys->get_lvl()].emplace(g_b);
      
      new_sys.push_back(l_a);
      new_sys.push_back(coll);
      new_sys.push_back(g_b);
      return;
    }
  
    template <class FUN_PTR_T>
    void med_abs(const FUN_PTR_T &sys, const FUN_PTR_T &obs,
        std::vector<FUN_PTR_T> &new_sys,
        const std::vector<std::pair<bool, std::pair<double, double>>> &col_struct,
        double t_step, const funnels::clock_ta_t &c_o,
        const funnels::label_t &col_lbl) {
      // Divides into 1 + 2*col_struct_size collision free sub-funnels
      // It is guaranteed that there is at least one collision
      
      assert(new_sys.empty());
      
      const std::string sys_name = sys->loc().name();
      FUN_PTR_T l_a = sys->make_copy(sys_name + "_l_a", sys->size());
      FUN_PTR_T coll_0 = sys->make_copy(sys_name + "_coll_0", sys->size());
      FUN_PTR_T g_b = sys->make_copy(sys_name + "_g_b", sys->size());
      
      bool l_a_feas, g_b_feas;
  
      // Check if the invariants are feasibly
      // for l_a and g_b
      l_a_feas = col_struct[0].second.first > (*(obs->t_ptr()))(0);
      g_b_feas = col_struct.back().second.second < (*(obs->t_ptr()))(obs->size()-1);
      
      // They all share the same traj
      if (l_a_feas)
        l_a->share_traj(sys);
      coll_0->share_traj(sys);
      if (g_b_feas)
        g_b->share_traj(sys);
      // coll_0 is committed and has no outgoing
      // transitions to avoid generation unnecessary states
      coll_0->_is_colliding=true;
      coll_0->loc().set_loc_typ(COMMITTED);
      coll_0->loc().add_label(col_lbl);
      
      // Add the invariants
      if (l_a_feas)
        l_a->loc().add_expr(c_o, "<", std::floor(col_struct[0].second.first*t_step));
      coll_0->loc().add_expr(c_o, ">=", std::floor(col_struct[0].second.first * t_step));
      coll_0->loc().add_expr(c_o, "<=", std::ceil(col_struct[0].second.second * t_step));
      if (g_b_feas)
        g_b->loc().add_expr(c_o, ">", std::ceil(col_struct.back().second.second*t_step));
    
      // Cyc
      if (l_a_feas)
        l_a->set_cyclic(sys->is_cyclic());
      coll_0->set_cyclic(sys->is_cyclic());
      if (g_b_feas)
        g_b->set_cyclic(sys->is_cyclic());
    
      if (l_a_feas)
        new_sys.push_back(l_a);
      new_sys.push_back(coll_0);
      
      
      // Add all collisions and intermediate except the first
      for(size_t k=1; k<col_struct.size(); k++){
        FUN_PTR_T inter_k = sys->make_copy(sys_name + "_i_" + std::to_string(k),
            sys->size());
        FUN_PTR_T coll_k = sys->make_copy(sys_name + "_coll_" +
            std::to_string(k), sys->size());
  
        inter_k->share_traj(sys);
        coll_k->share_traj(sys);
  
        inter_k->loc().add_expr(c_o, ">", std::ceil(col_struct[k-1].second.second * t_step));
        inter_k->loc().add_expr(c_o, "<", std::floor(col_struct[k].second.first * t_step));
        
        coll_k->loc().add_expr(c_o, ">=", std::floor(col_struct[k].second.first * t_step));
        coll_k->loc().add_expr(c_o, "<=", std::ceil(col_struct[k].second.second * t_step));
  
        coll_k->_is_colliding=true;
        coll_k->loc().set_loc_typ(COMMITTED);
        coll_k->loc().add_label(col_lbl);
  
        inter_k->set_cyclic(sys->is_cyclic());
        coll_k->set_cyclic(sys->is_cyclic());
  
        new_sys.push_back(inter_k);
        new_sys.push_back(coll_k);
      }
      // After all collisions
      if (g_b_feas)
        new_sys.push_back(g_b);
      
      // Set family
      for (auto & a_fun: new_sys){
        // Add the new ones to the family without removing old ones
        // Set the ptr
        a_fun->share_family(sys);
        sys->fun_family()[sys->get_lvl()].emplace(a_fun);
      }
      // Done
    }
    
    
    template<class FUN_PTR_T>
    bool operator()(const FUN_PTR_T &sys, const FUN_PTR_T &obs,
                    std::vector<FUN_PTR_T> &new_sys, double t_step,
                    const funnels::clock_ta_t &c_t,
                    const funnels::clock_ta_t &c_o,
                    const funnels::label_t &col_lbl) {
      new_sys.clear();
      
      auto col_struct = compute_outer_col_times(*sys, *obs);
      // Optimization, variable decomposition
      // todo include this better into the structure
      // todo make more robust
      if (col_struct.first && _abs == BASE &&
          col_struct.second.first == (double) obs->t().template topLeftCorner<1,1>()(0,0) &&
          col_struct.second.second == (double) obs->t().template bottomLeftCorner<1,1>()(0,0)){
        // The at each time point for the ctrl clock there exists a obstacle
        // clock value that causes collision
        // Cut the obstacle into 4 (?!?)
        _abs = MED;
        _obs_idx = new std::vector<size_t>(5);
        for (size_t k=0; k<5; k++) {
          (*_obs_idx)[k] = (size_t) (obs->t().size()*k/4);
        }
      }
      if(col_struct.first){
        new_sys.clear();
        switch(_abs){
          case BASE:
            base_abs(sys, obs, new_sys, col_struct.second, t_step, c_o,
                col_lbl);
            return !new_sys.empty();
          case MED:
            med_abs(sys, obs, new_sys, compute_outer_col_times(*sys, *obs,
                *_obs_idx), t_step, c_o, col_lbl);
            return !new_sys.empty();
          default:
            throw std::runtime_error("no viable abstraction");
            return false;
        }
      }
    }
    
    collision_abstract_t _abs = BASE;
    std::vector<size_t> *_sys_idx = nullptr;
    std::vector<size_t> *_obs_idx = nullptr;
  };
  
}
#endif //FUNNELS_CPP_COLLISIONS_HH
