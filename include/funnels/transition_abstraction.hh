//
// Created by philipp on 02.01.20.
//

#ifndef FUNNELS_CPP_TRANSITION_ABSTRACTION_HH
#define FUNNELS_CPP_TRANSITION_ABSTRACTION_HH

#include <Eigen/Dense>
#include <deque>
#include <stdexcept>

#include "funnels/transitions.hh"

namespace trans_abs{
  
  enum abst_lvl_t{
    NO_ABS,
    MED_ABS,
    ONE_ABS
  };
  
  ////////////////////////////////////////////
  // Switch
  
  size_t add_switch_no_abs(std::deque<edge_t> &edge_list, double t_step,
      const switching_trans_info_t &all_trans){
    // Add all transitions without further treatment
    
    for (size_t k=0; k<all_trans.size(); k++){
      edge_list.push_back(
          switching_transition(all_trans._src, all_trans._tgt,
                               all_trans._evt, all_trans._ctrl_clk,
                               all_trans._lcl_clk,
                               std::ceil(t_step*all_trans.a_vec[k]),
                               std::floor(t_step*all_trans.b_vec[k]),
                               std::round(t_step*all_trans.z_vec[k]),
                               all_trans.is_diag));
    }
    
    return all_trans.size();
  }
  
  size_t add_switch_med_abs(std::deque<edge_t> &edge_list, double t_step,
      const switching_trans_info_t &all_trans){
    // Replace transition block with approximations
    size_t n_trans_approx=0;
    
    throw std::runtime_error("TODO"); // todo
    
    return n_trans_approx;
  }
  
  size_t add_switch_one_abs(std::deque<edge_t> &edge_list, double t_step,
                           const switching_trans_info_t &all_trans){
    // Add all transitions without further treatment
    if (all_trans.size()==0){
      return 0;
    }
    edge_list.push_back(
        switching_transition(all_trans._src, all_trans._tgt,
                             all_trans._evt, all_trans._ctrl_clk,
                             all_trans._lcl_clk,
                             std::ceil(t_step*all_trans.a_vec[0]),
                             std::floor(t_step*all_trans.b_vec[0]),
                             std::round(t_step*all_trans.z_vec[0]),
                             all_trans.is_diag));
    
    return 1;
  }
  
  ////////////////////////////////////////////
  // Intersect
  
  size_t add_intersect_no_abs(std::deque<edge_t> &edge_list, double t_step,
                           const intersect_trans_info_t &all_trans){
    // Add all transitions without further treatment
    
    for (size_t k=0; k<all_trans.size(); k++){
      edge_list.push_back(
          intersecting_transition(all_trans._src, all_trans._tgt,
                               all_trans._evt, all_trans._ctrl_clk_src,
                               all_trans._ctrl_clk_tgt,
                               std::ceil(t_step*all_trans.a_vec[k]),
                               std::floor(t_step*all_trans.b_vec[k]),
                               std::round(t_step*all_trans.z_vec[k])));
    }
    
    return all_trans.size();
  }
  
  size_t add_intersect_med_abs(std::deque<edge_t> &edge_list, double t_step,
                            const intersect_trans_info_t &all_trans){
    // Replace transition block with approximations
    size_t n_trans_approx=0;
    
    throw std::runtime_error("TODO"); // todo
    
    return n_trans_approx;
  }
  
  class trans_abstract_t{
  public:
    void set_abst_lvl(abst_lvl_t new_lvl){
      _abst_lvl = new_lvl;
    }
    
    virtual size_t operator()(std::deque<edge_t> &edge_list, double t_step,
                              const switching_trans_info_t &all_trans){
      throw std::runtime_error("Virtual");
    }
    virtual size_t operator()(std::deque<edge_t> &edge_list, double t_step,
                              const intersect_trans_info_t &all_trans){
      throw std::runtime_error("Virtual");
    }

  protected:
    abst_lvl_t _abst_lvl=NO_ABS;
  };
  
  
  class switching_trans_abstract_t: public trans_abstract_t{
    
    size_t operator()(std::deque<edge_t> &edge_list, double t_step,
        const switching_trans_info_t &all_trans) {
  
      switch (_abst_lvl) {
        case NO_ABS:
          return add_switch_no_abs(edge_list, t_step, all_trans);
        case MED_ABS:
          return add_switch_med_abs(edge_list, t_step, all_trans);
        case ONE_ABS:
          return add_switch_one_abs(edge_list, t_step, all_trans);
      }
      throw std::runtime_error("Unknown abstraction");
      return 0;
    }
  };
  
  class intersect_trans_abstract_t: public trans_abstract_t{
    
    size_t operator()(std::deque<edge_t> &edge_list, double t_step,
                      const intersect_trans_info_t &all_trans){
      
      switch(_abst_lvl){
        case NO_ABS:
          return add_intersect_no_abs(edge_list, t_step, all_trans);
        case MED_ABS:
          return add_intersect_med_abs(edge_list, t_step, all_trans);
      }
      throw std::runtime_error("Unknown abstraction");
      return 0;
    }
  };
  
}

#endif //FUNNELS_CPP_TRANSITION_ABSTRACTION_HH
