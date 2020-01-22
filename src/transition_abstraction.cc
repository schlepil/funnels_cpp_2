//
// Created by philipp on 1/21/20.
//

#include "funnels/transition_abstraction.hh"

namespace trans_abs{
  
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
  
  ////////////////////////////////////////////
  // catch
  
  size_t add_catch_no_abs(std::deque<edge_t> &edge_list, double t_step,
                          const switching_trans_info_t &all_trans){
    return add_catch_no_abs_helper(edge_list, t_step, all_trans,
        all_trans._src, all_trans._tgt, all_trans._evt,
        all_trans._ctrl_clk, all_trans._ctrl_clk);
  }
  size_t add_catch_no_abs(std::deque<edge_t> &edge_list, double t_step,
                          const intersect_trans_info_t &all_trans){
    return add_catch_no_abs_helper(edge_list, t_step, all_trans,
        all_trans._src, all_trans._tgt, all_trans._evt,
        all_trans._ctrl_clk_src, all_trans._ctrl_clk_tgt);
  }
  
  size_t add_catch_med_abs(std::deque<edge_t> &edge_list, double t_step,
                           const switching_trans_info_t &all_trans){
    throw std::runtime_error("todo");
    return 0;
  }
  size_t add_catch_med_abs(std::deque<edge_t> &edge_list, double t_step,
                           const intersect_trans_info_t &all_trans){
    throw std::runtime_error("todo");
    return 0;
  }
  
  size_t add_catch_one_abs(std::deque<edge_t> &edge_list, double t_step,
                           const switching_trans_info_t &all_trans){
    throw std::runtime_error("todo");
    return 0;
  }
  size_t add_catch_one_abs(std::deque<edge_t> &edge_list, double t_step,
                           const intersect_trans_info_t &all_trans){
    throw std::runtime_error("todo");
    return 0;
  }
  
  size_t add_catch_no_tgt_t_abs(std::deque<edge_t> &edge_list, double t_step,
                                const switching_trans_info_t &all_trans){
    return catch_no_tgt_time_helper(edge_list, t_step, all_trans,
        all_trans._src, all_trans._tgt, all_trans._evt, all_trans._ctrl_clk);
  }
  size_t add_catch_no_tgt_t_abs(std::deque<edge_t> &edge_list, double t_step,
                                const intersect_trans_info_t &all_trans){
    return catch_no_tgt_time_helper(edge_list, t_step, all_trans,
        all_trans._src, all_trans._tgt, all_trans._evt, all_trans._ctrl_clk_src);
  }
  
  ////////////////////////////////////////////
  // Time less helper
  size_t catch_no_tgt_time_helper(std::deque<edge_t> &edge_list, double t_step,
      const trans_info_raw_t &all_trans, const location_t &src,
      const location_t &tgt, const event_t& evt, const clock_ta_t &clk){
    
    std::vector<std::pair<double, double>> all_pairs;
    bool did_merge;
    // Merge the new pairs
    for (size_t k=0; k<all_trans.size(); k++){
      did_merge = false;
      for (size_t i=0; i<all_pairs.size(); i++){
        if (((all_pairs[i].first <= all_trans.a_vec[k]) &&
            (all_trans.a_vec[k] <= all_pairs[i].second)) ||
            ((all_pairs[i].first <= all_trans.b_vec[k]) &&
             (all_trans.b_vec[k] <= all_pairs[i].second))){
          all_pairs[i].first = std::min(all_pairs[i].first, all_trans.a_vec[k]);
          all_pairs[i].second = std::max(all_pairs[i].second, all_trans.b_vec[k]);
          did_merge = true;
          break;
        }
      }
      if (!did_merge){
        all_pairs.emplace_back(all_trans.a_vec[k], all_trans.b_vec[k]);
      }
    }
    
    // Merge the intervals
    did_merge = true;
    while(did_merge){
      did_merge = false;
      for (size_t i=0; i<all_pairs.size(); i++){
        for (size_t j=i+1; j<all_pairs.size(); j++) {
          if (((all_pairs[i].first <= all_pairs[j].first) &&
               (all_pairs[j].first <= all_pairs[i].second)) ||
              ((all_pairs[i].first <= all_pairs[j].second) &&
               (all_pairs[j].second <= all_pairs[i].second))){
            all_pairs[i].first = std::min(all_pairs[i].first, all_pairs[j].first);
            all_pairs[i].second = std::max(all_pairs[i].second, all_pairs[j].second);
            did_merge = true;
            break;
          }
        } // j
        if (!did_merge){
          break;
        }
        } // i
      } // while
      
    // all_pairs contains now the largest disjoint intervals
    // Create the corresponding edges
    for (const auto & a_pair : all_pairs) {
      edge_t & this_edge = edge_list.emplace_back(src, tgt, evt);
      // Add the guard
      this_edge.guard().add_expr(clk, ">=", std::ceil(t_step*a_pair.first));
      this_edge.guard().add_expr(clk, "<=", std::floor(t_step*a_pair.second));
    }
    return all_pairs.size();
  }
  
  // No abstraction helper
  size_t add_catch_no_abs_helper(std::deque<edge_t> &edge_list, double t_step,
      const trans_info_raw_t &all_trans, const location_t &src,
      const location_t &tgt, const event_t& evt,
      const clock_ta_t &clk_src, const clock_ta_t &clk_tgt){
    
    for (size_t k=0; k< all_trans.size(); k++){
      edge_t &this_edge = edge_list.emplace_back(src, tgt, evt);
      // "position range" of the system
      this_edge.guard().add_expr(clk_src, ">=", std::ceil(t_step*all_trans.a_vec[k]));
      this_edge.guard().add_expr(clk_src, "<=", std::floor(t_step*all_trans.b_vec[k]));
      // "position" of the flag
      this_edge.guard().add_expr(clk_tgt, "==", std::round(t_step*all_trans.z_vec[k]));
    }
    return all_trans.size();
  }
  
  
  void trans_abstract_t::set_abst_lvl(abst_lvl_t new_lvl){
    _abst_lvl = new_lvl;
  }
  
  size_t trans_abstract_t::operator()(std::deque<edge_t> &edge_list, double t_step,
                            const switching_trans_info_t &all_trans){
    throw std::runtime_error("Virtual");
  }
  size_t trans_abstract_t::operator()(std::deque<edge_t> &edge_list, double t_step,
                            const intersect_trans_info_t &all_trans){
    throw std::runtime_error("Virtual");
  }
  
  size_t switching_trans_abstract_t::operator()(std::deque<edge_t> &edge_list, double t_step,
                    const switching_trans_info_t &all_trans) {
    
    switch (_abst_lvl) {
      case NO_ABS:
        return add_switch_no_abs(edge_list, t_step, all_trans);
      case MED_ABS:
        return add_switch_med_abs(edge_list, t_step, all_trans);
      case ONE_ABS:
        return add_switch_one_abs(edge_list, t_step, all_trans);
      case NO_TGT_T_ABS:
        throw std::runtime_error("This abstraction is not valid for switching");
    }
    throw std::runtime_error("Unknown abstraction");
    return 0;
  }

  size_t intersect_trans_abstract_t::operator()(std::deque<edge_t> &edge_list, double t_step,
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
  
  size_t catch_trans_abstract_t::operator()(std::deque<edge_t> &edge_list, double t_step,
                    const switching_trans_info_t &all_trans){
    switch (_abst_lvl) {
      case NO_ABS:
        return add_catch_no_abs(edge_list, t_step, all_trans);
      case MED_ABS:
        return add_catch_med_abs(edge_list, t_step, all_trans);
      case ONE_ABS:
        return add_catch_one_abs(edge_list, t_step, all_trans);
      case NO_TGT_T_ABS:
        add_catch_no_tgt_t_abs(edge_list, t_step, all_trans);
    }
    throw std::runtime_error("Unknown abstraction");
    return 0;
  }
  
  size_t catch_trans_abstract_t::operator()(std::deque<edge_t> &edge_list, double t_step,
                    const intersect_trans_info_t &all_trans){
    
    switch(_abst_lvl){
      case NO_ABS:
        return add_catch_no_abs(edge_list, t_step, all_trans);
      case MED_ABS:
        return add_catch_med_abs(edge_list, t_step, all_trans);
      case NO_TGT_T_ABS:
        add_catch_no_tgt_t_abs(edge_list, t_step, all_trans);
    }
    throw std::runtime_error("Unknown abstraction");
    return 0;
  }
  
  
  
  
}