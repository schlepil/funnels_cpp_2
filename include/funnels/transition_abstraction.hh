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
    ONE_ABS,
    NO_TGT_T_ABS
  };
  
  ////////////////////////////////////////////
  // Switch
  
  size_t add_switch_no_abs(std::deque<edge_t> &edge_list, double t_step,
      const switching_trans_info_t &all_trans);
  
  size_t add_switch_med_abs(std::deque<edge_t> &edge_list, double t_step,
      const switching_trans_info_t &all_trans);
  
  size_t add_switch_one_abs(std::deque<edge_t> &edge_list, double t_step,
                           const switching_trans_info_t &all_trans);
  
  ////////////////////////////////////////////
  // Intersect
  
  size_t add_intersect_no_abs(std::deque<edge_t> &edge_list, double t_step,
                           const intersect_trans_info_t &all_trans);
  
  size_t add_intersect_med_abs(std::deque<edge_t> &edge_list, double t_step,
                            const intersect_trans_info_t &all_trans);
  
  ////////////////////////////////////////////
  // Catch
  size_t add_catch_no_tgt_t_abs(std::deque<edge_t> &edge_list, double t_step,
                                const switching_trans_info_t &all_trans);
  
  size_t add_catch_no_tgt_t_abs(std::deque<edge_t> &edge_list, double t_step,
                                const intersect_trans_info_t &all_trans);
  
  ////////////////////////////////////////////
  // Time less helper
  size_t catch_no_tgt_time_helper(std::deque<edge_t> &edge_list, double t_step,
      const trans_info_raw_t &all_trans, const location_t &src, const location_t &tgt,
      const event_t& evt);


  ////////////////////////////////////////////
  class trans_abstract_t{
  public:
    void set_abst_lvl(abst_lvl_t new_lvl);
    
    virtual size_t operator()(std::deque<edge_t> &edge_list, double t_step,
                              const switching_trans_info_t &all_trans);
    
    virtual size_t operator()(std::deque<edge_t> &edge_list, double t_step,
                              const intersect_trans_info_t &all_trans);

  protected:
    abst_lvl_t _abst_lvl=NO_ABS;
  };
  
  
  class switching_trans_abstract_t: public trans_abstract_t{
    
    size_t operator()(std::deque<edge_t> &edge_list, double t_step,
        const switching_trans_info_t &all_trans);
  };
  
  class intersect_trans_abstract_t: public trans_abstract_t{
    
    size_t operator()(std::deque<edge_t> &edge_list, double t_step,
                      const intersect_trans_info_t &all_trans);
  };
  
  class   catch_trans_abstract_t: public trans_abstract_t{
  public:
    
    // Has to deal with both structures: switching and intersecting
    size_t operator()(std::deque<edge_t> &edge_list, double t_step,
                      const switching_trans_info_t &all_trans);
  
    size_t operator()(std::deque<edge_t> &edge_list, double t_step,
                      const intersect_trans_info_t &all_trans);
        
  };
  
} // trans_abs

#endif //FUNNELS_CPP_TRANSITION_ABSTRACTION_HH
