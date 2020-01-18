//
// Created by philipp on 1/10/20.
//

#ifndef FUNNELS_CPP_AUT_SURVIVAL_HH
#define FUNNELS_CPP_AUT_SURVIVAL_HH

#include <string>

#include "funnels/utils.hh"
#include "aut/proc_ta.hh"

namespace aut_surv{
  /*!
   * Automaton allowing to specify either
   * a minimal survival time
   * a maximally accorded time
   */
  class aut_surv:public ta::process_ta_t{
    
    using clock_ta_t = funnels::clock_ta_t;
  
  public:
    aut_surv(const std::string & proc_name, const clock_ta_t & clock,
        double end_time, bool is_max_time = true);
    
    void set_t_step(double t_step);
    
    std::string declare_proc()const;
  
    std::string declare_clk()const;
  
    std::string declare_event()const;
  
    std::string declare_sync()const;
  
    std::string declare_loc()const;
  
    std::string declare_edge();
  
    template <class T>
    void register_self(T & reg){
      reg._fun_proc.push_back( std::bind(&aut_surv::declare_proc, this) );
      reg._fun_clk.push_back( std::bind(&aut_surv::declare_clk, this) );
      reg._fun_event.push_back( std::bind(&aut_surv::declare_event, this) );
      reg._fun_sync.push_back( std::bind(&aut_surv::declare_sync, this) );
      reg._fun_loc.push_back( std::bind(&aut_surv::declare_loc, this) );
      reg._fun_edge.push_back( std::bind(&aut_surv::declare_edge, this) );
    }
    
  protected:
    const double _end_time;
    const bool _is_max;
    const funnels::process_t &_proc;
    const clock_ta_t &_clk;
    const funnels::location_t &_init_dummy, &_good, &_bad;
    const funnels::label_t &_good_lbl, &_bad_lbl;
    const funnels::event_t & _good2bad_evt, & _bad2good_evt;
    double _t_step=-1; // TODO ugly, has to be set before declaring
  };
  
}


#endif //FUNNELS_CPP_AUT_SURVIVAL_HH
