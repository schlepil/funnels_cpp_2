//
// Created by philipp on 21.01.20.
//

#ifndef FUNNELS_CPP_AUT_FORMULA_HH
#define FUNNELS_CPP_AUT_FORMULA_HH

#include <string>
#include <vector>

#include "aut/proc_ta.hh"

using namespace funnels;

namespace aut_formula{
  
  class aut_formula{
  public:
    aut_formula(std::string formula);
  
  
    template <class T>
    void register_self(T & reg) {
      return ta::register_self(reg, this);
    }
    
  public:
    const std::string _formula;
  };
  
  class aut_all_times_n{
  public:
    aut_all_times_n(std::string proc_name, std::vector<event_t *> event_vec,
        std::vector<sync_t *> &sync_vec, size_t n_repeat);
  
  
    std::string declare_proc()const;
  
    std::string declare_clk()const;
  
    std::string declare_event()const;
  
    std::string declare_sync()const;
  
    std::string declare_loc()const;
  
    std::string declare_edge()const;
  
    template <class T>
    void register_self(T & reg) {
      return ta::register_self(reg, this);
    }


  public:
    process_t &_proc; // Process associated to the formula
    std::vector<edge_t> _aut_edges;
    std::vector<location_t *> _aut_loc;
    location_t &_init_dummy;
    label_t &_acc_lbl;
    
    std::vector<event_t *> _event_vec;
    size_t _n_repeat;
    
  
  };
  
  
  
  
}

#endif //FUNNELS_CPP_AUT_FORMULA_HH
