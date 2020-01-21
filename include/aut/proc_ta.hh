//
// Created by phili on 29.12.2019.
//

#ifndef FUNNELS_CPP_PROC_TA_HH
#define FUNNELS_CPP_PROC_TA_HH

#include <iostream>

#include "funnels/transitions.hh"

namespace ta{
  class process_ta_t{
  public:
    virtual std::string declare_proc(){
      std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
      throw std::runtime_error("virtual");
      return "";
    };
    virtual std::string declare_clk(){
      std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
      throw std::runtime_error("virtual");
      return "";
    };
    virtual std::string declare_event(){
      std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
      throw std::runtime_error("virtual");
      return "";
    };
    virtual std::string declare_sync(){
      std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
      throw std::runtime_error("virtual");
      return "";
    };
    virtual std::string declare_loc(){
      std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
      throw std::runtime_error("virtual");
      return "";
    };
    virtual std::string declare_edge(){
      std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
      throw std::runtime_error("virtual");
      return "";
    };
  };
  
  template <class T, class U>
  static void register_self(T &reg, U *u){
    reg._fun_proc.push_back( std::bind(&U::declare_proc, u) );
    reg._fun_clk.push_back( std::bind(&U::declare_clk, u) );
    reg._fun_event.push_back( std::bind(&U::declare_event, u) );
    reg._fun_sync.push_back( std::bind(&U::declare_sync, u) );
    reg._fun_loc.push_back( std::bind(&U::declare_loc, u) );
    reg._fun_edge.push_back( std::bind(&U::declare_edge, u) );
  }
}

#endif //FUNNELS_CPP_PROC_TA_HH
