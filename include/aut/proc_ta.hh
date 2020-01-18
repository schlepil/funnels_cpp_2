//
// Created by phili on 29.12.2019.
//

#ifndef FUNNELS_CPP_PROC_TA_HH
#define FUNNELS_CPP_PROC_TA_HH

#include <iostream>

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
}

#endif //FUNNELS_CPP_PROC_TA_HH
