//
// Created by phili on 29.12.2019.
//

#ifndef FUNNELS_CPP_TA_HH
#define FUNNELS_CPP_TA_HH

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <functional>

namespace ta{
  
  class ta_t{
  
  public:
    ta_t(std::string init_str):_init_str(std::move(init_str)){};
  
    std::string declare_proc(){
      std::string temp_string="";
      for (auto f : _fun_proc){
        temp_string += f() + "\n";
      }
      return temp_string;
    }
  
    std::string declare_clk(){
      std::string temp_string;
      for (auto f : _fun_clk){
        temp_string += f() + "\n";
      }
      return temp_string;
    }
  
    std::string declare_event(){
      std::string temp_string;
      for (auto f : _fun_event){
        temp_string += f() + "\n";
      }
      return temp_string;
    }
  
    std::string declare_sync(){
      std::string temp_string;
      for (auto f : _fun_sync){
        temp_string += f() + "\n";
      }
      return temp_string;
    }
  
    std::string declare_loc(){
      std::string temp_string;
      for (auto f : _fun_loc){
        temp_string += f() + "\n";
      }
      return temp_string;
    }
  
    std::string declare_edge(){
      std::string temp_string;
      for (auto f : _fun_edge){
        temp_string += f() + "\n";
      }
      return temp_string;
    }
  
    std::string declare(){
      std::string temp_string = _init_str + "\n\n";
    
      temp_string += declare_proc() + "\n\n";
      temp_string += declare_clk() + "\n\n";
      temp_string += declare_event() + "\n\n";
      temp_string += declare_sync() + "\n\n";
      temp_string += declare_loc() + "\n\n";
      temp_string += declare_edge() + "\n\n";
    
      return temp_string;
    }

  public:
    std::deque<std::function<std::string()>> _fun_proc;
    std::deque<std::function<std::string()>> _fun_clk;
    std::deque<std::function<std::string()>> _fun_event;
    std::deque<std::function<std::string()>> _fun_sync;
    std::deque<std::function<std::string()>> _fun_loc;
    std::deque<std::function<std::string()>> _fun_edge;
    std::string _init_str;
    
  };
  
}

#endif //FUNNELS_CPP_TA_HH
