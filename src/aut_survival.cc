//
// Created by phili on 11.01.2020.
//

#include <funnels/transitions.hh>
#include "aut/aut_survival.hh"

namespace aut_surv {
  
  aut_surv::aut_surv(const std::string &proc_name,
                     const funnels::clock_ta_t &clock, double end_time,
                     bool is_max_time) :
      _end_time(end_time),
      _is_max(is_max_time),
      _proc(utils_ext::process_map.create_and_get(proc_name)),
      _clk(clock),
      _init_dummy(utils_ext::loc_map.create_and_get(proc_name + "_init_dummy",
          _proc, funnels::location_type_t::COMMITTED)),
      _good(utils_ext::loc_map.create_and_get(proc_name + "_good", _proc)),
      _bad(utils_ext::loc_map.create_and_get(proc_name + "_bad", _proc)),
      _good_lbl(utils_ext::label_map.create_and_get(proc_name + "_good_lbl")),
      _bad_lbl(utils_ext::label_map.create_and_get(proc_name + "_bad_lbl")),
      _good2bad_evt(
          utils_ext::event_map.create_and_get(proc_name + "_g2b_evt")),
      _bad2good_evt(
          utils_ext::event_map.create_and_get(proc_name + "_b2g_evt")) {
    _init_dummy.set_initial(true);
    _good.add_label(_good_lbl);
    _bad.add_label(_bad_lbl);
  }
  
  void aut_surv::set_t_step(double t_step) {
    _t_step = t_step;
  }
  
  std::string aut_surv::declare_proc() const {
    return _proc.declare() + "\n";
  }
  
  std::string aut_surv::declare_clk() const {
    return _clk.declare() + "\n";
  }
  
  std::string aut_surv::declare_event() const {
    std::string tmp_string = "";
    tmp_string += _good2bad_evt.declare() + "\n";
    tmp_string += _bad2good_evt.declare() + "\n";
    return tmp_string + "\n";
  }
  
  std::string aut_surv::declare_sync() const {
    return "";
  }
  
  std::string aut_surv::declare_loc() const {
    return _init_dummy.declare() + "\n" + _good.declare() + "\n" + _bad.declare() +
    "\n";
  }
  
  std::string aut_surv::declare_edge() {
    assert(_t_step>0.);
    if (_is_max){
      // Init
      edge_t tmp_edge_init = funnels::edge_t(_init_dummy, _good,
          utils_ext::event_map["init"]);
      tmp_edge_init.update().add_expr(_clk, "=", 0);
      // Transitions from good to bad once time exceeds max_time
      edge_t tmp_edge = funnels::edge_t(_good, _bad, _good2bad_evt);
      tmp_edge.guard().add_expr(_clk, ">=", std::ceil(_t_step*_end_time));
      return tmp_edge_init.declare() + "\n" + tmp_edge.declare() + "\n";
    }else{
      // Init
      edge_t tmp_edge_init = funnels::edge_t(_init_dummy, _bad,
                                             utils_ext::event_map["init"]);
      tmp_edge_init.update().add_expr(_clk, "=", 0);
      // Transitions from bad to good once time exceeds max_time,
      // which in this case should be called min time
      edge_t tmp_edge = funnels::edge_t(_bad, _good, _bad2good_evt);
      tmp_edge.guard().add_expr(_clk, ">=", std::ceil(_t_step*_end_time));
      return tmp_edge_init.declare() + "\n" + tmp_edge.declare() + "\n";
    }
  }
  
}









    
    