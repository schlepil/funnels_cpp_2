//
// Created by philipp on 21.01.20.
//

#include "aut/aut_formula.hh"


namespace aut_formula{
  
  // aut_formula
  
  aut_formula::aut_formula(std::string formula):
      _formula(formula){
    throw std::runtime_error("todo -> Adapt from spot");
  }
  
  // aut_all_times_n
  aut_all_times_n::aut_all_times_n(std::string proc_name,
      std::vector<event_t *> event_vec, std::vector<sync_t *> &sync_vec,
      size_t n_repeat):
      _proc(utils_ext::process_map.create_and_get(proc_name)),
      _init_dummy(utils_ext::loc_map.create_and_get(_proc.name()
          + "_init_dummy", _proc)),
      _acc_lbl(utils_ext::label_map.create_and_get(proc_name +
          "_ordered_" + std::to_string(n_repeat))),
      _event_vec(std::move(event_vec)),
      _n_repeat(n_repeat){
    
    edge_t *this_edge_ptr;
    
    // Add the process to all syncs
    assert(event_vec.size() == sync_vec.size());
    for (size_t k=0; k<event_vec.size(); k++){
      sync_vec[k]->add_sync(_proc, *event_vec[k]);
    }
    
    // Create all states and edges
    // Here init is not committed, as no clock has to be set
    // the process is in this state until the first event appears
    for (size_t n_re = 0; n_re < _n_repeat; n_re++){
      for (size_t n_evt = 0; n_evt<_event_vec.size(); n_evt++){
        // The new state
        _aut_loc.emplace_back(&utils_ext::loc_map.create_and_get( _proc.name() +
        "_" + std::to_string(n_re) + "_" + std::to_string(n_evt),
        _proc));
        // The edge to get there
        if(_aut_loc.size() > 1){
          // from one aux state to another
          _aut_edges.emplace_back(*_aut_loc[_aut_loc.size()-2],
              *_aut_loc[_aut_loc.size()-1], *_event_vec[n_evt]);
        }else{
          // Init trans
          _aut_edges.emplace_back(_init_dummy, *_aut_loc[0], *_event_vec[0]);
        }
      }
    }
    // Label the last state for acceptance
    _aut_loc.back()->add_label(_acc_lbl);
    // Done
  }
  
  std::string aut_all_times_n::declare_proc()const{
    return _proc.declare();
  }
  
  std::string aut_all_times_n::declare_clk()const{
    return std::string();
  }
  
  std::string aut_all_times_n::declare_event()const{
    // Events have to be already declared
    return std::string();
  }
  
  std::string aut_all_times_n::declare_sync()const{
    // Sync have to be already declared
    return std::string();
  }
  
  std::string aut_all_times_n::declare_loc()const{
    std::string tmp_string = "";
    for (const location_t *a_loc : _aut_loc){
      tmp_string += a_loc->declare() + "\n";
    }
    return tmp_string;
  }
  
  std::string aut_all_times_n::declare_edge()const{
    std::string tmp_string = "";
    for (const edge_t &a_edge : _aut_edges){
      tmp_string += a_edge.declare() + "\n";
    }
    return tmp_string;
  }

}