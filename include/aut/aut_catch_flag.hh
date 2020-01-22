//
// Created by philipp on 1/20/20.
//

#ifndef FUNNELS_CPP_AUT_CATCH_FLAG_HH
#define FUNNELS_CPP_AUT_CATCH_FLAG_HH

#include<memory>
#include <vector>

#include "funnels/utils.hh"
#include "aut/proc_ta.hh"
#include "funnels/funnel_computations.hh"
#include "funnels/transition_abstraction.hh"

using namespace funnels;

namespace aut_catch_flag{
  
  enum catch_cond_t{
    INTERSECT,
    INCLUSION
  };
  
  /*
   * todo mode 2,3,4
   * In total there are 4 modes to be considered, all except the first require
   * the funnel to be separated into two subparts
   * 1) Can with zeno : event self-edge
   * 2) Can once : event edge and regular edges from first to second subpart
   * 3) Must once : event edge from first to second part
   * 4) Must once, Can with zeno : event edge from first to second
   *    Second event self-edge
   *
   * Non-Zeno has to be ensured by formula
   */
  
  template <class FLAG_FUN_PTR>
  class aut_catch_flag_t : public ta::process_ta_t{
    using fun_t = typename FLAG_FUN_PTR::element_type::fun_t;
    using fun_ptr_t = FLAG_FUN_PTR;
    
  public:
    aut_catch_flag_t(bool is_timed, std::vector<FLAG_FUN_PTR> flag_fun,
        bool create_aut=false, catch_cond_t cond=INCLUSION):
        _create_aut(create_aut), _cond(cond),
        _proc(flag_fun[0]->loc().process()),
        _flag_fun(std::move(flag_fun)){
      
      // Create all events/flags/sync associated to the funnels
      for (size_t k=0;k<flag_fun.size();k++){
        assert(_proc == flag_fun[k]->loc().process());
        _all_loc.push_back(&(flag_fun[k]->loc()));
        _all_event.push_back(&utils_ext::event_map.create_and_get(
            flag_fun[k]->loc().name() + "_e"));
        _all_lbl.push_back(&utils_ext::label_map.create_and_get(
            flag_fun[k]->loc().name() + "_lbl"));
        _all_sync.push_back(&utils_ext::sync_map.create_and_get(
            flag_fun[k]->loc().name() + "_sync"));
      }
      
      if (is_timed){
        _clk_catch =
            &utils_ext::clock_map.create_and_get(_proc.name()+"_clk_catch");
      }
      
      // Create the associated auxilliary automaton
      if(_create_aut){
        // No restriction on the occurrence of the labels
        construct_no_restrict_aux_aut();
      }
      // Done
    }
    
    void construct_no_restrict_aux_aut(){
      // Create a init dummy state and a default state
      // make all funnel positions committed
      auto p_name = _proc.name();
      const location_t &default_loc =
          *_all_loc.emplace_back( & utils_ext::loc_map.create_and_get(p_name +
          "_default", _proc));
      const location_t &init_loc =
          *_all_loc.emplace_back( & utils_ext::loc_map.create_and_get(p_name +
          "_init_dummy", _proc));
      
      init_loc.set_loc_typ(COMMITTED);
      // Edge from init to default
      _aux_edges.emplace_back(init_loc, default_loc, utils_ext::event_map["init"]);
      
      for (size_t k=0; k<_flag_fun.size(); k++){
        // Set the catch funnel committed
        _flag_fun[k]->loc().set_loc_typ(COMMITTED);
        // edge from default to this label
        _aux_edges.emplace_back(default_loc, *_all_loc[k],
            *_all_event[k]);
        // edge from this label to default
        _aux_edges.emplace_back(*_all_loc[k], default_loc,
            utils_ext::event_map["no_action"]);
        // Add the sync
        _all_sync[k]->add_sync(_proc, *_all_event[k]);
        // Add the label to the state
        _flag_fun[k]->loc().add_label(*_all_lbl[k]);
      }
      // Done
    }
  
    /// Compute the collisions of all funnels in the sys with the obstacle
    /// \tparam FUN_SYS_PTR
    /// \param fun_sys_ptr
    template <class FUN_SYS_PTR>
    void compute_catches(const FUN_SYS_PTR & fun_sys_ptr, double t_step){
    
      // Add to the sync, make transition from and to the same funnel if
      // colliding
      
      // todo approximate the transitions
      // Check catch condition for each funnel
      size_t n_delta_trans, n_trans=0, i=0;
      for (const fun_ptr_t &fun : fun_sys_ptr->all_funnels()){
        n_delta_trans = 0;
        std::cout << "funnel " << fun->loc().name() << " (" << i++ << "/" <<
            fun_sys_ptr->all_funnels().size() << ")";
        // Skip colliding
        if(fun->_is_colliding){
          // "Fake" colliding tunnel
          continue;
        }
        // Set abstraction
        fun->_trans_abs = _abs_map[fun->loc().id()];
        // Compute the catches with each catch_funnel
        for (size_t k=0; k<_flag_fun.size(); k++){
          switch (_cond){
            case INCLUSION:
              n_delta_trans += compute_inclusion_trans_2(*fun, *(_flag_fun[k]),
                  t_step, fun_sys_ptr->ctrl_clk(), *_clk_catch,
                  fun->loc(), fun->loc(), *_all_event[k]);
              break;
            case INTERSECT:
              n_delta_trans += compute_intersecting_trans(*fun, *(_flag_fun[k]),
                  t_step, fun_sys_ptr->ctrl_clk(), *_clk_catch,
                  fun->loc(), fun->loc(), *_all_event[k]);
              break;
          }
        } // flag_fun
        std::cout << " has " << n_delta_trans << " catching transitions" << std::endl;
        n_trans += n_delta_trans;
      }// fun
      // Done
      std::cout << "There are " << n_trans << " catching transitions in total" << std::endl;
    }
  
    std::string declare_proc()const{
      std::string tmp_string = "";
      if (_create_aut){
        tmp_string = _proc.declare() + "\n";
      }
      return tmp_string;
    }
  
    std::string declare_clk()const{
      std::string tmp_string = "";
      if (_clk_catch != nullptr){
        tmp_string = _clk_catch->declare() + "\n";
      }
      return tmp_string;
    }
  
    std::string declare_event()const{
      std::string tmp_string = "";
      for (const event_t *a_ev : _all_event){
        tmp_string += a_ev->declare() + "\n";
      }
      return tmp_string;
    }
  
    std::string declare_sync()const{
      std::string tmp_string = "";
      for (const sync_t *a_sy : _all_sync){
        tmp_string += a_sy->declare() + "\n";
      }
      return tmp_string;
    }
  
    std::string declare_loc()const{
      std::string tmp_string = "";
      if (_create_aut){
        for (const location_t *a_loc : _all_loc){
          tmp_string += a_loc->declare() + "\n";
        }
      }
      return tmp_string;
    }
  
    std::string declare_edge()const{
      std::string tmp_string = "";
      if (_create_aut){
        for (const edge_t &a_edge : _aux_edges){
          tmp_string += a_edge.declare() + "\n";
        }
      }
      return tmp_string;
    }
  
    template <class T>
    void register_self(T & reg) {
      return ta::register_self(reg, this);
    }
    
    // Member variables
    bool _create_aut;
    catch_cond_t _cond;
    const process_t &_proc;
    clock_ta_t *_clk_catch = nullptr;
    std::vector<edge_t> _aux_edges;
    std::vector<const location_t *> _all_loc;
    std::vector<event_t *> _all_event;
    std::vector<label_t *> _all_lbl;
    std::vector<sync_t *> _all_sync;
    const std::vector<FLAG_FUN_PTR> _flag_fun;
  
    std::unordered_map<size_t,
        std::shared_ptr<trans_abs::catch_trans_abstract_t>> _abs_map;
  
  };

}

#endif //FUNNELS_CPP_AUT_CATCH_FLAG_HH
