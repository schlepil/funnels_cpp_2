//
// Created by phili on 29.12.2019.
//

#ifndef FUNNELS_CPP_AUT_COLLISION_HH
#define FUNNELS_CPP_AUT_COLLISION_HH

#include<memory>

#include "funnels/utils.hh"
#include "aut/proc_ta.hh"
#include <funnels/funnel_computations.hh>
#include "funnels/collisions.hh"

namespace aut_col{
  template <class OBS_FUNNEL>
  class aut_col: public ta::process_ta_t{
    using fun_t = typename OBS_FUNNEL::fun_t;
    using fun_ptr_t = typename OBS_FUNNEL::fun_ptr_t;
    using clock_ta_t = funnels::clock_ta_t;
  // TODO : check if really needed/desired that collision has its own process
  public:
    aut_col(const fun_ptr_t obs_funnel, const clock_ta_t &obs_clk):
        _obs_funnel(obs_funnel),
        _obs_clk(obs_clk),
        _proc(obs_funnel->loc().process()),
        _init_dummy(utils_ext::loc_map.create_and_get(_proc.name()+"_init_dummy",
            _proc, location_type_t::COMMITTED)),
        _init_edge(_init_dummy, _obs_funnel->loc(),
            utils_ext::event_map["init"]),
        _col_lbl(utils_ext::label_map.create_and_get(_proc.name()+"_collided")){
      // set init
      _init_dummy.set_initial(true);
      // init_edge
      _init_edge.update().add_expr(obs_clk, "=", 0);
    }
    
    /// Compute the collisions of all funnels in the sys with the obstacle
    /// \tparam FUN_SYS_PTR
    /// \param fun_sys_ptr
    template <class FUN_SYS_PTR>
    void compute_collisions(const FUN_SYS_PTR & fun_sys_ptr, double t_step){
      
      // Add to the sync, make transition from and to the same funnel if
      // colliding
      
      size_t i=0;
      bool does_collide;
      
      std::vector<fun_ptr_t> tmp_vec;
      bool p_is_init, p_is_parent;
      
      // todo approximate the transitions
      // Check collis ion for all funnels
      std::cout << "Computing collisions for " << _proc.name() << std::endl;
      std::vector<fun_ptr_t> all_funnels_cpy;
      all_funnels_cpy.reserve(fun_sys_ptr->all_funnels().size());
      all_funnels_cpy.insert(all_funnels_cpy.end(),
          fun_sys_ptr->all_funnels().begin(), fun_sys_ptr->all_funnels().end());
      for (const fun_ptr_t &fun : all_funnels_cpy){
        std::cout << "funnel " << fun->loc().name() << " (" << i << "/" <<
            all_funnels_cpy.size() << ")";
        // Set abstraction
        auto abs_comp = _abs_map[fun->loc().id()];
        // Compute intersections (overapprox)
        // This creates new edges with event in the src funnel
        tmp_vec.clear();
        does_collide = abs_comp(fun, _obs_funnel, tmp_vec, t_step,
            fun_sys_ptr->ctrl_clk(), _obs_clk, _col_lbl);
        if (does_collide){
          p_is_init = fun_sys_ptr->is_init(fun);
          p_is_parent = fun->get_lvl()==0;
          fun_sys_ptr->remove_funnel(fun);
          _abs_map.erase(fun->loc().id());
          for (auto &n_fun : tmp_vec){
            fun_sys_ptr->add_funnel(n_fun, p_is_parent, p_is_init);
          }
          std::cout << " does possibly collide, split into " << tmp_vec.size() << std::endl;
        }else{
          std::cout << "does not collide" << std::endl;
        }
        i++;
      }
    }
    
    std::string declare_proc()const{
      return _proc.declare() + "\n";
    }
    
    std::string declare_clk()const{
      return _obs_clk.declare() + "\n";
    }
    
    std::string declare_event()const{
      return "";
    }
    
    std::string declare_sync()const{
      return "";
    }
    
    std::string declare_loc()const{
      return _init_dummy.declare() + "\n" + _obs_funnel->loc().declare() + "\n";
    }
    
    std::string declare_edge(){
      return _init_edge.declare() + "\n";
    }
  
    template <class T>
    void register_self(T & reg){
      return ta::register_self(reg, this);
//      reg._fun_proc.push_back( std::bind(&aut_col::declare_proc, this) );
//      reg._fun_clk.push_back( std::bind(&aut_col::declare_clk, this) );
//      reg._fun_event.push_back( std::bind(&aut_col::declare_event, this) );
//      reg._fun_sync.push_back( std::bind(&aut_col::declare_sync, this) );
//      reg._fun_loc.push_back( std::bind(&aut_col::declare_loc, this) );
//      reg._fun_edge.push_back( std::bind(&aut_col::declare_edge, this) );
    }


  protected:
    fun_ptr_t _obs_funnel;
    const clock_ta_t &_obs_clk;
    const funnels::process_t &_proc;
    const funnels::location_t &_init_dummy;
    edge_t _init_edge;
    const funnels::label_t &_col_lbl;
  
    std::unordered_map<size_t, collision::avoid_collisions_t> _abs_map;
    
  };
}

#endif //FUNNELS_CPP_AUT_COLLISION_HH
