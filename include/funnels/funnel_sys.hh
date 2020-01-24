//
// Created by philipp on 17.01.20.
//

#ifndef FUNNELS_CPP_FUNNEL_SYS_HH
#define FUNNELS_CPP_FUNNEL_SYS_HH

#include <memory>
#include <vector>
#include <deque>
#include <unordered_map>

#include "funnels/utils.hh"

namespace funnels{
  
  template <class FUNNEL>
  class funnel_sys_t{
  public:
    using fun_t = typename FUNNEL::fun_t;
    using fun_ptr_t = typename FUNNEL::fun_ptr_t;
    
    using fun_sys_t = funnel_sys_t<FUNNEL>;
    using fun_sys_ptr_t = std::shared_ptr<fun_sys_t>;
    
    using fun_family_t = typename FUNNEL::fun_family_t;
    using fun_family_ptr_t = typename FUNNEL::fun_family_ptr_t;
    
    funnel_sys_t(const clock_ta_t &ctrl_clk, const clock_ta_t &lcl_clk,
        const process_t &proc,
        trans_abs::abst_lvl_t abst_lvl=trans_abs::ONE_ABS):
        
      _abst_lvl(abst_lvl), _ctrl_clk(ctrl_clk), _lcl_clk(lcl_clk),
      _init_dummy(utils_ext::loc_map.create_and_get(proc.name()
          +"_init_dummy", proc, funnels::location_type_t::COMMITTED)){
      _init_dummy.set_initial(true);
    }
    
    const process_t & process(){
      if(_all_funnels.empty()){
        throw std::runtime_error("Empty funnel sys has no process");
      }
      return _all_funnels[0]->loc().process();
    }
    
    void add_funnel(fun_ptr_t &funnel,
                    bool is_parent=false, bool is_init=false){
      
      if (_init_dummy.process() != funnel->loc().process()){
        throw std::runtime_error("All funnels in a sys have to belong to "
                                 "the same process");
      }
      
      _fun_map[funnel->loc().name()] = funnel;
      
      _all_funnels.emplace_back(funnel);
      if(is_parent){
        _parent_funnels.emplace(funnel);
      }
      if(is_init){
        _init_funnels.emplace(funnel);
      }
    }
    void add_funnel(fun_ptr_t &&funnel,
                    bool is_parent=false, bool is_init=false){
      
      if (_init_dummy.process() != funnel->loc().process()){
        throw std::runtime_error("All funnels in a sys have to belong to "
                                 "the same process");
      }
      
      _fun_map[funnel->loc().name()] = funnel;
      _all_funnels.emplace_back(funnel);
      if(is_parent){
        _parent_funnels.emplace(funnel);
      }
      if(is_init){
        _init_funnels.emplace(funnel);
      }
    }
    
    void remove_funnel(const fun_ptr_t &funnel){
      // Remove the funnel from _all_funnels / _parent_funnel
      // and _fun_map
      for(auto it = _all_funnels.begin(); it!= _all_funnels.end(); it++){
        if (*it == funnel){
          _all_funnels.erase(it);
          break;
        }
      }
      _parent_funnels.erase(funnel);
      _init_funnels.erase(funnel);
      _fun_map.erase(funnel->loc().name());
      // Remove him also from the family list
      funnel->fun_family()[funnel->get_lvl()].erase(funnel);
    }
    void remove_funnel(const std::string &funnel_name){
      auto it = _fun_map.find(funnel_name);
      if (it != _fun_map.end()){
        return remove_funnel(it->second);
      }
    }
    
    bool is_init(const fun_ptr_t &funnel)const{
      return _init_funnels.find(funnel) != _init_funnels.end();
    }
    
    void generate_transitions(const double t_step,
                              bool do_converging, bool do_lcl_switch){
      // Generate all internal transitions
      // ^^only switching and converging transitions are taken
      // into account
      // But first delete all
      
      if (_all_funnels.empty()){
        return;
      }
      
      size_t n_trans=0, n_trans_delta, n_trans_delta_d;
      std::cout << "Funnel sys " << _all_funnels[0]->loc().process().name() <<
                std::endl;
      std::cout << "Gen internal transitions" << std::endl;
      for (fun_ptr_t a_funnel : _all_funnels){
        // todo : current constraint is that locations corresponding to funnels
        // can only have the invariants corresponding to the length
        // Neither committed nor urgent is admissible
        a_funnel->gen_invariant(t_step, _ctrl_clk);
        a_funnel->gen_internal_trans(t_step, _ctrl_clk);
      }
      
      std::cout << "Gen converging transitions" << std::endl;
      // Construct the converging transitions
      if (do_converging){
        std::set<fun_ptr_t> treated_parents_;
        for ( auto parent_it = _parent_funnels.begin();
              parent_it != _parent_funnels.end(); ++parent_it){
          if(treated_parents_.find(*parent_it) !=
             treated_parents_.end()){
            // The parent funnels are treated set wise
            continue;
          }
          auto &this_fun_family = (*parent_it)->fun_family();
          treated_parents_.insert(this_fun_family[0].begin(),
                                  this_fun_family[0].end());
          for ( size_t lvl=0; lvl<this_fun_family.size()-1; lvl++){
            for (auto &a_parent : this_fun_family[lvl]){
              if(a_parent->_is_colliding){
                // "Fake" tunnel
                continue;
              }
              for ( size_t lvl_c=lvl+1; lvl_c<this_fun_family.size(); lvl_c++) {
                for (auto &a_child : this_fun_family[lvl_c]) {
                  n_trans += compute_converging_trans(*a_parent,
                                                      *a_child, t_step,
                                                      _ctrl_clk, _lcl_clk);
                } // children
              } // child lvl
            } // parents
          } // lvl
        } // parent_funnels
      }
      
      // Construct switching transitions
      size_t i=0;
      std::cout << "Gen switching transitions" << std::endl;
      // todo parallelize
      for (auto src_fun : _all_funnels){
        std::cout << "funnel " << src_fun->loc().name()  << " (" << i << "/" <<
                  _all_funnels.size() << ")";
        n_trans_delta = 0;
        if (src_fun->_is_colliding){
          // "Fake" colliding tunnel
          continue;
        }
        for (auto tgt_fun : _all_funnels){
          if (!do_lcl_switch && src_fun->in_family(tgt_fun)){
            // Do not compute transitions between funnels along
            // the same ref-traj
            continue;
          }
          if (src_fun == tgt_fun){
            continue;
          }
          // Set the correct abstraction
          src_fun->_trans_abs =
              _abs_map.insert(
                  {std::pair(src_fun->loc().id(), tgt_fun->loc().id()),
                   std::make_shared<trans_abs::switching_trans_abstract_t>(_abst_lvl)}).first->second;
          
          n_trans_delta_d = compute_inclusion_trans_1(*src_fun, *tgt_fun,
                                                    t_step, _ctrl_clk, _lcl_clk);
          n_trans_delta += n_trans_delta_d;
          if (n_trans_delta_d>0) {
            std::cout << n_trans_delta_d << " ; ";
          }
        }
        n_trans += n_trans_delta;
        std::cout << std::endl << " has " << n_trans_delta << " outgoing" << std::endl;
        i++;
      }
      std::cout << "A total of " << n_trans << "transitions" << std::endl;
    }
    
    std::string declare_proc()const{
      return _all_funnels[0]->loc().process().declare() + "\n";
    }
    
    std::string declare_clk()const{
      return _ctrl_clk.declare() + "\n" + _lcl_clk.declare() + "\n";
    }
    
    std::string declare_event()const{
      return "";
    }
    
    std::string declare_sync()const{
      return "";
    }
    
    std::string declare_loc()const{
      std::string temp_string="";
      // Init
      temp_string += _init_dummy.declare()+"\n";
      // Funnels
      for (auto v : _all_funnels){
        temp_string += v->loc().declare()+"\n";
      }
      return temp_string;
    }
    
    std::string declare_edge()const{
      std::string temp_string="";
      // Init
      for (auto &a_fun : _init_funnels){
        edge_t init_edge(_init_dummy, a_fun->loc(),
                         utils_ext::event_map["init"]);
        init_edge.update().add_expr( _ctrl_clk, "=", 0);
        init_edge.update().add_expr( _lcl_clk, "=", 0);
        
        temp_string += init_edge.declare()+"\n";
      }
      
      // Funnels
      for (auto v : _all_funnels){
        for (auto e : v->_edges){
          temp_string += e.declare()+"\n";
        }
      }
      return temp_string;
    }
    
    template <class T>
    void register_self(T & reg){
      reg._fun_proc.push_back( std::bind(&fun_sys_t::declare_proc, this) );
      reg._fun_clk.push_back( std::bind(&fun_sys_t::declare_clk, this) );
      reg._fun_event.push_back( std::bind(&fun_sys_t::declare_event, this) );
      reg._fun_sync.push_back( std::bind(&fun_sys_t::declare_sync, this) );
      reg._fun_loc.push_back( std::bind(&fun_sys_t::declare_loc, this) );
      reg._fun_edge.push_back( std::bind(&fun_sys_t::declare_edge, this) );
    }
    
    const std::deque<fun_ptr_t> &all_funnels(){
      return _all_funnels;
    }
    const std::set<fun_ptr_t> &all_parents(){
      return _parent_funnels;
    }
    
    const std::unordered_map<std::string, fun_ptr_t> &fun_map(){
      return _fun_map;
    }
    
    const clock_ta_t &ctrl_clk(){
      return _ctrl_clk;
    }
    
    const clock_ta_t &lcl_clk(){
      return _lcl_clk;
    }
    
    void clear_all(){
      for (auto & a_f : _all_funnels){
        a_f->clear_loc();
        a_f->clear_edges();
      }
    }
  
  public:
    trans_abs::abst_lvl_t _abst_lvl;
    location_t &_init_dummy;
    
    // declaring
    std::deque<fun_ptr_t> _all_funnels;
    std::set<fun_ptr_t> _parent_funnels;
    std::set<fun_ptr_t> _init_funnels;
    const clock_ta_t &_ctrl_clk, &_lcl_clk;
    // todo fragmentation?
    std::unordered_map<std::pair<size_t, size_t>,
    std::shared_ptr<trans_abs::switching_trans_abstract_t>, pair_hash>
        _abs_map;
    std::unordered_map<std::string, fun_ptr_t> _fun_map;
    
  };
  
}

#endif //FUNNELS_CPP_FUNNEL_SYS_HH
