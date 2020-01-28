//
// Created by philipp on 1/28/20.
//

#include <iostream>
#include <fstream>

// tchecker stuff
#include <tchecker_ext/algorithms/covreach_ext/run.hh>
#include <tchecker/zg/zg_ta.hh>

#include "tchecker/parsing/parsing.hh"
#include "tchecker/utils/log.hh"


// funnel stuff
#include "examples/example_1_utils.hh"
#include "heuristics/ex1_heu.hh"

// tchecker stuff
// Define all types
// The model
using algorithm_model_t = tchecker_ext::covreach_ext::details::zg::ta::
    algorithm_model_t<tchecker::zg::ta::elapsed_extraMplus_local_t>;
// and everything that derives from it
using model_t = algorithm_model_t::model_t;
using ts_t = algorithm_model_t::ts_t;
using graph_t = algorithm_model_t::graph_t;
using node_ptr_t = algorithm_model_t::node_ptr_t;
using state_predicate_t = algorithm_model_t::state_predicate_t;
// The inclusion
using cover_node_t = tchecker::covreach::cover_am_local_t<node_ptr_t, state_predicate_t>;
// Allocator
using builder_allocator_t = algorithm_model_t::builder_allocator_t;



std::unique_ptr<size_t> n_verif_obs = std::make_unique<size_t>(200);
std::unique_ptr<double_t> t_step = std::make_unique<double_t>(100);

int main(int arg, char *argv[]){
  
  std::string dummy_file = "./ta_fun_sys_dummy.tcheck";
  std::string tmp_aut_str;
  std::ofstream out_file;
  
  
  
  // Tchecker stuff
  tchecker::log_t log(&std::cerr);
  // options
  auto options_vec = get_options_vec();
  tchecker_ext::covreach_ext::options_t options(
      tchecker::range_t(options_vec.begin(), options_vec.end()), log);
  // Declaration
  tchecker::parsing::system_declaration_t * sysdecl = nullptr;
  model_t * model_ptr = nullptr;
  ts_t *ts_ptr = nullptr;
  cover_node_t *cover_node_ptr = nullptr;
  tchecker::label_index_t *label_index_ptr = nullptr;
  tchecker::covreach::accepting_labels_t<node_ptr_t> *accepting_labels_ptr = nullptr;
  tchecker::gc_t *gc_ptr = nullptr;
  graph_t *graph_ptr = nullptr;
  enum tchecker::covreach::outcome_t *outcome_ptr = nullptr;
  tchecker_ext::covreach_ext::stats_t *stats_ptr = nullptr;
  // DFS search
  tchecker_ext::covreach_ext::algorithm_t<ts_t,
      builder_allocator_t , graph_t, tchecker_ext::covreach_ext::threaded_lifo_waiting_t> *algorithm_ptr = nullptr;
  
  std::deque<ts_t> ts_vec; // Create one transition system per thread
  std::deque<builder_allocator_t> builder_alloc_vec;
  
  
  // Funnel stuff
  // Create the iteratively refined automaton
  
  // Playground is between -1 and 1 in x and y dimension
  // Obstacle moves along a scaled square box of side length 0.5
  // obstacle is a sphere of size 0.5
  size_t n_repeat = 2;
  // First argument: How often all flags have to be seen
  if (arg>=2){
    n_repeat = std::stoi(argv[1]);
  }
  // Second argument: t_step conversion from continuous time (double)
  //                  to discrete verif time (int)
  if (arg>=3){
    *t_step = std::stod(argv[2]);
  }
  // Third argument: n_verif_obs how many steps are used for obstacle evasion
  if (arg>=4){
    *n_verif_obs = std::stoi(argv[3]);
  }
  // Fourth argument : Whether or not the obstacle has a cyclic trajectory
  bool obs_is_cyclic=true;
  if(arg>=5){
    obs_is_cyclic = (bool)std::stoi(argv[4]);
  }
  
  utils_ext::clear_all_maps();
  // global clock
  funnels::clock_ta_t glob_clk = utils_ext::clock_map.create_and_get("c_g");
  // funnel controller clock
  funnels::clock_ta_t ctrl_clk = utils_ext::clock_map.create_and_get("c_t");
  // funnel local clock
  funnels::clock_ta_t lcl_clk = utils_ext::clock_map.create_and_get("c_h");
  // obstacle movement clock
  funnels::clock_ta_t obs_clk = utils_ext::clock_map.create_and_get("c_o");
  
  // process for funnel sys
  funnels::process_t &fun_sys_proc = utils_ext::process_map.create_and_get("proc_fun_sys_0");
  
  // Dummy event for switching between funnels / converging to funnels
  utils_ext::event_map.create_and_get("no_action");
  utils_ext::event_map.create_and_get("init");
  
  // Define the quadratic term of the Lyapunov function,
  // Here we use two different ones:
  // One for the dynamical system corresponding to a sphere in
  // in the state-space
  // One for the obstacle, corresponding to a cylinder in the state-space
  // (The obstacle blocks a position for all velocities)
  // Note approximation necessary
  
  // Create the funnel system (funnels will be added later
  fun_sys_ptr_t my_fun_sys = std::make_shared<fun_sys_f_t>(
      ctrl_clk, lcl_clk, fun_sys_proc, trans_abs::ONE_PER_BLOCK);
  
  // Create the obstacle automaton
  aut_col::aut_col<fun_t> my_aut_col(
      get_obstacle_box_fun(*n_verif_obs, obs_is_cyclic), obs_clk);
  
  // Create the survival automaton
  // longer then the cycle time of the obstacle
  aut_surv::aut_surv my_timer("surv", glob_clk, 1.5*4*n_repeat, false);
  
  // Create a flag-catch automaton
  auto flag_funnels = get_target_fun();
  aut_catch_flag::aut_catch_flag_t<fun_ptr_t> my_flag_catcher(false, flag_funnels,
                                                              false, aut_catch_flag::INCLUSION);
  
  // Create a process / automaton that represents a formula over the events
  // Here we want to visit all catch funnels in their given order exactly n-times
  aut_formula::aut_all_times_n my_formula("n_visit", my_flag_catcher._all_event,
                                          my_flag_catcher._all_sync, n_repeat);
  
  // Get initial funnel/position
  my_fun_sys->add_funnel(get_init_fun(), false, true);
  
  // Add it all together to obtain a TA
  ta::ta_t my_ta("system:funnel_ex_1\n\n");
  
  my_timer.register_self(my_ta);
  my_aut_col.register_self(my_ta);
  my_flag_catcher.register_self(my_ta);
  my_formula.register_self(my_ta);
  my_fun_sys->register_self(my_ta);
  
  // Auxilliary declarations
  // todo not very beautiful
  std::function<std::string()> aux_event =
      [](){return utils_ext::event_map["no_action"].declare()+
                  "\n" + utils_ext::event_map["init"].declare();};
  
  my_ta._fun_event.push_back(aux_event);
  
  // Timer update
  my_timer.set_t_step((*t_step));
  
  
  // Sync the processes
  my_flag_catcher.sync_process(my_fun_sys->process());
  
  
  // Now we can loop
  while (true){
    // Compute the automaton
    // todo : fix this to be incremental
    my_fun_sys->clear_all();
    // Compute transitions and collisions
    // COLLISSIONS FIRST!!
    my_aut_col.compute_collisions(my_fun_sys, (*t_step));
    // Now compute flags
    my_flag_catcher.compute_catches(my_fun_sys, (*t_step));
    my_fun_sys->generate_transitions((*t_step), true, false);
    
    out_file = std::ofstream(dummy_file);
    assert(out_file.is_open());
    out_file << my_ta.declare();
    out_file.close();
    
    // Create the tchecker model and run the algorithm to fill the graph
    sysdecl = nullptr;
    sysdecl = tchecker::parsing::parse_system_declaration(dummy_file, log);
    assert(sysdecl != nullptr);
  
    model_ptr = new model_t(*sysdecl, log);
    ts_ptr = new ts_t(*model_ptr);
  
    ts_vec.clear();
    for (unsigned int i=0; i<options.num_threads(); ++i){
      ts_vec.push_back(*ts_ptr);
    }
  
    // Depending on the extrapolation, cover_node is modified
    // compared to the standard version
    cover_node_ptr = new cover_node_t(algorithm_model_t::state_predicate_args(*model_ptr),
        algorithm_model_t::zone_predicate_args(*model_ptr));
  
    label_index_ptr = new tchecker::label_index_t(model_ptr->system().labels());
    for (std::string const & label : options.accepting_labels()) {
      if (label_index_ptr->find_value(label) == label_index_ptr->end_value_map())
        label_index_ptr->add(label);
    }
  
    // accepting_labels_t has member variables that are not thread safe
    // -> Use one copy per thread
    accepting_labels_ptr = new tchecker::covreach::accepting_labels_t<node_ptr_t>(*label_index_ptr, options.accepting_labels());
  
    gc_ptr = new tchecker::gc_t();
  
    graph_ptr = new graph_t(*gc_ptr,
                  std::tuple<tchecker::gc_t &, std::tuple<model_t &, std::size_t>, std::tuple<>>
                      (*gc_ptr, std::tuple<model_t &, std::size_t>(*model_ptr, options.block_size()), std::make_tuple()),
                  options.block_size(),
                  options.nodes_table_size(),
                  algorithm_model_t::node_to_key,
                  *cover_node_ptr);
  
    // Construct the helper allocator
    // Each builder allocator has its own transition (singleton) allocator, but all share the
    // node allocator with the graph
    builder_alloc_vec.clear();
    for (unsigned int i=0; i<options.num_threads(); ++i){
      builder_alloc_vec.emplace_back(*gc_ptr, graph_ptr->ts_allocator(), std::make_tuple());
    }
  
    gc_ptr->start();
  
    outcome_ptr = new tchecker::covreach::outcome_t();
    stats_ptr = new tchecker_ext::covreach_ext::stats_t();
    // DFS search
    algorithm_ptr = new tchecker_ext::covreach_ext::algorithm_t<ts_t,
        builder_allocator_t , graph_t, tchecker_ext::covreach_ext::threaded_lifo_waiting_t>();
  
    try {
      std::chrono::high_resolution_clock::time_point t_start
          = std::chrono::high_resolution_clock::now();
      std::tie(*outcome_ptr, *stats_ptr) = algorithm_ptr->run(ts_vec, builder_alloc_vec,
          *graph_ptr, *accepting_labels_ptr, options.num_threads(),
          options.n_notify());
    }
    catch (...) {
      gc_ptr->stop();
      graph_ptr->clear();
      graph_ptr->free_all();
      delete sysdecl;
      delete model_ptr; model_ptr = nullptr;
      delete ts_ptr; ts_ptr = nullptr;
      delete cover_node_ptr; cover_node_ptr = nullptr;
      delete label_index_ptr; label_index_ptr = nullptr;
      delete accepting_labels_ptr; accepting_labels_ptr = nullptr;
      delete gc_ptr; gc_ptr = nullptr;
      delete graph_ptr; graph_ptr = nullptr;
      delete outcome_ptr; outcome_ptr = nullptr;
      delete stats_ptr; stats_ptr = nullptr;
      delete algorithm_ptr; algorithm_ptr = nullptr;
      ts_vec.clear(); builder_alloc_vec.clear();
      throw;
    }
    
    if (*outcome_ptr != tchecker::covreach::REACHABLE){
      //Do the refinement
      iteration_from_graph(my_fun_sys, *graph_ptr, *sysdecl);
    }
  
    std::cout << "REACHABLE " << (*outcome_ptr == tchecker::covreach::REACHABLE ? "true" : "false") << std::endl;
  
    if (options.stats()) {
    
      std::cout << "Total stats are " << std::endl << options.num_threads() << std::endl;
    
      std::cout << "STORED_NODES " << graph_ptr->nodes_count() << std::endl;
      std::cout << *stats_ptr << std::endl;
    }
    
    // DONE
    gc_ptr->stop();
    graph_ptr->clear();
    graph_ptr->free_all();
    delete sysdecl;
    delete model_ptr; model_ptr = nullptr;
    delete ts_ptr; ts_ptr = nullptr;
    delete cover_node_ptr; cover_node_ptr = nullptr;
    delete label_index_ptr; label_index_ptr = nullptr;
    delete accepting_labels_ptr; accepting_labels_ptr = nullptr;
    delete gc_ptr; gc_ptr = nullptr;
    delete graph_ptr; graph_ptr = nullptr;
    delete outcome_ptr; outcome_ptr = nullptr;
    delete stats_ptr; stats_ptr = nullptr;
    delete algorithm_ptr; algorithm_ptr = nullptr;
    ts_vec.clear(); builder_alloc_vec.clear();
  
  
  
  
  
  
  
  }


}