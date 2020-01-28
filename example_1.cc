//
// Created by philipp on 1/10/20.
//
#include <iostream>

//#include "heuristics/ex1_heu.hh"
#include "examples/example_1_utils.hh"
#include "heuristics/ex1_fixed.hh"

std::unique_ptr<size_t> n_verif_obs = std::make_unique<size_t>(200);
std::unique_ptr<double_t> t_step = std::make_unique<double_t>(100);


int main(int arg, char *argv[]) {
  
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
  
  
  std::cout << "funnel sys" << std::endl;
  
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
  event_t &dummy = utils_ext::event_map.create_and_get("no_action");
  event_t &dummy2 = utils_ext::event_map.create_and_get("init");
  
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
  
  // Test expanding a funnel
  add_new_funnels(my_fun_sys->all_funnels()[0], 0.01, *my_fun_sys);
  for (auto ff : my_fun_sys->all_funnels()){
    std::cout << ff->loc().name() << " t_end: " << ff->t()(ff->size()-1)
        << " is_cyc: " << ff->is_cyclic() << std::endl;
  }
  
  // Add it all together to obtain a TA
  ta::ta_t my_ta("system:funnel_ex_1\n\n");
  
  my_timer.register_self(my_ta);
  my_aut_col.register_self(my_ta);
  my_flag_catcher.register_self(my_ta);
  my_formula.register_self(my_ta);
  my_fun_sys->register_self(my_ta);
  
  
  my_fun_sys->clear_all();
  // Compute transitions and collisions
  // COLLISSIONS FIRST!!
  my_aut_col.compute_collisions(my_fun_sys, (*t_step));
  // Now compute flags
  my_flag_catcher.compute_catches(my_fun_sys, (*t_step));
  my_fun_sys->generate_transitions((*t_step), true, false);
  
  // Auxilliary declarations
  // todo not very beautiful
  std::function<std::string()> aux_event =
      [](){return utils_ext::event_map["no_action"].declare()+
              "\n" + utils_ext::event_map["init"].declare();};
  
  my_ta._fun_event.push_back(aux_event);
  
  // Timer update
  my_timer.set_t_step((*t_step));
  
  std::cerr << my_ta.declare() << std::endl;
  
  return 0;
}