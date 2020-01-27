//
// Created by philipp on 1/10/20.
//
#include <iostream>
#include <Eigen/Core>
#include <memory>

#include "funnels/distances.hh"
#include "funnels/dynamics.hh"
#include "helpers/trajectory.hh"
#include "helpers/bounding_box.hh"
#include "funnels/funnels.hh"
#include "funnels/funnel_sys.hh"
#include "aut/aut_collision.hh"
#include "aut/aut_survival.hh"
#include "aut/aut_catch_flag.hh"
#include "aut/aut_formula.hh"
#include "aut/ta.hh"

//#include "heuristics/ex1_heu.hh"
#include "heuristics/ex1_fixed.hh"

using dist_t = partial_so2_dist_t;

using dyn_t = dynamics::kinematic_2d_sys_t;
using traj_t = trajectories::timed_way_points_t<dyn_t>;
using lyap_t = lyapunov::fixed_ellipsoidal_lyap_t<traj_t, dist_t>;
using cvx_hull_t = helper_geom::aligned_bounding_box_t<lyap_t>;
using fun_t = funnels::funnel_t<cvx_hull_t>;

using fun_ptr_t = fun_t::fun_ptr_t;
using fun_sys_f_t = funnel_sys_t<fun_t>;
using fun_sys_ptr_t = std::shared_ptr<fun_sys_f_t>;

using matrix_t = fun_t::matrix_t;
using vector_x_t = fun_t::vector_x_t;
using vector_u_t = fun_t::vector_u_t;
using vector_t_t = fun_t::vector_t_t;
using matrix_ptr_t = fun_t::matrix_ptr_t;
using vector_x_ptr_t = fun_t::vector_x_ptr_t;
using vector_u_ptr_t = fun_t::vector_u_ptr_t;
using vector_t_ptr_t = fun_t::vector_t_ptr_t;

using p_mat_t = lyap_t::s_mat_t;

std::unique_ptr<size_t> n_verif_obs = std::make_unique<size_t>(200);
std::unique_ptr<double_t> t_step = std::make_unique<double_t>(100);

// Predefinitions
dist_t my_dist;

//Funnel shapes for obstacle and system
p_mat_t get_P_sys(){
  return p_mat_t::Identity();
}
p_mat_t get_P_obs(){
  p_mat_t P_obs = p_mat_t::Zero();
  P_obs(0,0) = 1.; P_obs(1,1) = 1.;
  P_obs(2,2) = 1.e-10; P_obs(3,3) = 1.e-10;
  return P_obs;
}
// Funnel shape for targets
p_mat_t get_P_targ(){
  // Cylinder with radius 0.25
  p_mat_t P_targ = p_mat_t::Zero();
  P_targ(0, 0) = 8.; P_targ(1, 1) = 8.;
  P_targ(2, 2) = 1.e-16; P_targ(3, 3) = 1.e-16;
  return P_targ;
}

fun_ptr_t get_obstacle_box_fun(bool is_cyclic=false){
  // Creates an obstacle that moves along a square box of side length 0.5
  // The obstacle is a sphere of radius 0.5 in x-y-plane
  
  // Velocity of the obstacle is 0.25
  const double vel = 0.25;
  const double corner = 0.5;
  
  const double gamma_obs = 0.; // Obstacle does not contract
  const process_t &my_proc_obs = utils_ext::process_map.create_and_get
      ("obs_col");
  // Create the funnel
  // Size to cover almost all points in playfield
//  fun_ptr_t my_obs_fun = make_shared<fun_t>(4 * (*n_verif_obs), "obs_fun", my_proc_obs,
//                                            4.*get_P_obs(), gamma_obs, my_dist);
  
  //Small for testing
  fun_ptr_t my_obs_fun = make_shared<fun_t>(4 * (*n_verif_obs), "obs_fun", my_proc_obs,
                                            100.*get_P_obs(), gamma_obs, my_dist);
  //Fill it
  vector_t_ptr_t t_ptr = std::make_shared<vector_t_t>(4 * (*n_verif_obs));
  vector_u_ptr_t u_ptr = std::make_shared<vector_u_t>(my_obs_fun->dimu);
  matrix_ptr_t x_ptr = std::make_shared<matrix_t>(my_obs_fun->dimx, 4 * (*n_verif_obs));
  
  // set time
  t_ptr->setLinSpaced(0., 4.*corner/vel);
  // dummy for control
  u_ptr->fill(0);
  // Obstacle will start off upper left corner and go clock-wise
  //left upper -> right upper
  x_ptr->block(0, 0, 1, (*n_verif_obs)) =
      Eigen::VectorXd::LinSpaced((*n_verif_obs), -corner, corner).transpose();
  x_ptr->block(1, 0, 1, (*n_verif_obs)).fill(corner);
  x_ptr->block(2, 0, 1, (*n_verif_obs)).fill(vel);
  x_ptr->block(3, 0, 1, (*n_verif_obs)).fill(0);
  //right upper -> right lower
  x_ptr->block(0, (*n_verif_obs), 1, (*n_verif_obs)).fill(corner);
  x_ptr->block(1, (*n_verif_obs), 1, (*n_verif_obs)) =
      Eigen::VectorXd::LinSpaced((*n_verif_obs), corner, -corner).transpose();
  x_ptr->block(2, (*n_verif_obs), 1, (*n_verif_obs)).fill(0);
  x_ptr->block(3, (*n_verif_obs), 1, (*n_verif_obs)).fill(-vel);
  // right lower -> left lower
  x_ptr->block(0, 2 * (*n_verif_obs), 1, (*n_verif_obs)) =
      Eigen::VectorXd::LinSpaced((*n_verif_obs), corner, -corner).transpose();
  x_ptr->block(1, 2 * (*n_verif_obs), 1, (*n_verif_obs)).fill(-corner);
  x_ptr->block(2, 2 * (*n_verif_obs), 1, (*n_verif_obs)).fill(-vel);
  x_ptr->block(3, 2 * (*n_verif_obs), 1, (*n_verif_obs)).fill(0);
  //left lower -> left upper
  x_ptr->block(0, 3 * (*n_verif_obs), 1, (*n_verif_obs)).fill(-corner);
  x_ptr->block(1, 3 * (*n_verif_obs), 1, (*n_verif_obs)) =
      Eigen::VectorXd::LinSpaced((*n_verif_obs), -corner, corner).transpose();
  x_ptr->block(2, 3 * (*n_verif_obs), 1, (*n_verif_obs)).fill(0);
  x_ptr->block(3, 3 * (*n_verif_obs), 1, (*n_verif_obs)).fill(vel);
  
  // Set in funnel
  my_obs_fun->set_traj(t_ptr, u_ptr, x_ptr);
  // Set if cyclic
  my_obs_fun->set_cyclic(is_cyclic);
  return my_obs_fun;
}

std::vector<fun_ptr_t> get_target_fun(){
  // Creates two cylinders in the upper right corner and the lower left corner
  
  const double c_center = 0.8;
  const size_t n_targ=5;
  
  const process_t &my_proc_targ = utils_ext::process_map.create_and_get
      ("dummy_targ");
  // Create the funnel
  fun_ptr_t my_targ_fun_ur = make_shared<fun_t>(n_targ, "targ_fun_ur", my_proc_targ,
                                            get_P_targ(), 0.0, my_dist);
  
  fun_ptr_t my_targ_fun_ll = make_shared<fun_t>(n_targ, "targ_fun_ll", my_proc_targ,
                                                get_P_targ(), 0.0, my_dist);
  //Fill it
  vector_t_ptr_t t_ptr = std::make_shared<vector_t_t>(n_targ); // Dummy time in this case
  vector_u_ptr_t u_ptr = std::make_shared<vector_u_t>(my_targ_fun_ur->dimu);
  matrix_ptr_t x_ur_ptr = std::make_shared<matrix_t>(my_targ_fun_ur->dimx, n_targ);
  
  matrix_ptr_t x_ll_ptr = std::make_shared<matrix_t>(my_targ_fun_ll->dimx, n_targ);
  
  // set time
  t_ptr->setLinSpaced(0., 1.); // Dummy, not used in this ex
  // dummy for control
  u_ptr->fill(0);
  // right upper
  x_ur_ptr->block(0, 0, 2, n_targ).fill(c_center);
  //left lower
  x_ll_ptr->block(0, 0, 2, n_targ).fill(-c_center);
  
  // Set in funnel
  my_targ_fun_ur->set_traj(t_ptr, u_ptr, x_ur_ptr);
  my_targ_fun_ll->set_traj(t_ptr, u_ptr, x_ll_ptr);
  
  return {my_targ_fun_ur, my_targ_fun_ll};
}



fun_ptr_t get_init_fun(){
  
  // Initial funnel system starts off without velocity at
  // position [0.75,1.]in a very small funnel
  //
  // Create the funnel
  fun_ptr_t my_init_fun = make_shared<fun_t>(3, "fun_init",
      utils_ext::process_map["proc_fun_sys_0"], (100.*100.)*get_P_sys(),
      0.0, my_dist);
  //Fill it
  vector_u_t u = vector_u_t::Zero();
  vector_x_t x0 = vector_x_t::Zero();
  x0(0) = 0.45;
  x0(1)= 0.45;
  my_init_fun->compute(x0, 0., 0.1, u);
  my_init_fun->loc().set_initial(true);
  my_init_fun->start_family(my_init_fun);
  return my_init_fun;
}

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
  aut_col::aut_col<fun_t> my_aut_col(get_obstacle_box_fun(obs_is_cyclic), obs_clk);
  
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