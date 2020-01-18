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
using fun_sys_t = funnel_sys_t<fun_t>;
using fun_sys_ptr_t = std::shared_ptr<fun_sys_t>;

using matrix_t = fun_t::matrix_t;
using vector_x_t = fun_t::vector_x_t;
using vector_u_t = fun_t::vector_u_t;
using vector_t_t = fun_t::vector_t_t;
using matrix_ptr_t = fun_t::matrix_ptr_t;
using vector_x_ptr_t = fun_t::vector_x_ptr_t;
using vector_u_ptr_t = fun_t::vector_u_ptr_t;
using vector_t_ptr_t = fun_t::vector_t_ptr_t;

using p_mat_t = lyap_t::s_mat_t;

const size_t n_verif=100;
const double_t t_step=200;

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

fun_ptr_t get_obstacle_box_fun(){
  // Creates an obstacle that moves along a square box of side length 0.5
  // The obstacle is a sphere of radius 0.5 in x-y-plane
  
  // Velocity of the obstacle is 0.25
  const double vel = 0.25;
  const double corner = 0.5;
  
  const double gamma_obs = 0.; // Obstacle does not contract
  const process_t &my_proc_obs = utils_ext::process_map.create_and_get
      ("obs_col");
  // Create the funnel
  fun_ptr_t my_obs_fun = make_shared<fun_t>(4 * n_verif, "obs_fun", my_proc_obs,
                                            4.*get_P_obs(), gamma_obs, my_dist);
  //Fill it
  vector_t_ptr_t t_ptr = std::make_shared<vector_t_t>(4 * n_verif);
  vector_u_ptr_t u_ptr = std::make_shared<vector_u_t>(my_obs_fun->dimu);
  matrix_ptr_t x_ptr = std::make_shared<matrix_t>(my_obs_fun->dimx, 4 * n_verif);
  
  // set time
  t_ptr->setLinSpaced(0., 4.*corner/vel);
  // dummy for control
  u_ptr->fill(0);
  // Obstacle will start off upper left corner and go clock-wise
  //left upper -> right upper
  x_ptr->block(0, 0, 1, n_verif) =
      Eigen::VectorXd::LinSpaced(n_verif, -corner, corner).transpose();
  x_ptr->block(1, 0, 1, n_verif).fill(corner);
  x_ptr->block(2, 0, 1, n_verif).fill(vel);
  x_ptr->block(3, 0, 1, n_verif).fill(0);
  //right upper -> right lower
  x_ptr->block(0, n_verif, 1, n_verif).fill(corner);
  x_ptr->block(1, n_verif, 1, n_verif) =
      Eigen::VectorXd::LinSpaced(n_verif, corner, -corner).transpose();
  x_ptr->block(2, n_verif, 1, n_verif).fill(0);
  x_ptr->block(3, n_verif, 1, n_verif).fill(-vel);
  // right lower -> left lower
  x_ptr->block(0, 2 * n_verif, 1, n_verif) =
      Eigen::VectorXd::LinSpaced(n_verif, corner, -corner).transpose();
  x_ptr->block(1, 2 * n_verif, 1, n_verif).fill(-corner);
  x_ptr->block(2, 2 * n_verif, 1, n_verif).fill(-vel);
  x_ptr->block(3, 2 * n_verif, 1, n_verif).fill(0);
  //left lower -> left upper
  x_ptr->block(0, 3 * n_verif, 1, n_verif).fill(-corner);
  x_ptr->block(1, 3 * n_verif, 1, n_verif) =
      Eigen::VectorXd::LinSpaced(n_verif, -corner, corner).transpose();
  x_ptr->block(2, 3 * n_verif, 1, n_verif).fill(0);
  x_ptr->block(3, 3 * n_verif, 1, n_verif).fill(vel);
  
  // Set in funnel
  my_obs_fun->set_traj(t_ptr, u_ptr, x_ptr);
  
  return my_obs_fun;
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

int main() {
  
  // Playground is between -1 and 1 in x and y dimension
  // Obstacle moves along a scaled square box of side length 0.5
  // obstacle is a sphere of size 0.5
  
  
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
  fun_sys_ptr_t my_fun_sys = std::make_shared<fun_sys_t>(ctrl_clk, lcl_clk,
      fun_sys_proc);
  
  // Create the obstacle automaton
  aut_col::aut_col<fun_t> my_aut_col(get_obstacle_box_fun(), obs_clk);
  
  // Create the survival automaton
  // longer then the cycle time of the obstacle
  aut_surv::aut_surv my_timer("surv", glob_clk, 1.5*4*2., false);
  
  // Get initial funnel/position
  my_fun_sys->add_funnel(get_init_fun(), false, true);
  
  // Test expanding a funnel
  add_new_funnels(my_fun_sys->all_funnels()[0], 0.01, *my_fun_sys);
  for (auto ff : my_fun_sys->all_funnels()){
    std::cout << ff->loc().name() << " " << ff->t()(ff->size()-1) << std::endl;
  }
  
  // Add it all together to obtain a TA
  ta::ta_t my_ta("system:funnel_ex_1\n\n");
  
  my_timer.register_self(my_ta);
  my_aut_col.register_self(my_ta);
  my_fun_sys->register_self(my_ta);
  
  my_fun_sys->clear_all();
  // Compute transitions and collisions
  // COLLISSIONS FIRST!!
  my_aut_col.compute_collisions(my_fun_sys, t_step);
  my_fun_sys->generate_transitions(t_step, true, false);
  
  // Auxilliary declarations
  // todo not very beautiful
  std::function<std::string()> aux_event =
      [](){return utils_ext::event_map["no_action"].declare()+
              "\n" + utils_ext::event_map["init"].declare();};
  
  my_ta._fun_event.push_back(aux_event);
  
  // Timer update
  my_timer.set_t_step(t_step);
  
  std::cerr << my_ta.declare() << std::endl;
  
  return 0;
}