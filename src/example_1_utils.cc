//
// Created by philipp on 1/28/20.
//

#include "examples/example_1_utils.hh"

// Predefinitions
dist_t my_dist;

std::vector<std::pair<std::string, std::string>> get_options_vec(){
  std::vector<std::pair<std::string, std::string>> options;
  options.emplace_back("l", "n_visit_ordered_2");
  options.emplace_back("c", "aMl");
  options.emplace_back("m", "zg:elapsed:extraM+l");
  options.emplace_back("table-size", "7");
  return options;
}

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

// Create the obstacle funnel
fun_ptr_t get_obstacle_box_fun(size_t n_verif_obs, bool is_cyclic){
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
//  fun_ptr_t my_obs_fun = make_shared<fun_t>(4 * (n_verif_obs), "obs_fun", my_proc_obs,
//                                            4.*get_P_obs(), gamma_obs, my_dist);
  
  //Small for testing
  fun_ptr_t my_obs_fun = make_shared<fun_t>(4 * (n_verif_obs), "obs_fun", my_proc_obs,
                                            100.*get_P_obs(), gamma_obs, my_dist);
  //Fill it
  vector_t_ptr_t t_ptr = std::make_shared<vector_t_t>(4 * (n_verif_obs));
  vector_u_ptr_t u_ptr = std::make_shared<vector_u_t>(my_obs_fun->dimu);
  matrix_ptr_t x_ptr = std::make_shared<matrix_t>(my_obs_fun->dimx, 4 * (n_verif_obs));
  
  // set time
  t_ptr->setLinSpaced(0., 4.*corner/vel);
  // dummy for control
  u_ptr->fill(0);
  // Obstacle will start off upper left corner and go clock-wise
  //left upper -> right upper
  x_ptr->block(0, 0, 1, (n_verif_obs)) =
      Eigen::VectorXd::LinSpaced((n_verif_obs), -corner, corner).transpose();
  x_ptr->block(1, 0, 1, (n_verif_obs)).fill(corner);
  x_ptr->block(2, 0, 1, (n_verif_obs)).fill(vel);
  x_ptr->block(3, 0, 1, (n_verif_obs)).fill(0);
  //right upper -> right lower
  x_ptr->block(0, (n_verif_obs), 1, (n_verif_obs)).fill(corner);
  x_ptr->block(1, (n_verif_obs), 1, (n_verif_obs)) =
      Eigen::VectorXd::LinSpaced((n_verif_obs), corner, -corner).transpose();
  x_ptr->block(2, (n_verif_obs), 1, (n_verif_obs)).fill(0);
  x_ptr->block(3, (n_verif_obs), 1, (n_verif_obs)).fill(-vel);
  // right lower -> left lower
  x_ptr->block(0, 2 * (n_verif_obs), 1, (n_verif_obs)) =
      Eigen::VectorXd::LinSpaced((n_verif_obs), corner, -corner).transpose();
  x_ptr->block(1, 2 * (n_verif_obs), 1, (n_verif_obs)).fill(-corner);
  x_ptr->block(2, 2 * (n_verif_obs), 1, (n_verif_obs)).fill(-vel);
  x_ptr->block(3, 2 * (n_verif_obs), 1, (n_verif_obs)).fill(0);
  //left lower -> left upper
  x_ptr->block(0, 3 * (n_verif_obs), 1, (n_verif_obs)).fill(-corner);
  x_ptr->block(1, 3 * (n_verif_obs), 1, (n_verif_obs)) =
      Eigen::VectorXd::LinSpaced((n_verif_obs), -corner, corner).transpose();
  x_ptr->block(2, 3 * (n_verif_obs), 1, (n_verif_obs)).fill(0);
  x_ptr->block(3, 3 * (n_verif_obs), 1, (n_verif_obs)).fill(vel);
  
  // Set in funnel
  my_obs_fun->set_traj(t_ptr, u_ptr, x_ptr);
  // Set if cyclic
  my_obs_fun->set_cyclic(is_cyclic);
  return my_obs_fun;
}

// Create vector of target funnels
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

// Create initial funnel
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