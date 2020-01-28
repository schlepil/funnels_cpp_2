//
// Created by philipp on 1/28/20.
//

#ifndef FUNNELS_CPP_EXAMPLE_1_UTILS_HH
#define FUNNELS_CPP_EXAMPLE_1_UTILS_HH

#include <Eigen/Core>
#include <memory>

#include "funnels/distances.hh"
#include "funnels/dynamics.hh"
#include "helpers/trajectory.hh"
#include "funnels/funnel_sys.hh"
#include "aut/aut_collision.hh"
#include "aut/aut_survival.hh"
#include "aut/aut_catch_flag.hh"
#include "aut/aut_formula.hh"
#include "aut/ta.hh"

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

// Predefinitions
extern dist_t my_dist;

// Options
std::vector<std::pair<std::string, std::string>> get_options_vec();

//Funnel shapes for obstacle and system
p_mat_t get_P_sys();
p_mat_t get_P_obs();

// Funnel shape for targets
p_mat_t get_P_targ();

// Create the obstacle funnel
fun_ptr_t get_obstacle_box_fun(size_t n_verif_obs, bool is_cyclic=false);

// Create vector of target funnels
std::vector<fun_ptr_t> get_target_fun();

// Create initial funnel
fun_ptr_t get_init_fun();

#endif //FUNNELS_CPP_EXAMPLE_1_UTILS_HH
