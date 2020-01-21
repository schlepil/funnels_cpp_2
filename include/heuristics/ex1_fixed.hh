//
// Created by philipp on 1/16/20.
//

#ifndef FUNNELS_CPP_EX1_FIXED_HH
#define FUNNELS_CPP_EX1_FIXED_HH

#include <Eigen/Core>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <array>


#include <funnels/utils.hh>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795d
#endif

// Attention, in example 1, velocity coincides with the control
// signal
// Attention, assumes constant velocity/input

const double max_r_step = 0.5; // different funnel sizes in a family
const double min_r = 0.001; // Minimal radius
const double max_r = 0.5+5.*min_r; // Minimal radius
const double max_alpha_step = max_r_step*max_r_step; // Minimal step for alpha
const double min_alpha = min_r*min_r; // Minimal alpha for ellipsoids
const double max_alpha = max_r*max_r; // Minimal alpha for ellipsoids
const double max_vel = 1.; // Norm of velocity
const double min_vel_diff = 0.1; // Norm difference in velocity
const double min_ang_diff = 20.*M_PI/180.;
const double gamma_conv = 10.; // exponential convergence rate
const double max_traj_length = 3; // Length of the new segment
const double dt_step = 0.05; //time step between two verification points
const size_t n_max = 100;
const double max_time = 100.;
const double min_englobe_fac = 1.1;

const double plane_dist = 0.3;
const double inter_dist = 0.3;

Eigen::Matrix2d getR(double alpha);


template <class FUN_PTR_T>
std::pair<FUN_PTR_T, FUN_PTR_T> get_this_funnel(const FUN_PTR_T &src, double radius,
                          const Eigen::Vector4d &x0_O, const Eigen::Vector2d u){
  
  using fun_t = typename FUN_PTR_T::element_type;
  
  // Get maximal time before the maximal length is attained
  // or the playground is left
  Eigen::Vector4d x0 = x0_O;
  double n_u, t_max;
  if (u.norm() > 1e-3){
    n_u = u.norm()+1e-200;
    t_max = std::min(max_time, max_traj_length/n_u);
    // distance to x/y-border
    for (size_t i=0; i<2; i++){
      if(u(i)>=0.){
        t_max = std::min(t_max, (1.2-x0(i))/(u(i)+1e-200));
      }else{
        t_max = std::min(t_max, (-1.2-x0(i))/(u(i)+1e-200));
      }
    }
    if (t_max < 3.*dt_step){
      // trajectory is too short
      // initial is on the boundary and we are pointing outwards
      return std::pair(nullptr, nullptr);
    }
  }else{
    t_max = 3.*dt_step;
  }
  
  
  // Convert to number of points
  size_t this_n_verif = std::max((size_t)3, std::min((size_t) (t_max/dt_step), n_max));
  
  // todo check if this funnel already exists in the
  // funnel system
  // get the new name
  // Update the initial pos
  x0 = x0_O;
  
  std::stringstream sstream;
  sstream.setf(std::ios::fixed);
  sstream.precision(2);
  
  sstream << "fun_";
  for(size_t i=0; i<4; i++){
    sstream << (double) x0(i) << "_";
  }
  sstream << radius;
  std::string name = sstream.str();
  std::replace( name.begin(), name.end(), '.', 'p');
  std::replace( name.begin(), name.end(), '-', 'm');
  
  // If exactly this funnel exists, the name will exist
  if (utils_ext::loc_map.find(name)){
    return std::pair(nullptr, nullptr);
  }
  
  FUN_PTR_T new_fun1 = src->make_copy(name, 2.*this_n_verif);
  FUN_PTR_T new_fun2 = src->make_copy(name+"_twin", 2.*this_n_verif);
  
  // set the new radius
  new_fun1->set_P(src->get_P()*(src->get_alpha()/(radius*radius)));
  new_fun2->set_P(src->get_P()*(src->get_alpha()/(radius*radius)));
  // and convergence
  new_fun1->set_gamma(gamma_conv);
  new_fun2->set_gamma(gamma_conv);
  
  // Update pos and compute traj
  x0.block(2,0,2,1) -= -t_max*u;
  new_fun1->compute(x0, 0., 2.*t_max, u);
  // Mirror
  x0 = -x0_O;
  x0.block(2,0,2,1) -= -t_max*u;
  new_fun2->compute(x0, 0., 2.*t_max, u);
  
  
  // He starts a family
  new_fun1->start_family(new_fun1);
  new_fun2->start_family(new_fun2);
  
  return std::pair(new_fun1, new_fun2);
}

template <class FUN_PTR_T>
FUN_PTR_T get_this_funnel_1(const FUN_PTR_T &src, double radius,
    const Eigen::Vector4d &x0_O, const Eigen::Vector2d u){
  
  using fun_t = typename FUN_PTR_T::element_type;
  
  // Get maximal time before the maximal length is attained
  // or the playground is left
  Eigen::Vector4d x0 = x0_O;
  double n_u, t_max;
  if (u.norm() > 1e-3){
    n_u = u.norm()+1e-200;
    t_max = std::min(max_time, max_traj_length/n_u);
    // distance to x/y-border
    for (size_t i=0; i<2; i++){
      if(u(i)>=0.){
        t_max = std::min(t_max, (1.2-x0(i))/(u(i)+1e-200));
      }else{
        t_max = std::min(t_max, (-1.2-x0(i))/(u(i)+1e-200));
      }
    }
    if (t_max < 3.*dt_step){
      // trajectory is too short
      // initial is on the boundary and we are pointing outwards
      return nullptr;
    }
  }else{
    t_max = 3.*dt_step;
  }
  
  
  // Convert to number of points
  size_t this_n_verif = std::max((size_t)3, std::min((size_t) (t_max/dt_step), n_max));
  
  // todo check if this funnel already exists in the
  // funnel system
  // get the new name
  // Update the initial pos
  x0 = x0_O;
  
  std::stringstream sstream;
  sstream.setf(std::ios::fixed);
  sstream.precision(2);
  
  sstream << "fun_";
  for(size_t i=0; i<4; i++){
    sstream << (double) x0(i) << "_";
  }
  sstream << radius;
  std::string name = sstream.str();
  std::replace( name.begin(), name.end(), '.', 'p');
  std::replace( name.begin(), name.end(), '-', 'm');
  
  // If exactly this funnel exists, the name will exist
  if (utils_ext::loc_map.find(name)){
    return nullptr;
  }
  
  FUN_PTR_T new_fun = src->make_copy(name, this_n_verif);
  
  // set the new radius
  new_fun->set_P(src->get_P() * (src->get_alpha() / (radius * radius)));
  // and convergence
  new_fun->set_gamma(gamma_conv);
  
  // Update pos and compute traj
  x0.block(2,0,2,1) -= -t_max*u;
  new_fun->compute(x0, 0., 2. * t_max, u);
  // Mirror
  x0 = -x0_O;
  x0.block(2,0,2,1) -= -t_max*u;
  
  
  // He starts a family
  new_fun->start_family(new_fun);
  
  return new_fun;
}

template <class FUN_PTR_T>
std::vector<FUN_PTR_T> get_children(FUN_PTR_T &parent){
  
  using fun_t = typename FUN_PTR_T::element_type;
  using matrix_t = typename fun_t::matrix_t;
  
  FUN_PTR_T this_child_fun;
  FUN_PTR_T this_parent_fun = parent;
  
  
  std::stringstream sstream;
  std::string name;
  
  // Original unscaled matrix
  matrix_t P_p = parent->get_P()*parent->get_alpha();
  
  sstream.setf(std::ios::fixed);
  sstream.precision(2);
  
  std::vector<FUN_PTR_T> fun_vec_child;
  
  const double alpha_step = max_alpha_step;
  const double r_step = max_r_step;
  
  // Radius of parent
  double alpha = parent->get_alpha();
  
  if(std::sqrt(alpha) < min_r + r_step){
    return fun_vec_child;
  }
  
  alpha = std::max(min_alpha, (std::sqrt(alpha)-r_step)*(std::sqrt(alpha)-r_step));
  
  while(alpha>=min_alpha){
    // Get the name
    sstream.str(std::string());
    sstream << "fun_";
    auto x00 = parent->x0();
    for(size_t i=0; i<4; i++){
      sstream << (double) x00(i) << "_";
    }
    sstream << std::sqrt(alpha);
    name = sstream.str();
    std::replace( name.begin(), name.end(), '.', 'p');
    std::replace( name.begin(), name.end(), '-', 'm');
    if (utils_ext::loc_map.find(name)){
      // Already exists
      return fun_vec_child;
    }
    
    this_child_fun = this_parent_fun->make_copy(name, this_parent_fun->size());
    // set the new radius
    this_child_fun->set_P(P_p/alpha);
    this_child_fun->set_cyclic(this_parent_fun->is_cyclic());
    // And convergence
    this_child_fun->set_gamma(gamma_conv);
    
    fun_t::set_parent(this_parent_fun, this_child_fun);
    fun_vec_child.emplace_back(this_child_fun);
    this_parent_fun = this_child_fun;
    this_child_fun = nullptr;
    // Update alpha
    alpha = (std::sqrt(alpha)-r_step); // New alpha as r
    if (alpha <= min_r){
      break; // Necessary as negative radii become positive in next step
    }
    alpha *= alpha; // Convert to alpha
  }
  return fun_vec_child;
}


template<class FUN_PTR_T, class FUN_SYS>
void add_new_funnels(const FUN_PTR_T& src,
                     double t_src, FUN_SYS &fun_sys){
  
  // Src is dummy, only to keep the same signature
  
  using fun_t = typename FUN_PTR_T::element_type;
  
  // Return val
  FUN_PTR_T fun_parent1, fun_parent2;
  std::vector<FUN_PTR_T> fun_vec_child;
  
  Eigen::Vector2d pos_0;
  
  Eigen::Vector2d orth_p, vel_p, vel_1 = Eigen::Vector2d::Zero();
  vel_1(0) = 1.;
  
  Eigen::Vector4d x0;
  
  // The problem is purely symmetric
  
  // Grid of static funnels
  std::array<int, 2> signs = {-1,1};
  x0.setZero();
  vel_p.setZero();
  for (double x_s = plane_dist/2.; x_s <= 1.; x_s+=plane_dist){
    for (double y_s = plane_dist/2.; y_s <= 1.; y_s+=plane_dist) {
      for (auto s_x : signs){
        for (auto s_y : signs){
          x0(0) = s_x*x_s;
          x0(1) = s_y*y_s;
          fun_parent1 = get_this_funnel_1(src, max_r, x0, vel_p);
          if (fun_parent1 != nullptr) {
            // add
            fun_sys.add_funnel(fun_parent1, true);
            // all_children
            fun_vec_child = get_children(fun_parent1);
            for (auto a_f : fun_vec_child) {
              fun_sys.add_funnel(a_f);
            }
          }
        }
      }
    }
  }
  
  // "Dynamic" funnels
  // Loop over all velocities <-> planes
  for (double n_vel=inter_dist; n_vel<=1.0; n_vel+=inter_dist) {
      // Loop over all angles
    for (double d_alpha = 0.; d_alpha <= 2 * M_PI - d_alpha / 2;
         d_alpha += min_ang_diff) {
      // New velocity
      vel_p = n_vel * (getR(d_alpha) * vel_1);
      // New orthogonal
      orth_p = getR(M_PI / 2.) * getR(d_alpha) * vel_1;
      // Loop over all funnel origins
      pos_0 = orth_p * (plane_dist / 2.);
      while (true) {
        if ((pos_0.array().abs() >= 1.).any()) {
          // Out of the box
          break;
        }
    
        // Now we have a new parent funnel candidate
        x0.block(0, 0, 2, 1) = pos_0;
        // todo refactor norm vs normalized
        x0.block(2, 0, 2, 1) = vel_p;
        // Parent
        std::tie(fun_parent1, fun_parent2) = get_this_funnel(src, max_r, x0,
                                                             vel_p);
    
        if (fun_parent1 != nullptr) {
          // add
          fun_sys.add_funnel(fun_parent1, true);
          // all_children
          fun_vec_child = get_children(fun_parent1);
          for (auto a_f : fun_vec_child) {
            fun_sys.add_funnel(a_f);
          }
        }
        if (fun_parent2 != nullptr) {
          // add
          fun_sys.add_funnel(fun_parent2, true);
          // all_children
          fun_vec_child = get_children(fun_parent2);
          for (auto a_f : fun_vec_child) {
            fun_sys.add_funnel(a_f);
          }
        }
        // Update
        pos_0 += orth_p * plane_dist;
      } // origin pos
    } // angle
  } // plane
  return;
}

#endif //FUNNELS_CPP_EX1_FIXED_HH
