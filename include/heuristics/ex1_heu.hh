//
// Created by philipp on 13.01.20.
//

#ifndef FUNNELS_CPP_EX1_HEU_HH
#define FUNNELS_CPP_EX1_HEU_HH

#include <Eigen/Core>
#include <cmath>
#include <sstream>
#include <algorithm>


#include <funnels/utils.hh>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795d
#endif

// Attention, in example 1, velocity coincides with the control
// signal
// Attention, assumes constant velocity/input

const double max_r_step = 0.25; // different funnel sizes in a family
const double min_r = 0.001; // Minimal radius
const double max_r = 0.501; // Maximal radius
const double max_alpha_step = max_r_step*max_r_step; // Minimal step for alpha
const double min_alpha = min_r*min_r; // Minimal alpha for ellipsoids
const double max_alpha = max_r*max_r; // Minimal alpha for ellipsoids
const double max_vel = 1.; // Norm of velocity
const double min_vel_diff = 0.1; // Norm difference in velocity
const double min_ang_diff = 20.*M_PI/180.;
const double gamma_conv = 9.; // exponential convergence rate
const double max_traj_length = 0.2; // Length of the new segment
const double dt_step = 0.025; //time step between two verification points
const double max_time = 10.;
const double min_englobe_fac = 1.1;


Eigen::Matrix2d getR(double alpha);

template<class DERIVED>
double get_smallest_englobing(double r_src,
    const Eigen::MatrixBase<DERIVED> &delta_vel){
  return min_englobe_fac*(r_src + delta_vel.norm());
}

double get_smallest_englobing(double r_src, double d_v_norm);

template <class FUN_PTR_T>
FUN_PTR_T get_this_funnel(const FUN_PTR_T &src, double radius,
    const Eigen::Vector4d &x0, const Eigen::Vector2d u){
  
  using fun_t = typename FUN_PTR_T::element_type;
  
  // Get maximal time before the maximal length is attained
  // or the playground is left
  double n_u = u.norm()+1e-200;
  double t_max = std::min(max_time, max_traj_length/n_u);
  // distance to x/y-border
  for (size_t i=0; i<2; i++){
    if(x0(i)>=0. && u(i)>=0.){
      t_max = std::min(t_max, (1.-x0(i))/(u(i)+1e-200));
    }else if (x0(i)<=0. && u(i)<=0.){
      t_max = std::min(t_max, (1.+x0(i))/(u(i)+1e-200));
    }
  }
  if (t_max < 3.*dt_step){
    // trajectory is too short
    // initial is on the boundary and we are pointing outwards
    return nullptr;
  }
  
  // Convert to number of points
  size_t this_n_verif = t_max/dt_step + 3;
  
  // todo check if this funnel already exists in the
  // funnel system
  // get the new name
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
  new_fun->set_P(src->get_P()*(src->get_alpha()/(radius*radius)));
  // and convergence
  new_fun->set_gamma(gamma_conv);
  
  new_fun->compute(x0, 0., t_max, u);
  
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
    for(size_t i=0; i<4; i++){
      sstream << (double) parent->x0()(i) << "_";
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
    this_child_fun->set_cyclic(this_parent_fun->get_cyclic());
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
  
  using fun_t = typename FUN_PTR_T::element_type;
  using vector_t = typename fun_t::vector_t;
  using matrix_t = typename fun_t::matrix_t;
  
  // Return val
  FUN_PTR_T fun_parent;
  std::vector<FUN_PTR_T> fun_vec_child;
  
  // Radius of source
  double r_src = std::sqrt(src->get_alpha());
  if(r_src>=0.99*max_r){
    return; // Quick exit
  }
  
  Eigen::Vector2d v_src = src->x().block(2,0,2,1);
  double n_v_src = v_src.norm();
  Eigen::Vector2d v_src_n;
  if (n_v_src<1e-10){
    // The funnel is static -> any direction will do
    n_v_src = 0.;
    v_src_n(0) = 1.;
    v_src_n(1) = 0.;
  }else{
    v_src_n = v_src/(n_v_src+1e-200); // Normalized
  }
  
  
  //Initial position
  size_t idx_t = (src->t().array()-t_src).array().abs().minCoeff();
  // Branching point of new funnels
  Eigen::Vector2d pos_t = src->x().block(0,idx_t,2,1);
  
  // Create all possible successors with min_ang_diff
  Eigen::Vector2d new_vel_n, new_vel;
  Eigen::Vector4d x0;
  double n_new_vel, tgt_radius;
  double d_alpha = -M_PI;
  while (d_alpha <= M_PI-min_ang_diff/2){
    
    // Desired new velocity direction
    new_vel_n = getR(d_alpha)*v_src_n;
    
    // Loop over all allowed velocity norms
    for (double delta_new_vel = -2.*max_vel; delta_new_vel<=2.*max_vel;
        delta_new_vel+=min_vel_diff){
      n_new_vel = n_v_src + delta_new_vel;
      if(n_new_vel<-max_vel || n_new_vel > max_vel){
        // New velocity norm is forbidden
        continue;
      }
      if ((std::abs(n_new_vel)<1e-10) && (d_alpha != -M_PI/2) ){
        // No need to add many static funnels with "different"
        // directions
        continue;
      }
      tgt_radius = get_smallest_englobing(r_src, std::abs(delta_new_vel));
      if (tgt_radius>max_r){
        // Minimal funnel size needed to englobe is larger then
        // the maximal size
        continue;
      }
      // Now we have a new parent funnel candidate
      x0.block(0,0,2,1) = pos_t;
      // todo refactor norm vs normalized
      x0.block(2,0,2,1) = n_new_vel*new_vel_n;
      // Parent
      fun_parent = get_this_funnel(src, tgt_radius, x0, n_new_vel*new_vel_n);
      if (fun_parent == nullptr){
        continue;
      }else{
        // add
        fun_sys.add_funnel(fun_parent, true);
        // all_children
        fun_vec_child = get_children(fun_parent);
        for (auto a_f : fun_vec_child){
          fun_sys.add_funnel(a_f);
        }
      }
    } // Done velocity
    d_alpha += min_ang_diff; // Update angles
  }// Done angles
  // All calculated
  return;
}

//template<class FUN_SYS, class GRAPH>
//void iteration_from_graph( FUN_SYS)

#endif //FUNNELS_CPP_EX1_HEU_HH
