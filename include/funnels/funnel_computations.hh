//
// Created by phili on 25.12.2019.
//

#ifndef FUNNELS_CPP_FUNNEL_COMPUTATIONS_HH
#define FUNNELS_CPP_FUNNEL_COMPUTATIONS_HH

#include <cmath>
#include <cassert>

#include "lyapunov.hh"
#include "transitions.hh"
#include "funnels/fixed_ellip_comp.hh"

using namespace funnels;
using namespace lyapunov;

template<class FUNNEL>
size_t compute_converging_trans(FUNNEL &src, const FUNNEL& tgt,
    double t_step, const clock_ta_t &ctrl_clk, const clock_ta_t &lcl_clk){
  
  using cvx_hull_t = typename FUNNEL::cvx_hull_t;
  using lyap_t = typename cvx_hull_t::lyap_t;
  using dist_t = typename lyap_t::dist_t;
  using traj_t = typename lyap_t::traj_t;
  
  if (std::is_same<lyap_t, lyapunov::fixed_ellipsoidal_lyap_t<traj_t , dist_t>>::value){
    return compute_converging_trans_fixed_ellip(src, tgt, t_step, lcl_clk);
  }else{
    throw std::runtime_error("Converging not implemented");
  }
}

///////////////

template <class FUNNEL>
size_t compute_inclusion_trans_fixed_ellip(
    FUNNEL &src, const FUNNEL &tgt, double t_step,
    switching_trans_info_t & info) {
  
  // note : no need for a second clock, switching only allowed within the
  // same funnel system -> same ctrl_clk
  // Fill;
  switching_trans_info_raw_t info_raw =
      compute_covered_times_fixed_ellip(src, tgt);
  info.set_values(info_raw);
  
  return src._trans_abs->operator()(src._edges, t_step, info); // Done
}

template<class FUNNEL>
size_t compute_inclusion_trans(FUNNEL &src, const FUNNEL& tgt,
     double t_step, switching_trans_info_t &info){
  
  using cvx_hull_t = typename FUNNEL::cvx_hull_t;
  using lyap_t = typename cvx_hull_t::lyap_t;
  using dist_t = typename lyap_t ::dist_t;
  using traj_t = typename lyap_t ::traj_t;
  
  if (std::is_same<lyap_t, lyapunov::fixed_ellipsoidal_lyap_t<traj_t, dist_t>>::value){
    // Check if the inner bounding box of the src intersect with the
    // outer bounding box of the target
    if (src.cvx_hull().intersect_in_out(tgt.cvx_hull())) {
      return compute_inclusion_trans_fixed_ellip(src, tgt, t_step, info);
    }
  }else{
    throw std::runtime_error("Converging not implemented");
  }
  return 0; // CVX_HULLS did not meet criterion
}

template<class FUNNEL>
size_t compute_inclusion_trans_1(FUNNEL &src, const FUNNEL& tgt,
    double t_step, const clock_ta_t &ctrl_clk, const clock_ta_t &lcl_clk){
  
  switching_trans_info_t info(src.loc(), tgt.loc(),
      ctrl_clk, lcl_clk, utils_ext::event_map["no_action"]);
  
  return compute_inclusion_trans(src, tgt, t_step, info);
}

template<class FUNNEL>
size_t compute_inclusion_trans_2(FUNNEL &src, const FUNNEL& tgt,
    double t_step, const clock_ta_t &ctrl_clk, const clock_ta_t &lcl_clk,
    const location_t &src_loc, const location_t &tgt_loc,
    const event_t &evt){

  switching_trans_info_t info(src_loc, tgt_loc,
      ctrl_clk, lcl_clk, evt);
  
  return compute_inclusion_trans(src, tgt, t_step, info);
}

/////////////////////////////////////////////////////
// intersection

template <class FUNNEL>
size_t compute_intersecting_trans_fixed_ellip(
    FUNNEL &src, const FUNNEL &tgt, double t_step,
    intersect_trans_info_t &info) {
  
  // Fill
  auto info_raw = compute_intersecting_times_fixed_ellip(src, tgt);
  info.set_values(info_raw);
  
  // Set
  return src._trans_abs->operator()(src._edges, t_step, info); // Done
}


template<class FUNNEL>
size_t compute_intersecting_trans(FUNNEL &src, const FUNNEL& tgt,
    double t_step, intersect_trans_info_t &info){
  using cvx_hull_t = typename FUNNEL::cvx_hull_t;
  using lyap_t = typename cvx_hull_t::lyap_t;
  using dist_t = typename lyap_t ::dist_t;
  using traj_t = typename lyap_t ::traj_t;
  
  if (std::is_same<lyap_t, lyapunov::fixed_ellipsoidal_lyap_t<traj_t, dist_t>>::value){
    if (src.cvx_hull().intersect_out_out(tgt.cvx_hull())) {
      return compute_intersecting_trans_fixed_ellip(src, tgt, t_step, info);
    }
  }else{
    throw std::runtime_error("Converging not implemented");
  }
  return 0; // CVX_HULLS did not intersect
}


template<class FUNNEL>
size_t compute_intersecting_trans(FUNNEL &src, const FUNNEL& tgt,
                                  double t_step, const clock_ta_t &ctrl_clk_src, const clock_ta_t
                                  &ctrl_clk_tgt, const event_t & evt){
  
  intersect_trans_info_t info(src.loc(), tgt.loc(), ctrl_clk_src, ctrl_clk_tgt,
      evt);
  
  return compute_intersecting_trans(src, tgt, t_step, info);
}

template<class FUNNEL>
size_t compute_intersecting_trans(FUNNEL &src, const FUNNEL &tgt,
    double t_step, const clock_ta_t &ctrl_clk_src,
    const clock_ta_t &ctrl_clk_tgt, const location_t &src_loc,
    const location_t &tgt_loc, const event_t &evt){
  
  intersect_trans_info_t info(src_loc, tgt_loc, ctrl_clk_src, ctrl_clk_tgt,
                              evt);
  
  return compute_intersecting_trans(src, tgt, t_step, info);
}

// More basic computations
template <class FUNNEL>
std::vector<std::pair<bool, std::pair<double, double>>> compute_outer_col_times_fixed_ellip(
    const FUNNEL &sys, const FUNNEL &obs, const std::vector<size_t> & obs_idx) {


// The source funnel might intersect with tgt funnel[zeta] between
// alpha and beta
// Loop over each time-point in the source trajectory and
// compute the corresponding target interval
// todo find a way to minimize the number of transitions
  using fun_t = FUNNEL;
  using matrix_t = typename fun_t::matrix_t;
  using vector_t_t = typename fun_t::vector_t_t;
  
  assert(obs_idx.end()<=obs.t().size());
  
  const vector_t_t &t_sys = *sys.t_ptr();
  const vector_t_t &t_obs = *obs.t_ptr();
  const matrix_t &x_sys = *sys.x_ptr();
  const matrix_t &x_obs = *obs.x_ptr();
  Eigen::Matrix<bool, Eigen::Dynamic, 1> is_intersecting;
  
  double t_alpha_obs=0., t_beta_obs=0.;
  size_t lower_ind, upper_ind;
  bool collided=false;
  bool in_interval = false;

// Distance needs to be larger 1.+radius
// to guarantee non-intersection
// todo check if ok if obstacle has "infinite" velocity size
  double proj_min_dist = 1. + lyapunov::projected_max_radius(obs.C(), sys.C());
  proj_min_dist *= proj_min_dist; // Avoid sqrt
// Todo: shortcut considering the bounding box

  // Scan for first intersecting point between each obstacle index pair
  std::vector<std::pair<bool, std::pair<double, double>>> res_vec;
  
  for (size_t k=0; k<obs_idx.size()-1;k++) {
    t_alpha_obs=0., t_beta_obs=0.;
    lower_ind = obs_idx[k];
    upper_ind = obs_idx[k+1];
    for (size_t i=lower_ind; i<upper_ind; i++) {
      is_intersecting =
          (obs.C() * (obs.dist().cp_Mv(x_sys, VectorXd(x_obs.col(i)))))
              .colwise().squaredNorm().array() < proj_min_dist;
      if (is_intersecting.any()) {
        // Found the first colliding time point
        t_alpha_obs = t_obs(i);
        collided = true;
        break;
      }
    }
    if (!collided) {
      // collision free
      res_vec.push_back(std::pair(false, std::pair(0., 0.)));
      continue;
    }
    // Scan for last intersecting point
    for (size_t i = upper_ind-1; i >= lower_ind; i--) {
      is_intersecting =
          (obs.C() * (obs.dist().cp_Mv(x_sys, VectorXd(x_obs.col(i)))))
              .colwise().squaredNorm().array() < proj_min_dist;
      if (is_intersecting.any()) {
        // Found the first colliding time point
        t_beta_obs = t_obs(i);
        res_vec.push_back(std::pair(true, std::pair(t_alpha_obs,t_beta_obs)));
        break;
      }
    }
  }
  return res_vec;
}

template <class FUNNEL>
std::vector<std::pair<bool, std::pair<double, double>>> compute_outer_col_times_fixed_ellip(
    const FUNNEL &sys, const FUNNEL &obs) {
  
}



/*!
 * Computes the minimal and maximal time the obstacle intersects with the system
 * @tparam FUNNEL
 * @param src
 * @param tgt
 * @return
 */
template <class FUNNEL>
std::pair<bool, std::pair<double, double>> compute_outer_col_times(
    const FUNNEL &sys, const FUNNEL &obs){
  using lyap_t = typename FUNNEL::lyap_t;
  using traj_t = typename lyap_t::traj_t;
  using dist_t = typename lyap_t::dist_t;
  
  if (std::is_same<lyap_t, lyapunov::fixed_ellipsoidal_lyap_t<traj_t, dist_t>>::value){
    // If bounding outer bounding boxes do not intersect ->
    // No collision possible
    if (sys.cvx_hull().intersect_out_out(obs.cvx_hull())) {
      return compute_outer_col_times_fixed_ellip(sys, obs);
    }
  }else{
    throw std::runtime_error("Converging not implemented");
  }
  return {false, {0.,0.}}; // CVX_HULLS did not meet criterion
}


#endif //FUNNELS_CPP_FUNNEL_COMPUTATIONS_HH