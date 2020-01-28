//
// Created by philipp on 30.12.19.
//

#ifndef FUNNELS_CPP_FIXED_ELLIP_COMP_HH
#define FUNNELS_CPP_FIXED_ELLIP_COMP_HH

#include <iostream>

#include "funnels/lyapunov.hh"

#include "funnels/comp_utils.hh"
#include "funnels/trans_types_def.hh"

using namespace funnels;
using namespace lyapunov;

/////////////////////////////////////////
// Converging transitions

//template<class FUNNEL, class CLOCK>
//compute_converging_trans_fixed_ellip(
template<template<class> class FUNNEL, template<class> class CVX_HULL,
template<class,class> class LYAP, class TRAJ, class DIST, class CLOCK>
typename std::enable_if<
    std::is_same<FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>>,
    FUNNEL<CVX_HULL<lyapunov::fixed_ellipsoidal_lyap_t<TRAJ,DIST>>> >::value,
    size_t>::type
compute_converging_trans(
    FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>> &src,
    const FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>> &tgt, double t_step,
    const CLOCK &ctrl_clk, const CLOCK &lcl_clk){
  
  // ctrl_clk is dummy here
  
  assert((src.gamma()>=0.) && (tgt.gamma()>=0.) && "Fixed size funnels "
                                                   "need to converge");
  
  // Only one transition necessary, convergence time is independent of
  // the trajectory and the ctrl_clk
  
  // Get projected radius
  double r_proj = lyapunov::projected_max_radius(src.C(), tgt.C());
  if (r_proj>=1.0){
    return 0; // No sense to switch to larger
  }
  
  double t_min = 1./src.gamma()*std::log(1./r_proj);
  
  // Create a new corresponding to
  // lcl_clk>=t_min; lcl_clk:=0
  // todo: is_exact hard-coded to true
  src._edges.push_back(converging_transition(src.loc(), tgt.loc(),
                                             utils_ext::event_map["no_action"],
                                             lcl_clk, std::ceil(t_min*t_step), true));
  
  return 1;
}

////////////////////////////////////////////
// Switching transitions

template <class FUNNEL>
switching_trans_info_raw_t
compute_covered_times_fixed_ellip(FUNNEL &src, const FUNNEL &tgt){
  
  // Loop over each time-point in the source trajectory and
  // compute the corresponding target interval
  // todo find a way to minimize the number of transitions
  using fun_t = FUNNEL;
  using matrix_t = typename fun_t::matrix_t;
  using vector_t_t = typename fun_t::vector_t_t;
  
  const vector_t_t &t_src = *src.t_ptr();
  const vector_t_t &t_tgt = *tgt.t_ptr();
  const matrix_t &x_src = *src.x_ptr();
  const matrix_t &x_tgt = *tgt.x_ptr();
  Eigen::Matrix<bool, Eigen::Dynamic, 1> is_covered;
  
  size_t n_alpha, n_beta, n_zeta;
  bool in_interval_src=false;
  bool is_assigned;

//  Eigen::MatrixXd _tt;
  
  switching_trans_info_raw_t all_trans_raw;
  
  d_vec_t &a_vec = all_trans_raw.a_vec;
  d_vec_t &b_vec = all_trans_raw.b_vec;
  d_vec_t &z_vec = all_trans_raw.z_vec;
  
  // Regularity assumption
  // Each incoming time-point has exactly one
  // outgoing interval
  blck_idx_t &blck_idx = all_trans_raw.blck_idx;
  
  double proj_max_dist = 1.-lyapunov::projected_max_radius(tgt.C(), src.C());
  if (proj_max_dist<0.){
    // Short-cut : (projected) source funnel large than tgt funnel
    return all_trans_raw;
  }
  proj_max_dist *= proj_max_dist; //Avoid sqrt in computations
  
  
  // Scan over all tgt path for each point in src
  for (size_t i_in=0; i_in<t_tgt.size(); i_in++){
    n_zeta = i_in;
    is_assigned = false;
//    _tt = tgt.dist().cp_Mv(x_src,VectorXd(x_tgt.col(i_in)));
//    _tt = x_src.colwise() - x_tgt.col(i_in);
//    _tt = (tgt.C()*(_tt));
//    is_covered = _tt.colwise().squaredNorm().array() <= proj_max_dist;
    
    is_covered = (tgt.C()*(tgt.dist().cp_Mv(x_src,VectorXd(x_tgt.col(i_in)))))
                     .colwise().squaredNorm().array() <= proj_max_dist;
    
    n_alpha=0; n_beta=0;
    if (is_covered(0)){
      in_interval_src=true;
//      blck_idx.emplace_back(0, 0);
    }else{
      in_interval_src = false;
    }
    while(true){
      if(in_interval_src){
        // Find end of interval
        n_beta = find_next_low(n_alpha, is_covered);
        // Save the time points
        a_vec.push_back(t_src(n_alpha));
        b_vec.push_back(t_src(n_beta));
        z_vec.push_back(t_tgt(n_zeta));
        // Done with this interval
        in_interval_src = false;
        if (is_assigned){
          throw std::runtime_error("Violated regularity");
        }
        is_assigned = true;
        // Check if this is within the same block
        // This means we have to check if the src intervals
        // overlap, if not this transition starts a new block
        if (b_vec.size()>=2 &&
            (b_vec[b_vec.size() - 2] >
             a_vec[a_vec.size() - 1])){
          assert(!blck_idx.empty());
          blck_idx.back().second = a_vec.size()-1;
        }else{
          blck_idx.emplace_back(a_vec.size()-1, a_vec.size()-1);
        }
      }else{
        n_alpha = find_next_high(n_beta, is_covered);
        in_interval_src = true;
      }
      // end this loop through src
      if((n_alpha==is_covered.size()-1) || (n_beta==is_covered.size()-1)){
        break;
      }
    }
  } // End this tgt

#ifndef NDEBUG
  assert(a_vec.size()==b_vec.size() && a_vec.size()==z_vec.size());
  for (size_t k=0; k<a_vec.size();k++){
    assert(a_vec[k]<=b_vec[k]);
    assert(a_vec[k]>=0. && b_vec[k]>=0. && z_vec[k]>=0.);
  }
  for(const auto &a_pair : blck_idx){
    assert(a_pair.first>=0 && a_pair.first<=a_pair.second &&
        a_pair.second<a_vec.size());
  }

#endif
  
  return all_trans_raw;
}

template<template<class> class FUNNEL, template<class> class CVX_HULL,
    template<class,class> class LYAP, class TRAJ, class DIST>
typename std::enable_if<
    std::is_same<FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>>,
        FUNNEL<CVX_HULL<lyapunov::fixed_ellipsoidal_lyap_t<TRAJ,DIST>>> >::value,
    size_t>::type
compute_inclusion_trans(FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>> &src,
    const FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>>& tgt,
    double t_step, switching_trans_info_t &info) {
  
  // Check if the inner bounding box of the src intersect with the
  // outer bounding box of the target
    if (!src.cvx_hull().intersect_in_out(tgt.cvx_hull())) {
      return 0;
    }
  
  // note : no need for a second clock, switching only allowed within the
  // same funnel system -> same ctrl_clk
  // Fill;
  switching_trans_info_raw_t info_raw =
      compute_covered_times_fixed_ellip(src, tgt);
  info.set_values(info_raw);
  
  return src._trans_abs->operator()(src._edges, t_step, info); // Done
}

template<template<class> class FUNNEL, template<class> class CVX_HULL,
    template<class,class> class LYAP, class TRAJ, class DIST>
typename std::enable_if<
    std::is_same<FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>>,
        FUNNEL<CVX_HULL<lyapunov::fixed_ellipsoidal_lyap_t<TRAJ,DIST>>> >::value,
    size_t>::type
compute_inclusion_trans(FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>> &src,
    const FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>>& tgt,
    double t_step, const clock_ta_t &ctrl_clk, const clock_ta_t &lcl_clk){
  
  switching_trans_info_t info(src.loc(), tgt.loc(),
                              ctrl_clk, lcl_clk, utils_ext::event_map["no_action"]);
  
  return compute_inclusion_trans(src, tgt, t_step, info);
}

template<template<class> class FUNNEL, template<class> class CVX_HULL,
    template<class,class> class LYAP, class TRAJ, class DIST>
typename std::enable_if<
    std::is_same<FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>>,
        FUNNEL<CVX_HULL<lyapunov::fixed_ellipsoidal_lyap_t<TRAJ,DIST>>> >::value,
    size_t>::type
compute_inclusion_trans(FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>> &src,
    const FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>>& tgt,
    double t_step, const clock_ta_t &ctrl_clk, const clock_ta_t &lcl_clk,
    const location_t &src_loc, const location_t &tgt_loc,
    const event_t &evt){
  
  switching_trans_info_t info(src_loc, tgt_loc,
                              ctrl_clk, lcl_clk, evt);
  
  return compute_inclusion_trans(src, tgt, t_step, info);
}

////////////////////////////////////////////
// Intersecting transitions

template <class FUNNEL>
intersect_trans_info_raw_t
  compute_intersecting_times_fixed_ellip(FUNNEL &src, const FUNNEL &tgt){
    
  // The source funnel might intersect with tgt funnel[zeta] between
  // alpha and beta
  // Loop over each time-point in the source trajectory and
  // compute the corresponding target interval
  // todo find a way to minimize the number of transitions
  using fun_t = FUNNEL;
  using matrix_t = typename fun_t::matrix_t;
  using vector_t_t = typename fun_t::vector_t_t;
  
  const vector_t_t &t_src = *src.t_ptr();
  const vector_t_t &t_tgt = *tgt.t_ptr();
  const matrix_t &x_src = *src.x_ptr();
  const matrix_t &x_tgt = *tgt.x_ptr();
  Eigen::Matrix<bool, Eigen::Dynamic, 1> is_intersecting;
  
  size_t n_alpha, n_beta, n_zeta;
  bool in_interval=false;
  
  intersect_trans_info_raw_t all_trans_raw;
  
  d_vec_t &a_vec = all_trans_raw.a_vec;
  d_vec_t &b_vec = all_trans_raw.b_vec;
  d_vec_t &z_vec = all_trans_raw.z_vec;
  // Regularity assumptation
  // Each incoming time-point has exactly one
  // outgoing interval
  // todo
//  blck_idx_t &blck_idx = all_trans_raw.blck_idx;
  
  // Distance needs to be larger 1.+radius
  // to guarantee non-intersection
  // todo check if ok if obstacle has "infinite" velocity size
  double proj_min_dist = 1.+lyapunov::projected_max_radius(tgt.C(), src.C());
  proj_min_dist *= proj_min_dist; // Avoid sqrt
  // Todo: shortcut considering the bounding box
  
  // Scan over all tgt path for each point in src
  for (size_t i_in=0; i_in<t_tgt.size(); i_in++){
    n_zeta = i_in;
    is_intersecting = (tgt.C()*(tgt.dist().cp_Mv(x_src,VectorXd(x_tgt.col(i_in)))))
                     .colwise().squaredNorm().array() < proj_min_dist;
    
    n_alpha=0; n_beta=0;
    if (is_intersecting(0)){
      in_interval=true;
    }else{
      in_interval = false;
    }
    while(true){
      if(in_interval){
        // Find end of interval
        n_beta = find_next_low(n_alpha, is_intersecting);
        // Save the time points
        a_vec.push_back(t_src(n_alpha));
        b_vec.push_back(t_src(n_beta));
        z_vec.push_back(t_tgt(n_zeta));
        // Done with this interval
        in_interval = false;
      }else{
        n_alpha = find_next_high(n_beta, is_intersecting);
        in_interval = true;
      }
      // end this loop through src
      if((n_alpha==is_intersecting.size()-1) || (n_beta==is_intersecting.size()-1)){
        break;
      }
    }
  } // End this tgt
#ifndef NDEBUG
  assert(a_vec.size()==b_vec.size() && a_vec.size()==z_vec.size());
  for (size_t k=0; k<a_vec.size();k++){
    assert(a_vec[k]<=b_vec[k]);
    assert(a_vec[k]>=0. && b_vec[k]>=0. && z_vec[k]>=0.);
  }
#endif
  return all_trans_raw;
}

template<template<class> class FUNNEL, template<class> class CVX_HULL,
    template<class,class> class LYAP, class TRAJ, class DIST>
typename std::enable_if<
    std::is_same<FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>>,
        FUNNEL<CVX_HULL<lyapunov::fixed_ellipsoidal_lyap_t<TRAJ,DIST>>> >::value,
    size_t>::type
compute_intersecting_trans(FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>> &src,
    const FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>> &tgt, double t_step,
    intersect_trans_info_t &info) {
  
  // Fill
  auto info_raw = compute_intersecting_times_fixed_ellip(src, tgt);
  info.set_values(info_raw);
  
  // Set
  return src._trans_abs->operator()(src._edges, t_step, info); // Done
}

template<template<class> class FUNNEL, template<class> class CVX_HULL,
    template<class,class> class LYAP, class TRAJ, class DIST>
typename std::enable_if<
    std::is_same<FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>>,
        FUNNEL<CVX_HULL<lyapunov::fixed_ellipsoidal_lyap_t<TRAJ,DIST>>> >::value,
    size_t>::type
compute_intersecting_trans(FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>> &src,
    const FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>>& tgt,
    double t_step, const clock_ta_t &ctrl_clk_src, const clock_ta_t
    &ctrl_clk_tgt, const event_t & evt){
  
  intersect_trans_info_t info(src.loc(), tgt.loc(), ctrl_clk_src, ctrl_clk_tgt,
                              evt);
  
  return compute_intersecting_trans(src, tgt, t_step, info);
}

template<template<class> class FUNNEL, template<class> class CVX_HULL,
    template<class,class> class LYAP, class TRAJ, class DIST>
typename std::enable_if<
    std::is_same<FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>>,
        FUNNEL<CVX_HULL<lyapunov::fixed_ellipsoidal_lyap_t<TRAJ,DIST>>> >::value,
    size_t>::type
compute_intersecting_trans(FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>> &src,
    const FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>> &tgt,
    double t_step, const clock_ta_t &ctrl_clk_src,
    const clock_ta_t &ctrl_clk_tgt, const location_t &src_loc,
    const location_t &tgt_loc, const event_t &evt){
  
  intersect_trans_info_t info(src_loc, tgt_loc, ctrl_clk_src, ctrl_clk_tgt,
                              evt);
  
  return compute_intersecting_trans(src, tgt, t_step, info);
}

////////////////////////////////////////////////////////
// Collision computations

template<template<class> class FUNNEL, template<class> class CVX_HULL,
    template<class,class> class LYAP, class TRAJ, class DIST>
typename std::enable_if<
    std::is_same<FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>>,
        FUNNEL<CVX_HULL<lyapunov::fixed_ellipsoidal_lyap_t<TRAJ,DIST>>> >::value,
    col_struct_vec_t>::type
compute_outer_col_times(const FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>> &sys,
    const FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>> &obs,
    const idx_vec_t &obs_idx) {


// The source funnel might intersect with tgt funnel[zeta] between
// alpha and beta
// Loop over each time-point in the source trajectory and
// compute the corresponding target interval
// todo find a way to minimize the number of transitions
  using fun_t = FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>>;
  using matrix_t = typename fun_t::matrix_t;
  using vector_t_t = typename fun_t::vector_t_t;
  
  assert(obs_idx.back()<=obs.size());
  
  // Before doing anything else, verify that the convexhulls intersect
  if (!sys.cvx_hull().intersect_out_out(obs.cvx_hull())){
    return {{false,{0.,0.}}};
  }
  
  const vector_t_t &t_sys = *sys.t_ptr();
  const vector_t_t &t_obs = *obs.t_ptr();
  const matrix_t &x_sys = *sys.x_ptr();
  const matrix_t &x_obs = *obs.x_ptr();
  Eigen::Matrix<bool, Eigen::Dynamic, 1> is_intersecting;
  
  double t_alpha_obs=0., t_beta_obs=0.;
  size_t lower_ind, upper_ind;
  bool collided=false;
  bool in_interval = false;
  bool continuation_forward, continuation_backward;

// Distance needs to be larger 1.+radius
// to guarantee non-intersection
// todo check if ok if obstacle has "infinite" velocity size
  double proj_min_dist = 1.+lyapunov::projected_max_radius(obs.C(), sys.C());
  proj_min_dist *= proj_min_dist; // Avoid sqrt
// Todo: shortcut considering the bounding box
  
  // Scan for first intersecting point between each obstacle index pair
  std::vector<std::pair<bool, std::pair<double, double>>> res_vec;
  
  continuation_forward = false;
  continuation_backward = false;
  for (size_t k=0; k<obs_idx.size()-1;k++) {
    t_alpha_obs=0., t_beta_obs=0.;
    lower_ind = obs_idx[k];
    upper_ind = obs_idx[k+1];
    
    for (size_t i=lower_ind; i<upper_ind; i++) {
      is_intersecting =
          (obs.C() * (obs.dist().cp_Mv(x_sys, x_obs.col(i))))
              .colwise().squaredNorm().array() < proj_min_dist;
      if (is_intersecting.any()) {
        // Found the first colliding time point
        // Set the new only if it is not a continuation
        t_alpha_obs = t_obs(i);
        if (i==lower_ind){
          continuation_backward = true;
        }else{
          continuation_backward = false;
        }
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
          (obs.C() * (obs.dist().cp_Mv(x_sys, x_obs.col(i))))
              .colwise().squaredNorm().array() < proj_min_dist;
      if (is_intersecting.any()) {
        // Found the first colliding time point
        t_beta_obs = t_obs(i);
        if (continuation_forward && continuation_backward){
          assert(!res_vec.empty());
          // Update end
          res_vec.back().second.second = t_beta_obs;
        }else{
          // New interval
          res_vec.push_back(std::pair(true, std::pair(t_alpha_obs,t_beta_obs)));
        }
        if (i == upper_ind-1){
          continuation_forward = true;
        }else{
          continuation_forward = false;
        }
        break;
      }
    }
  }
  return res_vec;
}

template<template<class> class FUNNEL, template<class> class CVX_HULL,
    template<class,class> class LYAP, class TRAJ, class DIST>
typename std::enable_if<
    std::is_same<FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>>,
        FUNNEL<CVX_HULL<lyapunov::fixed_ellipsoidal_lyap_t<TRAJ,DIST>>> >::value,
    col_struct_t>::type
compute_outer_col_times(const FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>> &sys,
    const FUNNEL<CVX_HULL<LYAP<TRAJ,DIST>>> &obs) {
  // Compute the outer-most collision pair
  return compute_outer_col_times(sys, obs, {0,obs.size()})[0];
}




#endif //FUNNELS_CPP_FIXED_ELLIP_COMP_HH
