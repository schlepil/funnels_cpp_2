//
// Created by philipp on 30.12.19.
//

#ifndef FUNNELS_CPP_FIXED_ELLIP_COMP_HH
#define FUNNELS_CPP_FIXED_ELLIP_COMP_HH

#include <iostream>

#include "funnels/comp_utils.hh"
#include "funnels/trans_types_def.hh"

template<class FUNNEL, class CLOCK>
size_t compute_converging_trans_fixed_ellip(
    FUNNEL &src, const FUNNEL &tgt, double t_step,
    const CLOCK &lcl_clk){
  
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


/////////////////////////

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

////////////////////

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



#endif //FUNNELS_CPP_FIXED_ELLIP_COMP_HH
