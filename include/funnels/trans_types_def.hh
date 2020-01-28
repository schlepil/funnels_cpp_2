//
// Created by philipp on 02.01.20.
//

#ifndef FUNNELS_CPP_TRANS_TYPES_DEF_HH
#define FUNNELS_CPP_TRANS_TYPES_DEF_HH

#include <vector>
#include <tuple>

#include "funnels/utils.hh"

using namespace funnels;

using blck_idx_t = std::vector<std::pair<size_t , size_t>>;

using d_vec_t = std::vector<double>;

using col_struct_t = std::pair<bool, std::pair<double, double>>;
using col_struct_vec_t = std::vector<col_struct_t>;

using idx_vec_t = std::vector<size_t>;

struct trans_info_raw_t{
public:
  
  size_t size()const{
    return a_vec.size();
  }
  
  void set_values(trans_info_raw_t &raw_info){
    // todo
//    std::swap(a_vec, raw_info.a_vec);
//    std::swap(b_vec, raw_info.b_vec);
//    std::swap(z_vec, raw_info.z_vec);
    a_vec = raw_info.a_vec;
    b_vec = raw_info.b_vec;
    z_vec = raw_info.z_vec;
  }
  
  d_vec_t a_vec, b_vec, z_vec;
};

////////////////////////////////////////
// Switching

struct switching_trans_info_raw_t: public trans_info_raw_t{
public:
  
  void set_values(switching_trans_info_raw_t &raw_info){
    trans_info_raw_t::set_values(raw_info);
//    std::swap(blck_idx, raw_info.blck_idx);
    blck_idx = raw_info.blck_idx;
  }
  
  blck_idx_t blck_idx;
  
};

struct switching_trans_info_t: public switching_trans_info_raw_t{
public:
  switching_trans_info_t(const location_t &src, const location_t &tgt,
      const clock_ta_t &ctrl_clk, const clock_ta_t &lcl_clk,
      const event_t &evt=utils_ext::event_map["no_action"]):
      _src(src), _tgt(tgt), _ctrl_clk(ctrl_clk), _lcl_clk(lcl_clk),
      _evt(evt){};
  
  void set_values(switching_trans_info_t &other){
    switching_trans_info_raw_t::set_values(other);
  }
  void set_values(switching_trans_info_raw_t &other){
    switching_trans_info_raw_t::set_values(other);
  }
  
  bool is_diag=false;
  const location_t &_src, &_tgt;
  const clock_ta_t &_ctrl_clk, &_lcl_clk;
  const event_t &_evt;
};

////////////////////////////////////////
// Intersecting

struct intersect_trans_info_raw_t: public trans_info_raw_t{
public:
  
  void set_values(intersect_trans_info_raw_t &raw_info){
    trans_info_raw_t::set_values(raw_info);
  }
};

struct intersect_trans_info_t: public intersect_trans_info_raw_t{
public:
  intersect_trans_info_t(const location_t &src, const location_t &tgt,
      const clock_ta_t &ctrl_clk_src, const clock_ta_t &ctrl_clk_tgt,
      const event_t &evt):
      _src(src), _tgt(tgt), _ctrl_clk_src(ctrl_clk_src),
      _ctrl_clk_tgt(ctrl_clk_tgt), _evt(evt){};
  
  void set_values(intersect_trans_info_t & other){
    intersect_trans_info_raw_t::set_values(other);
  }
  void set_values(intersect_trans_info_raw_t & other){
    intersect_trans_info_raw_t::set_values(other);
  }
  
  const location_t &_src, &_tgt;
  const clock_ta_t &_ctrl_clk_src, &_ctrl_clk_tgt;
  const event_t &_evt;
};
#endif //FUNNELS_CPP_TRANS_TYPES_DEF_HH
