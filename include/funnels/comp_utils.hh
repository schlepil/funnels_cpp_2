//
// Created by philipp on 30.12.19.
//

#ifndef FUNNELS_CPP_COMP_UTILS_HH
#define FUNNELS_CPP_COMP_UTILS_HH

#include "funnels/lyapunov.hh"

/////////////////////////////////////////////////

inline size_t find_next_high(size_t start,
                             const Eigen::Matrix<bool, Eigen::Dynamic, 1> &vec){
  
  size_t idx = start;
  while((!vec(idx)) && (idx<vec.size()-1)){
    idx++;
  }
  return idx;
}


inline size_t find_next_low(size_t start,
                            const Eigen::Matrix<bool, Eigen::Dynamic, 1> &vec){
  
  size_t idx = start;
  while(vec(idx) && (idx<vec.size()-1)){
    idx++;
  }
  return idx;
}

#endif //FUNNELS_CPP_COMP_UTILS_HH
