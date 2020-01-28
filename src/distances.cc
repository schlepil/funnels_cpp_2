//
// Created by philipp on 1/28/20.
//

#include "funnels/distances.hh"

double wrap_so2(double x){
  return std::fmod(x, M_PI);
}