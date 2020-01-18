//
// Created by philipp on 12/13/19.
//

#ifndef FUNNELS_CPP_TRANSITIONS_HH
#define FUNNELS_CPP_TRANSITIONS_HH

#include <string>

#include "funnels/utils.hh"
#include "funnels/trans_types_def.hh"

namespace funnels{
  
  class edge_t{
  public:
    edge_t(const location_t &src, const location_t &tgt,
        const event_t &event);
    
    [[nodiscard]] std::string declare() const;
    
    provided_t & guard();
    do_t & update();
  
  protected:
    const location_t &_src;
    const location_t &_tgt;
    const event_t &_event;
    provided_t _guard;
    do_t _update;
  };
  
  // Utility functions
  // Switching transition
  edge_t switching_transition(const location_t &src, const location_t &tgt,
      const event_t &event, const clock_ta_t &ctrl_clk, const clock_ta_t &lcl_clk,
      size_t alpha, size_t beta, size_t zeta, bool is_diag=false);
  
  // Switching transition
  edge_t intersecting_transition(const location_t &src, const location_t &tgt,
      const event_t &event, const clock_ta_t &ctrl_clk_src,
      const clock_ta_t &ctrl_clk_tgt,
      size_t alpha, size_t beta, size_t zeta);
  
  // Converging transition
  edge_t converging_transition(const location_t &src, const location_t &tgt,
                               const event_t &event, const clock_ta_t &lcl_clk,
                               size_t alpha, bool is_exact=true); // It is
                               // normally always better to converge faster
  
}

#endif //FUNNELS_CPP_TRANSITIONS_HH
