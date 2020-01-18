set(UTIL_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/utils.cc"
        "${FUNNEL_INC_DIR}/funnels/utils.hh"
        PARENT_SCOPE)

set(TRANSITION_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/transitions.cc"
        "${FUNNEL_INC_DIR}/funnels/transitions.hh"
        PARENT_SCOPE)

set(FUNNEL_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/lyapunov.cc"
        "${FUNNEL_INC_DIR}/funnels/lyapunov.hh"
        "${CMAKE_CURRENT_SOURCE_DIR}/funnels.cc"
        "${FUNNEL_INC_DIR}/funnels/funnels.hh"
        "${FUNNEL_INC_DIR}/funnels/funnel_sys.hh"
        "${FUNNEL_INC_DIR}/funnels/funnel_computations.hh"
        "${FUNNEL_INC_DIR}/funnels/distances.hh"
        "${FUNNEL_INC_DIR}/funnels/collisions.hh"
        PARENT_SCOPE)

set(TA_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/ta.cc"
        "${FUNNEL_INC_DIR}/aut/ta.hh"
        "${CMAKE_CURRENT_SOURCE_DIR}/aut_survival.cc"
        "${FUNNEL_INC_DIR}/aut/aut_survival.hh"
        PARENT_SCOPE)

set(HELPER_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/bounding_box.cc"
        "${FUNNEL_INC_DIR}/helpers/bounding_box.hh"
        "${CMAKE_CURRENT_SOURCE_DIR}/trajectory.cc"
        "${FUNNEL_INC_DIR}/helpers/trajectory.hh"
        "${FUNNEL_INC_DIR}/funnels/dynamics.hh"
        "${CMAKE_CURRENT_SOURCE_DIR}/dynamics.cc"
        PARENT_SCOPE)

set(EX1_ADD_FILES_HEU
        "${CMAKE_CURRENT_SOURCE_DIR}/ex1_heu.cc"
        "${FUNNEL_INC_DIR}/heuristics/ex1_heu.hh"
        PARENT_SCOPE)

set(EX1_ADD_FILES_FIX
        "${CMAKE_CURRENT_SOURCE_DIR}/ex1_fixed.cc"
        "${FUNNEL_INC_DIR}/heuristics/ex1_fixed.hh"
        PARENT_SCOPE)
