cmake_minimum_required(VERSION 3.10)
project(funnels_cpp CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED)
find_package(OpenMP REQUIRED)

set(FUNNEL_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/eigen")

include_directories(${FUNNEL_INC_DIR})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/tchecker_ext/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/tchecker_ext/tchecker/include")

#add_subdirectory(json)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/tchecker_ext")


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -O3 -march=native -mtune=native")


add_library(libutil STATIC ${UTIL_FILES})
add_library(libtransition STATIC ${TRANSITION_FILES})
target_link_libraries(libtransition PRIVATE libutil)
add_library(libfunnels STATIC ${FUNNEL_FILES})
add_library(libta STATIC ${TA_FILES})
add_library(libhelper STATIC ${HELPER_FILES})

add_library(lib_ex1_heu STATIC ${EX1_ADD_FILES_HEU})
add_library(lib_ex1_fix STATIC ${EX1_ADD_FILES_FIX})
target_link_libraries(lib_ex1_heu PRIVATE libhelper)
target_link_libraries(lib_ex1_fix PRIVATE libhelper)


add_executable(funnels_cpp main.cpp)
target_link_libraries(funnels_cpp PUBLIC libutil)
target_link_libraries(funnels_cpp PUBLIC libtransition)
target_link_libraries(funnels_cpp PUBLIC libfunnels)
target_link_libraries(funnels_cpp PUBLIC libta)
target_link_libraries(funnels_cpp PUBLIC libhelper)
target_link_libraries(funnels_cpp PUBLIC m)
target_link_libraries(funnels_cpp PUBLIC stdc++)
target_link_libraries(funnels_cpp PUBLIC OpenMP::OpenMP_CXX)

add_executable(main2 main2.cc)
target_link_libraries(main2 PUBLIC m)
target_link_libraries(main2 PUBLIC stdc++)

add_executable(funnels_ex1 example_1.cc)
target_link_libraries(funnels_ex1 PUBLIC libutil)
target_link_libraries(funnels_ex1 PUBLIC libtransition)
target_link_libraries(funnels_ex1 PUBLIC libfunnels)
target_link_libraries(funnels_ex1 PUBLIC libta)
target_link_libraries(funnels_ex1 PUBLIC libhelper)
#target_link_libraries(funnels_ex1 PUBLIC lib_ex1_heu)
target_link_libraries(funnels_ex1 PUBLIC lib_ex1_fix)
target_link_libraries(funnels_ex1 PUBLIC m)
target_link_libraries(funnels_ex1 PUBLIC stdc++)
target_link_libraries(funnels_ex1 PUBLIC OpenMP::OpenMP_CXX)

add_executable(funnels_ex1_iter example_1_iter.cc)
target_link_libraries(funnels_ex1_iter PUBLIC libutil)
target_link_libraries(funnels_ex1_iter PUBLIC libtransition)
target_link_libraries(funnels_ex1_iter PUBLIC libfunnels)
target_link_libraries(funnels_ex1_iter PUBLIC libta)
target_link_libraries(funnels_ex1_iter PUBLIC libhelper)
target_link_libraries(funnels_ex1_iter PUBLIC lib_ex1_heu)
target_link_libraries(funnels_ex1_iter PUBLIC m)
target_link_libraries(funnels_ex1_iter PUBLIC stdc++)
target_link_libraries(funnels_ex1_iter PUBLIC OpenMP::OpenMP_CXX)
target_link_libraries(funnels_ex1_iter PUBLIC libtchecker_ext_static)
