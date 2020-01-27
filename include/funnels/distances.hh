//
// Created by philipp on 26.12.2019.
//

#ifndef FUNNELS_CPP_DISTANCES_HH
#define FUNNELS_CPP_DISTANCES_HH

#include <cassert>
#include <vector>
#include <Eigen/Core>
#include <cmath>

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

using namespace std;
using namespace Eigen;

double wrap_so2(double x){
  return std::fmod(x, M_PI);
}

class partial_so2_dist_t{
public:
  partial_so2_dist_t(vector<size_t> so2_dims=vector<size_t>()):
      _so2_dims(move(so2_dims)){};
  
//  template <class DERIVED0, class DERIVED1>
//  MatrixXd &operator()(const MatrixBase<DERIVED0> &x0,
//      const MatrixBase<DERIVED1> &x1){
//    // Multiple possibilities here
//    // so we have to do proper broadcasting
//    assert(x0.rows()==x1.rows());
//
//    if(x0.cols()==1 && x1.cols()==1){
//      // two vectors
//      _tmpMat = x0-x1;
//    }else if(x0.cols()==1 || x1.cols()==1){
//      if (x0.cols()==1){
//        _tmpMat = (-x1).colwise() + x0;
//      }else{
//        _tmpMat = x1.colwise()-x0;
//      }
//    }else if(x0.cols()==x1.cols()){
//      _tmpMat = x0-x1;
//    }
//

//    return _tmpMat;
//  }
  
  template<class DERIVED1, class DERIVED2>
  const MatrixXd & cp_vv(const MatrixBase<DERIVED1> &v0, const MatrixBase<DERIVED2> &v1) const {
    assert(v0.cols()==1 && v1.cols());
    _tmpMat = v0-v1;
    do_so2_wrap();
    return _tmpMat;
  }
  
  template<class DERIVEDV, class DERIVEDM>
  const MatrixXd & cp_vM(const MatrixBase<DERIVEDV> &v0, const MatrixBase<DERIVEDM> &m1) const {
    _tmpMat = (-m1).colwise() + v0;
    do_so2_wrap();
    return _tmpMat;
  }
  
  template<class DERIVEDM, class DERIVEDV>
  const MatrixXd & cp_Mv(const MatrixBase<DERIVEDM> &m0, const MatrixBase<DERIVEDV> &v1) const {
    _tmpMat = m0.colwise() - v1;
    do_so2_wrap();
    return _tmpMat;
  }
  
  template<class DERIVED1, class DERIVED2>
  const MatrixXd &cp_MM(const MatrixBase<DERIVED1> &m0, const MatrixBase<DERIVED2> &m1) const {
    _tmpMat = m0 - m1;
    do_so2_wrap();
    return _tmpMat;
  }
  
  
  

protected:
  
  inline void do_so2_wrap()const{
    // Take into account the angles
    for (size_t idx : _so2_dims){
      _tmpMat.col(idx).unaryExpr(&wrap_so2);
    }
  }
  
  const vector<size_t> _so2_dims;
  mutable MatrixXd _tmpMat;
};

#endif //FUNNELS_CPP_DISTANCES_HH
