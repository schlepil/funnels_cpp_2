//
// Created by phili on 26.12.2019.
//

#define _USE_MATH_DEFINES
#include <iostream>
#include <Eigen/Dense>
#include <vector>

#include <cmath>

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

using namespace Eigen;

std::vector<double> f(MatrixXd &M, MatrixXd &V, const MatrixXd &D, double rr){
  MatrixXd tt;
  Matrix<bool, Eigen::Dynamic, 1> is_covered;
  std::vector<double> d;
  
  
  for (size_t k=0; k<10; k++){
    tt = M.colwise()-V.col(k);
    tt = D*tt;
    is_covered = tt.colwise().squaredNorm().array() <= rr;
    std::cout << is_covered << std::endl;
    for(size_t i=0; i<M.cols(); i++){
      if(is_covered(i)){
        d.push_back(M(0,i));
      }
    }
  }
  return d;
}

int main(){
  
  MatrixXd M;
  MatrixXd D;
  MatrixXd V;
  std::vector<double> blub;
  
  M = MatrixXd::Random(10,1000);
  V = MatrixXd::Random(10,10);
  D = MatrixXd::Random(10,10);
  for (double k=0; k<1000; k+=10.){
    blub = f(M,V,D, k);
    for (auto bbb : blub){
      std::cout << bbb << std::endl;
    }
  }
  return 0;
}