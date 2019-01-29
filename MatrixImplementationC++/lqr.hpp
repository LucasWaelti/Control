#ifndef LQR_HPP_INCLUDED
#define LQR_HPP_INCLUDED

#include "matrix.hpp"

namespace LQR{

    static double threshold;

    void setThreshold(double thres);
    bool evaluate_convergence(Matrix S, Matrix S_old);
    Matrix lqr(Matrix Phi, Matrix Gamma, Matrix Q1, Matrix Q2);
    Matrix lqr_observer(Matrix Phi, Matrix C, Matrix Q1, Matrix Q2);
};

#endif // LQR_HPP_INCLUDED
