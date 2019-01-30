#ifndef LQR_HPP_INCLUDED
#define LQR_HPP_INCLUDED

#define THRESHOLD 0.001

#include "matrix.hpp"

namespace LQR{
    bool evaluate_convergence(Matrix S, Matrix S_old);

    Matrix controllability(Matrix Phi, Matrix Gamma);
    Matrix lqr(Matrix Phi, Matrix Gamma, Matrix Q1, Matrix Q2);

    Matrix observability(Matrix Phi, Matrix C);
    Matrix lqr_observer(Matrix Phi, Matrix C, Matrix Q1, Matrix Q2);
};

#endif // LQR_HPP_INCLUDED
