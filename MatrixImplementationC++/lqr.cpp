#include "lqr.hpp"

bool LQR::evaluate_convergence(Matrix S, Matrix S_old){
    Matrix diff = S - S_old;
    int dim[2];
    S.getDim(dim);
    double max_variation = 0;
    double current = 0;

    for(int i=0;i<dim[0];i++){
        for(int j=0; j<dim[1];j++){
            current = diff.get(i,j);
            if(current < 0)
                current = -current;
            if(max_variation < current)
                max_variation = current;
        }
    }

    if(max_variation < THRESHOLD)
        return true;
    else
        return false;
}

Matrix LQR::controllability(Matrix Phi, Matrix Gamma){
    int p = Phi.num_rows-1;
    Matrix G = Gamma;
    for(int i=1; i<=p; i++){
        G = G.concatRows((Phi^p)*Gamma);
    }
    G.setName("G");
    return G;
}

Matrix LQR::lqr(Matrix Phi, Matrix Gamma, Matrix Q1, Matrix Q2){
    // Compute the LQR gain for a state feedback controller
    // Q1 penalizes the state x(k), n x n
    // Q2 penalizes the input u(k)

    Matrix S = Q1;
    Matrix S_old = Q1*10;
    Matrix R;
    Matrix M;
    Matrix K;

    while(!evaluate_convergence(S,S_old)){
        R = Q2 + Gamma.trans()*S*Gamma;
        M = S - S*Gamma*R.inv()*Gamma.trans()*S;
        S_old = S;
        S = Phi.trans()*M*Phi + Q1;
    }

    K = R.inv()*Gamma.trans()*S*Phi;

    return K;
}

Matrix LQR::observability(Matrix Phi, Matrix C){
    int p = Phi.num_rows-1;
    Matrix Q = C;
    for(int i=1; i<=p; i++){
        Q = Q.concatCols(C*(Phi^p));
    }
    Q.setName("Q");
    return Q;
}

Matrix LQR::lqr_observer(Matrix Phi, Matrix C, Matrix Q1, Matrix Q2){
    // Compute the LQR gain for an observer
    // Q1 penalizes the modelling error delta(k), n x n
    // Q2 penalizes the measurement noise y(k)

    Matrix S = Q1;
    Matrix S_old = Q1*10;
    Matrix R;
    Matrix M;
    Matrix L;

    while(!evaluate_convergence(S,S_old)){
        R = Q2 + C*S*C.trans();
        M = S - S*C.trans()*R.inv()*C*S;
        S_old = S;
        S = Phi.trans()*M*Phi + Q1;
    }

    L = Phi*S*C.trans()*R.inv();

    return L;
}
