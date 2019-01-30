#include <iostream>

#include "matrix.hpp"
#include "lqr.hpp"

int main()
{
    std::cout << "Matrix Laboratory!" << std::endl;

    // Build a matrix by hand
    Matrix A;
    double values[9] = {1,2,2,
                        3,1,1,
                        4,3,5};
    A.buildMatrix("A",3,3,values);
    A.displayMatrix();
    std::cout << "Rank(A) = " << A.Rank() << std::endl;

    // Store its inverse
    Matrix B = A.inv();
    B.setName("B");
    B.displayMatrix();

    Matrix AB = A.concatRows(B);
    AB.setName("AB");
    AB.displayMatrix();

    Matrix Ab = A.concatCols(B);
    Ab.setName("Ab");
    Ab.displayMatrix();
    std::cout << "Rank(Ab) = " << Ab.Rank() << std::endl;

    // Add the two precedent matrices
    Matrix C = A + B;
    C.setName("C");
    C.displayMatrix();

    // Multiply the two precedent matrices
    Matrix D = B * C;
    D.setName("D");
    D.displayMatrix();

    // Take the transpose of the last matrix
    Matrix E = D.trans();
    E.setName("E");
    E.displayMatrix();

    // Multiply the last matrix by 2
    Matrix F = E;
    F = F*2;        // Note: 2*F is not supported!
    F.setName("F");
    F.displayMatrix();

    // Make a big final computation
    Matrix G = ((A+B)*C.inv() - E)/2;
    G.setName("G");
    G.displayMatrix();

    // Get G's dimensions
    int dim[2];
    G.getDim(dim);
    std::cout << "Size of G: " << dim[0] << ", " << dim[0] << std::endl;
    // Get a single value from G
    std::cout << "Value of G at 2,1: " << G.get(2,1) << std::endl;

    // Power of a matrix
    Matrix H;
    H.eye(3,3);
    H = H*2;
    H = H^3;
    H.setName("H^3");
    H.displayMatrix();




    // Create a vector
    Matrix v;
    double v_values[3] = {1,
                                2,
                                3};
    v.buildMatrix("v",3,1,v_values);
    v.displayMatrix();

    // Solve the system A*x = v <-> x = inv(A)*v
    Matrix x = A.inv()*v;
    x.setName("x");
    x.displayMatrix();





    // Implement a LQR controller
    Matrix Phi;
    double phi[4] = {2,1,-5,4};
    Phi.buildMatrix("Phi",2,2,phi);

    Matrix Gamma;
    double gamma[2] = {0,1};
    Gamma.buildMatrix("Gamma",2,1,gamma);

    //Matrix C;
    double c[4] = {1,0,0,0};
    C.buildMatrix("C",2,2,c);

    Matrix Q1;
    double q1[4] = {1,0,0,1};
    Q1.buildMatrix("Q1",2,2,q1);

    Matrix Q2;
    double q2[1] = {1};
    Q2.buildMatrix("Q2",1,1,q2);

    // Compute controllability matrix
    G = LQR::controllability(Phi,Gamma);
    G.displayMatrix();
    std::cout << "Rank of controllability matrix G: " << G.Rank() << std::endl;

    // Compute feedback gain
    Matrix K = LQR::lqr(Phi,Gamma,Q1,Q2);
    K.setName("K");
    K.displayMatrix();

    return 0;
}
