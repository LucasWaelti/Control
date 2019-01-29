#include <iostream>

#include "matrix.hpp"

int main()
{
    std::cout << "Matrix Laboratory!" << std::endl;

    // Build a matrix by hand
    Matrix A;
    const double values[9] = {1,2,2,
                              3,1,1,
                              2,2,1};
    A.buildMatrix("A",3,3,values);
    A.displayMatrix();

    // Store its inverse
    Matrix B = A.inv();
    B.setName("B");
    B.displayMatrix();

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

    // Create a vector
    Matrix v;
    const double v_values[3] = {1,
                                2,
                                3};
    v.buildMatrix("v",3,1,v_values);
    v.displayMatrix();

    // Solve the system A*x = v <-> x = inv(A)*v
    Matrix x = A.inv()*v;
    x.setName("x");
    x.displayMatrix();
    return 0;
}
