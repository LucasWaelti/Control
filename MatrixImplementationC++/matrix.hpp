#ifndef MATRIX_HPP_INCLUDED
#define MATRIX_HPP_INCLUDED

#include <iostream>
#include <vector>


class Matrix{
    public:

    std::string name;
    unsigned int num_rows;
    unsigned int num_cols;
    std::vector< std::vector<double> > M;

    void setName(std::string name);
    void buildMatrix(std::string name);
    void buildMatrix(std::string name,unsigned int rows,unsigned int cols,const double* values);
    void getDim(int* dim);
    double get(int row, int col);
    void setValue(int row, int col, double value);
    void eye(unsigned int rows, unsigned int cols);
    void zeros(unsigned int rows, unsigned int cols);
    void displayMatrix();
    int Rank();
    Matrix inv();
    Matrix trans();
    Matrix concatRows(Matrix R);
    Matrix concatCols(Matrix C);

    // Operator overloads
    Matrix operator+(const Matrix& m);
    Matrix operator-(const Matrix& m);
    Matrix operator*(const Matrix& m);
    Matrix operator*(const double& d);
    Matrix operator/(const double& d);
    Matrix operator^(const int&    p);
};

#endif // MATRIX_HPP_INCLUDED
