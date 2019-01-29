#include "matrix.hpp"

void Matrix::setName(std::string name){
    this->name = name;
}

void Matrix::buildMatrix(std::string name){
    this->name = name;

    // Get dimensions from the user
    std::cin >> this->num_rows;
    std::cin >> this->num_cols;

    // Resize the class' matrix
    this->zeros(this->num_rows,this->num_cols);

    for(unsigned int i=0; i<num_rows; i++){
        for(unsigned int j=0; j<num_cols; j++){
            std::cin >> M[i][j];
        }
    }

    return;
}
void Matrix::buildMatrix(std::string name, unsigned int rows, unsigned int cols, const double* values){

    this->name = name;

    // Get dimensions from the user
    this->num_rows = rows;
    this->num_cols = cols;

    // Resize the class' matrix
    this->zeros(this->num_rows,this->num_cols);

    unsigned int h = 0;
    for(unsigned int i=0; i<num_rows; i++){
        for(unsigned int j=0; j<num_cols; j++){
            this->M[i][j] = values[h];
            h++;
        }
    }

    return;
}

void Matrix::getDim(int* dim){
    dim[0] = (int)this->num_rows;
    dim[1] = (int)this->num_cols;
}

double Matrix::get(int row, int col){
    return this->M[row][col];
}

void Matrix::setValue(int row, int col, double value){
    this->M[row][col] = value;
}

void Matrix::eye(unsigned int rows, unsigned int cols){
    if(rows != cols){
        std::cout << "Warning in Matrix::eye(): matrix should be squared.\n";
    }
    this->zeros(rows,cols);
    for(unsigned int i=0; i<this->num_rows; i++){
        for(unsigned int j=0; j<this->num_cols; j++){
            if(i==j){
                this->M[i][j] = 1;
            }else{
                this->M[i][j] = 0;
            }
        }
    }
}
void Matrix::zeros(unsigned int rows, unsigned int cols){
    this->num_rows = rows;
    this->num_cols = cols;
    this->M.resize(this->num_rows);
    for(unsigned int i=0; i<this->num_rows; i++){
        this->M[i].resize(this->num_cols);
    }
}

void Matrix::displayMatrix(){
    std::cout << name << ":\n";
    if(M.size() > 0 && M[0].size() > 0){
        std::cout << "[";
        for(unsigned int i=0; i<M.size(); i++){
            for(unsigned int j=0; j<M[0].size(); j++){
                std::cout << M[i][j] << " ";
            }
            if(i != M.size()-1)
                std::cout << ";\n";
        }
        std::cout << "]\n";
    }
}

Matrix Matrix::inv(){

    Matrix result;

    if(this->num_cols != this->num_rows){
        std::cout << "Error: non square matrix does not have an inverse.\n";
        return result;
    }

    result.name = this->name + "inv";
    result.num_cols = this->num_cols;
    result.num_rows = this->num_rows;

    // Scalar case
    if(result.num_cols == 1){
        result.zeros(1,1);
        result.M[0][0] = 1/this->M[0][0];
        return result;
    }

    // Copy content of vector into a temporary one
    std::vector< std::vector<double> > m = this->M;
    std::vector< std::vector<double> > Minv;

    // Resize the inverse matrix
    Minv.resize(this->num_rows);
    for(unsigned int i=0; i<this->num_rows; i++){
        Minv[i].resize(this->num_cols);
    }
    // Set Minv as identity matrix
    for(unsigned int i=0; i<this->num_rows; i++){
        for(unsigned int j=0; j<this->num_cols; j++){
            if(i==j){
                Minv[i][j] = 1;
            }else{
                Minv[i][j] = 0;
            }
        }
    }

    // 1) Descent
    for(unsigned int i=0; i<this->num_rows; i++){
        // Get pivot of row i
        double pivot = m[i][i];
        if(pivot == 0)
            // Find a row with non zero element in column i
            for(unsigned int r=i+1;r<num_rows;r++)
                if(m[r][i] != 0){
                    for(unsigned int c=0;c<num_cols;c++){
                        m[i][c] += m[r][c];
                        Minv[i][c] += Minv[r][c];
                    }
                    pivot = m[i][i];
                    break;
                }
        // Divide the row by the pivot (M and Minv)
        for(unsigned int j=0; j<this->num_cols; j++){
            m[i][j] /= pivot;
            Minv[i][j] /= pivot;
        }
        // Cancel the value of the following rows
        double val = 0;
        for(unsigned int r=i+1;r<num_rows;r++){
            val = m[r][i];
            if(val == 0)
                continue;
            for(unsigned int c=0;c<num_cols;c++){
                m[r][c] -= m[i][c]*val;
                Minv[r][c] -= Minv[i][c]*val;
            }
        }
    }
    // 2) Ascent
    for(int i=num_rows-1; i>=0; i--){
        // Cancel the value of the upper rows
        double val = 0;
        for(int r=i-1;r>=0;r--){
            val = m[r][i];
            if(val == 0)
                continue;
            for(unsigned int c=0;c<num_cols;c++){
                m[r][c] -= m[i][c]*val;
                Minv[r][c] -= Minv[i][c]*val;
            }
        }
    }

    result.M = Minv;
    return result;
}

Matrix Matrix::trans(){

    Matrix result;

    result.name = this->name + "trans";
    result.num_cols = this->num_rows;
    result.num_rows = this->num_cols;
    result.zeros(result.num_rows,result.num_cols);

    for(unsigned int i=0;i<this->num_rows;i++){
        for(unsigned int j=0;j<this->num_cols;j++){
            result.M[j][i] = this->M[i][j];
        }
    }
    return result;
}

Matrix Matrix::operator+(const Matrix& m){

    Matrix result;

    if(this->num_cols!=m.num_cols || this->num_rows!=m.num_rows){
        std::cout << "Error: trying to add matrices with different sizes.\n";
        return result;
    }

    result.num_cols = this->num_cols;
    result.num_rows = this->num_rows;
    result.zeros(result.num_rows,result.num_cols);

    for(unsigned int i=0; i<num_rows; i++){
        for(unsigned int j=0; j<num_cols; j++){
            result.M[i][j] = this->M[i][j] + m.M[i][j];
        }
    }
    return result;
}
Matrix Matrix::operator-(const Matrix& m){

    Matrix result;

    if(this->num_cols!=m.num_cols || this->num_rows!=m.num_rows){
        std::cout << "Error: trying to subtract matrices with different sizes.\n";
        return result;
    }

    result.num_cols = this->num_cols;
    result.num_rows = this->num_rows;
    result.zeros(result.num_rows,result.num_cols);

    for(unsigned int i=0; i<num_rows; i++){
        for(unsigned int j=0; j<num_cols; j++){
            result.M[i][j] = this->M[i][j] - m.M[i][j];
        }
    }
    return result;
}
Matrix Matrix::operator*(const Matrix& m){

    Matrix result;

    if(this->num_cols != m.num_rows){
        std::cout << "Error: Incorrect dimensions for matrix multiplication.\n";
        return result;
    }

    result.num_cols = m.num_cols;
    result.num_rows = this->num_rows;
    result.zeros(result.num_rows,result.num_cols);

    for(unsigned int i=0; i<this->num_rows; i++){
        for(unsigned int j=0; j<m.num_cols; j++){
            result.M[i][j] = 0;
            for(unsigned int c=0; c<this->num_cols;c++){
                result.M[i][j] += this->M[i][c]*m.M[c][j];
            }
        }
    }
    return result;
}

Matrix Matrix::operator*(const double& d){
    Matrix result;

    result.num_cols = this->num_cols;
    result.num_rows = this->num_rows;
    result.zeros(result.num_rows,result.num_cols);

    for(unsigned int i=0; i<this->num_rows; i++){
        for(unsigned int j=0; j<this->num_cols; j++){
                result.M[i][j] = this->M[i][j]*d;
        }
    }
    return result;
}

Matrix Matrix::operator/(const double& d){
    Matrix result;

    result.num_cols = this->num_cols;
    result.num_rows = this->num_rows;
    result.zeros(result.num_rows,result.num_cols);

    for(unsigned int i=0; i<this->num_rows; i++){
        for(unsigned int j=0; j<this->num_cols; j++){
                result.M[i][j] = this->M[i][j]/d;
        }
    }
    return result;
}

//Check: https://en.cppreference.com/w/cpp/container/vector
