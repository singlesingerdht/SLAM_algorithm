#include <iostream>
using namespace std;
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>


using namespace std;

void rotationCal(Eigen::Vector3d norm_vec)
{
    norm_vec.normalize();
    if(norm_vec[0] < 0){
        norm_vec = -norm_vec;
    }

    Eigen::Vector3d dest_vec(1, 0, 0);
    Eigen::Vector3d axis = (dest_vec + norm_vec)/2;
    axis.normalize();
    Eigen::AngleAxisd rotation_vector(M_PI, axis);
    Eigen::Matrix3d rotation_matrix = rotation_vector.matrix();
    cout<< rotation_matrix << endl;
    Eigen::Vector3d eulerAngle = rotation_matrix.eulerAngles(2,1,0);
    cout<< eulerAngle << endl;
}

int main( int argc, char** argv )
{
    Eigen::Vector3d coefficient;
    coefficient[0] = 1;    coefficient[1] = 2;    coefficient[2] = 3;
    rotationCal(coefficient);
    return 0;
}
