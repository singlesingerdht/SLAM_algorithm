#include <iostream>

#include <ctime>

// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

#include <gtsam/geometry/SO3.h>

using namespace std;
using namespace Eigen;
using gtsam::Vector9;

#define MATRIX_SIZE 50

static Eigen::Block<const Vector9, 3, 1> dR(const Vector9& v) {
return v.segment<3>(0);
}
static Eigen::Block<const Vector9, 3, 1> dP(const Vector9& v) {
return v.segment<3>(3);
}
static Eigen::Block<const Vector9, 3, 1> dV(const Vector9& v) {
return v.segment<3>(6);
}


int main( int argc, char** argv )
{
    MatrixXd A(9,9), B(9,6), I(9,6), H(5,9);
    MatrixXd Q(6,6), R(5,5);
    MatrixXd P0(9,9), x0(9,1);
    double dt = 0.1;
    A << 1, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0, 0, 
            0, 0, 1, 0, 0, 0, 0, 0, 0, 
            0, 0, 0, 1, 0, 0, dt, 0, 0,
            0, 0, 0, 0, 1, 0, 0, dt, 0,
            0, 0, 0, 0, 0, 1, 0, 0, dt,
            0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1;
    B << 0, 0, 0, 0, 0, 0,
        dt, 0, 0, 0, 0, 0,
        0, dt, 0, 0, 0, 0,
        0, 0, dt, 0, 0, 0;
    H << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
    Q << 0.0001, 0, 0, 0, 0, 0,
                0, 0.0001, 0, 0, 0, 0,
                0.0, 0.0001, 0, 0, 0, 0,
                0, 0, 0, 0.0001, 0, 0,
                0.0, 0, 0, 0, 0.0001, 0,
                0.0, 0, 0, 0, 0, 0.0001;
    R << 0.0001, 0, 0, 0, 0,
            0, 0.0001, 0, 0, 0,
            0, 0, 0.0001, 0, 0,
            0, 0, 0, 0.0001, 0,
            0, 0, 0, 0, 0.0001;
    x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    P0 << 0.01, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0.01, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0.01, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0.01, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0.01, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0;
    KF(A, B, I, H, Q, R, P0, x0);

    // auto theta = dR(x);
    // gtsam::so3::DexpFunctor local(theta);
    // Matrix3d w_tangent_H_theta, invH;
    // const Vector3d w_tangent = // angular velocity mapped back to tangent space
    // local.applyInvDexp(w_body, A ? &w_tangent_H_theta : 0, C ? &invH : 0);
    // const SO3 R = local.expmap();
    // const Vector3 a_nav = R * a_body;
    // const double dt22 = 0.5 * dt * dt;

    return 0;
}
