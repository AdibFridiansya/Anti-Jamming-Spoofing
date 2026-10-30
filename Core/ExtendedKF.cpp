#include "ExtendedKF.h"

ExtendedKF::ExtendedKF() {
    x_err.setZero();
    P.setZero();
    Q.setZero();
    R.setZero();
    H.setZero();
    Phi.setIdentity();
    I15.setIdentity();

    // Eq. (14.67) - Groves: Loosely coupled measurement matrix (Position & Velocity only)
    H.block<6, 6>(0, 0) = Eigen::Matrix<float, 6, 6>::Identity();
}

void ExtendedKF::initialize(const Eigen::Matrix<float, 15, 15>& P0, 
                            const Eigen::Matrix<float, 15, 15>& init_Q, 
                            const Eigen::Matrix<float, 6, 6>& init_R) {
    P = P0;
    Q = init_Q;
    R = init_R;
}

void ExtendedKF::setQ(const Eigen::Matrix<float, 15, 15>& current_Q) {
    Q = current_Q;
}

Eigen::Matrix3f ExtendedKF::skewSymmetric(const Eigen::Vector3f& vec) const {
    Eigen::Matrix3f S;
    S <<    0.0f, -vec(2),  vec(1),
          vec(2),    0.0f, -vec(0),
         -vec(1),  vec(0),    0.0f;
    return S;
}

void ExtendedKF::predict(const Eigen::Matrix3f& cn_b, const Eigen::Vector3f& fn_ib, float dt) {
    Phi.setIdentity();
    
    // Eq. (14.50) & (3.34) - Transition Matrix Phi approx (I + F * dt)
    // 1. Error posisi digerakkan oleh error kecepatan
    Phi.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity() * dt;
    
    // 2. Error kecepatan digerakkan oleh error orientasi (Specific force cross-product)
    Phi.block<3, 3>(3, 6) = -skewSymmetric(fn_ib) * dt;
    
    // 3. Error kecepatan digerakkan oleh bias akselerometer
    Phi.block<3, 3>(3, 9) = cn_b * dt;
    
    // 4. Error orientasi digerakkan oleh bias giroskop
    Phi.block<3, 3>(6, 12) = -cn_b * dt;

    // Propagasi state error
    x_err = Phi * x_err;

    // Propagasi kovariansi error
    P = Phi * P * Phi.transpose() + Q;

    // Paksa simetri kovariansi untuk mencegah divergensi numerik
    P = 0.5f * (P + P.transpose());
}

void ExtendedKF::correct(const Eigen::Matrix<float, 6, 1>& dz) {
    // Eq. (3.20) - Innovation covariance (S)
    Eigen::Matrix<float, 6, 6> S = H * P * H.transpose() + R;

    // Selesaikan inversi menggunakan dekomposisi LDLT agar stabil terhadap singularitas
    Eigen::Matrix<float, 6, 15> HP = H * P;
    Eigen::Matrix<float, 6, 15> S_inv_HP = S.ldlt().solve(HP);
    Eigen::Matrix<float, 15, 6> K = S_inv_HP.transpose();

    // Eq. (3.24) - Error state update
    x_err = x_err + K * (dz - H * x_err);

    // Eq. (3.25) - Error covariance update dengan jaminan simetri
    P = (I15 - K * H) * P;
    P = 0.5f * (P + P.transpose());
}

Eigen::Matrix<float, 15, 1> ExtendedKF::getErrorState() const {
    return x_err;
}

void ExtendedKF::resetState() {
    x_err.setZero();
}