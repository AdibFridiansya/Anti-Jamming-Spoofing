#include "ExtendedKF.h"

ExtendedKF::ExtendedKF() {
    x_err.setZero();
    P.setZero();
    P0_diag.setOnes();
    Q.setZero();
    R.setZero();
    H.setZero();
    Phi.setIdentity();
    I15.setIdentity();

    dz_minus.setZero();
    S_cov.setIdentity();

    // Eq. (14.81) - System Noise PSD Parameters (Groves)
    Sa  = 0.05f * 0.05f;
    Sg  = 0.005f * 0.005f;
    Sba = 0.0001f * 0.0001f;
    Sbg = 0.00001f * 0.00001f;

    // Eq. (3.3) - First-Order Gauss-Markov Correlation Times (Groves)
    tau_ba = 0.0f;
    tau_bg = 0.0f;

    // Eq. (14.67) - Loosely Coupled Measurement Matrix (Groves)
    H.block<6, 6>(0, 0) = Eigen::Matrix<float, 6, 6>::Identity();
}

void ExtendedKF::initialize(const Eigen::Matrix<float, 15, 15>& P0, 
                            const Eigen::Matrix<float, 6, 6>& init_R) {
    P = P0;
    R = init_R;

    for (int i = 0; i < 15; ++i) {
        P0_diag(i) = std::max(P0(i, i), 1.0e-12f);
    }
}

// Eq. (2.50) - Skew-Symmetric Matrix (Groves)
Eigen::Matrix3f ExtendedKF::skewSymmetric(const Eigen::Vector3f& vec) const {
    Eigen::Matrix3f vector_skew;
    vector_skew <<    0.0f, -vec(2),  vec(1),
                    vec(2),    0.0f, -vec(0),
                   -vec(1),  vec(0),    0.0f;
    return vector_skew;
}

// Eq. (3.11) - Measurement Noise Covariance Matrix (Groves)
void ExtendedKF::setMeasurementNoise(const Eigen::Matrix<float, 6, 6>& R_k) {
    R = 0.5f * (R_k + R_k.transpose());
}

// Eq. (3.11) - Diagonal Measurement Noise Covariance Matrix (Groves)
void ExtendedKF::setMeasurementNoiseDiagonal(const Eigen::Matrix<float, 6, 1>& r_diag) {
    R.setZero();
    for (int i = 0; i < 6; ++i) {
        R(i, i) = std::max(r_diag(i), 1.0e-8f);
    }
}

// Eq. (3.2) & (14.81) - System Noise Power Spectral Density (Groves)
void ExtendedKF::setProcessNoisePSD(float Sa_psd, float Sg_psd, float Sba_psd, float Sbg_psd) {
    Sa  = Sa_psd;
    Sg  = Sg_psd;
    Sba = Sba_psd;
    Sbg = Sbg_psd;
}

// Eq. (3.3) - First-Order Gauss-Markov Correlation Time (Groves)
void ExtendedKF::setBiasCorrelationTimes(float tau_ba_s, float tau_bg_s) {
    tau_ba = tau_ba_s;
    tau_bg = tau_bg_s;
}

// Section 3.3.3 - Covariance Symmetry and Floor Enforcement (Groves)
void ExtendedKF::enforceCovarianceSymmetryAndFloor() {
    P = 0.5f * (P + P.transpose());

    static constexpr float MIN_VAR_FLOOR = 1.0e-12f;
    for (int i = 0; i < 15; ++i) {
        if (P(i, i) < MIN_VAR_FLOOR) {
            P(i, i) = MIN_VAR_FLOOR;
        }
    }
}

// Section 3.2.3 - System Propagation Phase (Groves)
void ExtendedKF::predict(const Eigen::Matrix3f& cn_b, const Eigen::Vector3f& fn_ib, float dt) {
    if (dt <= 0.0f) {
        return;
    }

    Phi.setIdentity();
    
    // Eq. (3.34) & (14.50) - First-Order State Transition Matrix (Groves)
    Phi.block<3, 3>(0, 3)  = Eigen::Matrix3f::Identity() * dt;
    Phi.block<3, 3>(3, 6)  = -skewSymmetric(fn_ib) * dt;
    Phi.block<3, 3>(3, 9)  = cn_b * dt;
    Phi.block<3, 3>(6, 12) = cn_b * dt;

    // Eq. (3.3) - First-Order Gauss-Markov State Transition (Groves)
    if (tau_ba > 0.0f) {
        Phi.block<3, 3>(9, 9) = Eigen::Matrix3f::Identity() * std::max(1.0f - (dt / tau_ba), 0.0f);
    }
    if (tau_bg > 0.0f) {
        Phi.block<3, 3>(12, 12) = Eigen::Matrix3f::Identity() * std::max(1.0f - (dt / tau_bg), 0.0f);
    }

    // Eq. (3.45) & (14.81) - Impulse Approximation System Noise Covariance Matrix (Groves)
    Q.setZero();
    Q.block<3, 3>(3, 3)   = Eigen::Matrix3f::Identity() * (Sa * dt);
    Q.block<3, 3>(6, 6)   = Eigen::Matrix3f::Identity() * (Sg * dt);
    Q.block<3, 3>(9, 9)   = Eigen::Matrix3f::Identity() * (Sba * dt);
    Q.block<3, 3>(12, 12) = Eigen::Matrix3f::Identity() * (Sbg * dt);

    // Eq. (3.14) - State Vector Propagation (Groves)
    x_err = Phi * x_err;

    // Eq. (3.46) - First-Order Error Covariance Propagation (Groves)
    Eigen::Matrix<float, 15, 15> half_Q = 0.5f * Q;
    P = Phi * (P + half_Q) * Phi.transpose() + half_Q;

    enforceCovarianceSymmetryAndFloor();
}

// Section 3.2.4 - Measurement Update Phase (Groves)
void ExtendedKF::correct(const Eigen::Matrix<float, 6, 1>& dz) {
    // Eq. (3.9) & (3.89) - Measurement Innovation (Groves)
    dz_minus = dz - H * x_err;

    // Eq. (3.20) - Innovation Covariance Matrix (Groves)
    S_cov = H * P * H.transpose() + R;
    S_cov = 0.5f * (S_cov + S_cov.transpose());

    // Eq. (3.21) - Kalman Gain Matrix (Groves)
    Eigen::Matrix<float, 6, 15> HP = H * P;
    Eigen::Matrix<float, 15, 6> K = S_cov.ldlt().solve(HP).transpose();

    // Eq. (3.24) - State Vector Update (Groves)
    x_err = x_err + K * dz_minus;

    // Eq. (3.58) - Joseph Form Error Covariance Update (Groves)
    Eigen::Matrix<float, 15, 15> I_KH = I15 - K * H;
    P = I_KH * P * I_KH.transpose() + K * R * K.transpose();

    enforceCovarianceSymmetryAndFloor();
}

// Section 3.2.7 - Sequential Measurement Update (Groves)
void ExtendedKF::correctSequential(const Eigen::Matrix<float, 6, 1>& dz) {
    dz_minus = dz - H * x_err;
    S_cov = H * P * H.transpose() + R;

    for (int j = 0; j < 6; ++j) {
        const float denom = P(j, j) + R(j, j);
        if (denom <= 1.0e-12f) {
            continue;
        }

        // Eq. (3.73) - Sequential Kalman Gain Vector (Groves)
        Eigen::Matrix<float, 15, 1> k_j = P.col(j) / denom;

        // Eq. (3.74) - Sequential State Vector Update (Groves)
        const float dz_j = dz(j) - x_err(j);
        x_err = x_err + k_j * dz_j;

        // Eq. (3.75) & (3.58) - Sequential Error Covariance Update (Groves)
        Eigen::Matrix<float, 15, 15> I_kh = I15;
        I_kh.col(j) -= k_j;
        P = I_kh * P * I_kh.transpose() + (k_j * R(j, j)) * k_j.transpose();
    }

    enforceCovarianceSymmetryAndFloor();
}

Eigen::Matrix<float, 15, 1> ExtendedKF::getErrorState() const {
    return x_err;
}

Eigen::Matrix<float, 15, 15> ExtendedKF::getCovariance() const {
    return P;
}

// Eq. (3.6) - State Uncertainty (Groves)
Eigen::Matrix<float, 15, 1> ExtendedKF::getStateUncertainties() const {
    return P.diagonal().cwiseMax(0.0f).cwiseSqrt();
}

// Eq. (3.71) - Normalized Error Covariance Matrix (Groves)
Eigen::Matrix<float, 15, 1> ExtendedKF::getNormalizedCovarianceDiagonal() const {
    Eigen::Matrix<float, 15, 1> norm_diag;
    for (int i = 0; i < 15; ++i) {
        norm_diag(i) = P(i, i) / P0_diag(i);
    }
    return norm_diag;
}

// Eq. (3.9) - Measurement Innovation Vector (Groves)
Eigen::Matrix<float, 6, 1> ExtendedKF::getLastInnovation() const {
    return dz_minus;
}

// Eq. (3.20) - Innovation Covariance Matrix (Groves)
Eigen::Matrix<float, 6, 6> ExtendedKF::getInnovationCovariance() const {
    return S_cov;
}

// Section 3.2.6 - Full Closed-Loop State Reset (Groves)
void ExtendedKF::resetState() {
    x_err.setZero();
}

// Section 3.2.6 - Mixed Closed/Open-Loop Navigation State Reset (Groves)
void ExtendedKF::resetNavigationStatesOnly() {
    x_err.head<9>().setZero();
}