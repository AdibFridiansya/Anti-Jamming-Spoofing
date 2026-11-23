#ifndef EXTENDED_KF_H
#define EXTENDED_KF_H

#ifndef EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_VECTORIZE
#endif

#ifndef EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#endif

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

class ExtendedKF {
public:
    ExtendedKF();

    void initialize(const Eigen::Matrix<float, 15, 15>& P0, 
                    const Eigen::Matrix<float, 6, 6>& init_R);

    // Eq. (3.14) & (3.46) - System Propagation Phase (Groves)
    void predict(const Eigen::Matrix3f& cn_b, const Eigen::Vector3f& fn_ib, float dt);

    // Eq. (3.21), (3.24), & (3.58) - Vector Measurement Update Phase (Groves)
    void correct(const Eigen::Matrix<float, 6, 1>& dz);

    // Eq. (3.73) - (3.75) - Sequential Measurement Update Phase (Groves)
    void correctSequential(const Eigen::Matrix<float, 6, 1>& dz);

    // Eq. (3.11) - Measurement Noise Covariance Matrix (Groves)
    void setMeasurementNoise(const Eigen::Matrix<float, 6, 6>& R_k);
    void setMeasurementNoiseDiagonal(const Eigen::Matrix<float, 6, 1>& r_diag);

    // Eq. (3.2) & (3.3) - System Noise PSD & Gauss-Markov Correlation Time (Groves)
    void setProcessNoisePSD(float Sa_psd, float Sg_psd, float Sba_psd, float Sbg_psd);
    void setBiasCorrelationTimes(float tau_ba_s, float tau_bg_s);

    // Section 3.2.6 - Closed-Loop State Reset (Groves)
    void resetState();
    void resetNavigationStatesOnly();

    Eigen::Matrix<float, 15, 1> getErrorState() const;
    Eigen::Matrix<float, 15, 15> getCovariance() const;
    Eigen::Matrix<float, 15, 1> getStateUncertainties() const;
    Eigen::Matrix<float, 15, 1> getNormalizedCovarianceDiagonal() const;
    Eigen::Matrix<float, 6, 1> getLastInnovation() const;
    Eigen::Matrix<float, 6, 6> getInnovationCovariance() const;

private:
    Eigen::Matrix<float, 15, 1> x_err;
    Eigen::Matrix<float, 15, 15> P;
    Eigen::Matrix<float, 15, 1> P0_diag;
    Eigen::Matrix<float, 15, 15> Q;
    Eigen::Matrix<float, 6, 6> R;
    Eigen::Matrix<float, 6, 15> H;
    Eigen::Matrix<float, 15, 15> Phi;
    Eigen::Matrix<float, 15, 15> I15;

    Eigen::Matrix<float, 6, 1> dz_minus;
    Eigen::Matrix<float, 6, 6> S_cov;

    float Sa;
    float Sg;
    float Sba;
    float Sbg;

    float tau_ba;
    float tau_bg;

    // Eq. (2.50) - Skew-Symmetric Matrix (Groves)
    Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& vec) const;

    // Section 3.3.3 - Covariance Symmetry and Floor Enforcement (Groves)
    void enforceCovarianceSymmetryAndFloor();
};

#endif // EXTENDED_KF_H