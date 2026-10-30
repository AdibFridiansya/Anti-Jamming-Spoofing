#pragma once

#ifndef EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_VECTORIZE
#endif

#ifndef EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#endif

#include <Eigen/Dense>

class ExtendedKF {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ExtendedKF();

    void initialize(const Eigen::Matrix<float, 15, 15>& P0, 
                    const Eigen::Matrix<float, 15, 15>& init_Q, 
                    const Eigen::Matrix<float, 6, 6>& init_R);

    void setQ(const Eigen::Matrix<float, 15, 15>& current_Q);

    Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& vec) const;

    void predict(const Eigen::Matrix3f& cn_b, const Eigen::Vector3f& fn_ib, float dt);

    void correct(const Eigen::Matrix<float, 6, 1>& dz);

    Eigen::Matrix<float, 15, 1> getErrorState() const;

    void resetState();

private:
    Eigen::Matrix<float, 15, 1> x_err;
    Eigen::Matrix<float, 15, 15> P;
    Eigen::Matrix<float, 15, 15> Q;
    Eigen::Matrix<float, 6, 6> R;
    Eigen::Matrix<float, 6, 15> H;
    Eigen::Matrix<float, 15, 15> Phi;
    Eigen::Matrix<float, 15, 15> I15;
};