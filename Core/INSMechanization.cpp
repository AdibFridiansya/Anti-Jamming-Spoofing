#include "INSMechanization.h"
#include <cmath>

// Figure 5.7
INSMechanization::INSMechanization() {
    Cn_b = Eigen::Matrix3f::Identity();
    vn_eb = Eigen::Vector3f::Zero();
    L_b = 0.0;
    lambda_b = 0.0;
    h_b = 0.0;
}

// Initialization of Navigation State: Position (L, lambda, h), Velocity (v), and Attitude (C^n_b)
void INSMechanization::initialize(double init_L_b, double init_lambda_b, double init_h_b, 
                                  float roll_rad, float pitch_rad, float yaw_rad){
    L_b = init_L_b;
    lambda_b = init_lambda_b;
    h_b = init_h_b;
    vn_eb = Eigen::Vector3f::Zero();

    Eigen::AngleAxisf rollAngle(roll_rad, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch_rad, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw_rad, Eigen::Vector3f::UnitZ());

    Cn_b = yawAngle * pitchAngle * rollAngle;                   // (C^n_b) Orientation Matrix
}

// Skew-Symmetric Matrix 
Eigen::Matrix3f INSMechanization::skewSymmetric(const Eigen::Vector3f& vec) const {
    Eigen::Matrix3f vector_skew;
    vector_skew <<    0.0f, -vec(2),  vec(1),
                    vec(2),    0.0f, -vec(0),
                   -vec(1),  vec(0),    0.0f;
    return vector_skew;
}

// Eq. (5.41) - Omega^n_ie
Eigen::Matrix3f INSMechanization::computeOmegan_ie(double L_b) const {
    const float sin_L = static_cast<float>(std::sin(L_b));
    const float cos_L = static_cast<float>(std::cos(L_b));

    const float w_sin = W_IE * sin_L;
    const float w_cos = W_IE * cos_L;

    Eigen::Matrix3f Omegan_ie;
    Omegan_ie <<    0.0f,   w_sin,    0.0f,
                  -w_sin,    0.0f,  -w_cos,
                    0.0f,   w_cos,    0.0f;

    return Omegan_ie;                           
}

// Eq. (2.105) & (2.106) - Earth Radii of Curvature
void INSMechanization::computeEarth(double L_b, double& RN_Lb, double& RE_Lb) const {
    const double sin_L = std::sin(L_b);
    const double sin_L_sqr = sin_L * sin_L;                                         

    const double denom_sqrt = std::sqrt(1.0 - E_SQR * sin_L_sqr);                   // sqrt(1 - e^2 * sin^2(L_b))
    RN_Lb = R_0 * (1.0 - E_SQR) / (denom_sqrt * denom_sqrt * denom_sqrt);           // Meridian radius (RN_Lb)
    RE_Lb = R_0 / denom_sqrt;                                                       // Transverse radius (RE_Lb)
}

// Eq. (5.44) - Omegan_en
Eigen::Matrix3f INSMechanization::computeOmegan_en(const Eigen::Vector3f& vn_eb, 
                                                   double L_b, double h_b, 
                                                   double RN_Lb, double RE_Lb) const {
    const double v_N = static_cast<double>(vn_eb(0));
    const double v_E = static_cast<double>(vn_eb(1));
    const double tan_Lb = std::tan(L_b);

    const float w_en_x = static_cast<float>(v_E / (RE_Lb + h_b));               // North
    const float w_en_y = static_cast<float>(-v_N / (RN_Lb + h_b));              // East
    const float w_en_z = static_cast<float>(-v_E * tan_Lb / (RE_Lb + h_b));     // Down

    Eigen::Matrix3f Omegan_en;
    Omegan_en <<     0.0f, -w_en_z,   w_en_y,
                   w_en_z,    0.0f,  -w_en_x,
                  -w_en_y,  w_en_x,     0.0f;

    return Omegan_en;
}

// Eq. (5.46) - Attitude Update 
void INSMechanization::updateAttitude(const Eigen::Vector3f& omegab_ib, float dt) {
    double RN_Lb, RE_Lb;
    computeEarth(L_b, RN_Lb, RE_Lb);

    Eigen::Matrix3f Omegan_ie = computeOmegan_ie(L_b);
    Eigen::Matrix3f Omegan_en = computeOmegan_en(vn_eb, L_b, h_b, RN_Lb, RE_Lb);
    Eigen::Matrix3f Omegab_ib = skewSymmetric(omegab_ib);
    Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity();

    // C_b^n(+) = C_b^n(-) * (I3 + Omega_ib_b * dt) - (Omega_ie_n + Omega_en_n) * C_b^n(-) * dt
    Eigen::Matrix3f Cn_b_plus;
    Eigen::Matrix3f Cn_b_plus = Cn_b * (I3 + Omegab_ib * dt) - (Omegan_ie + Omegan_en) * Cn_b * dt;
    
    Cn_b = Cn_b_plus;
    reorthogonalizeAttitude();
}

// DCM Reorthogonalization
void INSMechanization::reorthogonalizeAttitude() {
    Eigen::Vector3f x = Cn_b.col(0);
    Eigen::Vector3f y = Cn_b.col(1);

    float error = x.dot(y);
    Eigen::Vector3f x_orth = x - 0.5f * error * y;
    Eigen::Vector3f y_orth = y - 0.5f * error * x;
    Eigen::Vector3f z_orth = x_orth.cross(y_orth);

    Cn_b.col(0) = x_orth.normalized();
    Cn_b.col(1) = y_orth.normalized();
    Cn_b.col(2) = z_orth.normalized();
}