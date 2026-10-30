#include "INSMechanization.h"
#include <cmath>

// Figure 5.7[cite: 17]
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

// Eq. (5.41) - Omega^n_ie[cite: 18]
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

// Eq. (5.44) - Omegan_en[cite: 18]
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

// Eq. (2.134) - Earth gravity vector computation (g^n_b)
Eigen::Vector3f INSMechanization::computeGravity(double L_b, double h_b) const {
    // Somigliana Formula's
    const double sin_L = std::sin(L_b);
    const double sin_L_sqr = sin_L * sin_L;
    
    // ref. WGS-84
    const double g_0 = 9.7803253359; 
    const double gravity_num = g_0 * (1.0 + 0.00193185265241 * sin_L_sqr);
    const double denom_sqrt = std::sqrt(1.0 - E_SQR * sin_L_sqr);
    double gamma = gravity_num / denom_sqrt;
    
    // Eq. (5.122)
    double g = gamma * (1.0 - (2.0 / R_0) * h_b);
    
    // Eq. (2.135)
    return Eigen::Vector3f(0.0f, 0.0f, static_cast<float>(g));
}

// Eq. (5.46) - Attitude Update 
void INSMechanization::updateAttitude(const Eigen::Vector3f& omegab_ib, float dt) {
    double RN_Lb, RE_Lb;
    computeEarth(L_b, RN_Lb, RE_Lb);

    Eigen::Matrix3f Omegan_ie = computeOmegan_ie(L_b);
    Eigen::Matrix3f Omegan_en = computeOmegan_en(vn_eb, L_b, h_b, RN_Lb, RE_Lb);
    Eigen::Matrix3f Omegab_ib = skewSymmetric(omegab_ib);
    Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity();

    Eigen::Matrix3f Cn_b_plus;
    Cn_b_plus = Cn_b * (I3 + Omegab_ib * dt) - (Omegan_ie + Omegan_en) * Cn_b * dt;
    
    Cn_b = Cn_b_plus;
    reorthogonalizeAttitude();
}

// Reorthogonalization
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

// Eq. (5.47) - Specific-Force Frame Transformation[cite: 19]
Eigen::Vector3f INSMechanization::transformSpecificForce(const Eigen::Vector3f& fb_ib, 
                                                         const Eigen::Matrix3f& Cn_b_minus, 
                                                         const Eigen::Matrix3f& Cn_b_plus) const {
    Eigen::Matrix3f Cn_b_avg = 0.5f * (Cn_b_minus + Cn_b_plus);
    Eigen::Vector3f fn_ib = Cn_b_avg * fb_ib;
    
    return fn_ib;
}

// Eq. (5.54) - Velocity Update (First-Order Approximation)[cite: 20]
void INSMechanization::updateVelocity(const Eigen::Vector3f& fn_ib, float dt) {
    double RN_Lb, RE_Lb;
    computeEarth(L_b, RN_Lb, RE_Lb);

    Eigen::Vector3f gn_b = computeGravity(L_b, h_b); 
    Eigen::Matrix3f Omegan_ie = computeOmegan_ie(L_b);
    Eigen::Matrix3f Omegan_en = computeOmegan_en(vn_eb, L_b, h_b, RN_Lb, RE_Lb);
    Eigen::Vector3f vn_eb_plus = vn_eb + (fn_ib + gn_b - (Omegan_en + 2.0f * Omegan_ie) * vn_eb) * dt;

    vn_eb = vn_eb_plus;
}

// Eq. (5.56) - Position Update 
void INSMechanization::updatePosition(const Eigen::Vector3f& vn_eb_minus, 
                                      const Eigen::Vector3f& vn_eb_plus, 
                                      float dt) {
    double RN_minus, RE_minus;
    computeEarth(L_b, RN_minus, RE_minus);

    // Altitude 
    double h_plus = h_b - 0.5 * dt * (vn_eb_minus(2) + vn_eb_plus(2));

    // Latitude
    double L_plus = L_b + 0.5 * dt * (vn_eb_minus(0) / (RN_minus + h_b) + 
                                      vn_eb_plus(0) / (RN_minus + h_plus));

    double RN_plus, RE_plus;
    computeEarth(L_plus, RN_plus, RE_plus);

    // Longitude
    double lambda_plus = lambda_b + 0.5 * dt * (vn_eb_minus(1) / ((RE_minus + h_b) * std::cos(L_b)) + 
                                                vn_eb_plus(1) / ((RE_plus + h_plus) * std::cos(L_plus)));
    L_b      = L_plus;
    lambda_b = lambda_plus;
    h_b      = h_plus;
}

// Eq. (5.56) - Position Update (2nd Method)
// Altitude 
void INSMechanization::updateAltitude(float v_D_minus, float v_D_plus, float dt) {
    h_b = h_b - 0.5 * dt * (v_D_minus + v_D_plus);
}

// Latitude 
void INSMechanization::updateLatitude(float v_N_minus, float v_N_plus, 
                                      double h_minus, double h_plus, 
                                      float dt) {
    double RN_minus, RE_minus;
    computeEarth(L_b, RN_minus, RE_minus);
    
    L_b = L_b + 0.5 * dt * (v_N_minus / (RN_minus + h_minus) + 
                            v_N_plus  / (RN_minus + h_plus));
}

// ELongitude
void INSMechanization::updateLongitude(float v_E_minus, float v_E_plus, 
                                       double h_minus, double h_plus, 
                                       double L_minus, double L_plus, 
                                       float dt) {

    double RN_minus, RE_minus;
    computeEarth(L_minus, RN_minus, RE_minus);
    double RN_plus, RE_plus;
    computeEarth(L_plus, RN_plus, RE_plus);
    lambda_b = lambda_b + 0.5 * dt * (v_E_minus / ((RE_minus + h_minus) * std::cos(L_minus)) + 
                                      v_E_plus  / ((RE_plus  + h_plus)  * std::cos(L_plus)));
}