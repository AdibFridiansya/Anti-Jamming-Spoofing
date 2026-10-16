#pragma once
#include <Eigen/Dense>

class INSMechanization {
public:
    // WGS-84 & Physical Constants (Anon., NGA TR8350.2 / Groves Section 2.4.2)
    static constexpr float  W_IE   = 7.292115e-5f;         // (omega_ie) Earth's nominal rotation rate (rad/s)
    static constexpr double R_0    = 6378137.0;            // (R_0) Semi-major axis / Equatorial radius (m)
    static constexpr double E_SQR  = 0.00669437999014;     // (e^2) First eccentricity squared (-)

private:
    Eigen::Matrix3f Cn_b;       // (C^n_b) Body-to-Navigation Coordinate Transformation Matrix (DCM)
    Eigen::Vector3f vn_eb;      // (v^n_eb) Velocity resolved in Navigation frame [North, East, Down] (m/s)
    double L_b;                 // (L_b) Geodetic Latitude (rad)
    double lambda_b;            // (lambda_b) Geodetic Longitude (rad)
    double h_b;                 // (h_b) Geodetic Height above reference ellipsoid (m)
    
    // Skew-Symmetric Matrix / Cross-Product Operator [vec X]
    Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& vec) const;

    // Eq. (5.41) - Earth rotation rate skew-symmetric matrix (Omega^n_ie)
    Eigen::Matrix3f computeOmegan_ie(double L_b) const;

    // Eq. (2.105) & (2.106) - Meridian (R_N) and Transverse (R_E) radii of curvature
    void computeEarth(double L_b, double& R_N_Lb, double& R_E_Lb) const;

    // Eq. (5.44) - Transport rate skew-symmetric matrix (Omega^n_en)
    Eigen::Matrix3f computeOmegan_en(const Eigen::Vector3f& vn_eb, 
                                     double L_b, double h_b, 
                                     double R_N_Lb, double R_E_Lb) const;

    // Eq. (2.134) - Normal gravity vector computation (g^n_b)
    Eigen::Vector3f computeGravity(double L_b, double h_b) const;

    // Orthogonalization and Normalization of DCM
    void reorthogonalizeAttitude();

public:
    INSMechanization();
    ~INSMechanization() = default;

    void initialize(double init_L_b, double init_lambda_b, double init_h_b, 
                    float roll_rad, float pitch_rad, float yaw_rad);
   
    // Eq. (5.46) - Attitude Update (First-Order Approximation)
    void updateAttitude(const Eigen::Vector3f& omega_ib_b, float dt);

    // Eq. (5.47) - Specific-Force Frame Transformation
    Eigen::Vector3f transformSpecificForce(const Eigen::Vector3f& fb_ib, 
                                           const Eigen::Matrix3f& C_b_n_minus, 
                                           const Eigen::Matrix3f& C_b_n_plus) const;

    // Eq. (5.54) - Velocity Update (First-Order Approximation)
    void updateVelocity(const Eigen::Vector3f& fn_ib, float dt);

    // Getters
    Eigen::Matrix3f getOrientation() const { return Cn_b; }
    Eigen::Vector3f getVelocity() const { return vn_eb; }
    void getPosition(double& out_L_b, double& out_lambda_b, double& out_h_b) const {
        out_L_b = L_b;
        out_lambda_b = lambda_b;
        out_h_b = h_b;
    }
};