#include "JammingPropagation.h"
#include <algorithm>

JammingPropagation::JammingPropagation() {}

// Eq. (2.105) & (2.106) - Earth Radii of Curvature (Groves)
void JammingPropagation::computeEarth(double L_b, double& RN_Lb, double& RE_Lb) const {
    const double sin_L = std::sin(L_b);
    const double sin_L_sqr = sin_L * sin_L;

    const double denom_sqrt = std::sqrt(1.0 - E_SQR * sin_L_sqr);
    RN_Lb = R_0 * (1.0 - E_SQR) / (denom_sqrt * denom_sqrt * denom_sqrt);
    RE_Lb = R_0 / denom_sqrt;
}

// Eq. (9.27) - Free-Space Propagation Loss (Kaplan PDF 1, p. 571 / PDF p. 23)
float JammingPropagation::computeFreeSpaceLoss(float d_m, float freq_MHz) const {
    const double freq_Hz  = static_cast<double>(freq_MHz) * 1.0e6;
    const double lambda_j = C_LIGHT / freq_Hz;
    const double safe_d   = std::max(static_cast<double>(d_m), 1.0);

    const double Lp_dB = 20.0 * std::log10((4.0 * PI_VAL * safe_d) / lambda_j);
    return static_cast<float>(Lp_dB);
}

// Eq. (9.28) - (9.33) - Hata & Modified Hata Path Loss Models (Kaplan PDF 1, p. 574-575 / PDF p. 26-27)
float JammingPropagation::computeHataLoss(float d_km, float freq_MHz,
                                          float h_base_m, float h_mobile_m,
                                          PropagationModelType modelType) const {
    const double f  = std::max(static_cast<double>(freq_MHz), 150.0);
    const double hb = std::max(static_cast<double>(h_base_m), 1.0);
    // Domain empiris tinggi antena penerima Hata: 1 m <= h_m <= 10 m (Kaplan p. 573 & 595)
    const double hm = std::max(1.0, std::min(static_cast<double>(h_mobile_m), 10.0));
    const double d  = std::max(static_cast<double>(d_km), 0.001);

    const double log_f  = std::log10(f);
    const double log_hb = std::log10(hb);
    const double log_d  = std::log10(d);

    // Eq. (9.30) & (9.31) - Mobile Antenna Height Correction Factor (Kaplan p. 574)
    double a_hm = 0.0;
    if (modelType == PropagationModelType::HATA_URBAN_LARGE_CITY && f >= 400.0) {
        const double log_term = std::log10(11.75 * hm);
        a_hm = 3.2 * (log_term * log_term) - 4.97;
    } else {
        a_hm = (1.1 * log_f - 0.7) * hm + 0.8 - 1.56 * log_f;
    }

    // Eq. (9.28) & (9.29) - Original and Modified Hata Urban Loss (Kaplan p. 574)
    double LpN_dB = 0.0;
    if (f >= 1500.0) {
        LpN_dB = 46.3 + 33.9 * log_f - 13.82 * log_hb - a_hm + (44.9 - 6.55 * log_hb) * log_d;
    } else {
        LpN_dB = 69.55 + 26.16 * log_f - 13.82 * log_hb - a_hm + (44.9 - 6.55 * log_hb) * log_d;
    }

    // Eq. (9.32) & (9.33) - Suburban and Open Area Corrections (Kaplan p. 575)
    double L_out_dB = LpN_dB;
    if (modelType == PropagationModelType::HATA_SUBURBAN) {
        const double log_f28 = std::log10(f / 28.0);
        L_out_dB = LpN_dB - 2.0 * (log_f28 * log_f28) - 5.4;
    } else if (modelType == PropagationModelType::HATA_OPEN_AREA) {
        L_out_dB = LpN_dB - 4.78 * (log_f * log_f) + 18.33 * log_f - 40.94;
    }

    // Batas bawah fisika: rugi lintasan tidak lebih rendah dari Free-Space Path Loss
    const double fspl_dB = static_cast<double>(computeFreeSpaceLoss(static_cast<float>(d * 1000.0),
                                                                    static_cast<float>(f)));
    return static_cast<float>(std::max(L_out_dB, fspl_dB));
}

// Eq. (9.55) & Table 9.12 - Erceg Propagation Loss Model (Kaplan PDF 1, p. 594-596 / PDF p. 46-48)
float JammingPropagation::computeErcegLoss(float d_m, float freq_MHz,
                                           float h_base_m,
                                           PropagationModelType modelType) const {
    const double d0     = 100.0;
    const double safe_d = std::max(static_cast<double>(d_m), 1.0);

    // Model Erceg valid untuk d >= d_0 = 100 m; gunakan FSPL untuk jarak dekat
    if (safe_d <= d0) {
        return computeFreeSpaceLoss(static_cast<float>(safe_d), freq_MHz);
    }

    const double ht = std::max(10.0, std::min(static_cast<double>(h_base_m), 80.0));

    // Table 9.12 - Erceg Terrain Parameters termasuk mean shadowing loss mu_s (Kaplan p. 595)
    double a = 4.6, b = 0.0075, c_erc = 12.6, mu_s = 10.6;
    if (modelType == PropagationModelType::ERCEG_HILLY_LIGHT_TREE) {
        a = 4.0; b = 0.0065; c_erc = 17.1; mu_s = 9.6;
    } else if (modelType == PropagationModelType::ERCEG_FLAT_LIGHT_TREE) {
        a = 3.6; b = 0.0050; c_erc = 20.0; mu_s = 8.2;
    }

    const double A        = static_cast<double>(computeFreeSpaceLoss(static_cast<float>(d0), freq_MHz));
    const double mu_gamma = a - b * ht + (c_erc / ht);

    // [DITERTIBKAN] Eq. (9.55) & Figure 9.11 (Kaplan p. 594-596):
    // L = A + 10 * mu_gamma * log10(d / d_0) + mu_s (wajib sertakan mu_s sesuai Table 9.12)
    const double L_erceg_dB = A + 10.0 * mu_gamma * std::log10(safe_d / d0) + mu_s;
    const double fspl_dB    = static_cast<double>(computeFreeSpaceLoss(static_cast<float>(safe_d), freq_MHz));

    return static_cast<float>(std::max(L_erceg_dB, fspl_dB));
}

// Eq. (9.54), (9.57), & (9.58) - Vegetation and Building Penetration Loss (Kaplan PDF 1, p. 593 & 598)
float JammingPropagation::computeBlockageLoss(const JammerNodeConfig& jammer) const {
    double totalExcessLoss_dB = 0.0;

    // Eq. (9.54) - ITU-R Vegetation Attenuation Model (Kaplan p. 593)
    if (jammer.vegetationDepth_m > 0.0f) {
        const double f_MHz = static_cast<double>(jammer.freq_MHz);
        const double d_m   = static_cast<double>(jammer.vegetationDepth_m);

        double L_veg_dB = 0.2 * std::pow(f_MHz, 0.3) * std::pow(d_m, 0.6);
        if (jammer.isVegetationInLeaf) {
            L_veg_dB *= 1.30;
        }
        totalExcessLoss_dB += L_veg_dB;
    }

    // Eq. (9.57) - Building Penetration from Above (Kaplan p. 598)
    if (jammer.roofLoss_dB > 0.0f || jammer.numFloorsPenetrated > 0) {
        const double L_above_dB = static_cast<double>(jammer.roofLoss_dB) +
                                  static_cast<double>(jammer.numFloorsPenetrated) *
                                  static_cast<double>(jammer.floorLoss_dB);
        totalExcessLoss_dB += L_above_dB;
    }

    // Eq. (9.58) & Table 9.13 - Building Penetration through Walls (Kaplan p. 598-599)
    if (jammer.exteriorWallLoss_dB > 0.0f || jammer.numInteriorWalls > 0) {
        const double L_wall_dB = static_cast<double>(jammer.exteriorWallLoss_dB) +
                                 static_cast<double>(jammer.numInteriorWalls) *
                                 static_cast<double>(jammer.interiorWallLoss_dB);
        totalExcessLoss_dB += L_wall_dB;
    }

    return static_cast<float>(totalExcessLoss_dB);
}

// Eq. (9.26) - Incident Jammer Power Computation (Kaplan PDF 1, p. 570-571)
PropagationResult JammingPropagation::computePropagation(double currentTime_s,
                                                         double L_b, double lambda_b, double h_b,
                                                         float mobileHeightAGL_m,
                                                         const Eigen::Matrix3f& cn_b,
                                                         const JammerNodeConfig& jammer) const {
    PropagationResult res = {};
    res.isJammerActive       = false;
    res.incidentPower_Jr_dBW = -999.0f;

    if (!jammer.isEnabled || currentTime_s < jammer.startTime_s || currentTime_s > jammer.endTime_s) {
        return res;
    }
    res.isJammerActive = true;

    // Eq. (14.60) & (14.61) - Local NED Relative Position Vector (Groves)
    double RN_Lb, RE_Lb;
    computeEarth(L_b, RN_Lb, RE_Lb);

    const float d_N = static_cast<float>((jammer.L_j - L_b) * (RN_Lb + h_b));
    const float d_E = static_cast<float>((jammer.lambda_j - lambda_b) * (RE_Lb + h_b) * std::cos(L_b));
    const float d_D = static_cast<float>(-(jammer.h_j - h_b));

    res.r_bj_n       = Eigen::Vector3f(d_N, d_E, d_D);
    res.slantRange_m = std::max(res.r_bj_n.norm(), 1.0f);

    // Eq. (8.41) & (8.43) - LOS Unit Vector and Angles in Local NED (Groves)
    const Eigen::Vector3f un_bj = res.r_bj_n / res.slantRange_m;
    res.elevAngle_ned_rad = std::asin(std::max(-1.0f, std::min(-un_bj(2), 1.0f)));
    res.azimAngle_ned_rad = std::atan2(un_bj(1), un_bj(0));

    // Eq. (2.12) - NED to Body Frame LOS Transformation (Groves)
    const Eigen::Vector3f ub_bj = cn_b.transpose() * un_bj;
    res.elevAngle_body_rad = std::asin(std::max(-1.0f, std::min(-ub_bj(2), 1.0f)));
    res.azimAngle_body_rad = std::atan2(ub_bj(1), ub_bj(0));

    // Section 9.2.3 - Radio Horizon Limit using AGL Heights & 4/3 Effective Earth Radius (Kaplan p. 585)
    const double h_b_agl     = std::max(static_cast<double>(mobileHeightAGL_m), 0.1);
    const double h_j_agl     = std::max(static_cast<double>(jammer.baseHeightAGL_m), 0.1);
    const double R_eff       = (4.0 / 3.0) * R_0;
    const double d_horizon_m = std::sqrt(2.0 * R_eff * h_b_agl) + std::sqrt(2.0 * R_eff * h_j_agl);
    res.isBeyondHorizon      = (static_cast<double>(res.slantRange_m) > d_horizon_m);

    // Section 9.2.2.4 - Jammer Effective Isotropic Radiated Power (Kaplan p. 571)
    const float Jt_dBW = 10.0f * std::log10(std::max(jammer.transmitPower_W, 1.0e-9f));
    res.eirp_dBW       = Jt_dBW + jammer.antennaGain_dBic;

    // Eq. (9.27) - (9.33) & (9.55) - Primary Propagation Path Loss (Kaplan)
    const float d_km    = res.slantRange_m * 1.0e-3f;
    const float fspl_dB = computeFreeSpaceLoss(res.slantRange_m, jammer.freq_MHz);
    switch (jammer.propModel) {
        case PropagationModelType::FREE_SPACE:
            res.pathLoss_dB = fspl_dB;
            break;
        case PropagationModelType::HATA_URBAN_SMALL_MED_CITY:
        case PropagationModelType::HATA_URBAN_LARGE_CITY:
        case PropagationModelType::HATA_SUBURBAN:
        case PropagationModelType::HATA_OPEN_AREA:
            res.pathLoss_dB = computeHataLoss(d_km, jammer.freq_MHz, jammer.baseHeightAGL_m,
                                              mobileHeightAGL_m, jammer.propModel);
            break;
        case PropagationModelType::ERCEG_HILLY_MOD_HEAVY_TREE:
        case PropagationModelType::ERCEG_HILLY_LIGHT_TREE:
        case PropagationModelType::ERCEG_FLAT_LIGHT_TREE:
            res.pathLoss_dB = computeErcegLoss(res.slantRange_m, jammer.freq_MHz,
                                               jammer.baseHeightAGL_m, jammer.propModel);
            break;
    }

    // Section 9.4.1 - 9.4.3 - Excess Blockage and Beyond-Horizon Diffraction Loss (Kaplan)
    res.blockageLoss_dB = computeBlockageLoss(jammer);
    if (res.isBeyondHorizon) {
        res.blockageLoss_dB += 40.0f;
    }

    // Eq. (9.26) - Received Incident Jammer Power at Antenna Input (Kaplan p. 570)
    res.incidentPower_Jr_dBW = res.eirp_dBW - (res.pathLoss_dB + res.blockageLoss_dB);

    return res;
}