#include "JammingTrackingLoop.h"
#include <algorithm>

JammingTrackingLoop::JammingTrackingLoop() {}

// =========================================================================================
// Helper: Inisialisasi Konfigurasi Tracking Loop Default sesuai Kaplan (PDF 2) Section 8.9
//         dan Kaplan (PDF 1) Table 9.4 (L1 C/A threshold ~27 dB-Hz, Pilot ~17 dB-Hz)
// =========================================================================================
TrackingLoopConfig JammingTrackingLoop::createDefaultTrackingConfig(GNSSSignalType sigType,
                                                                    ReceiverAidingMode aidMode) {
    TrackingLoopConfig cfg = {};
    cfg.aidingMode = aidMode;

    // Tentukan apakah sinyal memiliki kanal pilot dataless (Kaplan PDF 1 Table 9.4-9.5)
    const bool isPilot = (sigType == GNSSSignalType::GPS_L2_CL ||
                          sigType == GNSSSignalType::GPS_L5_Q5 ||
                          sigType == GNSSSignalType::GPS_L1_M_TDDM ||
                          sigType == GNSSSignalType::GPS_L1C_PILOT ||
                          sigType == GNSSSignalType::GALILEO_E1_OS);

    cfg.usePilotChannel      = isPilot;
    cfg.pllOrder             = 3;
    cfg.pllBandwidth_Hz      = (aidMode == ReceiverAidingMode::INS_VELOCITY_AIDED) ? 2.0f : 15.0f;
    cfg.pllIntegrationTime_s = (sigType == GNSSSignalType::GPS_L1_CA) ? 0.020f : 0.010f;

    // Parameter Osilator & Getaran Standar (Kaplan PDF 2 Hal. 478-479, contoh Fig. 8.70)
    cfg.oscVibSensitivity_g  = 1.0e-9f;
    cfg.vibPSD_g2_Hz         = 0.005f;
    cfg.vibFreqMin_Hz        = 20.0f;
    cfg.vibFreqMax_Hz        = 2000.0f;
    cfg.oscAllanDeviation    = 1.0e-10f;

    // Parameter FLL Backup (Kaplan PDF 2 Section 8.9.8, p. 485)
    cfg.enableFLLBackup      = true;
    cfg.fllBandwidth_Hz      = 2.0f;
    cfg.fllIntegrationTime_s = 0.010f;

    // Parameter DLL (Kaplan PDF 2 Section 8.9.9, Fig. 8.74, p. 489-490)
    cfg.useCoherentDLL          = isPilot;
    cfg.dllBandwidth_Hz         = 0.1f;
    cfg.dllIntegrationTime_s    = isPilot ? 0.100f : 0.020f;
    cfg.correlatorSpacing_chips = (sigType == GNSSSignalType::GPS_L1_M_TDDM) ? 0.125f : 1.0f;

    // Geometri DOP Nominal
    cfg.hdop = 1.2f;
    cfg.vdop = 2.0f;

    return cfg;
}

// =========================================================================================
// [1] Kaplan (PDF 2) Ch. 8, Section 8.9.2 - Carrier Frequency & Wavelength (p. 476 / PDF p. 3)
// =========================================================================================
void JammingTrackingLoop::getCarrierFrequency(GNSSSignalType sigType,
                                              double& fL_Hz,
                                              double& lambdaL_m) const {
    switch (sigType) {
        case GNSSSignalType::GPS_L2_CL:
            fL_Hz = 1227.60e6;
            break;
        case GNSSSignalType::GPS_L5_Q5:
            fL_Hz = 1176.45e6;
            break;
        case GNSSSignalType::GPS_L1_CA:
        case GNSSSignalType::GPS_L1_PY:
        case GNSSSignalType::GPS_L1_M_TDDM:
        case GNSSSignalType::GPS_L1C_PILOT:
        case GNSSSignalType::GALILEO_E1_OS:
        default:
            fL_Hz = 1575.42e6;
            break;
    }
    lambdaL_m = C_LIGHT / fL_Hz;
}

// =========================================================================================
// [2] Kaplan (PDF 2) Ch. 8, Section 8.9.3, Eq. (8.73) - Vibration-Induced Oscillator Phase Noise
//     Lokasi: Halaman Buku 478 (Halaman PDF 5)
//     Rumus : sigma_v = (360 * f_L * S_v / (2 * pi)) * sqrt(P_v * (1/f_min - 1/f_max)) (deg)
// =========================================================================================
float JammingTrackingLoop::computeVibrationPhaseError(double fL_Hz,
                                                      const TrackingLoopConfig& trkCfg) const {
    if (trkCfg.vibPSD_g2_Hz <= 0.0f || trkCfg.oscVibSensitivity_g <= 0.0f) {
        return 0.0f;
    }

    const double f_min = std::max(static_cast<double>(trkCfg.vibFreqMin_Hz), 1.0);
    const double f_max = std::max(static_cast<double>(trkCfg.vibFreqMax_Hz), f_min + 1.0);
    const double P_v   = static_cast<double>(trkCfg.vibPSD_g2_Hz);
    const double S_v   = static_cast<double>(trkCfg.oscVibSensitivity_g);

    const double integral_val = P_v * ((1.0 / f_min) - (1.0 / f_max));
    const double sigma_v = (360.0 * fL_Hz * S_v / (2.0 * PI_VAL)) * std::sqrt(std::max(integral_val, 0.0));
    return static_cast<float>(sigma_v);
}

// =========================================================================================
// [3] Kaplan (PDF 2) Ch. 8, Section 8.9.4, Eq. (8.76) & (8.77) - Allan Deviation Phase Noise
//     Lokasi: Halaman Buku 479 (Halaman PDF 6)
//     Rumus : theta_A2 = 144 * sigma_A * f_L / B_n (Orde-2)
//             theta_A3 = 160 * sigma_A * f_L / B_n (Orde-3)
// =========================================================================================
float JammingTrackingLoop::computeAllanPhaseError(double fL_Hz,
                                                  const TrackingLoopConfig& trkCfg) const {
    const double Bn      = std::max(static_cast<double>(trkCfg.pllBandwidth_Hz), 0.1);
    const double sigma_A = std::max(static_cast<double>(trkCfg.oscAllanDeviation), 0.0);
    const double coeff   = (trkCfg.pllOrder == 2) ? 144.0 : 160.0;

    const double theta_A = coeff * (sigma_A * fL_Hz) / Bn;
    return static_cast<float>(theta_A);
}

// =========================================================================================
// [4] Kaplan (PDF 2) Ch. 8, Section 8.9.5, Eq. (8.78) & (8.79) - PLL Dynamic Stress Error (3-sigma)
//     Lokasi: Halaman Buku 481 (Halaman PDF 8)
//     Rumus : theta_e2 = 0.2809 * (d2R/dt2) / B_n^2  (Orde-2)
//             theta_e3 = 0.4828 * (d3R/dt3) / B_n^3  (Orde-3)
// =========================================================================================
float JammingTrackingLoop::computePLLDynamicStress(double lambdaL_m,
                                                   const TrackingLoopConfig& trkCfg,
                                                   float losAccel_m_s2,
                                                   float losJerk_m_s3) const {
    // Kaplan (PDF 1) Section 9.2.3.2, p. 587 & (PDF 2) Section 8.9.5, p. 481:
    // External INS velocity aiding removes dynamic stress from the closed carrier loop
    const double aidingFactor = (trkCfg.aidingMode == ReceiverAidingMode::INS_VELOCITY_AIDED) ? 0.05 : 1.0;
    const double Bn           = std::max(static_cast<double>(trkCfg.pllBandwidth_Hz), 0.1);
    const double deg_per_m    = 360.0 / lambdaL_m;

    if (trkCfg.pllOrder == 2) {
        // Eq. (8.78) - Second-order loop dynamic stress error
        const double d2R_dt2 = std::abs(static_cast<double>(losAccel_m_s2)) * aidingFactor * deg_per_m;
        return static_cast<float>(0.2809 * d2R_dt2 / (Bn * Bn));
    } else {
        // Eq. (8.79) - Third-order loop dynamic stress error
        const double d3R_dt3 = std::abs(static_cast<double>(losJerk_m_s3)) * aidingFactor * deg_per_m;
        return static_cast<float>(0.4828 * d3R_dt3 / (Bn * Bn * Bn));
    }
}

// =========================================================================================
// [5] Kaplan (PDF 2) Ch. 8, Section 8.9.2, Eq. (8.70) - PLL Thermal Noise Error (1-sigma, deg)
//     Lokasi: Halaman Buku 475 (Halaman PDF 2)
// =========================================================================================
float JammingTrackingLoop::computePLLThermalNoise(double cn0_linear_Hz,
                                                  const TrackingLoopConfig& trkCfg) const {
    const double safe_cn0 = std::max(cn0_linear_Hz, 1.0e-6);
    const double Bn       = std::max(static_cast<double>(trkCfg.pllBandwidth_Hz), 0.1);
    const double T        = std::max(static_cast<double>(trkCfg.pllIntegrationTime_s), 1.0e-4);

    double squaringTerm = 1.0;
    if (!trkCfg.usePilotChannel) {
        // Data channel (Costas PLL): squaring loss = 1 / (2 * T * C/N0)
        squaringTerm = 1.0 + (1.0 / (2.0 * T * safe_cn0));
    }

    const double sigma_tPLL = (360.0 / (2.0 * PI_VAL)) * std::sqrt((Bn / safe_cn0) * squaringTerm);
    return static_cast<float>(sigma_tPLL);
}

// =========================================================================================
// [6] Kaplan (PDF 2) Ch. 8, Section 8.9.8, Eq. (8.82)-(8.85) - FLL Thermal & Dynamic Errors
//     Lokasi: Halaman Buku 484-485 (Halaman PDF 11-12)
// =========================================================================================
void JammingTrackingLoop::computeFLLMetrics(double cn0_linear_Hz, double cn0_dBHz, double lambdaL_m,
                                            const TrackingLoopConfig& trkCfg, float losJerk_m_s3,
                                            float& sigma_tFLL_Hz, float& sigma_tFLL_m_s, float& fe_3sig_Hz) const {
    const double safe_cn0 = std::max(cn0_linear_Hz, 1.0e-6);
    const double Bn       = std::max(static_cast<double>(trkCfg.fllBandwidth_Hz), 0.1);
    const double T        = std::max(static_cast<double>(trkCfg.fllIntegrationTime_s), 1.0e-4);

    // Kaplan p. 485: F = 1 at high C/N0, F = 2 near threshold
    const double F = (cn0_dBHz < 30.0) ? 2.0 : 1.0;

    // Eq. (8.83) - FLL thermal noise frequency error (Hz)
    const double sig_Hz = (1.0 / (2.0 * PI_VAL * T)) *
                          std::sqrt(((4.0 * F * Bn) / safe_cn0) * (1.0 + (1.0 / (T * safe_cn0))));

    // Eq. (8.84) - FLL thermal noise velocity / pseudorange-rate error (m/s)
    const double sig_ms = lambdaL_m * sig_Hz;

    // Eq. (8.85) - Second-order FLL dynamic stress error (Hz, 3-sigma):
    // omega_0 = B_n / 0.53 (Table 8.23, Kaplan p. 485)
    const double aidingFactor = (trkCfg.aidingMode == ReceiverAidingMode::INS_VELOCITY_AIDED) ? 0.05 : 1.0;
    const double omega0       = Bn / 0.53;
    const double d3R_dt3      = std::abs(static_cast<double>(losJerk_m_s3)) * aidingFactor * (360.0 / lambdaL_m);
    const double fe_Hz        = d3R_dt3 / (360.0 * omega0 * omega0);

    sigma_tFLL_Hz  = static_cast<float>(sig_Hz);
    sigma_tFLL_m_s = static_cast<float>(sig_ms);
    fe_3sig_Hz     = static_cast<float>(fe_Hz);
}

// =========================================================================================
// [7] Kaplan (PDF 2) Ch. 8, Section 8.9.9 Eq. (8.90) & Section 8.9.10 Eq. (8.93)
//     Lokasi: Halaman Buku 488 (PDF p. 15) & Halaman Buku 493 (PDF p. 20)
//     Deskripsi: DLL 1-sigma thermal noise code tracking error (chips)
// =========================================================================================
float JammingTrackingLoop::computeDLLThermalNoiseChips(double cn0_linear_Hz,
                                                       GNSSSignalType sigType,
                                                       float Rc_chips_s,
                                                       float Bfe_Hz,
                                                       const TrackingLoopConfig& trkCfg,
                                                       bool isCoherentActive) const {
    const double safe_cn0 = std::max(cn0_linear_Hz, 1.0e-6);
    const double Bn       = std::max(static_cast<double>(trkCfg.dllBandwidth_Hz), 0.01);
    const double T        = std::max(static_cast<double>(trkCfg.dllIntegrationTime_s), 1.0e-3);
    const double Rc       = static_cast<double>(Rc_chips_s);
    const double Tc       = 1.0 / Rc;
    const double Bfe      = std::max(static_cast<double>(Bfe_Hz), 1.0e5);

    // Kasus Khusus: GPS M-Code BOC(10,5) -> Kaplan Eq. (8.93), Hal. Buku 493 (Hal. PDF 20)
    if (sigType == GNSSSignalType::GPS_L1_M_TDDM) {
        double term1 = 0.0;
        double squaringLoss = 1.0;
        const double clamped_Bfe = std::max(16.0e6, std::min(Bfe, 30.0e6));

        if (clamped_Bfe <= 24.5e6) {
            const double bw_term = 0.66 * clamped_Bfe - 7.7e6;
            term1 = Bn / ((2.0 * PI_VAL) * (2.0 * PI_VAL) * safe_cn0 * bw_term * bw_term);
            if (!isCoherentActive) {
                const double denom_sq = std::max(7.3e-8 * clamped_Bfe - 0.96, 0.1) * T * safe_cn0;
                squaringLoss = 1.0 + (1.0 / denom_sq);
            }
        } else {
            const double bw_term = 0.007 * clamped_Bfe + 8.4e6;
            term1 = Bn / ((2.0 * PI_VAL) * (2.0 * PI_VAL) * safe_cn0 * bw_term * bw_term);
            if (!isCoherentActive) {
                squaringLoss = 1.0 + (1.0 / (0.837 * T * safe_cn0));
            }
        }
        return static_cast<float>(Rc * std::sqrt(term1 * squaringLoss));
    }

    // Kasus Utama: BPSK-R(n) (GPS L1 C/A, P(Y), L2 CL, L5 Q5, dll.) -> Kaplan Eq. (8.90), Hal. 488
    const double D       = std::max(0.05, std::min(static_cast<double>(trkCfg.correlatorSpacing_chips), 1.5));
    const double Bfe_Tc  = Bfe * Tc;
    const double inv_Bfe = 1.0 / Bfe_Tc;           // R_c / B_fe
    const double pi_Bfe  = PI_VAL * inv_Bfe;       // pi * R_c / B_fe

    double variance_chips2 = 0.0;
    if (D >= pi_Bfe) {
        // Cabang Atas Eq. (8.90): D >= pi * R_c / B_fe
        const double sq = isCoherentActive ? 1.0 : (1.0 + 2.0 / (T * safe_cn0 * (2.0 - D)));
        variance_chips2 = (Bn / (2.0 * safe_cn0)) * D * sq;
    } else if (D > inv_Bfe) {
        // Cabang Tengah Eq. (8.90): R_c / B_fe < D < pi * R_c / B_fe
        const double diff = D - inv_Bfe;
        const double shape_term = inv_Bfe + (Bfe_Tc / (PI_VAL - 1.0)) * (diff * diff);
        const double sq = isCoherentActive ? 1.0 : (1.0 + 2.0 / (T * safe_cn0 * (2.0 - D)));
        variance_chips2 = (Bn / (2.0 * safe_cn0)) * shape_term * sq;
    } else {
        // Cabang Bawah Eq. (8.90): D <= R_c / B_fe
        const double sq = isCoherentActive ? 1.0 : (1.0 + 1.0 / (T * safe_cn0));
        variance_chips2 = (Bn / (2.0 * safe_cn0)) * inv_Bfe * sq;
    }

    return static_cast<float>(std::sqrt(std::max(variance_chips2, 0.0)));
}

// =========================================================================================
// [8] Fungsi Utama Tahap 3: Evaluasi PLL/FLL/DLL, State Machine Loss-of-Lock, & Proyeksi DOP
// =========================================================================================
TrackingLoopResult JammingTrackingLoop::evaluateTrackingLoop(const ReceiverFrontEndConfig& rxCfg,
                                                             const CorrelatorResult& corrRes,
                                                             const TrackingLoopConfig& trkCfg,
                                                             float losAccel_m_s2,
                                                             float losJerk_m_s3) const {
    TrackingLoopResult res = {};

    double fL_Hz = 1575.42e6, lambdaL_m = 0.190293673;
    getCarrierFrequency(rxCfg.signalType, fL_Hz, lambdaL_m);

    const double eff_cn0_lin = static_cast<double>(corrRes.effective_CN0_linear_Hz);
    const double eff_cn0_dB  = static_cast<double>(corrRes.effective_CN0_dBHz);
    const double unj_cn0_dB  = static_cast<double>(corrRes.unjammed_CN0_dBHz);
    const double unj_cn0_lin = std::pow(10.0, unj_cn0_dB / 10.0);

    // -------------------------------------------------------------------------------------
    // 8A. Evaluasi PLL (Carrier Phase Loop) - Kaplan Eq. (8.69)-(8.79)
    // -------------------------------------------------------------------------------------
    res.sigma_tPLL_deg        = computePLLThermalNoise(eff_cn0_lin, trkCfg);
    res.sigma_v_deg           = computeVibrationPhaseError(fL_Hz, trkCfg);
    res.theta_A_deg           = computeAllanPhaseError(fL_Hz, trkCfg);
    res.theta_e_3sig_deg      = computePLLDynamicStress(lambdaL_m, trkCfg, losAccel_m_s2, losJerk_m_s3);
    res.pllThreshold_1sig_deg = trkCfg.usePilotChannel ? 30.0f : 15.0f;

    // Eq. (8.69): sigma_PLL = sqrt(sigma_tPLL^2 + sigma_v^2 + theta_A^2) + theta_e / 3
    const float pll_rss = std::sqrt(res.sigma_tPLL_deg * res.sigma_tPLL_deg +
                                    res.sigma_v_deg * res.sigma_v_deg +
                                    res.theta_A_deg * res.theta_A_deg);
    res.total_PLL_1sig_deg = pll_rss + (res.theta_e_3sig_deg / 3.0f);
    res.isPLLLocked        = (!corrRes.isFrontEndSaturated) &&
                             (res.total_PLL_1sig_deg <= res.pllThreshold_1sig_deg);

    // -------------------------------------------------------------------------------------
    // 8B. Evaluasi FLL (Carrier Frequency Loop / Pseudorange-Rate) - Kaplan Eq. (8.82)-(8.85)
    // -------------------------------------------------------------------------------------
    float sigma_rhodot_jam = 0.0f;
    computeFLLMetrics(eff_cn0_lin, eff_cn0_dB, lambdaL_m, trkCfg, losJerk_m_s3,
                      res.sigma_tFLL_Hz, sigma_rhodot_jam, res.fe_3sig_Hz);

    // Eq. (8.82): 3 * sigma_tFLL + f_e <= 1 / (4 * T_FLL)
    res.fllThreshold_3sig_Hz = 1.0f / (4.0f * std::max(trkCfg.fllIntegrationTime_s, 1.0e-4f));
    res.isFLLLocked          = (!corrRes.isFrontEndSaturated) &&
                               ((3.0f * res.sigma_tFLL_Hz + res.fe_3sig_Hz) <= res.fllThreshold_3sig_Hz);

    // Hitung baseline unjammed FLL untuk mendapatkan excess jamming velocity noise
    float unj_fll_Hz = 0.0f, sigma_rhodot_unj = 0.0f, unj_fe_Hz = 0.0f;
    computeFLLMetrics(unj_cn0_lin, unj_cn0_dB, lambdaL_m, trkCfg, losJerk_m_s3,
                      unj_fll_Hz, sigma_rhodot_unj, unj_fe_Hz);

    // -------------------------------------------------------------------------------------
    // 8C. Evaluasi DLL (Code Tracking Loop / Pseudorange) - Kaplan Eq. (8.86)-(8.93)
    // -------------------------------------------------------------------------------------
    // Coherent DLL hanya boleh aktif saat PLL benar-benar terkunci (Kaplan PDF 2 p. 491 & 495)
    const bool coherentNow = trkCfg.useCoherentDLL && res.isPLLLocked;

    res.sigma_tDLL_chips = computeDLLThermalNoiseChips(eff_cn0_lin, rxCfg.signalType,
                                                       corrRes.chipRate_Rc, rxCfg.frontEndBandwidth_Hz,
                                                       trkCfg, coherentNow);

    // Eq. (8.91) - DLL Dynamic Stress Error R_e (Kaplan p. 492-493):
    // Carrier-aided code meniadakan dynamic stress DLL selama carrier terkunci / aided
    const double meters_per_chip = C_LIGHT / static_cast<double>(corrRes.chipRate_Rc);
    if (res.isPLLLocked || res.isFLLLocked) {
        res.Re_3sig_chips = 0.0f;
    } else if (trkCfg.aidingMode == ReceiverAidingMode::INS_VELOCITY_AIDED) {
        // Sisa error kecil dari open-loop INS velocity aiding (5% residual jerk)
        const double omega0_dll = static_cast<double>(trkCfg.dllBandwidth_Hz) / 0.7845;
        const double jerk_chips = (std::abs(static_cast<double>(losJerk_m_s3)) * 0.05) / meters_per_chip;
        res.Re_3sig_chips       = static_cast<float>(jerk_chips / (omega0_dll * omega0_dll * omega0_dll));
    } else {
        res.Re_3sig_chips = 0.0f;
    }

    // Eq. (8.86): 3 * sigma_tDLL + R_e <= D / 2 (chips)
    res.dllThreshold_3sig_chips = 0.5f * trkCfg.correlatorSpacing_chips;
    res.isDLLLocked             = (!corrRes.isFrontEndSaturated) &&
                                  ((3.0f * res.sigma_tDLL_chips + res.Re_3sig_chips) <= res.dllThreshold_3sig_chips);

    // Hitung baseline unjammed DLL untuk mendapatkan excess jamming pseudorange noise
    const float unj_dll_chips = computeDLLThermalNoiseChips(unj_cn0_lin, rxCfg.signalType,
                                                            corrRes.chipRate_Rc, rxCfg.frontEndBandwidth_Hz,
                                                            trkCfg, trkCfg.useCoherentDLL);

    // -------------------------------------------------------------------------------------
    // 8D. State Machine Status Kunci Penerima - Kaplan (PDF 1) p. 561 & 587; (PDF 2) p. 486
    // -------------------------------------------------------------------------------------
    if (corrRes.isFrontEndSaturated || !res.isDLLLocked) {
        res.lockState         = TrackingLockState::LOST_LOCK;
        res.px4_fix_type      = 0;
        res.px4_vel_ned_valid = false;
    } else if (res.isPLLLocked) {
        res.lockState         = TrackingLockState::FULL_LOCK_PLL;
        res.px4_fix_type      = 3;
        res.px4_vel_ned_valid = true;
    } else if (trkCfg.enableFLLBackup && res.isFLLLocked) {
        res.lockState         = TrackingLockState::FLL_BACKUP_LOCK;
        res.px4_fix_type      = 3;
        res.px4_vel_ned_valid = true;
    } else if (trkCfg.aidingMode == ReceiverAidingMode::INS_VELOCITY_AIDED) {
        // Carrier loop terbuka, tetapi INS menjaga code DLL tetap mengunci (Kaplan PDF 1 p. 587)
        res.lockState         = TrackingLockState::OPEN_CARRIER_AIDED_DLL;
        res.px4_fix_type      = 3;
        res.px4_vel_ned_valid = false;
    } else {
        // Unaided receiver: DLL segera putus begitu carrier loop (PLL & FLL) kehilangan kunci (Kaplan PDF 1 p. 561)
        res.lockState         = TrackingLockState::LOST_LOCK;
        res.px4_fix_type      = 0;
        res.px4_vel_ned_valid = false;
    }

    // -------------------------------------------------------------------------------------
    // 8E. Konversi Error 1D (Meter & m/s) dan Proyeksi DOP ke Local NED Frame
    // -------------------------------------------------------------------------------------
    const float sigma_rho_jam = static_cast<float>(static_cast<double>(res.sigma_tDLL_chips) * meters_per_chip);
    const float sigma_rho_unj = static_cast<float>(static_cast<double>(unj_dll_chips) * meters_per_chip);

    res.sigma_rho_total_m       = sigma_rho_jam;
    res.sigma_rhodot_total_m_s  = sigma_rhodot_jam;

    // Excess jamming variance: sigma_excess = sqrt(max(0, sigma_jam^2 - sigma_unjammed^2))
    const float var_rho_ex    = std::max(0.0f, sigma_rho_jam * sigma_rho_jam - sigma_rho_unj * sigma_rho_unj);
    const float var_rhodot_ex = std::max(0.0f, sigma_rhodot_jam * sigma_rhodot_jam - sigma_rhodot_unj * sigma_rhodot_unj);

    res.sigma_rho_excess_m      = std::sqrt(var_rho_ex);
    res.sigma_rhodot_excess_m_s = std::sqrt(var_rhodot_ex);

    // Proyeksi DOP ke Local NED (NDOP = EDOP = HDOP / sqrt(2), VDOP = vdop)
    const float inv_sqrt2 = 0.70710678f;
    const float sig_pNE   = trkCfg.hdop * inv_sqrt2 * res.sigma_rho_excess_m;
    const float sig_pD    = trkCfg.vdop * res.sigma_rho_excess_m;
    const float sig_vNE   = trkCfg.hdop * inv_sqrt2 * res.sigma_rhodot_excess_m_s;
    const float sig_vD    = trkCfg.vdop * res.sigma_rhodot_excess_m_s;

    res.sigma_pos_excess_ned_m   = Eigen::Vector3f(sig_pNE, sig_pNE, sig_pD);
    res.sigma_vel_excess_ned_m_s = Eigen::Vector3f(sig_vNE, sig_vNE, sig_vD);

    // Metrik excess untuk kolom PX4 (eph, epv, s_variance_m_s)
    res.eph_excess_m     = trkCfg.hdop * res.sigma_rho_excess_m;
    res.epv_excess_m     = trkCfg.vdop * res.sigma_rho_excess_m;
    res.s_var_excess_m_s = trkCfg.hdop * res.sigma_rhodot_excess_m_s;

    return res;
}