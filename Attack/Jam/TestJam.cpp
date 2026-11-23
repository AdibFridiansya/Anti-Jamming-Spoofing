#ifndef EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_VECTORIZE
#endif

#ifndef EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#endif

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include "JammingPropagation.h"
#include "JammingCorrelator.h"
#include "JammingTrackingLoop.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static bool checkNear(const std::string& label, float actual, float expected, float tol) {
    const float err = std::abs(actual - expected);
    const bool pass = (err <= tol);
    std::cout << "  [" << (pass ? "PASS" : "FAIL") << "] "
              << std::left << std::setw(28) << label
              << " | Actual: " << std::right << std::setw(9) << std::fixed << std::setprecision(3) << actual
              << " | Target: " << std::setw(9) << expected
              << " | Tol: +/-" << std::setprecision(2) << tol << "\n";
    return pass;
}

static std::string lockStateToString(TrackingLockState s) {
    switch (s) {
        case TrackingLockState::FULL_LOCK_PLL:          return "FULL_LOCK_PLL";
        case TrackingLockState::FLL_BACKUP_LOCK:        return "FLL_BACKUP_LOCK";
        case TrackingLockState::OPEN_CARRIER_AIDED_DLL: return "OPEN_CARRIER_DLL";
        case TrackingLockState::LOST_LOCK:              return "LOST_LOCK";
        default:                                        return "UNKNOWN";
    }
}

int main() {
    JammingPropagation  propEngine;
    JammingCorrelator   corrEngine;
    JammingTrackingLoop trkEngine;

    const double DEG_TO_RAD = M_PI / 180.0;
    const double R_EARTH    = 6378137.0;

    // Posisi dasar wahana (Lat = -6.9 deg, Lon = 107.6 deg)
    const double L_b      = -6.9 * DEG_TO_RAD;
    const double lambda_b = 107.6 * DEG_TO_RAD;

    bool allPass = true;

    // =========================================================================
    // TEST 1: Free-Space Propagation (Kaplan PDF 1, Eq. 9.26-9.27, Hal. 571-572)
    // =========================================================================
    std::cout << "=====================================================================\n";
    std::cout << "[TEST 1] Validasi Free-Space Path Loss (Kaplan Hal. 571-572)\n";
    std::cout << "=====================================================================\n";
    {
        const double h_b = 100.0;
        const double d_target_m = 22700.0;

        const double e2 = 0.00669437999014;
        const double sinL = std::sin(L_b);
        const double RN = R_EARTH * (1.0 - e2) / std::pow(1.0 - e2 * sinL * sinL, 1.5);

        JammerNodeConfig jam = {};
        jam.isEnabled        = true;
        jam.startTime_s      = 0.0;
        jam.endTime_s        = 100.0;
        jam.L_j              = L_b + d_target_m / (RN + h_b);
        jam.lambda_j         = lambda_b;
        jam.h_j              = h_b;
        jam.baseHeightAGL_m  = 100.0f;
        jam.freq_MHz         = 1575.42f;
        jam.transmitPower_W  = 2.0f;
        jam.antennaGain_dBic = 3.0f;
        jam.propModel        = PropagationModelType::FREE_SPACE;

        Eigen::Matrix3f cn_b = Eigen::Matrix3f::Identity();
        PropagationResult pRes = propEngine.computePropagation(10.0, L_b, lambda_b, h_b, 100.0f, cn_b, jam);

        allPass &= checkNear("Slant Range (m)", pRes.slantRange_m, 22700.0f, 5.0f);
        allPass &= checkNear("EIRP (dBW)", pRes.eirp_dBW, 6.01f, 0.05f);
        allPass &= checkNear("Free-Space Loss Lp (dB)", pRes.pathLoss_dB, 123.52f, 0.10f);
        allPass &= checkNear("Incident Power Jr (dBW)", pRes.incidentPower_Jr_dBW, -117.50f, 0.10f);
    }

    // =========================================================================
    // TEST 2: Modified Hata Urban (Kaplan PDF 1, Eq. 9.29-9.30, Hal. 575)
    // =========================================================================
    std::cout << "\n=====================================================================\n";
    std::cout << "[TEST 2] Validasi Modified Hata Urban (Kaplan Hal. 575)\n";
    std::cout << "=====================================================================\n";
    {
        const double h_b = 1.5;
        const double d_target_m = 497.0;
        const double e2 = 0.00669437999014;
        const double sinL = std::sin(L_b);
        const double RN = R_EARTH * (1.0 - e2) / std::pow(1.0 - e2 * sinL * sinL, 1.5);

        JammerNodeConfig jam = {};
        jam.isEnabled        = true;
        jam.startTime_s      = 0.0;
        jam.endTime_s        = 100.0;
        jam.L_j              = L_b + d_target_m / (RN + h_b);
        jam.lambda_j         = lambda_b;
        jam.h_j              = h_b;
        jam.freq_MHz         = 1575.42f;
        jam.transmitPower_W  = 2.0f;
        jam.antennaGain_dBic = 3.0f;
        jam.baseHeightAGL_m  = 30.0f;
        jam.propModel        = PropagationModelType::HATA_URBAN_SMALL_MED_CITY;

        Eigen::Matrix3f cn_b = Eigen::Matrix3f::Identity();
        PropagationResult pRes = propEngine.computePropagation(10.0, L_b, lambda_b, h_b, 1.5f, cn_b, jam);

        allPass &= checkNear("Modified Hata Loss Lp1 (dB)", pRes.pathLoss_dB, 123.54f, 0.15f);
        allPass &= checkNear("Incident Power Jr (dBW)", pRes.incidentPower_Jr_dBW, -117.53f, 0.15f);
    }

    // =========================================================================
    // TEST 3: Correlator & Effective C/N0 (Kaplan PDF 1, Eq. 9.18-9.24, Hal. 562-564)
    // =========================================================================
    std::cout << "\n=====================================================================\n";
    std::cout << "[TEST 3] Validasi Front-End & Effective C/N0 (Kaplan Hal. 562-564)\n";
    std::cout << "=====================================================================\n";
    {
        ReceiverFrontEndConfig rxCfg = JammingCorrelator::createDefaultReceiverConfig(GNSSSignalType::GPS_L1_CA);

        ActiveJammerInput jamIn = {};
        jamIn.prop.isJammerActive       = true;
        jamIn.prop.incidentPower_Jr_dBW = -117.50f;
        jamIn.prop.elevAngle_body_rad   = static_cast<float>(5.0 * DEG_TO_RAD);
        jamIn.sig.waveform              = JammerWaveformType::WIDEBAND_BLWN_NULL;
        jamIn.sig.polarization          = JammerPolarization::RHCP;

        std::vector<ActiveJammerInput> list;
        list.push_back(jamIn);

        CorrelatorResult cRes = corrEngine.evaluateCorrelator(rxCfg, list);

        allPass &= checkNear("T_receiver (K)", cRes.T_receiver_K, 169.6f, 0.2f);
        allPass &= checkNear("N0 (dBW/Hz)", cRes.N0_dBW_Hz, -204.29f, 0.05f);
        allPass &= checkNear("Unjammed C/N0 (dB-Hz)", cRes.unjammed_CN0_dBHz, 45.29f, 0.05f);
        allPass &= checkNear("J/S Ratio (dB)", cRes.total_JS_dB, 41.00f, 0.10f);
        allPass &= checkNear("Effective C/N0 (dB-Hz)", cRes.effective_CN0_dBHz, 27.00f, 0.10f);
    }

    // =========================================================================
    // TEST 4: Efek Antenna Tilt (Roll Drone) & Kapasitas Null CRPA (N - 1)
    // =========================================================================
    std::cout << "\n=====================================================================\n";
    std::cout << "[TEST 4] Validasi Fisika Antenna Tilt & Batas CRPA (N - 1 Nulls)\n";
    std::cout << "=====================================================================\n";
    {
        const double h_b = 100.0;
        const double e2 = 0.00669437999014;
        const double sinL = std::sin(L_b);
        const double RE = R_EARTH / std::sqrt(1.0 - e2 * sinL * sinL);

        JammerNodeConfig jam = {};
        jam.isEnabled        = true;
        jam.startTime_s      = 0.0;
        jam.endTime_s        = 100.0;
        jam.L_j              = L_b;
        jam.lambda_j         = lambda_b + 100.0 / ((RE + h_b) * std::cos(L_b));
        jam.h_j              = 0.0;
        jam.baseHeightAGL_m  = 1.0f;
        jam.freq_MHz         = 1575.42f;
        jam.transmitPower_W  = 0.001f;
        jam.antennaGain_dBic = 0.0f;
        jam.propModel        = PropagationModelType::FREE_SPACE;

        Eigen::Matrix3f cn_b_level = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f cn_b_roll  = Eigen::AngleAxisf(static_cast<float>(45.0 * DEG_TO_RAD), Eigen::Vector3f::UnitX()).toRotationMatrix();

        PropagationResult pLevel = propEngine.computePropagation(10.0, L_b, lambda_b, h_b, 100.0f, cn_b_level, jam);
        PropagationResult pRoll  = propEngine.computePropagation(10.0, L_b, lambda_b, h_b, 100.0f, cn_b_roll, jam);

        ReceiverFrontEndConfig rxCfg = JammingCorrelator::createDefaultReceiverConfig(GNSSSignalType::GPS_L1_CA);
        JammerSignalConfig jSig = { JammerWaveformType::WIDEBAND_BLWN_NULL, JammerPolarization::RHCP, 0.0f, 1.0f };

        std::vector<ActiveJammerInput> listLevel;
        listLevel.push_back({ pLevel, jSig });
        std::vector<ActiveJammerInput> listRoll;
        listRoll.push_back({ pRoll, jSig });

        CorrelatorResult cLevel = corrEngine.evaluateCorrelator(rxCfg, listLevel);
        CorrelatorResult cRoll  = corrEngine.evaluateCorrelator(rxCfg, listRoll);

        std::cout << "  [INFO] Level Flight -> Elev Body: " << std::setprecision(1) << pLevel.elevAngle_body_rad * 57.2958f
                  << " deg | (C/N0)_eff: " << std::setprecision(2) << cLevel.effective_CN0_dBHz << " dB-Hz\n";
        std::cout << "  [INFO] Roll +45 deg -> Elev Body: " << std::setprecision(1) << pRoll.elevAngle_body_rad * 57.2958f
                  << " deg | (C/N0)_eff: " << std::setprecision(2) << cRoll.effective_CN0_dBHz << " dB-Hz\n";
        const bool tiltPass = (cRoll.effective_CN0_dBHz < cLevel.effective_CN0_dBHz - 5.0f);
        std::cout << "  [" << (tiltPass ? "PASS" : "FAIL") << "] Antenna Tilt memperparah jamming saat wahana miring ke arah jammer\n";
        allPass &= tiltPass;

        rxCfg.enableCRPA       = true;
        rxCfg.crpaNumElements  = 4;
        rxCfg.crpaNullDepth_dB = -40.0f;

        ActiveJammerInput strongJam = {};
        strongJam.prop.isJammerActive       = true;
        strongJam.prop.incidentPower_Jr_dBW = -117.5f;
        strongJam.prop.elevAngle_body_rad   = static_cast<float>(5.0 * DEG_TO_RAD);
        strongJam.sig                       = jSig;

        std::vector<ActiveJammerInput> threeJammers(3, strongJam);
        std::vector<ActiveJammerInput> fourJammers(4, strongJam);

        CorrelatorResult c3 = corrEngine.evaluateCorrelator(rxCfg, threeJammers);
        CorrelatorResult c4 = corrEngine.evaluateCorrelator(rxCfg, fourJammers);

        std::cout << "  [INFO] CRPA(4 elem) vs 3 Jammers -> Nulled: " << c3.crpaNulledCount
                  << " | (C/N0)_eff: " << c3.effective_CN0_dBHz << " dB-Hz\n";
        std::cout << "  [INFO] CRPA(4 elem) vs 4 Jammers -> Nulled: " << c4.crpaNulledCount
                  << " | (C/N0)_eff: " << c4.effective_CN0_dBHz << " dB-Hz\n";
        const bool crpaPass = (c3.crpaNulledCount == 3 && c3.effective_CN0_dBHz > 44.0f &&
                               c4.crpaNulledCount == 3 && c4.effective_CN0_dBHz < 28.0f);
        std::cout << "  [" << (crpaPass ? "PASS" : "FAIL") << "] CRPA 4-elemen menahan 3 jammer & jebol pada jammer ke-4\n";
        allPass &= crpaPass;
    }

    // =========================================================================
    // TEST 5: Validasi Osilator & Stres Dinamika PLL/FLL (Kaplan PDF 2, Hal. 478-485)
    // =========================================================================
    std::cout << "\n=====================================================================\n";
    std::cout << "[TEST 5] Validasi Osilator & Dynamic Stress (Kaplan PDF 2 Hal. 478-485)\n";
    std::cout << "=====================================================================\n";
    {
        ReceiverFrontEndConfig rxCfg = JammingCorrelator::createDefaultReceiverConfig(GNSSSignalType::GPS_L1_CA);
        TrackingLoopConfig trkCfg    = JammingTrackingLoop::createDefaultTrackingConfig(GNSSSignalType::GPS_L1_CA, ReceiverAidingMode::UNAIDED);

        // Parameter contoh soal resmi Kaplan PDF 2 Hal. 478, 479, 481, & 485
        trkCfg.pllOrder             = 3;
        trkCfg.pllBandwidth_Hz      = 15.0f;    // Hal. 479 & 481
        trkCfg.oscVibSensitivity_g  = 1.0e-9f;  // Hal. 478: Sv = 1e-9 parts/g
        trkCfg.vibPSD_g2_Hz         = 0.005f;   // Hal. 478: P(fm) = 0.005 g^2/Hz
        trkCfg.vibFreqMin_Hz        = 20.0f;    // Hal. 478: 20 Hz
        trkCfg.vibFreqMax_Hz        = 2000.0f;  // Hal. 478: 2000 Hz
        trkCfg.oscAllanDeviation    = 1.0e-10f; // Hal. 479: sigma_A = 1e-10
        trkCfg.fllBandwidth_Hz      = 2.0f;     // Hal. 485: Bn_FLL = 2 Hz
        trkCfg.fllIntegrationTime_s = 0.005f;   // Hal. 485: T_FLL = 5 ms -> threshold = 50 Hz

        std::vector<ActiveJammerInput> noJam;
        CorrelatorResult cUnj   = corrEngine.evaluateCorrelator(rxCfg, noJam);
        TrackingLoopResult tRes = trkEngine.evaluateTrackingLoop(rxCfg, cUnj, trkCfg, 0.0f, 98.0f);

        allPass &= checkNear("Vibration sigma_v (deg)", tRes.sigma_v_deg, 1.42f, 0.02f);         // Eq. (8.73) Hal. 478
        allPass &= checkNear("Allan Dev theta_A3 (deg)", tRes.theta_A_deg, 1.68f, 0.02f);        // Eq. (8.77) Hal. 479
        allPass &= checkNear("PLL 3-sig Stress (deg)", tRes.theta_e_3sig_deg, 26.52f, 0.05f);    // Eq. (8.79) Hal. 481
        allPass &= checkNear("FLL 3-sig Stress fe (Hz)", tRes.fe_3sig_Hz, 36.16f, 0.25f);        // Eq. (8.85) Hal. 485
        allPass &= checkNear("FLL 3-sig Thresh (Hz)", tRes.fllThreshold_3sig_Hz, 50.00f, 0.05f); // Eq. (8.82) Hal. 484
    }

    // =========================================================================
    // TEST 6: Validasi DLL Error (Kaplan PDF 2 Fig. 8.74a Hal. 490) & Zero Excess
    // =========================================================================
    std::cout << "\n=====================================================================\n";
    std::cout << "[TEST 6] Validasi DLL Error Fig. 8.74(a) & Zero-Excess Saat Aman\n";
    std::cout << "=====================================================================\n";
    {
        ReceiverFrontEndConfig rxCfg = JammingCorrelator::createDefaultReceiverConfig(GNSSSignalType::GPS_L1_CA);
        rxCfg.frontEndBandwidth_Hz   = 1.7e6f; // Fig. 8.74(a): B_fe = 1.7 MHz

        TrackingLoopConfig trkCfg = JammingTrackingLoop::createDefaultTrackingConfig(GNSSSignalType::GPS_L1_CA, ReceiverAidingMode::UNAIDED);
        trkCfg.dllBandwidth_Hz         = 0.1f;
        trkCfg.dllIntegrationTime_s    = 0.020f;
        trkCfg.correlatorSpacing_chips = 1.0f;

        // 6A. Kondisi aman (tanpa jammer) -> excess noise HARUS persis 0.0
        std::vector<ActiveJammerInput> noJam;
        CorrelatorResult cSafe   = corrEngine.evaluateCorrelator(rxCfg, noJam);
        TrackingLoopResult tSafe = trkEngine.evaluateTrackingLoop(rxCfg, cSafe, trkCfg, 0.0f, 0.0f);

        allPass &= checkNear("Unjammed Excess Pos (m)", tSafe.sigma_rho_excess_m, 0.0f, 0.001f);
        allPass &= checkNear("Unjammed Excess Vel (m/s)", tSafe.sigma_rhodot_excess_m_s, 0.0f, 0.001f);

        // 6B. Kondisi batas ambang Fig. 8.74(a): C/N0 = 10.9 dB-Hz -> sigma_rho ~ 48.06 m (1/6 chip = 48.8 m)
        CorrelatorResult cLow = cSafe;
        cLow.effective_CN0_dBHz      = 10.9f;
        cLow.effective_CN0_linear_Hz = static_cast<float>(std::pow(10.0, 10.9 / 10.0));

        TrackingLoopResult tLow = trkEngine.evaluateTrackingLoop(rxCfg, cLow, trkCfg, 0.0f, 0.0f);
        allPass &= checkNear("DLL Error @10.9 dB-Hz (m)", tLow.sigma_rho_total_m, 48.06f, 1.00f);
        allPass &= checkNear("DLL 3-sig Thresh (chips)", tLow.dllThreshold_3sig_chips, 0.50f, 0.01f);
    }

    // =========================================================================
    // TEST 7: Uji End-to-End (Tahap 1 -> 2 -> 3) Unaided vs INS Velocity-Aided
    // =========================================================================
    std::cout << "\n=====================================================================\n";
    std::cout << "[TEST 7] Uji End-to-End Transisi Lock: Unaided vs INS Velocity-Aided\n";
    std::cout << "=====================================================================\n";
    {
        ReceiverFrontEndConfig rxCfg = JammingCorrelator::createDefaultReceiverConfig(GNSSSignalType::GPS_L1_CA);
        TrackingLoopConfig trkUnaid  = JammingTrackingLoop::createDefaultTrackingConfig(GNSSSignalType::GPS_L1_CA, ReceiverAidingMode::UNAIDED);
        TrackingLoopConfig trkAided  = JammingTrackingLoop::createDefaultTrackingConfig(GNSSSignalType::GPS_L1_CA, ReceiverAidingMode::INS_VELOCITY_AIDED);
        trkAided.oscAllanDeviation   = 1.0e-11f; // Kaplan PDF 2 Fig. 8.70 (Hal. 482): Osilator presisi tinggi untuk narrowband 2-Hz PLL

        const double h_b  = 100.0;
        const double e2   = 0.00669437999014;
        const double sinL = std::sin(L_b);
        const double RN   = R_EARTH * (1.0 - e2) / std::pow(1.0 - e2 * sinL * sinL, 1.5);
        Eigen::Matrix3f cn_b = Eigen::Matrix3f::Identity();

        const double testDistances_m[3] = { 30000.0, 5000.0, 2000.0 };
        TrackingLockState unaidStates[3];
        TrackingLockState aidedStates[3];

        for (int i = 0; i < 3; ++i) {
            JammerNodeConfig jam = {};
            jam.isEnabled        = true;
            jam.startTime_s      = 0.0;
            jam.endTime_s        = 100.0;
            jam.L_j              = L_b + testDistances_m[i] / (RN + h_b);
            jam.lambda_j         = lambda_b;
            jam.h_j              = h_b + testDistances_m[i] * std::tan(5.0 * DEG_TO_RAD); // Elevasi +5 deg
            jam.baseHeightAGL_m  = 100.0f;
            jam.freq_MHz         = 1575.42f;
            jam.transmitPower_W  = 2.0f;
            jam.antennaGain_dBic = 3.0f;
            jam.propModel        = PropagationModelType::FREE_SPACE;

            PropagationResult pRes  = propEngine.computePropagation(10.0, L_b, lambda_b, h_b, 100.0f, cn_b, jam);
            JammerSignalConfig jSig = { JammerWaveformType::WIDEBAND_BLWN_NULL, JammerPolarization::RHCP, 0.0f, 1.0f };

            std::vector<ActiveJammerInput> jamList;
            jamList.push_back({ pRes, jSig });

            CorrelatorResult cRes     = corrEngine.evaluateCorrelator(rxCfg, jamList);
            TrackingLoopResult rUnaid = trkEngine.evaluateTrackingLoop(rxCfg, cRes, trkUnaid, 0.0f, 0.0f);
            TrackingLoopResult rAided = trkEngine.evaluateTrackingLoop(rxCfg, cRes, trkAided, 0.0f, 0.0f);

            unaidStates[i] = rUnaid.lockState;
            aidedStates[i] = rAided.lockState;

            std::cout << "  [DIST " << std::right << std::setw(5) << std::fixed << std::setprecision(0) << testDistances_m[i] << " m] "
                      << "(C/N0)_eff=" << std::setw(5) << std::setprecision(2) << cRes.effective_CN0_dBHz << " dB-Hz"
                      << " | Unaided: " << std::left << std::setw(16) << lockStateToString(rUnaid.lockState)
                      << " | INS-Aided: " << std::setw(16) << lockStateToString(rAided.lockState)
                      << " | sig_pN=" << std::right << std::setw(5) << rAided.sigma_pos_excess_ned_m(0) << " m\n";
        }

        // Validasi logika transisi sesuai Kaplan PDF 1 Hal. 561 & 587:
        // 30 km: Keduanya FULL_LOCK_PLL
        //  5 km: Unaided LOST_LOCK, INS-Aided bertahan pada OPEN_CARRIER_AIDED_DLL
        //  2 km: Keduanya LOST_LOCK (Free-Inertial Coasting)
        const bool transPass = (unaidStates[0] == TrackingLockState::FULL_LOCK_PLL &&
                                aidedStates[0] == TrackingLockState::FULL_LOCK_PLL &&
                                unaidStates[1] == TrackingLockState::LOST_LOCK &&
                                aidedStates[1] == TrackingLockState::OPEN_CARRIER_AIDED_DLL &&
                                unaidStates[2] == TrackingLockState::LOST_LOCK &&
                                aidedStates[2] == TrackingLockState::LOST_LOCK);

        std::cout << "  [" << (transPass ? "PASS" : "FAIL")
                  << "] Transisi Lock Unaided vs INS-Aided sesuai Kaplan Section 9.2.3.2\n";
        allPass &= transPass;
    }

    std::cout << "\n=====================================================================\n";
    std::cout << "HASIL AKHIR VERIFIKASI TAHAP 1, 2, & 3: " << (allPass ? "SEMUA PASS!" : "ADA YANG GAGAL!") << "\n";
    std::cout << "=====================================================================\n";
    return allPass ? 0 : 1;
}