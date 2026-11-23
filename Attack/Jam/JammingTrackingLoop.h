#ifndef JAMMING_TRACKING_LOOP_H
#define JAMMING_TRACKING_LOOP_H

#include "JammingCorrelator.h"
#include <Eigen/Dense>
#include <cmath>

// Mode Aiding dari INS ke Tracking Loop GNSS - Kaplan (PDF 1) Section 9.2.3.2, p. 587 (PDF p. 39)
enum class ReceiverAidingMode {
    UNAIDED,                    // Penerima komersial standar (seperti modul GPS PX4): DLL putus bila carrier putus
    INS_VELOCITY_AIDED          // INS membantu Doppler: DLL tetap bertahan (open carrier) saat PLL/FLL putus
};

// Status Kunci Penerima GNSS (State Machine) - Kaplan (PDF 1) p. 561 & 587; (PDF 2) p. 486
enum class TrackingLockState {
    FULL_LOCK_PLL,              // PLL + DLL terkunci normal (fix_type = 3, vel_ned_valid = true)
    FLL_BACKUP_LOCK,            // PLL putus, FLL backup + DLL masih terkunci (fix_type = 3, vel_ned_valid = true)
    OPEN_CARRIER_AIDED_DLL,     // Hanya mode INS_VELOCITY_AIDED: Carrier putus, DLL masih terkunci (vel_ned_valid = false)
    LOST_LOCK                   // Total Loss of Lock / Free-Inertial Coasting (fix_type = 0, vel_ned_valid = false)
};

// Konfigurasi Parameter Tracking Loop, Osilator, & DOP Penerima
// Rujukan: Kaplan (PDF 2) Ch. 8, Section 8.9.1 - 8.9.10, p. 474-494 (PDF 2 p. 1-21)
struct TrackingLoopConfig {
    ReceiverAidingMode aidingMode;

    // Parameter PLL - Kaplan Section 8.9.1 - 8.9.5, p. 474-481
    bool usePilotChannel;           // true = 4-quadrant Pilot (thresh 30 deg), false = 2-quadrant Costas Data (thresh 15 deg)
    int pllOrder;                   // 2 = second-order PLL (Eq. 8.76, 8.78), 3 = third-order PLL (Eq. 8.77, 8.79)
    float pllBandwidth_Hz;          // B_n,PLL (Hz): misal 15.0 Hz (unaided) atau 2.0 Hz (INS aided)
    float pllIntegrationTime_s;     // T_PLL (s): misal 0.010 s atau 0.020 s untuk GPS L1 C/A

    // Parameter Osilator Referensi & Getaran Wahana - Kaplan Section 8.9.3 & 8.9.4, p. 478-479
    float oscVibSensitivity_g;      // S_v: Oscillator vibration sensitivity (parts/g), default 1.0e-9
    float vibPSD_g2_Hz;             // P(f_m): Flat random vibration PSD (g^2/Hz), default 0.005
    float vibFreqMin_Hz;            // f_min (Hz), default 20.0
    float vibFreqMax_Hz;            // f_max (Hz), default 2000.0
    float oscAllanDeviation;        // sigma_A(tau): Short-term Allan deviation (Delta f / f), default 1.0e-10

    // Parameter FLL (Backup Carrier & Pengukuran Doppler) - Kaplan Section 8.9.8, p. 484-485
    bool enableFLLBackup;           // true = gunakan FLL sebagai cadangan PLL (Kaplan p. 486)
    float fllBandwidth_Hz;          // B_n,FLL (Hz), default 2.0 Hz
    float fllIntegrationTime_s;     // T_FLL (s), default 0.005 s (5 ms) atau 0.010 s

    // Parameter DLL (Code Tracking & Pengukuran Pseudorange) - Kaplan Section 8.9.9, p. 486-493
    bool useCoherentDLL;            // true = coherent DLL tanpa squaring loss saat PLL terkunci (p. 488)
    float dllBandwidth_Hz;          // B_n,DLL (Hz), default 0.1 Hz
    float dllIntegrationTime_s;     // T_DLL (s), default 0.020 s (C/A) atau 0.100 s (Pilot)
    float correlatorSpacing_chips;  // D: Early-to-late spacing (chips), default 1.0 chip (atau 0.1 narrow)

    // Parameter Geometri DOP (Proyeksi 1D Range -> 3D Local NED)
    float hdop;                     // Horizontal Dilution of Precision, default 1.2
    float vdop;                     // Vertical Dilution of Precision, default 2.0
};

// Struktur Keluaran Tahap 3 (Status Kunci & Standar Deviasi Error NED untuk EKF/CSV)
struct TrackingLoopResult {
    TrackingLockState lockState;
    int px4_fix_type;               // 3 jika posisi valid, 0 jika LOST_LOCK (kompatibel main.cpp Anda)
    bool px4_vel_ned_valid;         // true jika Doppler/FLL valid, false jika carrier putus (kompatibel main.cpp)

    // Diagnostik Error PLL (deg 1-sigma & 3-sigma) - Kaplan Eq. (8.69)-(8.79)
    bool isPLLLocked;
    float sigma_tPLL_deg;           // PLL 1-sigma thermal noise (deg)
    float sigma_v_deg;              // Vibration-induced oscillator phase noise (deg)
    float theta_A_deg;              // Allan deviation oscillator phase noise (deg)
    float theta_e_3sig_deg;         // PLL 3-sigma dynamic stress error (deg)
    float total_PLL_1sig_deg;       // Total 1-sigma PLL error (deg)
    float pllThreshold_1sig_deg;    // 15 deg (Data) atau 30 deg (Pilot)

    // Diagnostik Error FLL (Hz & m/s) - Kaplan Eq. (8.82)-(8.85)
    bool isFLLLocked;
    float sigma_tFLL_Hz;            // FLL 1-sigma thermal frequency error (Hz)
    float fe_3sig_Hz;               // FLL 3-sigma dynamic stress error (Hz)
    float fllThreshold_3sig_Hz;     // 1 / (4 * T_FLL) (Hz)

    // Diagnostik Error DLL (chips & meter) - Kaplan Eq. (8.86)-(8.93)
    bool isDLLLocked;
    float sigma_tDLL_chips;         // DLL 1-sigma thermal code error (chips)
    float Re_3sig_chips;            // DLL 3-sigma dynamic stress error (chips)
    float dllThreshold_3sig_chips;  // D / 2 (chips)

    // Error 1D Range & Range-Rate (Total vs Excess Jamming)
    float sigma_rho_total_m;        // Total 1-sigma pseudorange noise (m)
    float sigma_rho_excess_m;       // Tambahan 1-sigma pseudorange noise akibat jamming saja (m)
    float sigma_rhodot_total_m_s;   // Total 1-sigma pseudorange-rate noise (m/s)
    float sigma_rhodot_excess_m_s;  // Tambahan 1-sigma pseudorange-rate noise akibat jamming saja (m/s)

    // Proyeksi Error Excess ke Local NED Frame (Untuk Injeksi Noise ke GPSDataPacket)
    Eigen::Vector3f sigma_pos_excess_ned_m;   // [sigma_N, sigma_E, sigma_D]^T (m)
    Eigen::Vector3f sigma_vel_excess_ned_m_s; // [sigma_vN, sigma_vE, sigma_vD]^T (m/s)

    // Metrik Akurasi Tambahan untuk Kolom PX4 (eph, epv, s_variance_m_s)
    float eph_excess_m;             // HDOP * sigma_rho_excess_m (m)
    float epv_excess_m;             // VDOP * sigma_rho_excess_m (m)
    float s_var_excess_m_s;         // HDOP * sigma_rhodot_excess_m_s (m/s)
};

class JammingTrackingLoop {
public:
    JammingTrackingLoop();

    // Membuat konfigurasi default Tracking Loop (Kaplan Ch. 8.9 & Table 9.4)
    static TrackingLoopConfig createDefaultTrackingConfig(GNSSSignalType sigType = GNSSSignalType::GPS_L1_CA,
                                                          ReceiverAidingMode aidMode = ReceiverAidingMode::UNAIDED);

    // Fungsi Utama Tahap 3: Evaluasi PLL, FLL, DLL, Ambang Batas, dan Proyeksi DOP ke Local NED
    // losAccel_m_s2 : Dinamika percepatan line-of-sight wahana (m/s^2) - dipakai PLL orde-2
    // losJerk_m_s3  : Dinamika jerk line-of-sight wahana (m/s^3) - dipakai PLL orde-3 & FLL orde-2
    TrackingLoopResult evaluateTrackingLoop(const ReceiverFrontEndConfig& rxCfg,
                                            const CorrelatorResult& corrRes,
                                            const TrackingLoopConfig& trkCfg,
                                            float losAccel_m_s2 = 0.0f,
                                            float losJerk_m_s3  = 0.0f) const;

private:
    static constexpr double C_LIGHT = 299792458.0;
    static constexpr double PI_VAL  = 3.14159265358979323846;

    // [1] Kaplan (PDF 2) Section 8.9.2, p. 476 (PDF 2 p. 3) - Carrier Frequency & Wavelength
    void getCarrierFrequency(GNSSSignalType sigType, double& fL_Hz, double& lambdaL_m) const;

    // [2] Kaplan (PDF 2) Section 8.9.3, Eq. (8.73), p. 478 (PDF 2 p. 5)
    float computeVibrationPhaseError(double fL_Hz, const TrackingLoopConfig& trkCfg) const;

    // [3] Kaplan (PDF 2) Section 8.9.4, Eq. (8.76)-(8.77), p. 479 (PDF 2 p. 6)
    float computeAllanPhaseError(double fL_Hz, const TrackingLoopConfig& trkCfg) const;

    // [4] Kaplan (PDF 2) Section 8.9.5, Eq. (8.78)-(8.79), p. 481 (PDF 2 p. 8)
    float computePLLDynamicStress(double lambdaL_m, const TrackingLoopConfig& trkCfg,
                                  float losAccel_m_s2, float losJerk_m_s3) const;

    // [5] Kaplan (PDF 2) Section 8.9.2, Eq. (8.70), p. 475 (PDF 2 p. 2)
    float computePLLThermalNoise(double cn0_linear_Hz, const TrackingLoopConfig& trkCfg) const;

    // [6] Kaplan (PDF 2) Section 8.9.8, Eq. (8.82)-(8.85), p. 484-485 (PDF 2 p. 11-12)
    void computeFLLMetrics(double cn0_linear_Hz, double cn0_dBHz, double lambdaL_m,
                           const TrackingLoopConfig& trkCfg, float losJerk_m_s3,
                           float& sigma_tFLL_Hz, float& sigma_tFLL_m_s, float& fe_3sig_Hz) const;

    // [7] Kaplan (PDF 2) Section 8.9.9 Eq. (8.90) p. 488 & Section 8.9.10 Eq. (8.93) p. 493
    float computeDLLThermalNoiseChips(double cn0_linear_Hz, GNSSSignalType sigType,
                                      float Rc_chips_s, float Bfe_Hz,
                                      const TrackingLoopConfig& trkCfg,
                                      bool isCoherentActive) const;
};

#endif // JAMMING_TRACKING_LOOP_H