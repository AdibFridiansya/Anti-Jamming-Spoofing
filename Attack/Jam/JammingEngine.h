#ifndef JAMMING_ENGINE_H
#define JAMMING_ENGINE_H

#include "JammingPropagation.h"
#include "JammingCorrelator.h"
#include "JammingTrackingLoop.h"

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <random>

// Definisi Satu Entitas Jammer Lengkap di Peta Mission Planner (Geometri + Sinyal RF)
struct JammerEntity {
    std::string id;                 // Nama/ID node jammer di UI (misal "Jammer_1")
    JammerNodeConfig propCfg;       // Posisi, jadwal aktif, daya EIRP, model medan (Tahap 1)
    JammerSignalConfig sigCfg;      // Tipe modulasi CW/Matched/BLWN/Pulsed & polarisasi (Tahap 2)
};

// State Wahana pada Epoch/Tick Saat Ini (Dikirim dari Antigravity atau Log PX4)
struct VehicleKinematicsInput {
    double missionTime_s;           // Waktu relatif sejak awal misi (detik)
    double lat_rad;                 // Latitude wahana (rad)
    double lon_rad;                 // Longitude wahana (rad)
    double alt_m;                   // Altitude ellipsoid wahana (m)
    float heightAGL_m;              // Ketinggian di atas permukaan tanah (m)
    Eigen::Matrix3f cn_b;           // Matriks rotasi Body-to-NED (C^n_b) untuk efek Antenna Tilt
    float losAccel_m_s2;            // Magnitudo dinamika percepatan wahana (m/s^2)
    float losJerk_m_s3;             // Magnitudo dinamika jerk wahana (m/s^3)
};

// Paket Data GPS Bersih/Terganggu (Selaras dengan struct GPSDataPacket di main.cpp Anda)
struct GNSSObservationPacket {
    double timestamp_us;            // Timestamp asli (mikrodetik)
    double lat_rad;
    double lon_rad;
    double alt_m;
    float vn;
    float ve;
    float vd;
    float eph;
    float epv;
    float s_variance;
    int fix_type;
    bool vel_ned_valid;
    int satellites_used;
};

// Keluaran Lengkap per-Tick dari JammingEngine (Untuk EKF + Panel Grafik Antigravity)
struct JammingStepOutput {
    GNSSObservationPacket jammedGps;            // Paket GPS yang sudah dimodifikasi oleh jamming
    std::vector<PropagationResult> propResults; // Hasil propagasi masing-masing jammer aktif
    CorrelatorResult corrResult;                // Hasil (C_s/N_0)_eff, J/S, CRPA, saturasi LNA
    TrackingLoopResult trkResult;               // Hasil PLL/FLL/DLL, status lock, & sigma NED
    Eigen::Vector3f injectedPosNoise_ned_m;     // Realisasi noise posisi NED yang disuntikkan (m)
    Eigen::Vector3f injectedVelNoise_ned_m_s;   // Realisasi noise kecepatan NED yang disuntikkan (m/s)
};

// Opsi Konfigurasi Global Simulator Jamming
struct JammingSimOptions {
    unsigned int randomSeed;        // Seed PRNG agar hasil simulasi reproducible
    bool reportDegradedAccuracy;    // true: update eph/epv/s_var (Aware), false: biarkan normal (Unaware/uji AIME)
    bool zeroVelocityOnOpenCarrier; // true: nolkan/tandai vel_ned_valid=false saat OPEN_CARRIER_AIDED_DLL
};

class JammingEngine {
public:
    JammingEngine();

    // Konfigurasi penerima, skenario jammer, dan opsi simulasi
    void configure(const ReceiverFrontEndConfig& rxConfig,
                   const TrackingLoopConfig& trkConfig,
                   const JammingSimOptions& simOpts);

    void clearJammers();
    void addJammer(const JammerEntity& jammer);
    void setJammers(const std::vector<JammerEntity>& jammerList);
    void resetRNG(unsigned int seed);

    // [API 1 - Untuk Mission Planner / Antigravity]: Hitung radius zona ancaman jammer (m)
    // Kaplan (PDF 1) Eq. (9.22), (9.23), & Eq. (9.27), Hal. Buku 563, 567, 572
    float computeThreatRadius_m(const JammerEntity& jammer,
                                float nominalMobileHeightAGL_m = 50.0f,
                                float nominalJammerElev_deg = 5.0f,
                                float targetCN0Thresh_dBHz = -1.0f) const;

    // [API 2 - Untuk Real-Time Tick-by-Tick Simulation di Antigravity]
    JammingStepOutput step(const VehicleKinematicsInput& vehState,
                           const GNSSObservationPacket& cleanGps);

    // [API 3 - Untuk Recorded Data Testing (Groves Section 17.7.2, Hal. 725)]
    // Membaca CSV GPS normal PX4 (+ opsional CSV attitude PX4 untuk Antenna Tilt),
    // lalu menghasilkan CSV GPS jammed yang 100% kompatibel dengan main.cpp Anda
    // beserta file log telemetri RF jamming.
    bool injectToPX4CSV(const std::string& inputGpsCsvPath,
                        const std::string& inputAttitudeCsvPath,
                        const std::string& outputJammedGpsCsvPath,
                        const std::string& outputTelemetryLogCsvPath);

private:
    static constexpr double R_0     = 6378137.0;
    static constexpr double E_SQR   = 0.00669437999014;
    static constexpr double C_LIGHT = 299792458.0;
    static constexpr double PI_VAL  = 3.14159265358979323846;

    JammingPropagation  propModule;
    JammingCorrelator   corrModule;
    JammingTrackingLoop trkModule;

    ReceiverFrontEndConfig rxCfg;
    TrackingLoopConfig     trkCfg;
    JammingSimOptions      opts;
    std::vector<JammerEntity> jammers;

    std::mt19937 rngEngine;
    std::normal_distribution<double> normalDist;

    // Groves Eq. (2.105)-(2.106) - Earth Radii of Curvature
    void computeEarth(double L_b, double& RN_Lb, double& RE_Lb) const;

    // Konversi kuaternion PX4 (q0,q1,q2,q3) menjadi matriks rotasi Body-to-NED (C^n_b)
    static Eigen::Matrix3f quaternionToCnb(double q0, double q1, double q2, double q3);
};

#endif // JAMMING_ENGINE_H