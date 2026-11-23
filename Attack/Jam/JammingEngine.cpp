#include "JammingEngine.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <cmath>

JammingEngine::JammingEngine()
    : rngEngine(42u), normalDist(0.0, 1.0) {
    rxCfg  = JammingCorrelator::createDefaultReceiverConfig(GNSSSignalType::GPS_L1_CA);
    trkCfg = JammingTrackingLoop::createDefaultTrackingConfig(GNSSSignalType::GPS_L1_CA, ReceiverAidingMode::UNAIDED);

    opts.randomSeed                = 42u;
    opts.reportDegradedAccuracy    = true;
    opts.zeroVelocityOnOpenCarrier = true;
}

void JammingEngine::configure(const ReceiverFrontEndConfig& rxConfig,
                              const TrackingLoopConfig& trkConfig,
                              const JammingSimOptions& simOpts) {
    rxCfg  = rxConfig;
    trkCfg = trkConfig;
    opts   = simOpts;
    resetRNG(opts.randomSeed);
}

void JammingEngine::clearJammers() {
    jammers.clear();
}

void JammingEngine::addJammer(const JammerEntity& jammer) {
    jammers.push_back(jammer);
}

void JammingEngine::setJammers(const std::vector<JammerEntity>& jammerList) {
    jammers = jammerList;
}

void JammingEngine::resetRNG(unsigned int seed) {
    opts.randomSeed = seed;
    rngEngine.seed(seed);
    normalDist.reset();
}

// Groves Eq. (2.105) & (2.106) - Earth Radii of Curvature
void JammingEngine::computeEarth(double L_b, double& RN_Lb, double& RE_Lb) const {
    const double sin_L = std::sin(L_b);
    const double sin_L_sqr = sin_L * sin_L;
    const double denom_sqrt = std::sqrt(1.0 - E_SQR * sin_L_sqr);

    RN_Lb = R_0 * (1.0 - E_SQR) / (denom_sqrt * denom_sqrt * denom_sqrt);
    RE_Lb = R_0 / denom_sqrt;
}

// Konversi kuaternion PX4 (FRD-to-NED) ke DCM C^n_b
Eigen::Matrix3f JammingEngine::quaternionToCnb(double q0, double q1, double q2, double q3) {
    const double norm = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm < 1.0e-9) {
        return Eigen::Matrix3f::Identity();
    }
    const double w = q0 / norm;
    const double x = q1 / norm;
    const double y = q2 / norm;
    const double z = q3 / norm;

    Eigen::Matrix3f cn_b;
    cn_b(0, 0) = static_cast<float>(1.0 - 2.0 * (y * y + z * z));
    cn_b(0, 1) = static_cast<float>(2.0 * (x * y - w * z));
    cn_b(0, 2) = static_cast<float>(2.0 * (x * z + w * y));

    cn_b(1, 0) = static_cast<float>(2.0 * (x * y + w * z));
    cn_b(1, 1) = static_cast<float>(1.0 - 2.0 * (x * x + z * z));
    cn_b(1, 2) = static_cast<float>(2.0 * (y * z - w * x));

    cn_b(2, 0) = static_cast<float>(2.0 * (x * z - w * y));
    cn_b(2, 1) = static_cast<float>(2.0 * (y * z + w * x));
    cn_b(2, 2) = static_cast<float>(1.0 - 2.0 * (x * x + y * y));
    return cn_b;
}

// =========================================================================================
// [1] Kaplan (PDF 1) Eq. (9.22), Eq. (9.23), & Eq. (9.27), Hal. Buku 563, 567, 572
//     Menghitung radius zona ancaman (meter) agar UI Mission Planner dapat menggambar
//     lingkaran jangkauan efektif sebuah node jammer pada peta.
// =========================================================================================
float JammingEngine::computeThreatRadius_m(const JammerEntity& jammer,
                                           float nominalMobileHeightAGL_m,
                                           float nominalJammerElev_deg,
                                           float targetCN0Thresh_dBHz) const {
    if (!jammer.propCfg.isEnabled || jammer.propCfg.transmitPower_W <= 0.0f) {
        return 0.0f;
    }

    // Jika ambang batas tidak ditentukan manual, gunakan ambang standar Table 9.4 (27 dB-Hz Data, 17 dB-Hz Pilot)
    const float thresh_dBHz = (targetCN0Thresh_dBHz > 0.0f)
                                  ? targetCN0Thresh_dBHz
                                  : (trkCfg.usePilotChannel ? 17.0f : 27.0f);

    // Cari jarak d menggunakan metode biseksi [1 m, 100.000 m] agar akurat untuk SEMUA model
    // (Free-Space, Modified Hata Urban/Suburban/Open, Erceg, Blockage, Polarisasi, & CRPA!)
    double d_low  = 1.0;
    double d_high = 100000.0;

    const double L_ref      = jammer.propCfg.L_j;
    const double lambda_ref = jammer.propCfg.lambda_j;
    const double h_ref      = jammer.propCfg.h_j;
    double RN_ref = R_0, RE_ref = R_0;
    computeEarth(L_ref, RN_ref, RE_ref);

    const double elev_rad = static_cast<double>(nominalJammerElev_deg) * (PI_VAL / 180.0);
    const Eigen::Matrix3f cn_b_level = Eigen::Matrix3f::Identity();

    auto evalCN0AtDistance = [&](double dist_m) -> float {
        // Tempatkan wahana uji sejauh dist_m ke Selatan jammer dengan elevasi nominal
        const double L_veh = L_ref - (dist_m * std::cos(elev_rad)) / (RN_ref + h_ref);
        const double h_veh = h_ref - dist_m * std::sin(elev_rad);

        JammerNodeConfig testCfg = jammer.propCfg;
        testCfg.startTime_s = -1.0;
        testCfg.endTime_s   = 1.0;

        PropagationResult pRes = propModule.computePropagation(0.0, L_veh, lambda_ref, h_veh,
                                                               nominalMobileHeightAGL_m, cn_b_level, testCfg);
        pRes.elevAngle_body_rad = static_cast<float>(elev_rad);

        std::vector<ActiveJammerInput> oneJam;
        oneJam.push_back({ pRes, jammer.sigCfg });
        CorrelatorResult cRes = corrModule.evaluateCorrelator(rxCfg, oneJam);
        return cRes.effective_CN0_dBHz;
    };

    // Jika pada jarak 1 m saja C/N0 masih aman di atas ambang, radius ancaman = 0 m
    if (evalCN0AtDistance(d_low) > thresh_dBHz) {
        return 0.0f;
    }
    // Jika pada jarak 100 km masih terganggu, kembalikan batas atas
    if (evalCN0AtDistance(d_high) <= thresh_dBHz) {
        return static_cast<float>(d_high);
    }

    // Biseksi 28 iterasi -> ketelitian radius < 0.5 meter
    for (int iter = 0; iter < 28; ++iter) {
        const double d_mid = 0.5 * (d_low + d_high);
        if (evalCN0AtDistance(d_mid) <= thresh_dBHz) {
            d_low = d_mid;
        } else {
            d_high = d_mid;
        }
    }

    return static_cast<float>(0.5 * (d_low + d_high));
}

// =========================================================================================
// [2] Fungsi Real-Time Tick-by-Tick (Menghubungkan Tahap 1 -> 2 -> 3 -> Injeksi GPS)
//     Siap dipanggil langsung oleh loop simulasi Mission Planner di Antigravity
// =========================================================================================
JammingStepOutput JammingEngine::step(const VehicleKinematicsInput& vehState,
                                      const GNSSObservationPacket& cleanGps) {
    JammingStepOutput out = {};
    out.jammedGps = cleanGps;
    out.injectedPosNoise_ned_m.setZero();
    out.injectedVelNoise_ned_m_s.setZero();

    // -------------------------------------------------------------------------------------
    // Pipeline Tahap 1: Evaluasi Propagasi 3D untuk seluruh node jammer di peta
    // -------------------------------------------------------------------------------------
    std::vector<ActiveJammerInput> corrInputs;
    corrInputs.reserve(jammers.size());
    out.propResults.reserve(jammers.size());

    for (size_t i = 0; i < jammers.size(); ++i) {
        PropagationResult pRes = propModule.computePropagation(vehState.missionTime_s,
                                                               vehState.lat_rad,
                                                               vehState.lon_rad,
                                                               vehState.alt_m,
                                                               vehState.heightAGL_m,
                                                               vehState.cn_b,
                                                               jammers[i].propCfg);
        out.propResults.push_back(pRes);
        corrInputs.push_back({ pRes, jammers[i].sigCfg });
    }

    // -------------------------------------------------------------------------------------
    // Pipeline Tahap 2: Evaluasi Antena (FRPA/CRPA), Front-End, & Korelator -> (C_s/N_0)_eff
    // -------------------------------------------------------------------------------------
    out.corrResult = corrModule.evaluateCorrelator(rxCfg, corrInputs);

    // -------------------------------------------------------------------------------------
    // Pipeline Tahap 3: Evaluasi Tracking Loop (PLL/FLL/DLL), Lock State, & Proyeksi DOP NED
    // -------------------------------------------------------------------------------------
    out.trkResult = trkModule.evaluateTrackingLoop(rxCfg, out.corrResult, trkCfg,
                                                   vehState.losAccel_m_s2, vehState.losJerk_m_s3);

    // -------------------------------------------------------------------------------------
    // Tahap 4: Injeksi Dampak Jamming ke Paket Observasi GNSS
    // -------------------------------------------------------------------------------------
    if (out.trkResult.lockState == TrackingLockState::LOST_LOCK) {
        // Total Loss of Lock -> main.cpp Anda otomatis melewati ekf.correct() (Free-Inertial Coasting)
        out.jammedGps.fix_type        = 0;
        out.jammedGps.vel_ned_valid   = false;
        out.jammedGps.satellites_used = 0;
        if (opts.reportDegradedAccuracy) {
            out.jammedGps.eph        = 999.0f;
            out.jammedGps.epv        = 999.0f;
            out.jammedGps.s_variance = 99.0f;
        }
        return out;
    }

    // Bangkitkan realisasi derau Gaussian excess pada bingkai Local NED
    const float dN = out.trkResult.sigma_pos_excess_ned_m(0)   * static_cast<float>(normalDist(rngEngine));
    const float dE = out.trkResult.sigma_pos_excess_ned_m(1)   * static_cast<float>(normalDist(rngEngine));
    const float dD = out.trkResult.sigma_pos_excess_ned_m(2)   * static_cast<float>(normalDist(rngEngine));

    const float dvN = out.trkResult.sigma_vel_excess_ned_m_s(0) * static_cast<float>(normalDist(rngEngine));
    const float dvE = out.trkResult.sigma_vel_excess_ned_m_s(1) * static_cast<float>(normalDist(rngEngine));
    const float dvD = out.trkResult.sigma_vel_excess_ned_m_s(2) * static_cast<float>(normalDist(rngEngine));

    out.injectedPosNoise_ned_m   = Eigen::Vector3f(dN, dE, dD);
    out.injectedVelNoise_ned_m_s = Eigen::Vector3f(dvN, dvE, dvD);

    // Konversi gangguan posisi NED (m) kembali ke Geodetik LLA - Groves Eq. (14.60)-(14.61)
    double RN_Lb = R_0, RE_Lb = R_0;
    computeEarth(cleanGps.lat_rad, RN_Lb, RE_Lb);
    const double cos_L = std::max(std::cos(cleanGps.lat_rad), 1.0e-6);

    out.jammedGps.lat_rad = cleanGps.lat_rad + static_cast<double>(dN) / (RN_Lb + cleanGps.alt_m);
    out.jammedGps.lon_rad = cleanGps.lon_rad + static_cast<double>(dE) / ((RE_Lb + cleanGps.alt_m) * cos_L);
    out.jammedGps.alt_m   = cleanGps.alt_m   - static_cast<double>(dD); // Down = -Altitude

    // Injeksi gangguan kecepatan NED
    out.jammedGps.vn = cleanGps.vn + dvN;
    out.jammedGps.ve = cleanGps.ve + dvE;
    out.jammedGps.vd = cleanGps.vd + dvD;

    // Status kunci PX4 sesuai state machine Tahap 3
    out.jammedGps.fix_type = std::min(cleanGps.fix_type, out.trkResult.px4_fix_type);
    if (out.trkResult.lockState == TrackingLockState::OPEN_CARRIER_AIDED_DLL && opts.zeroVelocityOnOpenCarrier) {
        out.jammedGps.vel_ned_valid = false;
    } else {
        out.jammedGps.vel_ned_valid = cleanGps.vel_ned_valid && out.trkResult.px4_vel_ned_valid;
    }

    // Update kolom akurasi dinamis PX4 secara root-sum-square jika mode Aware Receiver aktif
    if (opts.reportDegradedAccuracy) {
        out.jammedGps.eph = std::sqrt(cleanGps.eph * cleanGps.eph +
                                      out.trkResult.eph_excess_m * out.trkResult.eph_excess_m);
        out.jammedGps.epv = std::sqrt(cleanGps.epv * cleanGps.epv +
                                      out.trkResult.epv_excess_m * out.trkResult.epv_excess_m);
        out.jammedGps.s_variance = std::sqrt(cleanGps.s_variance * cleanGps.s_variance +
                                             out.trkResult.s_var_excess_m_s * out.trkResult.s_var_excess_m_s);
    }

    return out;
}

// =========================================================================================
// [3] Groves (PDF 3) Ch. 17, Section 17.7.2 - Recorded Data Testing (Hal. Buku 725 / PDF 25)
//     Injektor Batch CSV PX4: Memodifikasi dataset vehicle_gps_position.csv tanpa menyentuh
//     satu baris pun kode EKF/main.cpp Anda!
// =========================================================================================
bool JammingEngine::injectToPX4CSV(const std::string& inputGpsCsvPath,
                                   const std::string& inputAttitudeCsvPath,
                                   const std::string& outputJammedGpsCsvPath,
                                   const std::string& outputTelemetryLogCsvPath) {
    // 3A. Muat seluruh data sikap (kuaternion) dari vehicle_attitude.csv jika tersedia
    struct AttSample {
        double timestamp_us;
        Eigen::Matrix3f cn_b;
    };
    std::vector<AttSample> attSamples;

    if (!inputAttitudeCsvPath.empty()) {
        std::ifstream attFile(inputAttitudeCsvPath);
        if (attFile.is_open()) {
            std::string hLine;
            if (std::getline(attFile, hLine)) {
                hLine.erase(std::remove(hLine.begin(), hLine.end(), '\r'), hLine.end());
                hLine.erase(std::remove(hLine.begin(), hLine.end(), '\n'), hLine.end());

                int idx_t = -1, idx_q[4] = {-1, -1, -1, -1};
                std::stringstream ssH(hLine);
                std::string col;
                int c = 0;
                while (std::getline(ssH, col, ',')) {
                    col.erase(std::remove(col.begin(), col.end(), '\"'), col.end());
                    if (col == "timestamp") idx_t = c;
                    else if (col == "q[0]") idx_q[0] = c;
                    else if (col == "q[1]") idx_q[1] = c;
                    else if (col == "q[2]") idx_q[2] = c;
                    else if (col == "q[3]") idx_q[3] = c;
                    c++;
                }

                if (idx_t >= 0 && idx_q[0] >= 0 && idx_q[1] >= 0 && idx_q[2] >= 0 && idx_q[3] >= 0) {
                    const int maxIdx = std::max(idx_t, std::max(std::max(idx_q[0], idx_q[1]), std::max(idx_q[2], idx_q[3])));
                    std::string line;
                    while (std::getline(attFile, line)) {
                        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
                        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
                        if (line.empty()) continue;

                        std::stringstream ss(line);
                        std::string val;
                        std::vector<std::string> row;
                        while (std::getline(ss, val, ',')) row.push_back(val);
                        if (static_cast<int>(row.size()) <= maxIdx) continue;

                        try {
                            AttSample s;
                            s.timestamp_us = std::stod(row[idx_t]);
                            s.cn_b = quaternionToCnb(std::stod(row[idx_q[0]]), std::stod(row[idx_q[1]]),
                                                     std::stod(row[idx_q[2]]), std::stod(row[idx_q[3]]));
                            attSamples.push_back(s);
                        } catch (...) {}
                    }
                }
            }
            attFile.close();
        }
    }

    // 3B. Buka berkas GPS input dan berkas keluaran
    std::ifstream inGps(inputGpsCsvPath);
    if (!inGps.is_open()) {
        std::cerr << "[JammingEngine ERROR] Gagal membuka GPS CSV: " << inputGpsCsvPath << "\n";
        return false;
    }

    std::ofstream outGps(outputJammedGpsCsvPath);
    std::ofstream outLog(outputTelemetryLogCsvPath);
    if (!outGps.is_open() || !outLog.is_open()) {
        std::cerr << "[JammingEngine ERROR] Gagal membuat file output CSV!\n";
        return false;
    }

    // Parse header GPS CSV (identik dengan aturan kolom GPSReader pada main.cpp Anda)
    std::string headerLine;
    if (!std::getline(inGps, headerLine)) {
        return false;
    }
    headerLine.erase(std::remove(headerLine.begin(), headerLine.end(), '\r'), headerLine.end());
    headerLine.erase(std::remove(headerLine.begin(), headerLine.end(), '\n'), headerLine.end());

    // Tulis ulang header yang sama persis ke output GPS CSV agar 100% kompatibel dengan main.cpp
    outGps << headerLine << "\n";

    // Header file log telemetri RF Jamming (untuk plotting grafik analisis & UI Antigravity)
    outLog << "timestamp_s,mission_time_s,active_jammers,crpa_nulled,js_dB,ci_cs_dB,"
           << "unjammed_cn0_dBHz,eff_cn0_dBHz,lna_saturated,cw_leak_risk,lock_state,"
           << "fix_type,vel_valid,pll_1sig_deg,fll_1sig_Hz,dll_1sig_chips,"
           << "sig_posN_ex_m,sig_posD_ex_m,sig_velN_ex_ms,inj_dN_m,inj_dE_m,inj_dD_m\n";

    int idx_time = -1, idx_lat = -1, idx_lon = -1, idx_alt = -1;
    int idx_vn = -1, idx_ve = -1, idx_vd = -1;
    int idx_eph = -1, idx_epv = -1, idx_s_var = -1;
    int idx_fix_type = -1, idx_vel_valid = -1, idx_sat_used = -1;
    bool is_lat_scaled_int = false, is_lon_scaled_int = false, is_alt_scaled_mm = false;

    {
        std::stringstream ss(headerLine);
        std::string col;
        int idx = 0;
        while (std::getline(ss, col, ',')) {
            col.erase(std::remove(col.begin(), col.end(), '\"'), col.end());
            if (col == "timestamp") idx_time = idx;
            else if (col == "latitude_deg") { idx_lat = idx; is_lat_scaled_int = false; }
            else if (col == "lat") { idx_lat = idx; is_lat_scaled_int = true; }
            else if (col == "longitude_deg") { idx_lon = idx; is_lon_scaled_int = false; }
            else if (col == "lon") { idx_lon = idx; is_lon_scaled_int = true; }
            else if (col == "altitude_ellipsoid_m") { idx_alt = idx; is_alt_scaled_mm = false; }
            else if (col == "altitude_msl_m" && idx_alt == -1) { idx_alt = idx; is_alt_scaled_mm = false; }
            else if (col == "alt_ellipsoid" && idx_alt == -1) { idx_alt = idx; is_alt_scaled_mm = true; }
            else if (col == "alt" && idx_alt == -1) { idx_alt = idx; is_alt_scaled_mm = true; }
            else if (col == "vel_n_m_s") idx_vn = idx;
            else if (col == "vel_e_m_s") idx_ve = idx;
            else if (col == "vel_d_m_s") idx_vd = idx;
            else if (col == "eph") idx_eph = idx;
            else if (col == "epv") idx_epv = idx;
            else if (col == "s_variance_m_s") idx_s_var = idx;
            else if (col == "fix_type") idx_fix_type = idx;
            else if (col == "vel_ned_valid") idx_vel_valid = idx;
            else if (col == "satellites_used") idx_sat_used = idx;
            idx++;
        }
    }

    if (idx_time < 0 || idx_lat < 0 || idx_lon < 0 || idx_alt < 0 ||
        idx_vn < 0 || idx_ve < 0 || idx_vd < 0) {
        std::cerr << "[JammingEngine ERROR] Kolom wajib GPS CSV tidak ditemukan!\n";
        return false;
    }

    // 3C. Iterasi setiap baris GPS PX4, sinkronkan attitude & dinamika, lalu injeksikan jamming
    double t0_us = -1.0;
    double homeAlt_m = 0.0;
    double prevTime_s = -1.0;
    Eigen::Vector3f prevVel = Eigen::Vector3f::Zero();
    Eigen::Vector3f prevAcc = Eigen::Vector3f::Zero();
    size_t attCursor = 0;

    std::string line;
    while (std::getline(inGps, line)) {
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string val;
        std::vector<std::string> row;
        while (std::getline(ss, val, ',')) row.push_back(val);

        const int reqMax = std::max(std::max(idx_time, idx_lat),
                                    std::max(std::max(idx_lon, idx_alt),
                                             std::max(idx_vn, std::max(idx_ve, idx_vd))));
        if (static_cast<int>(row.size()) <= reqMax) continue;

        GNSSObservationPacket cleanPkt = {};
        try {
            cleanPkt.timestamp_us = std::stod(row[idx_time]);
            double rawLat = std::stod(row[idx_lat]);
            double rawLon = std::stod(row[idx_lon]);
            double rawAlt = std::stod(row[idx_alt]);

            const bool latScaled = is_lat_scaled_int || (std::abs(rawLat) > 90.0);
            const bool lonScaled = is_lon_scaled_int || (std::abs(rawLon) > 180.0);
            if (latScaled) rawLat *= 1.0e-7;
            if (lonScaled) rawLon *= 1.0e-7;
            if (is_alt_scaled_mm) rawAlt *= 1.0e-3;

            cleanPkt.lat_rad = rawLat * (PI_VAL / 180.0);
            cleanPkt.lon_rad = rawLon * (PI_VAL / 180.0);
            cleanPkt.alt_m   = rawAlt;
            cleanPkt.vn      = std::stof(row[idx_vn]);
            cleanPkt.ve      = std::stof(row[idx_ve]);
            cleanPkt.vd      = std::stof(row[idx_vd]);

            cleanPkt.eph             = (idx_eph >= 0 && static_cast<int>(row.size()) > idx_eph) ? std::stof(row[idx_eph]) : 0.8f;
            cleanPkt.epv             = (idx_epv >= 0 && static_cast<int>(row.size()) > idx_epv) ? std::stof(row[idx_epv]) : 1.37f;
            cleanPkt.s_variance      = (idx_s_var >= 0 && static_cast<int>(row.size()) > idx_s_var) ? std::stof(row[idx_s_var]) : 0.26f;
            cleanPkt.fix_type        = (idx_fix_type >= 0 && static_cast<int>(row.size()) > idx_fix_type) ? std::stoi(row[idx_fix_type]) : 3;
            cleanPkt.vel_ned_valid   = (idx_vel_valid >= 0 && static_cast<int>(row.size()) > idx_vel_valid) ? (std::stoi(row[idx_vel_valid]) != 0) : true;
            cleanPkt.satellites_used = (idx_sat_used >= 0 && static_cast<int>(row.size()) > idx_sat_used) ? std::stoi(row[idx_sat_used]) : 10;

            if (t0_us < 0.0) {
                t0_us     = cleanPkt.timestamp_us;
                homeAlt_m = cleanPkt.alt_m;
            }
            const double missionTime_s = (cleanPkt.timestamp_us - t0_us) * 1.0e-6;

            // Estimasi dinamika percepatan & jerk sesaat dari kecepatan GPS berurutan
            Eigen::Vector3f curVel(cleanPkt.vn, cleanPkt.ve, cleanPkt.vd);
            float losAccel = 0.0f, losJerk = 0.0f;
            if (prevTime_s >= 0.0 && (missionTime_s - prevTime_s) > 1.0e-3) {
                const float dt_gps = static_cast<float>(missionTime_s - prevTime_s);
                Eigen::Vector3f curAcc = (curVel - prevVel) / dt_gps;
                Eigen::Vector3f curJrk = (curAcc - prevAcc) / dt_gps;
                losAccel = curAcc.norm();
                losJerk  = curJrk.norm();
                prevAcc  = curAcc;
            }
            prevVel    = curVel;
            prevTime_s = missionTime_s;

            // Cari matriks sikap C^n_b terdekat dari vehicle_attitude.csv
            Eigen::Matrix3f curCnb = Eigen::Matrix3f::Identity();
            if (!attSamples.empty()) {
                while ((attCursor + 1) < attSamples.size() &&
                       attSamples[attCursor + 1].timestamp_us <= cleanPkt.timestamp_us) {
                    attCursor++;
                }
                curCnb = attSamples[attCursor].cn_b;
            }

            VehicleKinematicsInput vehIn = {};
            vehIn.missionTime_s = missionTime_s;
            vehIn.lat_rad       = cleanPkt.lat_rad;
            vehIn.lon_rad       = cleanPkt.lon_rad;
            vehIn.alt_m         = cleanPkt.alt_m;
            vehIn.heightAGL_m   = std::max(1.0f, static_cast<float>(cleanPkt.alt_m - homeAlt_m));
            vehIn.cn_b          = curCnb;
            vehIn.losAccel_m_s2 = losAccel;
            vehIn.losJerk_m_s3  = losJerk;

            // Jalankan 1 tick penuh JammingEngine
            JammingStepOutput stepOut = step(vehIn, cleanPkt);

            // Tulis balik kolom yang terpengaruh ke vektor string `row` (format identik file asli)
            const double outLatDeg = stepOut.jammedGps.lat_rad * (180.0 / PI_VAL);
            const double outLonDeg = stepOut.jammedGps.lon_rad * (180.0 / PI_VAL);
            const double outAltM   = stepOut.jammedGps.alt_m;

            auto fmtDouble = [](double v, int prec) -> std::string {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(prec) << v;
                return oss.str();
            };

            row[idx_lat] = latScaled ? std::to_string(static_cast<long long>(std::llround(outLatDeg * 1.0e7)))
                                     : fmtDouble(outLatDeg, 8);
            row[idx_lon] = lonScaled ? std::to_string(static_cast<long long>(std::llround(outLonDeg * 1.0e7)))
                                     : fmtDouble(outLonDeg, 8);
            row[idx_alt] = is_alt_scaled_mm ? std::to_string(static_cast<long long>(std::llround(outAltM * 1.0e3)))
                                            : fmtDouble(outAltM, 4);

            row[idx_vn] = fmtDouble(stepOut.jammedGps.vn, 4);
            row[idx_ve] = fmtDouble(stepOut.jammedGps.ve, 4);
            row[idx_vd] = fmtDouble(stepOut.jammedGps.vd, 4);

            if (idx_eph >= 0 && static_cast<int>(row.size()) > idx_eph)
                row[idx_eph] = fmtDouble(stepOut.jammedGps.eph, 4);
            if (idx_epv >= 0 && static_cast<int>(row.size()) > idx_epv)
                row[idx_epv] = fmtDouble(stepOut.jammedGps.epv, 4);
            if (idx_s_var >= 0 && static_cast<int>(row.size()) > idx_s_var)
                row[idx_s_var] = fmtDouble(stepOut.jammedGps.s_variance, 4);
            if (idx_fix_type >= 0 && static_cast<int>(row.size()) > idx_fix_type)
                row[idx_fix_type] = std::to_string(stepOut.jammedGps.fix_type);
            if (idx_vel_valid >= 0 && static_cast<int>(row.size()) > idx_vel_valid)
                row[idx_vel_valid] = stepOut.jammedGps.vel_ned_valid ? "1" : "0";
            if (idx_sat_used >= 0 && static_cast<int>(row.size()) > idx_sat_used)
                row[idx_sat_used] = std::to_string(stepOut.jammedGps.satellites_used);

            // Tulis baris CSV GPS jammed
            for (size_t c = 0; c < row.size(); ++c) {
                outGps << row[c] << (c + 1 < row.size() ? "," : "\n");
            }

            // Tulis baris log telemetri RF Jamming
            outLog << std::fixed << std::setprecision(6) << (cleanPkt.timestamp_us * 1.0e-6) << ","
                   << std::setprecision(3) << missionTime_s << ","
                   << stepOut.corrResult.activeJammerCount << ","
                   << stepOut.corrResult.crpaNulledCount << ","
                   << std::setprecision(2) << stepOut.corrResult.total_JS_dB << ","
                   << stepOut.corrResult.total_Ci_Cs_dB << ","
                   << stepOut.corrResult.unjammed_CN0_dBHz << ","
                   << stepOut.corrResult.effective_CN0_dBHz << ","
                   << (stepOut.corrResult.isFrontEndSaturated ? 1 : 0) << ","
                   << (stepOut.corrResult.isCWLeakthroughRisk ? 1 : 0) << ","
                   << static_cast<int>(stepOut.trkResult.lockState) << ","
                   << stepOut.jammedGps.fix_type << ","
                   << (stepOut.jammedGps.vel_ned_valid ? 1 : 0) << ","
                   << std::setprecision(3) << stepOut.trkResult.total_PLL_1sig_deg << ","
                   << stepOut.trkResult.sigma_tFLL_Hz << ","
                   << std::setprecision(5) << stepOut.trkResult.sigma_tDLL_chips << ","
                   << std::setprecision(3) << stepOut.trkResult.sigma_pos_excess_ned_m(0) << ","
                   << stepOut.trkResult.sigma_pos_excess_ned_m(2) << ","
                   << stepOut.trkResult.sigma_vel_excess_ned_m_s(0) << ","
                   << stepOut.injectedPosNoise_ned_m(0) << ","
                   << stepOut.injectedPosNoise_ned_m(1) << ","
                   << stepOut.injectedPosNoise_ned_m(2) << "\n";
        } catch (...) {
            continue;
        }
    }

    inGps.close();
    outGps.close();
    outLog.close();
    return true;
}