#ifndef EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_VECTORIZE
#endif

#ifndef EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#endif

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "JammingEngine.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Pencarian berkas CSV yang identik dengan main.cpp Anda
static std::string resolveFilePath(const std::string& directory,
                                   const std::string& keyword,
                                   const std::vector<std::string>& candidates) {
    for (size_t i = 0; i < candidates.size(); ++i) {
        std::ifstream test(candidates[i]);
        if (test.is_open()) {
            test.close();
            return candidates[i];
        }
    }

#ifdef _WIN32
    WIN32_FIND_DATAA findData;
    const std::string searchPattern = directory + "*" + keyword + "*.csv";
    HANDLE hFind = FindFirstFileA(searchPattern.c_str(), &findData);
    if (hFind != INVALID_HANDLE_VALUE) {
        std::string foundFile = directory + std::string(findData.cFileName);
        FindClose(hFind);
        return foundFile;
    }
#endif

    return candidates.front();
}

// Helper: Mengintip koordinat & durasi terbang dari CSV GPS PX4 agar jammer ditaruh tepat di rute terbang
struct FlightSummary {
    bool valid;
    double totalDuration_s;
    double midLat_rad;
    double midLon_rad;
    double groundAlt_m;
    int totalRows;
};

static FlightSummary inspectPX4Flight(const std::string& gpsCsvPath) {
    FlightSummary info = {};
    info.valid = false;

    std::ifstream file(gpsCsvPath);
    if (!file.is_open()) return info;

    std::string headerLine;
    if (!std::getline(file, headerLine)) return info;
    headerLine.erase(std::remove(headerLine.begin(), headerLine.end(), '\r'), headerLine.end());
    headerLine.erase(std::remove(headerLine.begin(), headerLine.end(), '\n'), headerLine.end());

    int idx_t = -1, idx_lat = -1, idx_lon = -1, idx_alt = -1;
    bool lat_int = false, lon_int = false, alt_mm = false;

    std::stringstream ssH(headerLine);
    std::string col;
    int c = 0;
    while (std::getline(ssH, col, ',')) {
        col.erase(std::remove(col.begin(), col.end(), '\"'), col.end());
        if (col == "timestamp") idx_t = c;
        else if (col == "latitude_deg") { idx_lat = c; lat_int = false; }
        else if (col == "lat") { idx_lat = c; lat_int = true; }
        else if (col == "longitude_deg") { idx_lon = c; lon_int = false; }
        else if (col == "lon") { idx_lon = c; lon_int = true; }
        else if (col == "altitude_ellipsoid_m") { idx_alt = c; alt_mm = false; }
        else if (col == "altitude_msl_m" && idx_alt == -1) { idx_alt = c; alt_mm = false; }
        else if (col == "alt_ellipsoid" && idx_alt == -1) { idx_alt = c; alt_mm = true; }
        else if (col == "alt" && idx_alt == -1) { idx_alt = c; alt_mm = true; }
        c++;
    }

    if (idx_t < 0 || idx_lat < 0 || idx_lon < 0 || idx_alt < 0) return info;

    std::vector<double> times_us, lats_rad, lons_rad, alts_m;
    const int maxReq = std::max(std::max(idx_t, idx_lat), std::max(idx_lon, idx_alt));
    std::string line;

    while (std::getline(file, line)) {
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string val;
        std::vector<std::string> row;
        while (std::getline(ss, val, ',')) row.push_back(val);
        if (static_cast<int>(row.size()) <= maxReq) continue;

        try {
            double t = std::stod(row[idx_t]);
            double la = std::stod(row[idx_lat]);
            double lo = std::stod(row[idx_lon]);
            double al = std::stod(row[idx_alt]);
            if (lat_int || std::abs(la) > 90.0) la *= 1.0e-7;
            if (lon_int || std::abs(lo) > 180.0) lo *= 1.0e-7;
            if (alt_mm) al *= 1.0e-3;

            times_us.push_back(t);
            lats_rad.push_back(la * (M_PI / 180.0));
            lons_rad.push_back(lo * (M_PI / 180.0));
            alts_m.push_back(al);
        } catch (...) {}
    }

    if (times_us.size() < 10) return info;

    const size_t midIdx  = times_us.size() / 2;
    info.valid           = true;
    info.totalRows       = static_cast<int>(times_us.size());
    info.totalDuration_s = (times_us.back() - times_us.front()) * 1.0e-6;
    info.midLat_rad      = lats_rad[midIdx];
    info.midLon_rad      = lons_rad[midIdx];
    info.groundAlt_m     = alts_m.front(); // Asumsi awal terbang di permukaan tanah (home)
    return info;
}

// Helper: Membaca ringkasan statistik dari jamming_telemetry_log.csv
static void printTelemetrySummary(const std::string& logCsvPath) {
    std::ifstream file(logCsvPath);
    if (!file.is_open()) return;

    std::string line;
    std::getline(file, line); // skip header

    int total = 0, cntFull = 0, cntFLL = 0, cntOpenDLL = 0, cntLost = 0;
    float minCN0 = 999.0f, maxJS = -999.0f, maxSigPosN = 0.0f;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string val;
        std::vector<std::string> row;
        while (std::getline(ss, val, ',')) row.push_back(val);
        if (row.size() < 18) continue;

        try {
            const float js_dB  = std::stof(row[4]);
            const float effCN0 = std::stof(row[7]);
            const int state    = std::stoi(row[10]);
            const float sigN   = std::stof(row[16]);

            if (effCN0 < minCN0) minCN0 = effCN0;
            if (js_dB > maxJS && js_dB > -900.0f) maxJS = js_dB;
            if (sigN > maxSigPosN && state != static_cast<int>(TrackingLockState::LOST_LOCK)) {
                maxSigPosN = sigN;
            }

            if (state == static_cast<int>(TrackingLockState::FULL_LOCK_PLL))          cntFull++;
            else if (state == static_cast<int>(TrackingLockState::FLL_BACKUP_LOCK))   cntFLL++;
            else if (state == static_cast<int>(TrackingLockState::OPEN_CARRIER_AIDED_DLL)) cntOpenDLL++;
            else cntLost++;

            total++;
        } catch (...) {}
    }

    std::cout << "\n==================== RINGKASAN HASIL JAMMING ====================\n";
    std::cout << "  Total Epoch GPS Diproses : " << total << " baris\n";
    std::cout << "  - FULL_LOCK_PLL          : " << cntFull << " epoch\n";
    std::cout << "  - FLL_BACKUP_LOCK        : " << cntFLL << " epoch\n";
    std::cout << "  - OPEN_CARRIER_AIDED_DLL : " << cntOpenDLL << " epoch\n";
    std::cout << "  - LOST_LOCK (Outage)     : " << cntLost << " epoch\n";
    std::cout << "  Puncak (J/S)_dB          : " << std::fixed << std::setprecision(2) << maxJS << " dB\n";
    std::cout << "  Minimum (C/N0)_eff       : " << minCN0 << " dB-Hz\n";
    std::cout << "  Max Excess sigma_N (Lock): " << maxSigPosN << " m\n";
    std::cout << "=================================================================\n";
}

int main() {
    std::cout << "=================================================================\n";
    std::cout << "[INFO] Simulator Injeksi Jamming RF -> Dataset PX4 (Standalone)\n";
    std::cout << "=================================================================\n";

    // 1. Resolusi path dataset asli PX4 (sama dengan main.cpp Anda)
    const std::string dataDir = "E:/Tugas Akhir/New Firmware/Sim/data/";

    const std::string pathCleanGPS = resolveFilePath(dataDir, "vehicle_gps_position", {
        dataDir + "vehicle_gps_position.csv",
        dataDir + "vehicle_gps_position_0.csv",
        "vehicle_gps_position.csv",
        "Dataset_vehicle_gps_position_0.csv"
    });

    const std::string pathAtt = resolveFilePath(dataDir, "vehicle_attitude", {
        dataDir + "vehicle_attitude.csv",
        dataDir + "vehicle_attitude_0.csv",
        "vehicle_attitude.csv"
    });

    const std::string pathOutJammedGPS = "vehicle_gps_position_jammed.csv";
    const std::string pathOutTelemetry = "jamming_telemetry_log.csv";

    // 2. Inspeksi lintasan penerbangan asli PX4
    FlightSummary flt = inspectPX4Flight(pathCleanGPS);
    if (!flt.valid) {
        std::cerr << "[ERROR] Gagal membaca dataset GPS asli di: " << pathCleanGPS << "\n";
        std::cin.get();
        return 1;
    }

    std::cout << "[OK] Dataset GPS Asli : " << pathCleanGPS << " (" << flt.totalRows << " baris)\n";
    std::cout << "[OK] Durasi Terbang   : " << std::fixed << std::setprecision(2) << flt.totalDuration_s << " detik\n";
    std::cout << "[OK] Titik Tengah Rute: Lat=" << std::setprecision(7) << flt.midLat_rad * (180.0 / M_PI)
              << ", Lon=" << flt.midLon_rad * (180.0 / M_PI)
              << ", GroundAlt=" << std::setprecision(2) << flt.groundAlt_m << " m\n";

    // 3. Konfigurasi Penerima GNSS pada Drone (GPS L1 C/A Komersial Unaided)
    ReceiverFrontEndConfig rxCfg = JammingCorrelator::createDefaultReceiverConfig(GNSSSignalType::GPS_L1_CA);
    rxCfg.enableCRPA               = false; // Ubah ke true untuk uji antena anti-jamming CRPA
    rxCfg.enableNarrowbandExcision = false;
    rxCfg.enablePulseBlanking      = false;

    TrackingLoopConfig trkCfg = JammingTrackingLoop::createDefaultTrackingConfig(
        GNSSSignalType::GPS_L1_CA, ReceiverAidingMode::UNAIDED
    );

    JammingSimOptions simOpts = {};
    simOpts.randomSeed                = 12345u; // Seed tetap agar eksperimen reproducible
    simOpts.reportDegradedAccuracy    = true;   // true: EKF sadar (eph/epv naik); false: EKF naif (nis_chi2 meledak!)
    simOpts.zeroVelocityOnOpenCarrier = true;

    JammingEngine engine;
    engine.configure(rxCfg, trkCfg, simOpts);

    // 4. Rancang Skenario: Taruh 1 Orang Pemegang Jammer Portabel di Darat
    //    Lokasi : 80 meter di sebelah Timur titik tengah rute terbang drone
    //    Jadwal : Menyala mulai 35% s/d 65% durasi terbang (setelah EKF konvergen)
    //    Daya   : 0.005 Watt (5 mW) BLWN -> cukup membuat transisi degradasi -> Loss of Lock -> Recovery!
    const double R_EARTH = 6378137.0;
    const double offsetEast_m = 80.0;

    JammerEntity jammer1 = {};
    jammer1.id = "Operator_Jammer_1";

    jammer1.propCfg.isEnabled           = true;
    jammer1.propCfg.startTime_s         = flt.totalDuration_s * 0.35;
    jammer1.propCfg.endTime_s           = flt.totalDuration_s * 0.65;
    jammer1.propCfg.L_j                 = flt.midLat_rad;
    jammer1.propCfg.lambda_j            = flt.midLon_rad + offsetEast_m / (R_EARTH * std::cos(flt.midLat_rad));
    jammer1.propCfg.h_j                 = flt.groundAlt_m + 1.5; // Dipegang orang setinggi 1.5 m
    jammer1.propCfg.baseHeightAGL_m     = 1.5f;
    jammer1.propCfg.freq_MHz            = 1575.42f;
    jammer1.propCfg.transmitPower_W     = 0.005f;                // 5 mW portable jammer
    jammer1.propCfg.antennaGain_dBic    = 0.0f;
    jammer1.propCfg.propModel           = PropagationModelType::FREE_SPACE;
    jammer1.propCfg.vegetationDepth_m   = 0.0f;
    jammer1.propCfg.isVegetationInLeaf  = false;

    jammer1.sigCfg.waveform             = JammerWaveformType::WIDEBAND_BLWN_NULL;
    jammer1.sigCfg.polarization         = JammerPolarization::LINEAR; // Antena monopole linear
    jammer1.sigCfg.customBandwidth_Hz   = 0.0f;
    jammer1.sigCfg.pulseDutyCycle       = 1.0f;

    engine.addJammer(jammer1);

    // 5. Hitung Radius Zona Ancaman untuk Visualisasi Peta Mission Planner (Antigravity)
    const float threatRadius_m = engine.computeThreatRadius_m(jammer1, 30.0f, 5.0f);
    std::cout << "\n[SKENARIO] " << jammer1.id << " aktif pada t = ["
              << std::setprecision(1) << jammer1.propCfg.startTime_s << " s .. "
              << jammer1.propCfg.endTime_s << " s]\n";
    std::cout << "[SKENARIO] Estimasi Radius Zona Jamming (Peta Mission Planner): "
              << std::setprecision(1) << threatRadius_m << " meter\n";

    // 6. Jalankan Injeksi Batch ke CSV PX4 (Groves Section 17.7.2 - Recorded Data Testing)
    std::cout << "\n[INFO] Menjalankan injeksi jamming ke dataset PX4...\n";
    if (!engine.injectToPX4CSV(pathCleanGPS, pathAtt, pathOutJammedGPS, pathOutTelemetry)) {
        std::cerr << "[ERROR] Injeksi Jamming gagal!\n";
        std::cin.get();
        return 1;
    }

    printTelemetrySummary(pathOutTelemetry);

    std::cout << "[SUKSES] Output GPS Jammed tersimpan di : " << pathOutJammedGPS << "\n";
    std::cout << "[SUKSES] Output Telemetri RF tersimpan  : " << pathOutTelemetry << "\n";
    std::cout << "\nTekan Enter untuk selesai...";
    std::cin.get();
    return 0;
}