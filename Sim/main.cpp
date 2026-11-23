#ifndef EIGEN_DONT_VECTORIZE
#define EIGEN_DONT_VECTORIZE
#endif

#ifndef EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#endif

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cmath>
#include <algorithm>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "SensorReader.h"
#include "../Core/IMUPreProcessing.h"
#include "../Core/INSMechanization.h"
#include "../Core/ExtendedKF.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct GPSDataPacket {
    double timestamp; // usec
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
    bool has_dynamic_acc;
    bool isValid;
};

// Pembaca data GNSS PX4 berbasis nama header CSV
class GPSReader {
public:
    explicit GPSReader(const std::string& path) : filename(path) {}

    ~GPSReader() {
        if (file.is_open()) {
            file.close();
        }
    }

    bool initialize() {
        file.open(filename);
        if (!file.is_open()) {
            return false;
        }

        std::string headerLine;
        if (!std::getline(file, headerLine)) {
            return false;
        }

        headerLine.erase(std::remove(headerLine.begin(), headerLine.end(), '\r'), headerLine.end());
        headerLine.erase(std::remove(headerLine.begin(), headerLine.end(), '\n'), headerLine.end());

        std::stringstream ss(headerLine);
        std::string col;
        int index = 0;

        while (std::getline(ss, col, ',')) {
            col.erase(std::remove(col.begin(), col.end(), '\"'), col.end());

            if (col == "timestamp") idx_time = index;
            else if (col == "latitude_deg") { idx_lat = index; is_lat_scaled_int = false; }
            else if (col == "lat") { idx_lat = index; is_lat_scaled_int = true; }
            else if (col == "longitude_deg") { idx_lon = index; is_lon_scaled_int = false; }
            else if (col == "lon") { idx_lon = index; is_lon_scaled_int = true; }
            else if (col == "altitude_ellipsoid_m") { idx_alt = index; is_alt_scaled_mm = false; }
            else if (col == "altitude_msl_m" && idx_alt == -1) { idx_alt = index; is_alt_scaled_mm = false; }
            else if (col == "alt_ellipsoid" && idx_alt == -1) { idx_alt = index; is_alt_scaled_mm = true; }
            else if (col == "alt" && idx_alt == -1) { idx_alt = index; is_alt_scaled_mm = true; }
            else if (col == "vel_n_m_s") idx_vn = index;
            else if (col == "vel_e_m_s") idx_ve = index;
            else if (col == "vel_d_m_s") idx_vd = index;
            else if (col == "eph") idx_eph = index;
            else if (col == "epv") idx_epv = index;
            else if (col == "s_variance_m_s") idx_s_var = index;
            else if (col == "fix_type") idx_fix_type = index;
            else if (col == "vel_ned_valid") idx_vel_valid = index;
            else if (col == "satellites_used") idx_sat_used = index;

            index++;
        }

        if (idx_time == -1 || idx_lat == -1 || idx_lon == -1 || idx_alt == -1 ||
            idx_vn == -1 || idx_ve == -1 || idx_vd == -1) {
            return false;
        }

        return true;
    }

    GPSDataPacket readNext() {
        GPSDataPacket pkt = {};
        pkt.isValid = false;
        pkt.fix_type = 3;
        pkt.vel_ned_valid = true;
        pkt.satellites_used = 0;
        pkt.has_dynamic_acc = false;

        std::string line;
        if (std::getline(file, line)) {
            line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
            line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
            if (line.empty()) {
                return pkt;
            }

            std::stringstream ss(line);
            std::string val;
            std::vector<std::string> row;
            while (std::getline(ss, val, ',')) {
                row.push_back(val);
            }

            const int req_max = std::max({idx_time, idx_lat, idx_lon, idx_alt, idx_vn, idx_ve, idx_vd});
            if (static_cast<int>(row.size()) <= req_max) {
                return pkt;
            }

            try {
                pkt.timestamp = std::stod(row[idx_time]);

                double raw_lat = std::stod(row[idx_lat]);
                double raw_lon = std::stod(row[idx_lon]);
                double raw_alt = std::stod(row[idx_alt]);

                if (is_lat_scaled_int || std::abs(raw_lat) > 90.0) {
                    raw_lat *= 1.0e-7;
                }
                if (is_lon_scaled_int || std::abs(raw_lon) > 180.0) {
                    raw_lon *= 1.0e-7;
                }
                if (is_alt_scaled_mm) {
                    raw_alt *= 1.0e-3;
                }

                pkt.lat_rad = raw_lat * (M_PI / 180.0);
                pkt.lon_rad = raw_lon * (M_PI / 180.0);
                pkt.alt_m   = raw_alt;

                pkt.vn = std::stof(row[idx_vn]);
                pkt.ve = std::stof(row[idx_ve]);
                pkt.vd = std::stof(row[idx_vd]);

                if (idx_eph >= 0 && idx_epv >= 0 && idx_s_var >= 0 &&
                    static_cast<int>(row.size()) > std::max({idx_eph, idx_epv, idx_s_var})) {
                    pkt.eph        = std::stof(row[idx_eph]);
                    pkt.epv        = std::stof(row[idx_epv]);
                    pkt.s_variance = std::stof(row[idx_s_var]);
                    pkt.has_dynamic_acc = std::isfinite(pkt.eph) &&
                                          std::isfinite(pkt.epv) &&
                                          std::isfinite(pkt.s_variance);
                }

                if (idx_fix_type >= 0 && static_cast<int>(row.size()) > idx_fix_type) {
                    pkt.fix_type = std::stoi(row[idx_fix_type]);
                }
                if (idx_vel_valid >= 0 && static_cast<int>(row.size()) > idx_vel_valid) {
                    pkt.vel_ned_valid = (std::stoi(row[idx_vel_valid]) != 0);
                }
                if (idx_sat_used >= 0 && static_cast<int>(row.size()) > idx_sat_used) {
                    pkt.satellites_used = std::stoi(row[idx_sat_used]);
                }

                pkt.isValid = true;
            } catch (...) {
                pkt.isValid = false;
            }
        }
        return pkt;
    }

    bool hasDynamicAccuracyColumns() const {
        return (idx_eph >= 0 && idx_epv >= 0 && idx_s_var >= 0);
    }

private:
    std::string filename;
    std::ifstream file;

    int idx_time      = -1;
    int idx_lat       = -1;
    int idx_lon       = -1;
    int idx_alt       = -1;
    int idx_vn        = -1;
    int idx_ve        = -1;
    int idx_vd        = -1;
    int idx_eph       = -1;
    int idx_epv       = -1;
    int idx_s_var     = -1;
    int idx_fix_type  = -1;
    int idx_vel_valid = -1;
    int idx_sat_used  = -1;

    bool is_lat_scaled_int = false;
    bool is_lon_scaled_int = false;
    bool is_alt_scaled_mm  = false;
};

// Cek keberadaan berkas dari daftar kandidat path maupun wildcard folder data
static std::string resolveFilePath(const std::string& directory,
                                   const std::string& keyword,
                                   const std::vector<std::string>& candidates) {
    for (const auto& path : candidates) {
        std::ifstream test(path);
        if (test.is_open()) {
            test.close();
            return path;
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

// Baca sikap awal (roll, pitch, yaw dalam radian) dari kuaternion PX4 vehicle_attitude
static bool loadInitialAttitudeFromCSV(const std::string& path,
                                       float& roll_rad, float& pitch_rad, float& yaw_rad) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }

    std::string headerLine;
    if (!std::getline(file, headerLine)) {
        return false;
    }
    headerLine.erase(std::remove(headerLine.begin(), headerLine.end(), '\r'), headerLine.end());
    headerLine.erase(std::remove(headerLine.begin(), headerLine.end(), '\n'), headerLine.end());

    int idx_q[4] = {-1, -1, -1, -1};
    std::stringstream ssHeader(headerLine);
    std::string col;
    int idx = 0;
    while (std::getline(ssHeader, col, ',')) {
        col.erase(std::remove(col.begin(), col.end(), '\"'), col.end());
        if (col == "q[0]") idx_q[0] = idx;
        else if (col == "q[1]") idx_q[1] = idx;
        else if (col == "q[2]") idx_q[2] = idx;
        else if (col == "q[3]") idx_q[3] = idx;
        idx++;
    }

    if (idx_q[0] == -1 || idx_q[1] == -1 || idx_q[2] == -1 || idx_q[3] == -1) {
        return false;
    }

    std::string dataLine;
    if (!std::getline(file, dataLine)) {
        return false;
    }
    dataLine.erase(std::remove(dataLine.begin(), dataLine.end(), '\r'), dataLine.end());
    dataLine.erase(std::remove(dataLine.begin(), dataLine.end(), '\n'), dataLine.end());

    std::stringstream ssData(dataLine);
    std::string val;
    std::vector<std::string> row;
    while (std::getline(ssData, val, ',')) {
        row.push_back(val);
    }

    const int max_q = std::max({idx_q[0], idx_q[1], idx_q[2], idx_q[3]});
    if (static_cast<int>(row.size()) <= max_q) {
        return false;
    }

    try {
        const double q0 = std::stod(row[idx_q[0]]);
        const double q1 = std::stod(row[idx_q[1]]);
        const double q2 = std::stod(row[idx_q[2]]);
        const double q3 = std::stod(row[idx_q[3]]);

        // Konversi kuaternion PX4 (FRD-to-NED) ke sudut Euler Z-Y-X
        roll_rad  = static_cast<float>(std::atan2(2.0 * (q0 * q1 + q2 * q3),
                                                  1.0 - 2.0 * (q1 * q1 + q2 * q2)));
        const double sin_p = std::max(-1.0, std::min(2.0 * (q0 * q2 - q3 * q1), 1.0));
        pitch_rad = static_cast<float>(std::asin(sin_p));
        yaw_rad   = static_cast<float>(std::atan2(2.0 * (q0 * q3 + q1 * q2),
                                                  1.0 - 2.0 * (q2 * q2 + q3 * q3)));
        return true;
    } catch (...) {
        return false;
    }
}

// Eq. (2.25) - Euler Angles Extraction from C^n_b (Groves)
static void extractEulerDeg(const Eigen::Matrix3f& cn_b,
                            float& roll_deg, float& pitch_deg, float& yaw_deg) {
    const float rad2deg = static_cast<float>(180.0 / M_PI);
    const float sin_pitch = std::max(-1.0f, std::min(-cn_b(2, 0), 1.0f));

    pitch_deg = std::asin(sin_pitch) * rad2deg;
    roll_deg  = std::atan2(cn_b(2, 1), cn_b(2, 2)) * rad2deg;
    yaw_deg   = std::atan2(cn_b(1, 0), cn_b(0, 0)) * rad2deg;
}

int main() {
    try {
        std::cout << "====================================================\n";
        std::cout << "[INFO] Memulai Simulasi Fusi ESKF 15-State INS/GNSS\n";
        std::cout << "====================================================\n";

        // Konfigurasi mode eksekusi ExtendedKF
        constexpr bool USE_SEQUENTIAL_UPDATE = false; // true: Eq. (3.73)-(3.75), false: Eq. (3.21) & (3.58)
        constexpr bool USE_DYNAMIC_GNSS_R    = true;  // true: perbarui R_k dari eph, epv, s_variance_m_s

        // Resolusi path dataset (mendukung nama dengan maupun tanpa awalan/akhiran ULog)
        const std::string dataDir = "E:/Tugas Akhir/New Firmware/Sim/data/";

        const std::string pathIMU = resolveFilePath(dataDir, "sensor_combined", {
            dataDir + "sensor_combined.csv",
            dataDir + "sensor_combined_0.csv",
            "sensor_combined.csv",
            "Dataset_sensor_combined_0.csv"
        });

        const std::string pathGPS = resolveFilePath(dataDir, "vehicle_gps_position", {
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

        // 1. Inisialisasi pembaca sensor
        SensorReader imuReader(pathIMU);
        if (!imuReader.initialize()) {
            std::cerr << "[ERROR] Gagal membuka dataset IMU di: " << pathIMU << "\n";
            std::cin.get();
            return 1;
        }
        std::cout << "[OK] Dataset IMU dibuka: " << pathIMU << "\n";

        GPSReader gpsReader(pathGPS);
        if (!gpsReader.initialize()) {
            std::cerr << "[ERROR] Gagal membuka dataset GPS di: " << pathGPS << "\n";
            std::cin.get();
            return 1;
        }
        std::cout << "[OK] Dataset GPS dibuka: " << pathGPS << "\n";
        std::cout << "[INFO] Kolom akurasi dinamis GNSS (eph/epv/s_variance): "
                  << (gpsReader.hasDynamicAccuracyColumns() ? "TERDETEKSI" : "TIDAK TERDETEKSI")
                  << "\n";

        // 2. Sinkronisasi awal sampel GPS 3D fix dan sampel IMU pertama
        GPSDataPacket firstGPS = {};
        while (true) {
            firstGPS = gpsReader.readNext();
            if (!firstGPS.isValid) {
                std::cerr << "[ERROR] Tidak ditemukan baris GPS 3D fix yang valid!\n";
                std::cin.get();
                return 1;
            }
            if (firstGPS.fix_type >= 3) {
                break;
            }
        }

        IMUDataPacket firstIMU = imuReader.readNextRow();
        if (!firstIMU.isDataValid) {
            std::cerr << "[ERROR] Gagal membaca baris IMU pertama!\n";
            std::cin.get();
            return 1;
        }

        GPSDataPacket nextGPS = gpsReader.readNext();
        while (nextGPS.isValid && nextGPS.timestamp <= firstIMU.timestamp) {
            if (nextGPS.fix_type >= 3) {
                firstGPS = nextGPS;
            }
            nextGPS = gpsReader.readNext();
        }

        std::cout << "[OK] Titik Awal GPS: t=" << std::fixed << std::setprecision(3)
                  << firstGPS.timestamp * 1.0e-6 << " s, Lat=" << std::setprecision(7)
                  << firstGPS.lat_rad * (180.0 / M_PI)
                  << ", Lon=" << firstGPS.lon_rad * (180.0 / M_PI)
                  << ", Alt(ellipsoid)=" << std::setprecision(3) << firstGPS.alt_m << " m\n";

        // 3. Inisialisasi sikap awal dari vehicle_attitude (fallback dari baris 0 log PX4)
        float init_roll  =  0.046976f;
        float init_pitch = -0.006194f;
        float init_yaw   =  1.172044f;

        if (loadInitialAttitudeFromCSV(pathAtt, init_roll, init_pitch, init_yaw)) {
            std::cout << "[OK] Sikap awal dimuat dari: " << pathAtt << "\n";
        } else {
            std::cout << "[INFO] Menggunakan sikap awal default terkalibrasi (rad): "
                      << "roll=" << init_roll << ", pitch=" << init_pitch
                      << ", yaw=" << init_yaw << "\n";
        }

        INSMechanization ins;
        ins.initialize(firstGPS.lat_rad, firstGPS.lon_rad, firstGPS.alt_m,
                       init_roll, init_pitch, init_yaw);

        IMUPreProcessing preProc;
        preProc.setBiasesStatic(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
        preProc.setScaleCrossMatrices(Eigen::Matrix3f::Identity(),
                                      Eigen::Matrix3f::Identity(),
                                      Eigen::Matrix3f::Zero());

        // 4. Inisialisasi 15-State Error-State Extended Kalman Filter
        ExtendedKF ekf;

        // Eq. (3.5) - Initial Error Covariance Matrix P0 (Groves)
        Eigen::Matrix<float, 15, 15> P0 = Eigen::Matrix<float, 15, 15>::Zero();
        P0.block<3, 3>(0, 0)   = Eigen::Matrix3f::Identity() * (1.0f * 1.0f);         // Posisi (m^2)
        P0.block<3, 3>(3, 3)   = Eigen::Matrix3f::Identity() * (0.2f * 0.2f);         // Kecepatan (m/s)^2
        P0.block<3, 3>(6, 6)   = Eigen::Matrix3f::Identity() * (0.01745f * 0.01745f); // Sikap (~1 deg)^2
        P0.block<3, 3>(9, 9)   = Eigen::Matrix3f::Identity() * (0.05f * 0.05f);       // Bias akselerometer
        P0.block<3, 3>(12, 12) = Eigen::Matrix3f::Identity() * (0.005f * 0.005f);     // Bias giroskop

        // Eq. (14.81) - System Noise PSD Parameters (Groves)
        const float Sa_psd  = 0.05f * 0.05f;       // (m/s^2)^2 / Hz (terverifikasi dari getaran terbang)
        const float Sg_psd  = 0.005f * 0.005f;     // (rad/s)^2 / Hz
        const float Sba_psd = 0.0001f * 0.0001f;   // (m/s^2)^2 / s
        const float Sbg_psd = 0.00001f * 0.00001f; // (rad/s)^2 / s
        ekf.setProcessNoisePSD(Sa_psd, Sg_psd, Sba_psd, Sbg_psd);

        // Eq. (3.3) - First-Order Gauss-Markov Correlation Times (0.0f = Random Walk) (Groves)
        ekf.setBiasCorrelationTimes(0.0f, 0.0f);

        // Eq. (3.11) - Nominal Measurement Noise Covariance Matrix R (Groves)
        const float nominal_sigma_pos = 0.80f; // m (rata-rata eph log PX4)
        const float nominal_sigma_alt = 1.37f; // m (rata-rata epv log PX4)
        const float nominal_sigma_vel = 0.26f; // m/s (rata-rata s_variance_m_s log PX4)

        Eigen::Matrix<float, 6, 6> R_init = Eigen::Matrix<float, 6, 6>::Zero();
        R_init(0, 0) = nominal_sigma_pos * nominal_sigma_pos;
        R_init(1, 1) = nominal_sigma_pos * nominal_sigma_pos;
        R_init(2, 2) = nominal_sigma_alt * nominal_sigma_alt;
        R_init(3, 3) = nominal_sigma_vel * nominal_sigma_vel;
        R_init(4, 4) = nominal_sigma_vel * nominal_sigma_vel;
        R_init(5, 5) = (nominal_sigma_vel * 1.5f) * (nominal_sigma_vel * 1.5f);

        ekf.initialize(P0, R_init);

        // 5. Siapkan berkas keluaran trajektori dan diagnostik EKF
        const std::string trajOutputPath = "ins_ekf_fused_trajectory.csv";
        const std::string diagOutputPath = "ekf_gnss_diagnostics.csv";

        std::ofstream outFile(trajOutputPath);
        std::ofstream diagFile(diagOutputPath);
        if (!outFile.is_open() || !diagFile.is_open()) {
            std::cerr << "[ERROR] Gagal membuat file output CSV!\n";
            std::cin.get();
            return 1;
        }

        outFile << "timestamp,latitude_deg,longitude_deg,height_m,"
                << "v_N,v_E,v_D,roll_deg,pitch_deg,yaw_deg,"
                << "bias_ax,bias_ay,bias_az,bias_gx,bias_gy,bias_gz,"
                << "sig_pN,sig_pE,sig_pD,sig_vN,sig_vE,sig_vD,"
                << "sig_psiN,sig_psiE,sig_psiD\n";

        diagFile << "timestamp,dz_pN,dz_pE,dz_pD,dz_vN,dz_vE,dz_vD,"
                 << "nis_chi2,r_pN,r_pD,r_vN,"
                 << "normP_pN,normP_vN,normP_psiD,normP_baX,normP_bgZ\n";

        GPSDataPacket currentGPS = firstGPS;
        Eigen::Vector3f ba_est = Eigen::Vector3f::Zero();
        Eigen::Vector3f bg_est = Eigen::Vector3f::Zero();

        double prev_imu_timestamp = firstIMU.timestamp;
        int step = 0;
        int gnss_updates = 0;

        std::cout << "[INFO] Memproses fusi data IMU dan GNSS...\n";

        while (true) {
            IMUDataPacket imuPkt = imuReader.readNextRow();
            if (!imuPkt.isDataValid) {
                std::cout << "\n[SELESAI] Total sampel IMU diproses: " << step
                          << ", total update GNSS: " << gnss_updates << ".\n";
                break;
            }

            float dt = static_cast<float>((imuPkt.timestamp - prev_imu_timestamp) * 1.0e-6);
            prev_imu_timestamp = imuPkt.timestamp;
            if (dt <= 0.0f || dt > 0.05f) {
                dt = 0.006f;
            }

            // Tahap 1: IMU Pre-Processing (Groves Eq. 4.19-4.21)
            preProc.inputRawData(imuPkt);
            ProcessedIMUData procImu = preProc.getProcessedData();
            if (!procImu.isValid) {
                continue;
            }

            // Kompensasi bias sensor dinamis (Closed-Loop Feedback)
            Eigen::Vector3f fb_ib     = procImu.acc - ba_est;
            Eigen::Vector3f omegab_ib = procImu.gyr - bg_est;

            // Tahap 2: Mekanisasi INS Local NED (Groves Bab 5)
            Eigen::Matrix3f cn_b_minus = ins.getOrientation();
            Eigen::Vector3f vn_minus   = ins.getVelocity();

            ins.updateAttitude(omegab_ib, dt);
            Eigen::Matrix3f cn_b_plus = ins.getOrientation();

            Eigen::Vector3f fn_ib = ins.transformSpecificForce(fb_ib, cn_b_minus, cn_b_plus);
            ins.updateVelocity(fn_ib, dt);
            Eigen::Vector3f vn_plus = ins.getVelocity();

            ins.updatePosition(vn_minus, vn_plus, dt);

            // Tahap 3: Fase Prediksi EKF (Groves Eq. 3.14 & 3.46)
            ekf.predict(cn_b_plus, fn_ib, dt);

            // Tahap 4: Fase Koreksi EKF saat paket GNSS tersedia
            while (nextGPS.isValid && nextGPS.timestamp <= imuPkt.timestamp) {
                currentGPS = nextGPS;

                if (currentGPS.fix_type >= 3 && currentGPS.vel_ned_valid) {
                    // Eq. (3.11) - Dynamic Measurement Noise Covariance Matrix R_k (Groves)
                    Eigen::Matrix<float, 6, 1> r_diag;
                    if (USE_DYNAMIC_GNSS_R && currentGPS.has_dynamic_acc) {
                        const float sig_h = std::max(currentGPS.eph, 0.5f);
                        const float sig_v = std::max(currentGPS.epv, 1.0f);
                        const float sig_s = std::max(currentGPS.s_variance, 0.1f);

                        r_diag << sig_h * sig_h,
                                  sig_h * sig_h,
                                  sig_v * sig_v,
                                  sig_s * sig_s,
                                  sig_s * sig_s,
                                  (sig_s * 1.5f) * (sig_s * 1.5f);
                    } else {
                        r_diag << nominal_sigma_pos * nominal_sigma_pos,
                                  nominal_sigma_pos * nominal_sigma_pos,
                                  nominal_sigma_alt * nominal_sigma_alt,
                                  nominal_sigma_vel * nominal_sigma_vel,
                                  nominal_sigma_vel * nominal_sigma_vel,
                                  (nominal_sigma_vel * 1.5f) * (nominal_sigma_vel * 1.5f);
                    }
                    ekf.setMeasurementNoiseDiagonal(r_diag);

                    double cur_L, cur_lambda, cur_h;
                    ins.getPosition(cur_L, cur_lambda, cur_h);
                    Eigen::Vector3f cur_vel = ins.getVelocity();

                    double RN_Lb, RE_Lb;
                    ins.computeEarth(cur_L, RN_Lb, RE_Lb);

                    // Eq. (14.102) - Loosely Coupled Measurement Innovation Vector (Groves)
                    Eigen::Matrix<float, 6, 1> dz;
                    dz(0) = static_cast<float>((cur_L - currentGPS.lat_rad) * (RN_Lb + cur_h));
                    dz(1) = static_cast<float>((cur_lambda - currentGPS.lon_rad) * (RE_Lb + cur_h) * std::cos(cur_L));
                    dz(2) = static_cast<float>(-(cur_h - currentGPS.alt_m));
                    dz(3) = cur_vel(0) - currentGPS.vn;
                    dz(4) = cur_vel(1) - currentGPS.ve;
                    dz(5) = cur_vel(2) - currentGPS.vd;

                    if (USE_SEQUENTIAL_UPDATE) {
                        ekf.correctSequential(dz);
                    } else {
                        ekf.correct(dz);
                    }

                    // Ekstraksi diagnostik inovasi (Chi-Square NIS) dan metrik observability
                    Eigen::Matrix<float, 6, 1> dz_minus = ekf.getLastInnovation();
                    Eigen::Matrix<float, 6, 6> S_cov    = ekf.getInnovationCovariance();
                    Eigen::Matrix<float, 15, 1> norm_P  = ekf.getNormalizedCovarianceDiagonal();
                    const float nis_chi2 = dz_minus.dot(S_cov.ldlt().solve(dz_minus));

                    // Eq. (14.60)-(14.63) - Closed-Loop Error Feedback (Groves)
                    Eigen::Matrix<float, 15, 1> x_err = ekf.getErrorState();
                    ins.applyErrorFeedback(x_err.segment<3>(0),
                                           x_err.segment<3>(3),
                                           x_err.segment<3>(6));

                    ba_est += x_err.segment<3>(9);
                    bg_est += x_err.segment<3>(12);

                    // Section 3.2.6 - Full Closed-Loop State Reset (Groves)
                    ekf.resetState();

                    diagFile << std::fixed << std::setprecision(6) << currentGPS.timestamp * 1.0e-6 << ","
                             << std::setprecision(4)
                             << dz_minus(0) << "," << dz_minus(1) << "," << dz_minus(2) << ","
                             << dz_minus(3) << "," << dz_minus(4) << "," << dz_minus(5) << ","
                             << nis_chi2 << ","
                             << r_diag(0) << "," << r_diag(2) << "," << r_diag(3) << ","
                             << std::setprecision(6)
                             << norm_P(0) << "," << norm_P(3) << "," << norm_P(8) << ","
                             << norm_P(9) << "," << norm_P(14) << "\n";

                    gnss_updates++;
                }

                nextGPS = gpsReader.readNext();
            }

            // Pencatatan trajektori, sudut Euler, bias, dan ketidakpastian state (Groves Eq. 3.6)
            double out_L, out_lambda, out_h;
            ins.getPosition(out_L, out_lambda, out_h);
            Eigen::Vector3f out_v    = ins.getVelocity();
            Eigen::Matrix3f out_cn_b = ins.getOrientation();
            Eigen::Matrix<float, 15, 1> sig = ekf.getStateUncertainties();

            float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
            extractEulerDeg(out_cn_b, roll_deg, pitch_deg, yaw_deg);

            outFile << std::fixed << std::setprecision(6) << imuPkt.timestamp * 1.0e-6 << ","
                    << std::setprecision(8) << out_L * (180.0 / M_PI) << ","
                    << std::setprecision(8) << out_lambda * (180.0 / M_PI) << ","
                    << std::setprecision(3) << out_h << ","
                    << std::setprecision(4) << out_v(0) << "," << out_v(1) << "," << out_v(2) << ","
                    << std::setprecision(4) << roll_deg << "," << pitch_deg << "," << yaw_deg << ","
                    << std::setprecision(6) << ba_est(0) << "," << ba_est(1) << "," << ba_est(2) << ","
                    << std::setprecision(6) << bg_est(0) << "," << bg_est(1) << "," << bg_est(2) << ","
                    << std::setprecision(4)
                    << sig(0) << "," << sig(1) << "," << sig(2) << ","
                    << sig(3) << "," << sig(4) << "," << sig(5) << ","
                    << sig(6) << "," << sig(7) << "," << sig(8) << "\n";

            step++;
            if (step % 200 == 0) {
                outFile.flush();
                diagFile.flush();
            }
            if (step % 2000 == 0) {
                std::cout << "." << std::flush;
            }
        }

        outFile.close();
        diagFile.close();
        std::cout << "\n[SUKSES] Trajektori tersimpan di: " << trajOutputPath << "\n";
        std::cout << "[SUKSES] Diagnostik EKF tersimpan di: " << diagOutputPath << "\n";
        std::cout << "Tekan Enter untuk selesai...";
        std::cin.get();

    } catch (const std::exception& e) {
        std::cerr << "\n[EXCEPTION ERROR] Terjadi kendala: " << e.what() << "\n";
        std::cin.get();
        return 1;
    }

    return 0;
}