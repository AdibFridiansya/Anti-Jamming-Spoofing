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

#include "SensorReader.h"
#include "../Core/INSMechanization.h"
#include "../Core/ExtendedKF.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Struktur data paket observasi GPS
struct GPSDataPacket {
    double timestamp; // usec
    double lat_rad;
    double lon_rad;
    double alt_m;
    float vn;
    float ve;
    float vd;
    bool isValid;
};

// Kelas pembaca data GPS terurut waktu
class GPSReader {
public:
    explicit GPSReader(const std::string& path) : filename(path) {}
    
    bool initialize() {
        file.open(filename);
        if (!file.is_open()) return false;
        std::string header;
        std::getline(file, header);
        return true;
    }

    GPSDataPacket readNext() {
        GPSDataPacket pkt = {};
        pkt.isValid = false;
        std::string line;
        if (std::getline(file, line)) {
            line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
            line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
            if (line.empty()) return pkt;
            
            std::stringstream ss(line);
            std::string val;
            std::vector<std::string> row;
            while (std::getline(ss, val, ',')) {
                row.push_back(val);
            }
            
            if (row.size() < 20) return pkt;

            try {
                pkt.timestamp = std::stod(row[0]);
                pkt.lat_rad   = std::stod(row[2]) * (M_PI / 180.0);
                pkt.lon_rad   = std::stod(row[3]) * (M_PI / 180.0);
                pkt.alt_m     = std::stod(row[5]); // altitude_ellipsoid_m
                pkt.vn        = std::stof(row[17]);
                pkt.ve        = std::stof(row[18]);
                pkt.vd        = std::stof(row[19]);
                pkt.isValid   = true;
            } catch (...) {
                pkt.isValid = false;
            }
        }
        return pkt;
    }

private:
    std::string filename;
    std::ifstream file;
};

int main() {
    try {
        std::cout << "====================================================\n";
        std::cout << "[INFO] Memulai Simulasi Fusi EKF INS/GNSS...\n";
        std::cout << "====================================================\n";

        // Deteksi path (Mendukung path lokal direktori eksekusi maupun path absolut tugas akhir)
        std::string pathIMU = "Dataset_sensor_combined_0.csv";
        std::string pathGPS = "Dataset_vehicle_gps_position_0.csv";

        std::ifstream testIMU(pathIMU);
        if (!testIMU.is_open()) {
            pathIMU = "E:/Tugas Akhir/New Firmware/Sim/data/Dataset_sensor_combined_0.csv";
            pathGPS = "E:/Tugas Akhir/New Firmware/Sim/data/Dataset_vehicle_gps_position_0.csv";
        } else {
            testIMU.close();
        }

        // 1. Inisialisasi Pembaca Sensor
        SensorReader imuReader(pathIMU);
        if (!imuReader.initialize()) {
            std::cerr << "[ERROR] Gagal membuka dataset IMU di: " << pathIMU << "\n";
            std::cerr << "Tekan Enter untuk menutup...";
            std::cin.get();
            return 1;
        }
        std::cout << "[OK] Dataset IMU berhasil dibuka: " << pathIMU << "\n";

        GPSReader gpsReader(pathGPS);
        if (!gpsReader.initialize()) {
            std::cerr << "[ERROR] Gagal membuka dataset GPS di: " << pathGPS << "\n";
            std::cerr << "Tekan Enter untuk menutup...";
            std::cin.get();
            return 1;
        }
        std::cout << "[OK] Dataset GPS berhasil dibuka: " << pathGPS << "\n";

        // 2. Baca sampel GPS pertama untuk inisialisasi state posisi
        GPSDataPacket firstGPS = gpsReader.readNext();
        if (!firstGPS.isValid) {
            std::cerr << "[ERROR] Gagal membaca baris GPS pertama!\n";
            std::cerr << "Tekan Enter untuk menutup...";
            std::cin.get();
            return 1;
        }
        std::cout << "[OK] Titik Awal GPS: Lat=" << std::fixed << std::setprecision(7) 
                  << firstGPS.lat_rad * (180.0 / M_PI) 
                  << ", Lon=" << firstGPS.lon_rad * (180.0 / M_PI) 
                  << ", Alt=" << firstGPS.alt_m << " m\n";

        // 3. Inisialisasi INS Mechanization
        float init_roll  =  0.05016f;  
        float init_pitch = -0.00688f;  
        float init_yaw   =  1.17208f;  

        INSMechanization ins;
        ins.initialize(firstGPS.lat_rad, firstGPS.lon_rad, firstGPS.alt_m, init_roll, init_pitch, init_yaw);

        // 4. Inisialisasi Extended Kalman Filter (15-State)
ExtendedKF ekf;
        Eigen::Matrix<float, 15, 15> P0 = Eigen::Matrix<float, 15, 15>::Identity() * 0.01f;
        P0.block<3, 3>(0, 0)   = Eigen::Matrix3f::Identity() * 1.0f;
        P0.block<3, 3>(3, 3)   = Eigen::Matrix3f::Identity() * 0.1f;
        P0.block<3, 3>(6, 6)   = Eigen::Matrix3f::Identity() * (0.01745f * 0.01745f);
        P0.block<3, 3>(9, 9)   = Eigen::Matrix3f::Identity() * (0.05f * 0.05f);
        P0.block<3, 3>(12, 12) = Eigen::Matrix3f::Identity() * (0.005f * 0.005f);

        Eigen::Matrix<float, 15, 15> Q = Eigen::Matrix<float, 15, 15>::Zero();
        float Sa  = 0.05f * 0.05f;
        float Sg  = 0.005f * 0.005f;
        float Sba = 0.0001f * 0.0001f;
        float Sbg = 0.00001f * 0.00001f;
        
        Eigen::Matrix<float, 6, 6> R = Eigen::Matrix<float, 6, 6>::Zero();
        float sigma_pos = 1.0f;
        float sigma_vel = 0.1f;
        R(0, 0) = sigma_pos * sigma_pos;
        R(1, 1) = sigma_pos * sigma_pos;
        R(2, 2) = (sigma_pos * 2.0f) * (sigma_pos * 2.0f);
        R(3, 3) = sigma_vel * sigma_vel;
        R(4, 4) = sigma_vel * sigma_vel;
        R(5, 5) = (sigma_vel * 2.0f) * (sigma_vel * 2.0f);

        ekf.initialize(P0, Q, R);

        // 5. Siapkan File Output
        std::string outputPath = "ins_ekf_fused_trajectory.csv";
        std::ofstream outFile(outputPath);
        if (!outFile.is_open()) {
            std::cerr << "[ERROR] Gagal membuat file output CSV!\n";
            std::cerr << "Tekan Enter untuk menutup...";
            std::cin.get();
            return 1;
        }

        outFile << "timestamp,latitude_deg,longitude_deg,height_m,v_N,v_E,v_D,bias_ax,bias_ay,bias_az,bias_gx,bias_gy,bias_gz\n";
        outFile.flush();

        GPSDataPacket currentGPS = firstGPS;
        GPSDataPacket nextGPS = gpsReader.readNext();

        Eigen::Vector3f ba_est = Eigen::Vector3f::Zero();
        Eigen::Vector3f bg_est = Eigen::Vector3f::Zero();

        double prev_imu_timestamp = 0.0;
        int step = 0;

        std::cout << "[INFO] Memproses fusi 21.000+ data sensor...\n";

        while (true) {
            IMUDataPacket imuPkt = imuReader.readNextRow();
            if (!imuPkt.isDataValid) {
                std::cout << "\n[SELESAI] Total sampel berhasil diproses: " << step << " baris.\n";
                break;
            }

            if (prev_imu_timestamp == 0.0) {
                prev_imu_timestamp = imuPkt.timestamp;
                continue;
            }

            float dt = static_cast<float>((imuPkt.timestamp - prev_imu_timestamp) * 1e-6);
            prev_imu_timestamp = imuPkt.timestamp;
            if (dt <= 0.0f || dt > 0.05f) dt = 0.006f;

            // Kompensasi bias sensor estimasi
            Eigen::Vector3f raw_acc(imuPkt.accX, imuPkt.accY, imuPkt.accZ);
            Eigen::Vector3f raw_gyr(imuPkt.gyrX, imuPkt.gyrY, imuPkt.gyrZ);
            Eigen::Vector3f acc = (raw_acc - ba_est).cast<float>();
            Eigen::Vector3f gyr = (raw_gyr - bg_est).cast<float>();
            
            // --- Siklus Mekanisasi INS ---
            Eigen::Matrix3f cn_b_minus = ins.getOrientation();
            Eigen::Vector3f vn_minus   = ins.getVelocity();

            ins.updateAttitude(gyr, dt);
            Eigen::Matrix3f cn_b_plus = ins.getOrientation();

            Eigen::Vector3f fn_ib = ins.transformSpecificForce(acc, cn_b_minus, cn_b_plus);
            ins.updateVelocity(fn_ib, dt);
            Eigen::Vector3f vn_plus = ins.getVelocity();

            ins.updatePosition(vn_minus, vn_plus, dt);

            // --- Fase Prediksi EKF ---
            Q.setZero();
            Q.block<3, 3>(3, 3)   = Eigen::Matrix3f::Identity() * (Sa * dt);
            Q.block<3, 3>(6, 6)   = Eigen::Matrix3f::Identity() * (Sg * dt);
            Q.block<3, 3>(9, 9)   = Eigen::Matrix3f::Identity() * (Sba * dt);
            Q.block<3, 3>(12, 12) = Eigen::Matrix3f::Identity() * (Sbg * dt);

            ekf.setQ(Q); // Salurkan matriks Q ke EKF
            ekf.predict(cn_b_plus, fn_ib, dt);

            // --- Fase Koreksi EKF (Saat Paket GPS Masuk) ---
            while (nextGPS.isValid && nextGPS.timestamp <= imuPkt.timestamp) {
                currentGPS = nextGPS;

                double cur_L, cur_lambda, cur_h;
                ins.getPosition(cur_L, cur_lambda, cur_h);
                Eigen::Vector3f cur_vel = ins.getVelocity();

                double RN_Lb, RE_Lb;
                ins.computeEarth(cur_L, RN_Lb, RE_Lb);

                // Vektor Inovasi dz
                Eigen::Matrix<float, 6, 1> dz;
                dz(0) = static_cast<float>((cur_L - currentGPS.lat_rad) * (RN_Lb + cur_h));
                dz(1) = static_cast<float>((cur_lambda - currentGPS.lon_rad) * (RE_Lb + cur_h) * std::cos(cur_L));
                dz(2) = static_cast<float>(-(cur_h - currentGPS.alt_m));

                dz(3) = cur_vel(0) - currentGPS.vn;
                dz(4) = cur_vel(1) - currentGPS.ve;
                dz(5) = cur_vel(2) - currentGPS.vd;

                ekf.correct(dz);

                Eigen::Matrix<float, 15, 1> x_err = ekf.getErrorState();

                // Closed-loop feedback ke INS mekanisasi
                ins.applyErrorFeedback(x_err.segment<3>(0), 
                                       x_err.segment<3>(3), 
                                       x_err.segment<3>(6));

                ba_est += x_err.segment<3>(9);
                bg_est += x_err.segment<3>(12);

                ekf.resetState();
                nextGPS = gpsReader.readNext();
            }

            // Tulis baris ke file output
            double out_L, out_lambda, out_h;
            ins.getPosition(out_L, out_lambda, out_h);
            Eigen::Vector3f out_v = ins.getVelocity();

            outFile << std::fixed << std::setprecision(6) << imuPkt.timestamp * 1e-6 << ","
                    << std::setprecision(8) << out_L * (180.0 / M_PI) << ","
                    << std::setprecision(8) << out_lambda * (180.0 / M_PI) << ","
                    << std::setprecision(3) << out_h << ","
                    << std::setprecision(4) << out_v(0) << "," << out_v(1) << "," << out_v(2) << ","
                    << std::setprecision(6) << ba_est(0) << "," << ba_est(1) << "," << ba_est(2) << ","
                    << std::setprecision(6) << bg_est(0) << "," << bg_est(1) << "," << bg_est(2) << "\n";

            step++;
            if (step % 100 == 0) {
                outFile.flush(); // Flush rutin per 100 baris agar data tersimpan aman
            }
            if (step % 2000 == 0) {
                std::cout << "." << std::flush;
            }
        }

        outFile.close();
        std::cout << "\n[SUKSES] Data trajektori tersimpan (" << step << " baris).\n";
        std::cout << "Lokasi file: " << outputPath << "\n";
        std::cout << "Tekan Enter untuk selesai...";
        std::cin.get();

    } catch (const std::exception& e) {
        std::cerr << "\n[EXCEPTION ERROR] Terjadi kendala: " << e.what() << "\n";
        std::cerr << "Tekan Enter untuk menutup...";
        std::cin.get();
        return 1;
    }

    return 0;
}