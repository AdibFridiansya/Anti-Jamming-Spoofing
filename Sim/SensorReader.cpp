#include "SensorReader.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>

SensorReader::SensorReader(const std::string& filepath) : filename(filepath) {}

SensorReader::~SensorReader() {
    if (file.is_open()) {
        file.close();
    }
}

bool SensorReader::initialize() {
    file.open(filename);
    if (!file.is_open()) {
        std::cerr << "[Error] Gagal membuka file: " << filename << std::endl;
        return false;
    }

    std::string headerLine;
    if (std::getline(file, headerLine)) {
        // Hapus karakter \r dan \n dari baris header
        headerLine.erase(std::remove(headerLine.begin(), headerLine.end(), '\r'), headerLine.end());
        headerLine.erase(std::remove(headerLine.begin(), headerLine.end(), '\n'), headerLine.end());

        std::stringstream ss(headerLine);
        std::string col;
        int index = 0;
        
        while (std::getline(ss, col, ',')) {
            // Bersihkan tanda kutip jika ditambahkan oleh Excel
            col.erase(std::remove(col.begin(), col.end(), '\"'), col.end());

            if (col == "timestamp") idx_time = index;
            else if (col == "accelerometer_m_s2[0]") idx_acc[0] = index;
            else if (col == "accelerometer_m_s2[1]") idx_acc[1] = index;
            else if (col == "accelerometer_m_s2[2]") idx_acc[2] = index;
            else if (col == "gyro_rad[0]") idx_gyr[0] = index;
            else if (col == "gyro_rad[1]") idx_gyr[1] = index;
            else if (col == "gyro_rad[2]") idx_gyr[2] = index;
            index++;
        }
    }

    // Validasi ketat seluruh sumbu (X, Y, Z)
    if (idx_time == -1 || 
        idx_acc[0] == -1 || idx_acc[1] == -1 || idx_acc[2] == -1 || 
        idx_gyr[0] == -1 || idx_gyr[1] == -1 || idx_gyr[2] == -1) {
        std::cerr << "\n[Error] Format kolom CSV tidak lengkap!\n";
        std::cerr << "Indeks terdeteksi: time=" << idx_time 
                  << ", acc=[" << idx_acc[0] << "," << idx_acc[1] << "," << idx_acc[2] << "]"
                  << ", gyr=[" << idx_gyr[0] << "," << idx_gyr[1] << "," << idx_gyr[2] << "]\n";
        return false;
    }

    return true;
}

IMUDataPacket SensorReader::readNextRow() {
    IMUDataPacket packet = {};
    packet.isDataValid = false;

    std::string line;
    if (std::getline(file, line)) {
        // Hapus karakter carriage return
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        
        if (line.empty()) return packet;

        std::stringstream ss(line);
        std::string val;
        std::vector<std::string> row;
        
        while (std::getline(ss, val, ',')) {
            row.push_back(val);
        }

        int max_idx = std::max({idx_time, idx_acc[0], idx_acc[1], idx_acc[2], idx_gyr[0], idx_gyr[1], idx_gyr[2]});
        if (static_cast<int>(row.size()) <= max_idx) {
            return packet;
        }

        try {
            packet.timestamp = std::stod(row[idx_time]);
            packet.accX = std::stof(row[idx_acc[0]]);
            packet.accY = std::stof(row[idx_acc[1]]);
            packet.accZ = std::stof(row[idx_acc[2]]);
            packet.gyrX = std::stof(row[idx_gyr[0]]);
            packet.gyrY = std::stof(row[idx_gyr[1]]);
            packet.gyrZ = std::stof(row[idx_gyr[2]]);
            packet.temperature = 0.0f;
            packet.isDataValid = true;
        } catch (...) {
            packet.isDataValid = false;
        }
    }

    return packet;
}