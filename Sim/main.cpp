#include <iostream>
#include <iomanip>
#include "SensorSim.h"
#include "IMUPreProcessing.h"

int main() {
    // Sesuaikan path CSV dengan lokasi Anda menjalankan program
    SensorSim simData("E:/Tugas Akhir/New Firmware/Sim/IMUsimulation.csv");
    IMUPreProcessing imuPreProc;

    if (!simData.initialize()) {
        return 1; // Keluar jika gagal membaca file
    }

    std::cout << "\n--- Memulai Simulasi Pre-Processing IMU ---\n";
    std::cout << std::fixed << std::setprecision(4); // Format desimal

    int dataCount = 0;

    while (true) {
        // 1. Ekstrak baris CSV
        IMUDataPacket rawPacket = simData.readNextRow();
        
        // Cek akhir file (EOF)
        if (!rawPacket.isDataValid) {
            std::cout << "\n[Selesai] Akhir file CSV tercapai. Total data: " << dataCount << "\n";
            break;
        }

        // 2. Masukkan ke Pre-Processing (Kalibrasi/Kompensasi Bias)
        imuPreProc.inputRawData(rawPacket);
        ProcessedIMUData processedPacket = imuPreProc.getProcessedData();

        // 3. Cetak dan Verifikasi Data
        if (processedPacket.isValid) {
            std::cout << "Time: " << processedPacket.timestamp << " s | "
                      << "Acc: [" << processedPacket.acc.x() << ", " 
                                  << processedPacket.acc.y() << ", " 
                                  << processedPacket.acc.z() << "] | "
                      << "Gyr: [" << processedPacket.gyr.x() << ", " 
                                  << processedPacket.gyr.y() << ", " 
                                  << processedPacket.gyr.z() << "]\n";
            dataCount++;
        }
    }

    return 0;
}