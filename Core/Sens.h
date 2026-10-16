#pragma once

#include <stdint.h>

// Inertial Measurement Sensor (IMU)
struct IMUDataPacket {
    double timestamp;       
    // Accel (m/s^2) - Specific Force
    float accX;
    float accY;
    float accZ;
    // Gyro (rad/s) - Angular Rate
    float gyrX;
    float gyrY;
    float gyrZ;
    
    float temperature;     
    bool isDataValid;       
};

// Global Navigation Satelite System (GNSS) Module
struct GNSSDataPacket {
    double timestamp;       // Hasil konversi dari iTOW (GPS Millisecond Time of Week)
    
    double latitude;        // Derajat desimal (dari lon/lat bita skala 1e-7)
    double longitude;
    float altitude;         // Ketinggian MSL (hMSL)
    
    float velN;             // Kecepatan NED (velN, velE, velD)
    float velE;
    float velD;
    
    float hAcc;             // Horizontal Accuracy Estimate (m)
    float vAcc;             // Vertical Accuracy Estimate (m)
    float sAcc;             // Speed Accuracy Estimate (m/s)
    float pDOP;             // Position DOP (Skala 0.01)
    
    // Parameter Status dan Validasi u-blox
    uint8_t fixType;        // 0: No Fix, 2: 2D, 3: 3D, 4: GNSS+Dead Reckoning
    uint8_t flags;          // Bit 0: gnssFixOK, Bit 1: diffSoln (SBAS/DGPS applied)
    uint8_t numSV;          // Jumlah satelit yang digunakan dalam solusi
    
    bool isDataValid;
};