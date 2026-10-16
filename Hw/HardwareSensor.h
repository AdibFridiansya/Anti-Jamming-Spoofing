#pragma once
#include "SensorTypes.h"

class HardwareSensor {
private:
    bool isInitialized;
    
    // Buffer DMA (1 byte alamat dummy + 14 byte data)
    uint8_t spiTxBuf[15];
    uint8_t spiRxBuf[15];

    // Konstanta Skala (Contoh MPU6050 pada +-8g dan +-2000dps)
    static constexpr float ACCEL_SCALE = (9.80665f / 4096.0f);
    static constexpr float GYRO_SCALE  = (0.0174532925f / 16.4f);
    
    // Konstanta Skala Temperatur MPU6050
    static constexpr float TEMP_OFFSET = 36.53f;
    static constexpr float TEMP_SCALE  = (1.0f / 340.0f);

public:
    HardwareSensor();
    bool initialize();                                                      
    IMUDataPacket readData();
    void triggerDMARead(); // Fungsi untuk memulai siklus DMA
};