#include "HardwareSensor.h"
// #include "stm32SERIESxx_hal.h"

// extern SPI_HandleTypeDef hspi1;
// volatile bool dmaTransferComplete = false;

HardwareSensor::HardwareSensor() : isInitialized(false) {
    //buffer . Byte pertama adalah alamat register Accel (0x3B) + bit Read (0x80)
    spiTxBuf[0] = 0x3B | 0x80;
    for (int i = 1; i < 15; i++) spiTxBuf[i] = 0x00; // Dummy clock bytes
}

bool HardwareSensor::initialize() {
    // uint8_t config_pwr[2] = {0x6B, 0x00}; // Wake up command
    
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
    // HAL_SPI_Transmit(&hspi1, config_pwr, 2, 100);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS HIGH

    isInitialized = true;
    
    triggerDMARead();
    return true;
}

void HardwareSensor::triggerDMARead() {
    if (!isInitialized) return;
    
    // dmaTransferComplete = false;
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
    
    // Meminta DMA untuk bertukar 15 bita secara non-blocking
    // HAL_SPI_TransmitReceive_DMA(&hspi1, spiTxBuf, spiRxBuf, 15);
}

IMUDataPacket HardwareSensor::readData() {
    IMUDataPacket packet = {}; 
    packet.isDataValid = false;

    // Hanya proses jika interupsi DMA sebelumnya sudah menyatakan transfer selesai
    // if (!isInitialized || !dmaTransferComplete) return packet;

    // Parsing 14 bita data murni (abaikan spiRxBuf[0] karena itu balasan saat mengirim alamat)
    int16_t accX_raw = (spiRxBuf[1] << 8) | spiRxBuf[2];
    int16_t accY_raw = (spiRxBuf[3] << 8) | spiRxBuf[4];
    int16_t accZ_raw = (spiRxBuf[5] << 8) | spiRxBuf[6];
    
    int16_t temp_raw = (spiRxBuf[7] << 8) | spiRxBuf[8];
    
    int16_t gyrX_raw = (spiRxBuf[9] << 8) | spiRxBuf[10];
    int16_t gyrY_raw = (spiRxBuf[11] << 8) | spiRxBuf[12];
    int16_t gyrZ_raw = (spiRxBuf[13] << 8) | spiRxBuf[14];

    // Konversi ke Satuan Standar Internasional (SI)
    packet.accX = static_cast<float>(accX_raw) * ACCEL_SCALE;
    packet.accY = static_cast<float>(accY_raw) * ACCEL_SCALE;
    packet.accZ = static_cast<float>(accZ_raw) * ACCEL_SCALE;
    
    packet.temperature = (static_cast<float>(temp_raw) * TEMP_SCALE) + TEMP_OFFSET;
    
    packet.gyrX = static_cast<float>(gyrX_raw) * GYRO_SCALE;
    packet.gyrY = static_cast<float>(gyrY_raw) * GYRO_SCALE;
    packet.gyrZ = static_cast<float>(gyrZ_raw) * GYRO_SCALE;

    // Timestamp perangkat keras via timer (misal TIM2 atau SysTick)
    // packet.timestamp = getHardwareTimeInSeconds();
    packet.timestamp = 0.01; 

    packet.isDataValid = true;

    // Langsung picu siklus DMA berikutnya di latar belakang sebelum fungsi ini berakhir
    triggerDMARead();

    return packet;
}