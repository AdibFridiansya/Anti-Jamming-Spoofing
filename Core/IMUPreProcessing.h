#pragma once
#include "Sens.h"
#include <Eigen/Dense>

// Struct khusus untuk keluaran dari Pre-Processing menuju Mechanization
struct ProcessedIMUData {
    Eigen::Vector3f acc; // f^b_ib (m/s^2)
    Eigen::Vector3f gyr; // w^b_ib (rad/s)
    double timestamp;
    bool isValid;
};

class IMUPreProcessing {
private:
    Eigen::Vector3f bias_AccStatic;
    Eigen::Vector3f bias_GyrStatic;

    Eigen::Matrix3f matrix_AccScaleCross;
    Eigen::Matrix3f matrix_GyrScaleCross;
    Eigen::Matrix3f matrix_GyrGDependent;

    IMUDataPacket currentData; // Hanya satu wadah universal

public:
    IMUPreProcessing();
    ~IMUPreProcessing() = default; // Destructor default sudah cukup

    // Hanya satu fungsi input yang dibutuhkan
    void inputRawData(const IMUDataPacket& rawData);
    
    void setBiasesStatic(const Eigen::Vector3f& b_as, const Eigen::Vector3f& b_gs);
    void setScaleCrossMatrices(const Eigen::Matrix3f& M_a, const Eigen::Matrix3f& M_g, const Eigen::Matrix3f& G_g);

    ProcessedIMUData getProcessedData();
};