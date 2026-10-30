#pragma once
#include <string>
#include <fstream>
#include "Sens.h"

class SensorReader {
private:
    std::ifstream file;
    std::string filename;
    
    // Menyimpan indeks kolom (karena dataset memiliki banyak kolom)
    int idx_time = -1;
    int idx_acc[3] = {-1, -1, -1};
    int idx_gyr[3] = {-1, -1, -1};

public:
    explicit SensorReader(const std::string& filepath);
    ~SensorReader();

    bool initialize();
    IMUDataPacket readNextRow();
};