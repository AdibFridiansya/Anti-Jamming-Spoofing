#pragma once

#include "Sens.h"
#include <string>
#include <fstream>

class SensorSim {
private:
    std::ifstream file;
    std::string filename;

public:
    explicit SensorSim(const std::string& filepath);
    ~SensorSim();

    bool initialize();
    IMUDataPacket readNextRow();
};