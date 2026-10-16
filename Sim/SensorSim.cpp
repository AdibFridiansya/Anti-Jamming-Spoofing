#include "SensorSim.h"
#include <iostream>
#include <cstdio>

SensorSim::SensorSim(const std::string& filepath) : filename(filepath) {}

SensorSim::~SensorSim() {
    if (file.is_open()) {
        file.close();
    }
}

bool SensorSim::initialize() {
    file.open(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open the " << filename << std::endl;
        return false;
    }
    return true;
}

IMUDataPacket SensorSim::readNextRow() {
    IMUDataPacket packet = {};
    packet.isDataValid = false; 

    if (!file.is_open()) return packet;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#' || line.find("time") != std::string::npos) {
            continue; 
        }
        int parsed = std::sscanf(line.c_str(), "%lf,%f,%f,%f,%f,%f,%f",
                                 &packet.timestamp,
                                 &packet.accX, &packet.accY, &packet.accZ,
                                 &packet.gyrX, &packet.gyrY, &packet.gyrZ);
        if (parsed == 7) {
            packet.isDataValid = true;
            return packet;
        }
    }
    return packet; 
}