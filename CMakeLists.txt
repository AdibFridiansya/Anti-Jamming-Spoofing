cmake_minimum_required(VERSION 3.10)
project(INS_Navigation_Sim)

# Gunakan standar C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Cari pustaka Eigen3 (Pastikan sudah terinstal di OS Anda)
find_package(Eigen3 REQUIRED)

# Masukkan folder agar compiler tahu di mana mencari file .h
include_directories(core sim)

# Kumpulkan file .cpp yang akan dikompilasi
set(SOURCES
    sim/main.cpp
    sim/SensorSim.cpp
    core/IMUPreProcessing.cpp
)

# Buat executable bernama "ins_sim"
add_executable(ins_sim ${SOURCES})
target_include_directories(ins_sim PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/Core"
    "${CMAKE_CURRENT_SOURCE_DIR}/Sim"
    "${CMAKE_CURRENT_SOURCE_DIR}/Hw"
)

# Tautkan target dengan pustaka Eigen
target_link_libraries(ins_sim Eigen3::Eigen)