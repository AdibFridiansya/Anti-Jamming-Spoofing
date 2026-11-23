cmake_minimum_required(VERSION 3.15)
project(INSSimulation CXX)

# Gunakan C++14 yang didukung penuh dan stabil oleh MinGW GCC 6.3.0
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(
    -DEIGEN_HAS_STD_INVOKE_RESULT=0
    -DEIGEN_DONT_VECTORIZE
    -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra -mstackrealign")

# --- TARGET 1: Program Utama Simulasi INS/EKF ---
set(SOURCES
    Sim/main.cpp
    Sim/SensorReader.cpp
    Core/INSMechanization.cpp
    Core/ExtendedKF.cpp
    Core/IMUPreProcessing.cpp
)

add_executable(ins_sim ${SOURCES})

target_include_directories(ins_sim PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core"
    "${CMAKE_CURRENT_SOURCE_DIR}/Sim"
    "${CMAKE_CURRENT_SOURCE_DIR}/Hw"
    "${CMAKE_CURRENT_SOURCE_DIR}/Attack/Jam"
)

# --- TARGET 2: Program Uji Mandiri Jamming Tahap 1 & 2 ---
add_executable(run
    Attack/Jam/run.cpp
    Attack/Jam/JammingPropagation.cpp
    Attack/Jam/JammingCorrelator.cpp
    Attack/Jam/JammingTrackingLoop.cpp
    Attack/Jam/JammingEngine.cpp
)

target_include_directories(run PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/Attack/Jam"
)