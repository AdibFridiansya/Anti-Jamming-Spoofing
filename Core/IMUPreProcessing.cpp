/* 
 The matrix addition (I_3 + ^M) and the matrix inversion operation must be computed offline (Eq. 4.21)
 in a separate calibration software (e.g., MATLAB, Python) to avoid heavy real-time CPU load. 
 This C++ firmware only stores and applies the final pre-calculated matrices.
*/
#include "IMUPreProcessing.h"

IMUPreProcessing::IMUPreProcessing() {
    bias_AccStatic = Eigen::Vector3f(0.012f, -0.005f, 0.001f);
    bias_GyrStatic = Eigen::Vector3f(0.002f, 0.001f, -0.003f);
    /* Calibration (Vector Changed) Template Exmp.
    bias_AccStatic = Eigen::Vector3f(0.012f, -0.005f, 0.001f);
    bias_GyrStatic = Eigen::Vector3f(0.002f, 0.001f, -0.003f);
    */

    matrix_AccScaleCross = Eigen::Matrix3f::Identity();
    matrix_GyrScaleCross = Eigen::Matrix3f::Identity();
    matrix_GyrGDependent = Eigen::Matrix3f::Zero();                                     
    /* Calibration (Identity Matrix Changed) Template Exmp.
    matrix_AccScaleCross << 1.002f,  0.001f,  0.000f,
                            0.001f,  0.998f,  0.002f,
                            0.000f,  0.002f,  1.001f;
    matrix_GyrScaleCross << 1.001f,  0.000f,  0.000f,
                            0.000f,  1.000f,  0.000f,
                            0.000f,  0.000f,  0.999f;
    matrix_GyrGDependent << 0.0001f, 0.0000f, 0.0000f,
                            0.0000f, 0.0001f, 0.0000f,
                            0.0000f, 0.0000f, 0.0001f;
    */
}

void IMUPreProcessing::inputRawData(const IMUDataPacket& rawData) {
    currentData = rawData;
}

void IMUPreProcessing::setBiasesStatic(const Eigen::Vector3f& b_as, const Eigen::Vector3f& b_gs) {
    bias_AccStatic = b_as;
    bias_GyrStatic = b_gs;
}

void IMUPreProcessing::setScaleCrossMatrices(const Eigen::Matrix3f& M_a, const Eigen::Matrix3f& M_g, const Eigen::Matrix3f& G_g) {
    matrix_AccScaleCross = M_a;                                                       // or ^(M_a)
    matrix_GyrScaleCross = M_g;                                                       // or ^(M_g)
    matrix_GyrGDependent = G_g;                                                       // or ^(G_g)
}

ProcessedIMUData IMUPreProcessing::getProcessedData() {
    ProcessedIMUData outData;
    outData.isValid = false;

    if (!currentData.isDataValid) {
        return outData;
    }

    Eigen::Vector3f f_raw(currentData.accX, currentData.accY, currentData.accZ);      // ~(f^b_ib)  
    Eigen::Vector3f w_raw(currentData.gyrX, currentData.gyrY, currentData.gyrZ);      // ~(w^b_ib)

    // Eq.(4.13), b_ad (bias_AccDynamic) & b_gd(bias_GyrDynamic) will be Continued in EKF Equations
    Eigen::Vector3f f_unbiased = f_raw - bias_AccStatic;                                                                                                                                                                                                                                        
    Eigen::Vector3f w_unbiased = w_raw - bias_GyrStatic;                                                              

    // from Eq.(4.13) and Eq.(4.15), hence Eq.(4.19)
    Eigen::Vector3f f_carat = matrix_AccScaleCross * f_unbiased;                      // ^(f^b_ib)

    // Eq.(4.20)
    Eigen::Vector3f w_g_corrected = w_unbiased - (matrix_GyrGDependent * f_carat);                                      
    Eigen::Vector3f w_carat = matrix_GyrScaleCross * w_g_corrected;                   // ^(w^b_ib)   

    /* 
     'f_carat' and 'w_carat' hold the estimated values (^f) and (^w). 
     However, to maintain architectural consistency and simplicity in the software, 
     (^f) and (^w) are changed to be (f) and (w). 
    */
     
    outData.acc = f_carat;                      // f 
    outData.gyr = w_carat;                      // w
    outData.timestamp = currentData.timestamp;  // timestamp
    
    outData.isValid = true;
    return outData;
}