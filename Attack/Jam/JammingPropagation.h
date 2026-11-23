#ifndef JAMMING_PROPAGATION_H
#define JAMMING_PROPAGATION_H

#include <Eigen/Dense>
#include <cmath>

// Pilihan Model Propagasi sesuai Kaplan (PDF 1) Ch. 9, Section 9.2.2.4 & Section 9.4.2
enum class PropagationModelType {
    FREE_SPACE,                 // Kaplan Eq. (9.27), p. 571 (PDF 1 p. 23)
    HATA_URBAN_SMALL_MED_CITY,  // Kaplan Eq. (9.28)-(9.30), p. 574 (PDF 1 p. 26)
    HATA_URBAN_LARGE_CITY,      // Kaplan Eq. (9.28)-(9.29) & (9.31), p. 574 (PDF 1 p. 26)
    HATA_SUBURBAN,              // Kaplan Eq. (9.32), p. 575 (PDF 1 p. 27)
    HATA_OPEN_AREA,             // Kaplan Eq. (9.33), p. 575 (PDF 1 p. 27)
    ERCEG_HILLY_MOD_HEAVY_TREE, // Kaplan Eq. (9.55) & Table 9.12 Col 1, p. 594-595 (PDF 1 p. 46-47)
    ERCEG_HILLY_LIGHT_TREE,     // Kaplan Eq. (9.55) & Table 9.12 Col 2, p. 594-595 (PDF 1 p. 46-47)
    ERCEG_FLAT_LIGHT_TREE       // Kaplan Eq. (9.55) & Table 9.12 Col 3, p. 594-595 (PDF 1 p. 46-47)
};

// Struktur Konfigurasi Titik Jammer (Ditempatkan User pada Peta Mission Planner)
struct JammerNodeConfig {
    bool isEnabled;
    double startTime_s;         // Waktu mulai jammer menyala (detik)
    double endTime_s;           // Waktu selesai jammer menyala (detik)

    // Posisi Geodetik Jammer
    double L_j;                 // Latitude jammer (rad)
    double lambda_j;            // Longitude jammer (rad)
    double h_j;                 // Altitude ellipsoid jammer (m)

    // Parameter Pemancar RF - Kaplan (PDF 1) Section 9.2.2.4, p. 571 (PDF 1 p. 23)
    float freq_MHz;             // f [MHz]: Frekuensi transmisi jammer (misal L1 = 1575.42 MHz)
    float transmitPower_W;      // J_t: Jammer transmit power into its antenna (W)
    float antennaGain_dBic;     // (G_t)_dB: Jammer transmitter antenna gain (dBic)
    float baseHeightAGL_m;      // h_base [m]: Height of the base (transmitter) antenna (m)

    // Parameter Lingkungan & Penghalang - Kaplan (PDF 1) Section 9.4, p. 591-599 (PDF 1 p. 43-51)
    PropagationModelType propModel;
    
    // Section 9.4.1 - Vegetation, Eq. (9.54), p. 593 (PDF 1 p. 45)
    float vegetationDepth_m;    // d [m]: Jarak lintasan menembus vegetasi/pohon (m)
    bool isVegetationInLeaf;    // true = pohon berdaun (+30% atenuasi dB, p. 593 / PDF 1 p. 45)

    // Section 9.4.3 - Man-Made Structures, Eq. (9.57) & (9.58), p. 598 (PDF 1 p. 50)
    float roofLoss_dB;          // L_roof: Roof penetration loss (1 to 30 dB, Eq. 9.57)
    int numFloorsPenetrated;    // n_floor: Number of floors penetrated (Eq. 9.57)
    float floorLoss_dB;         // L_floor: Loss per floor (1 to 10 dB, Eq. 9.57)
    float exteriorWallLoss_dB;  // L_ext: Exterior wall penetration loss (1 to 30 dB, Eq. 9.58 & Table 9.13)
    int numInteriorWalls;       // n_int: Number of interior walls penetrated (Eq. 9.58)
    float interiorWallLoss_dB;  // L_int: Loss per interior wall (1 to 10 dB, Eq. 9.58)
};

// Struktur Keluaran Tahap 1 (Geometri 3D & Daya Jammer Tiba di Input Antena Wahana)
struct PropagationResult {
    bool isJammerActive;        // Status aktif jammer pada timestamp saat ini
    bool isBeyondHorizon;       // Status terhalang kelengkungan bumi (Kaplan Section 9.2.3, p. 585)

    Eigen::Vector3f r_bj_n;     // Vektor posisi relatif wahana-ke-jammer di Local NED [d_N, d_E, d_D]^T (m)
    float slantRange_m;         // d: Range to jammer (m) - Kaplan Eq. (9.27)

    float elevAngle_ned_rad;    // Sudut elevasi jammer terhadap horison lokal NED (rad)
    float azimAngle_ned_rad;    // Sudut azimuth jammer terhadap Utara lokal NED (rad)

    float elevAngle_body_rad;   // Sudut elevasi jammer relatif bidang antena wahana (rad) - efek Antenna Tilt
    float azimAngle_body_rad;   // Sudut azimuth jammer relatif sumbu X body wahana (rad)

    float eirp_dBW;             // (EIRP)_dB: Effective isotropic radiated power of jammer (dBW)
    float pathLoss_dB;          // (L_p)_dB: Jammer power propagation loss (dB)
    float blockageLoss_dB;      // Excess loss akibat vegetasi (Eq. 9.54) + gedung (Eq. 9.57-9.58) (dB)
    float incidentPower_Jr_dBW; // (J_r)_dB: Incident (received) jammer power at antenna input (dBW)
};

class JammingPropagation {
public:
    JammingPropagation();

    // Fungsi Utama Tahap 1: Menghitung Geometri 3D & (J_r)_dB
    PropagationResult computePropagation(double currentTime_s,
                                         double L_b, double lambda_b, double h_b,
                                         float mobileHeightAGL_m,
                                         const Eigen::Matrix3f& Cn_b,
                                         const JammerNodeConfig& jammer) const;

private:
    // Konstanta WGS-84 & Fisika (Konsisten dengan INSMechanization.h)
    static constexpr double R_0      = 6378137.0;           // WGS-84 Equatorial radius (m)
    static constexpr double E_SQR    = 0.00669437999014;    // WGS-84 Eccentricity squared (e^2)
    static constexpr double C_LIGHT  = 299792458.0;         // Speed of light in vacuum (m/s)
    static constexpr double PI_VAL   = 3.14159265358979323846;

    // [1] Groves Ch. 2, Section 2.4.2, Eq. (2.105) & (2.106) - Earth Radii of Curvature
    //     (Referensi diambil langsung dari INSMechanization.cpp baris 44-51)
    void computeEarth(double L_b, double& RN_Lb, double& RE_Lb) const;

    // [2] Kaplan (PDF 1) Ch. 9, Section 9.2.2.4, Eq. (9.27), p. 571 (PDF p. 23)
    float computeFreeSpaceLoss(float d_m, float freq_MHz) const;

    // [3] Kaplan (PDF 1) Ch. 9, Section 9.2.2.4, Eq. (9.28)-(9.33), p. 574-575 (PDF p. 26-27)
    float computeHataLoss(float d_km, float freq_MHz, 
                          float h_base_m, float h_mobile_m, 
                          PropagationModelType modelType) const;

    // [4] Kaplan (PDF 1) Ch. 9, Section 9.4.2, Eq. (9.55) & Table 9.12, p. 594-595 (PDF p. 46-47)
    float computeErcegLoss(float d_m, float freq_MHz, 
                           float h_base_m, 
                           PropagationModelType modelType) const;

    // [5] Kaplan (PDF 1) Ch. 9, Section 9.4.1 Eq. (9.54) p. 593 (PDF p. 45) &
    //     Section 9.4.3 Eq. (9.57)-(9.58) p. 598 (PDF p. 50)
    float computeBlockageLoss(const JammerNodeConfig& jammer) const;
};

#endif // JAMMING_PROPAGATION_H