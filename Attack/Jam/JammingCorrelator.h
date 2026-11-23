#ifndef JAMMING_CORRELATOR_H
#define JAMMING_CORRELATOR_H

#include "JammingPropagation.h"
#include <vector>
#include <cmath>

// Jenis Sinyal GNSS Target - Kaplan (PDF 1) Table 9.3, 9.4, & 9.5, p. 560, 564-565 (PDF 1 p. 12, 16-17)
enum class GNSSSignalType {
    GPS_L1_CA,          // BPSK-R(1),  Rc = 1.023e6,  (C_Ri)_dB = -158.5 dBW
    GPS_L1_PY,          // BPSK-R(10), Rc = 10.23e6,  (C_Ri)_dB = -161.5 dBW
    GPS_L2_CL,          // BPSK-R(1),  Rc = 1.023e6,  (C_Ri)_dB = -163.0 dBW (IIF)
    GPS_L5_Q5,          // BPSK-R(10), Rc = 10.23e6,  (C_Ri)_dB = -157.9 dBW (IIF)
    GPS_L1_M_TDDM,      // BOCs(10,5), Rc = 5.115e6,  (C_Ri)_dB = -161.0 dBW
    GPS_L1C_PILOT,      // MBOC,       Rc = 1.023e6,  (C_Ri)_dB = -158.25 dBW
    GALILEO_E1_OS       // MBOC,       Rc = 1.023e6,  (C_Ri)_dB = -160.0 dBW
};

// Jenis Spektrum Sinyal Jammer - Kaplan (PDF 1) Table 9.1 & Table 9.3, p. 550 & 560 (PDF 1 p. 2 & 12)
enum class JammerWaveformType {
    NARROWBAND_CW,          // Case 1: Eq. (9.10) - Single tone / CW at spectral peak
    MATCHED_SPECTRUM,       // Case 2: Eq. (9.11)-(9.12) - Spectrum matched to GNSS signal
    WIDEBAND_BLWN_NULL,     // Case 3: Eq. (9.17) - Bandlimited white noise spanning null-to-null
    WIDEBAND_BLWN_CUSTOM,   // Case 3: Eq. (9.15) - Flat noise with custom bandwidth beta_i (Hz)
    WIDEBAND_PULSED         // Burst/Pulse jammer with duty cycle - Section 9.2.3.1, p. 586
};

// Polarisasi Antena Jammer - Kaplan (PDF 1) Section 9.2.2.2, p. 563 (PDF 1 p. 15)
enum class JammerPolarization {
    RHCP,                   // Right-Hand Circularly Polarized (sama dengan sinyal GNSS)
    LINEAR                  // Linearly Polarized (rugi mismatch ~3 dB terhadap antena RHCP)
};

// Parameter Tambahan Sinyal untuk Setiap Node Jammer
struct JammerSignalConfig {
    JammerWaveformType waveform;
    JammerPolarization polarization;
    float customBandwidth_Hz;   // beta_i: Hanya dipakai jika WIDEBAND_BLWN_CUSTOM (Eq. 9.15)
    float pulseDutyCycle;       // d_c in (0, 1]: Hanya dipakai jika WIDEBAND_PULSED (Section 9.2.3.1)
};

// Konfigurasi Antena & RF Front-End Penerima GNSS di Wahana
// Rujukan: Kaplan (PDF 1) Table 9.6, p. 565 (PDF 1 p. 17) & Section 9.2.3, p. 585-588
struct ReceiverFrontEndConfig {
    GNSSSignalType signalType;

    // Parameter Thermal Noise & Implementation Loss - Kaplan Eq. (9.20) & Table 9.6
    float gainTowardSV_dBic;        // (G_SVi)_dB: Default = 1.5 dBic
    float antennaNoiseTemp_K;       // T_ant: Default = 100.0 K
    float noiseFigure_dB;           // (N_f)_dB: Default = 2.0 dB at 290 K
    float implLoss_dB;              // L_dB: Receiver implementation + A/D loss, Default = 2.0 dB
    float frontEndBandwidth_Hz;     // B_fe: Receiver front-end bandwidth (Hz), e.g., 1.7e6 or 15.0e6 (Eq. 9.35)
    float lnaSaturationPower_dBW;   // Batas daya input RF saturasi LNA (dBW), misal -40.0 dBW (-10 dBm)

    // Opsi Mitigasi Penerima - Kaplan Section 9.2.3.1 & 9.2.3.2; Groves Section 10.3.2 & 10.3.3
    bool enableCRPA;                // true = gunakan CRPA, false = FRPA standar
    int crpaNumElements;            // N: Jumlah elemen CRPA (dapat men-null maks N - 1 jammer)
    float crpaNullDepth_dB;         // Kedalaman null-steering CRPA (negatif, -30 s/d -45 dB, p. 587)
    bool enablePulseBlanking;       // true = pulse blanking aktif (Section 9.2.3.1, p. 586)
    bool enableNarrowbandExcision;  // true = OFFT/ATF narrowband filter aktif (Section 9.2.3.1, p. 585-586)
};

// Pasangan Input per Jammer untuk Evaluasi Korelator (Menggabungkan hasil Tahap 1 + Konfigurasi Sinyal)
struct ActiveJammerInput {
    PropagationResult prop;         // Keluaran dari JammingPropagation (Tahap 1)
    JammerSignalConfig sig;         // Konfigurasi modulasi & polarisasi jammer
};

// Struktur Keluaran Tahap 2 (Masukan untuk Tracking Loop di Tahap 3 & Grafik UI Mission Planner)
struct CorrelatorResult {
    // Baseline tanpa jamming - Eq. (9.20) & Eq. (9.35)
    float T_receiver_K;             // Receiver system temperature (K)
    float N0_dBW_Hz;                // (N_0)_dB: Thermal noise spectral density (dBW/Hz)
    float Nt_dBW;                   // (N_t)_dB: Thermal noise floor in B_fe (dBW) - Eq. (9.35)
    float unjammed_Cs_dBW;          // (C_s)_dB: Recovered signal power (dBW)
    float unjammed_CN0_dBHz;        // (C_s/N_0)_dB: Unjammed C/N0 (dB-Hz)

    // Metrik Jamming Agregat - Eq. (9.18), (9.21), (9.24)
    int activeJammerCount;          // Jumlah jammer yang sedang menyala
    int crpaNulledCount;            // Jumlah jammer yang berhasil diredam CRPA
    bool isFrontEndSaturated;       // true jika total daya jammer menjenuhkan LNA (p. 554)
    bool isCWLeakthroughRisk;       // true jika GPS L1 C/A terkena CW di atas thermal floor (Section 9.2.2.5)

    float total_JS_dB;              // (J/S)_dB: Total rasio jammer-to-signal di input antena (dB)
    float total_Ci_Cs_dB;           // (C_i/C_s)_dB: Total rasio interferensi-to-signal pasca antena (dB)
    float effective_CN0_dBHz;       // (C_s/N_0)_eff,dB: Effective C/N0 output korelator prompt (dB-Hz)
    float effective_CN0_linear_Hz;  // (C_s/N_0)_eff dalam skala linear (Hz) -> dipakai langsung di Tahap 3
    float chipRate_Rc;              // R_c: Spreading code chipping rate (chips/s) -> dipakai di Tahap 3
    float minReceivedPower_CRi_dBW; // (C_Ri)_dB: Minimum received signal power (dBW)
};

class JammingCorrelator {
public:
    JammingCorrelator();

    // Membuat konfigurasi default penerima (Kaplan Table 9.6, p. 565)
    static ReceiverFrontEndConfig createDefaultReceiverConfig(GNSSSignalType sigType = GNSSSignalType::GPS_L1_CA);

    // Fungsi Utama Tahap 2: Menghitung (C_s/N_0)_eff untuk satu atau banyak titik jammer aktif
    CorrelatorResult evaluateCorrelator(const ReceiverFrontEndConfig& rxCfg,
                                        const std::vector<ActiveJammerInput>& jammers) const;

private:
    // Konstanta Fisika - Kaplan (PDF 1) Section 9.2.2.2, p. 562 (PDF 1 p. 14)
    static constexpr double K_BOLTZMANN = 1.38e-23;         // Boltzmann's constant (J/K)
    static constexpr double T_REF_K     = 290.0;            // Reference temperature (K)
    static constexpr float  RAD_TO_DEG  = 57.295779513f;

    // [1] Kaplan (PDF 1) Table 9.4 & Table 9.5, p. 564-565 (PDF 1 p. 16-17)
    void getSignalProperties(GNSSSignalType sigType, float& Rc_chips_s, float& CRi_dBW) const;

    // [2] Kaplan (PDF 1) Section 9.2.2.1 Eq. (9.10)-(9.17) & Table 9.3, p. 557-560 (PDF 1 p. 9-12)
    float computeQualityFactorQ(GNSSSignalType sigType, 
                                const JammerSignalConfig& jamSig, 
                                float Rc_chips_s) const;

    // [3] Kaplan (PDF 1) Section 9.2.2.2 Eq. (9.20) p. 561-562 & Section 9.2.2.5 Eq. (9.35) p. 578
    void computeUnjammedFrontEnd(const ReceiverFrontEndConfig& rxCfg,
                                 float CRi_dBW,
                                 CorrelatorResult& out) const;

    // [4] Kaplan (PDF 1) Section 9.2.2.2 p. 562-563 (PDF 1 p. 14-15) & Section 9.2.3 p. 585 (PDF 1 p. 37)
    float computeReceiverAntennaGain(float elevAngle_body_rad,
                                     JammerPolarization jamPol) const;
};

#endif // JAMMING_CORRELATOR_H