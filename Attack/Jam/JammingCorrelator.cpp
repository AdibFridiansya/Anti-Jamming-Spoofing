#include "JammingCorrelator.h"
#include <algorithm>

JammingCorrelator::JammingCorrelator() {}

// Table 9.6 & Section 9.2.2.5 / 9.2.3 - Default Receiver Front-End Parameters (Kaplan PDF 1, p. 565)
ReceiverFrontEndConfig JammingCorrelator::createDefaultReceiverConfig(GNSSSignalType sigType) {
    ReceiverFrontEndConfig cfg = {};
    cfg.signalType               = sigType;
    cfg.gainTowardSV_dBic        = 1.5f;
    cfg.antennaNoiseTemp_K       = 100.0f;
    cfg.noiseFigure_dB           = 2.0f;
    cfg.implLoss_dB              = 2.0f;
    cfg.frontEndBandwidth_Hz     = 1.7e6f;
    cfg.lnaSaturationPower_dBW   = -40.0f;

    cfg.enableCRPA               = false;
    cfg.crpaNumElements          = 7;
    cfg.crpaNullDepth_dB         = -35.0f;
    cfg.enablePulseBlanking      = false;
    cfg.enableNarrowbandExcision = false;
    return cfg;
}

// Table 9.4 & Table 9.5 - GNSS Chipping Rate and Minimum Received Signal Power (Kaplan PDF 1, p. 564-565)
void JammingCorrelator::getSignalProperties(GNSSSignalType sigType,
                                            float& Rc_chips_s,
                                            float& CRi_dBW) const {
    switch (sigType) {
        case GNSSSignalType::GPS_L1_CA:
            Rc_chips_s = 1.023e6f;  CRi_dBW = -158.5f;  break;
        case GNSSSignalType::GPS_L1_PY:
            Rc_chips_s = 10.23e6f;  CRi_dBW = -161.5f;  break;
        case GNSSSignalType::GPS_L2_CL:
            Rc_chips_s = 1.023e6f;  CRi_dBW = -163.0f;  break;
        case GNSSSignalType::GPS_L5_Q5:
            Rc_chips_s = 10.23e6f;  CRi_dBW = -157.9f;  break;
        case GNSSSignalType::GPS_L1_M_TDDM:
            Rc_chips_s = 5.115e6f;  CRi_dBW = -161.0f;  break;
        case GNSSSignalType::GPS_L1C_PILOT:
            Rc_chips_s = 1.023e6f;  CRi_dBW = -158.25f; break;
        case GNSSSignalType::GALILEO_E1_OS:
            Rc_chips_s = 1.023e6f;  CRi_dBW = -160.0f;  break;
        default:
            Rc_chips_s = 1.023e6f;  CRi_dBW = -158.5f;  break;
    }
}

// Eq. (9.10), (9.12), (9.15), (9.17) & Table 9.3 - Jamming Resistance Quality Factor Q (Kaplan PDF 1, p. 557-560)
float JammingCorrelator::computeQualityFactorQ(GNSSSignalType sigType,
                                               const JammerSignalConfig& jamSig,
                                               float Rc_chips_s) const {
    if (jamSig.waveform == JammerWaveformType::WIDEBAND_BLWN_CUSTOM) {
        const float safe_beta = std::max(jamSig.customBandwidth_Hz, Rc_chips_s);
        return safe_beta / Rc_chips_s;
    }

    float Q_narrow  = 1.0f;
    float Q_matched = 1.5f;
    float Q_blwn    = 2.2f;

    if (sigType == GNSSSignalType::GPS_L1_M_TDDM) {
        Q_narrow = 2.3f;  Q_matched = 4.0f;  Q_blwn = 7.2f;
    } else if (sigType == GNSSSignalType::GPS_L1C_PILOT || sigType == GNSSSignalType::GALILEO_E1_OS) {
        Q_narrow = 2.1f;  Q_matched = 3.6f;  Q_blwn = 5.1f;
    }

    switch (jamSig.waveform) {
        case JammerWaveformType::NARROWBAND_CW:
            return Q_narrow;
        case JammerWaveformType::MATCHED_SPECTRUM:
            return Q_matched;
        case JammerWaveformType::WIDEBAND_BLWN_NULL:
        case JammerWaveformType::WIDEBAND_PULSED:
        default:
            return Q_blwn;
    }
}

// Eq. (9.20) & Eq. (9.35) - Unjammed Front-End C/N0 and Thermal Noise Power (Kaplan PDF 1, p. 561-562 & 578)
void JammingCorrelator::computeUnjammedFrontEnd(const ReceiverFrontEndConfig& rxCfg,
                                                float CRi_dBW,
                                                CorrelatorResult& out) const {
    const double Nf_linear = std::pow(10.0, static_cast<double>(rxCfg.noiseFigure_dB) / 10.0);
    const double T_rx      = T_REF_K * (Nf_linear - 1.0);
    const double N0_dB     = 10.0 * std::log10(K_BOLTZMANN * (static_cast<double>(rxCfg.antennaNoiseTemp_K) + T_rx));

    const double Cs_dB     = static_cast<double>(CRi_dBW) +
                             static_cast<double>(rxCfg.gainTowardSV_dBic) -
                             static_cast<double>(rxCfg.implLoss_dB);
    const double CN0_dBHz  = Cs_dB - N0_dB;

    const double safe_Bfe  = std::max(static_cast<double>(rxCfg.frontEndBandwidth_Hz), 1.0e3);
    const double Nt_dB     = N0_dB + 10.0 * std::log10(safe_Bfe);

    out.T_receiver_K      = static_cast<float>(T_rx);
    out.N0_dBW_Hz         = static_cast<float>(N0_dB);
    out.Nt_dBW            = static_cast<float>(Nt_dB);
    out.unjammed_Cs_dBW   = static_cast<float>(Cs_dB);
    out.unjammed_CN0_dBHz = static_cast<float>(CN0_dBHz);
}

// Section 9.2.2.2 & Section 9.2.3 - FRPA Gain Pattern, Body Masking, and Polarization Loss (Kaplan PDF 1, p. 562-563 & 585)
float JammingCorrelator::computeReceiverAntennaGain(float elevAngle_body_rad,
                                                    JammerPolarization jamPol) const {
    const float elev_deg = elevAngle_body_rad * RAD_TO_DEG;

    float gain_dBic = -3.0f;
    if (elev_deg >= 5.0f) {
        const float frac = std::max(0.0f, std::min((elev_deg - 5.0f) / 85.0f, 1.0f));
        gain_dBic = -3.0f + 4.5f * std::sin(frac * 1.5707963f);
    } else if (elev_deg >= 0.0f) {
        gain_dBic = -4.0f + (elev_deg / 5.0f) * 1.0f;
    } else {
        const float below_frac = std::max(0.0f, std::min((-elev_deg) / 30.0f, 1.0f));
        gain_dBic = -4.0f - 11.0f * below_frac;
    }

    if (jamPol == JammerPolarization::LINEAR) {
        const float pol_factor = std::max(0.0f, std::min(std::abs(elev_deg) / 15.0f, 1.0f));
        gain_dBic -= 3.0f * pol_factor;
    }

    return gain_dBic;
}

// Eq. (9.18), (9.21), (9.24), & Section 9.2.3 - Effective C/N0 Correlator Evaluation (Kaplan PDF 1)
CorrelatorResult JammingCorrelator::evaluateCorrelator(const ReceiverFrontEndConfig& rxCfg,
                                                       const std::vector<ActiveJammerInput>& jammers) const {
    CorrelatorResult res = {};

    float Rc_chips_s = 1.023e6f;
    float CRi_dBW    = -158.5f;
    getSignalProperties(rxCfg.signalType, Rc_chips_s, CRi_dBW);

    res.chipRate_Rc              = Rc_chips_s;
    res.minReceivedPower_CRi_dBW = CRi_dBW;
    computeUnjammedFrontEnd(rxCfg, CRi_dBW, res);

    struct JammerEvalItem {
        int index;
        float Jr_dBW;
        float GJ_dB;
        float Ci_pre_mitigation_dBW;
    };
    std::vector<JammerEvalItem> activeList;

    for (size_t i = 0; i < jammers.size(); ++i) {
        if (!jammers[i].prop.isJammerActive) {
            continue;
        }

        const float GJ_dB = computeReceiverAntennaGain(jammers[i].prop.elevAngle_body_rad,
                                                       jammers[i].sig.polarization);
        const float Ci_pre_dBW = jammers[i].prop.incidentPower_Jr_dBW + GJ_dB;

        JammerEvalItem item;
        item.index                 = static_cast<int>(i);
        item.Jr_dBW                = jammers[i].prop.incidentPower_Jr_dBW;
        item.GJ_dB                 = GJ_dB;
        item.Ci_pre_mitigation_dBW = Ci_pre_dBW;
        activeList.push_back(item);
    }

    res.activeJammerCount = static_cast<int>(activeList.size());

    if (activeList.empty()) {
        res.total_JS_dB             = -999.0f;
        res.total_Ci_Cs_dB          = -999.0f;
        res.effective_CN0_dBHz      = res.unjammed_CN0_dBHz;
        res.effective_CN0_linear_Hz = static_cast<float>(std::pow(10.0, static_cast<double>(res.unjammed_CN0_dBHz) / 10.0));
        return res;
    }

    // Section 9.2.3.2 (Kaplan p. 587-588) & Section 10.3.2 (Groves p. 453) - CRPA Null-Steering Priority
    std::sort(activeList.begin(), activeList.end(),
              [](const JammerEvalItem& a, const JammerEvalItem& b) {
                  return a.Ci_pre_mitigation_dBW > b.Ci_pre_mitigation_dBW;
              });

    const int maxCrpaNulls = rxCfg.enableCRPA ? std::max(0, rxCfg.crpaNumElements - 1) : 0;

    double sum_Jr_W              = 0.0;
    double sum_Ci_lna_W          = 0.0;
    double sum_Ci_post_W         = 0.0;
    double sum_interference_term = 0.0;
    double totalBlankedFraction  = 0.0;

    for (size_t k = 0; k < activeList.size(); ++k) {
        const JammerEvalItem& item   = activeList[k];
        const ActiveJammerInput& jam = jammers[static_cast<size_t>(item.index)];

        float eff_GJ_dB = item.GJ_dB;
        if (static_cast<int>(k) < maxCrpaNulls) {
            eff_GJ_dB += rxCfg.crpaNullDepth_dB;
            res.crpaNulledCount++;
        }

        sum_Jr_W += std::pow(10.0, static_cast<double>(item.Jr_dBW) / 10.0);

        // Daya RF di input LNA analog (sebelum implementation loss & mitigasi digital)
        const double Ci_rf_dBW = static_cast<double>(item.Jr_dBW) + static_cast<double>(eff_GJ_dB);
        // Eq. (9.21) - Daya interferensi di korelator setelah front-end loss L_dB (Kaplan p. 562)
        double Ci_dBW          = Ci_rf_dBW - static_cast<double>(rxCfg.implLoss_dB);
        bool skipInterference  = false;

        // Section 9.2.3.1 (Kaplan p. 586) & Section 10.3.3 (Groves p. 454) - Pulsed Jammer & Pulse Blanking
        if (jam.sig.waveform == JammerWaveformType::WIDEBAND_PULSED) {
            const double dc = std::max(0.01, std::min(static_cast<double>(jam.sig.pulseDutyCycle), 0.99));
            if (rxCfg.enablePulseBlanking) {
                totalBlankedFraction = std::min(0.95, totalBlankedFraction + dc);
                skipInterference     = true;
            } else {
                // [DITERTIBKAN] Kaplan p. 586: Tanpa pulse blanking / instant recovery AGC,
                // LNA analog terhantam daya puncak pulsa (Ci_rf_dBW), sedangkan di korelator
                // daya interferensi dirata-ratakan oleh duty cycle (dc_dB).
                const double dc_dB = 10.0 * std::log10(dc);
                Ci_dBW       += dc_dB;
                sum_Ci_lna_W += std::pow(10.0, Ci_rf_dBW / 10.0);
            }
        } else {
            sum_Ci_lna_W += std::pow(10.0, Ci_rf_dBW / 10.0);
        }

        // Section 9.2.3.1 & Section 9.2.2.5 (Kaplan p. 578-579 & 586) - Narrowband Excision & C/A Leak-Through
        if (jam.sig.waveform == JammerWaveformType::NARROWBAND_CW) {
            if (rxCfg.enableNarrowbandExcision && Ci_dBW > static_cast<double>(res.Nt_dBW)) {
                Ci_dBW = static_cast<double>(res.Nt_dBW);
            }
            // [DITERTIBKAN] Kaplan p. 578 & 586: GPS L1 C/A tetap berisiko CW leak-through
            // MESKIPUN telah ditekan ke thermal noise floor (N_t)_dB oleh narrowband excision!
            if (rxCfg.signalType == GNSSSignalType::GPS_L1_CA &&
                Ci_dBW >= static_cast<double>(res.Nt_dBW) - 1.0) {
                res.isCWLeakthroughRisk = true;
            }
        }

        if (!skipInterference) {
            sum_Ci_post_W += std::pow(10.0, Ci_dBW / 10.0);

            // Eq. (9.21) - Interference-to-Signal Power Ratio (Kaplan p. 562)
            const double Ci_Cs_dB     = Ci_dBW - static_cast<double>(res.unjammed_Cs_dBW);
            const double Ci_Cs_linear = std::pow(10.0, Ci_Cs_dB / 10.0);

            // Eq. (9.18) - Interference Spectral Term Accumulation (Kaplan p. 560)
            const float Q_m = computeQualityFactorQ(rxCfg.signalType, jam.sig, Rc_chips_s);
            sum_interference_term += Ci_Cs_linear / (static_cast<double>(Q_m) * static_cast<double>(Rc_chips_s));
        }
    }

    // Section 9.2.1.2 - Analog Front-End LNA Saturation Check (Kaplan p. 554)
    const double total_Jr_dBW = 10.0 * std::log10(std::max(sum_Jr_W, 1.0e-30));
    res.isFrontEndSaturated   = (sum_Ci_lna_W > 0.0) &&
                                (10.0 * std::log10(sum_Ci_lna_W) >= static_cast<double>(rxCfg.lnaSaturationPower_dBW));

    // Eq. (9.24) & (9.21) - Total Incident J/S and Post-Mitigation Ci/Cs (Kaplan p. 569 & 562)
    res.total_JS_dB = static_cast<float>(total_Jr_dBW - static_cast<double>(CRi_dBW));
    if (sum_Ci_post_W > 0.0) {
        const double total_Ci_dBW = 10.0 * std::log10(sum_Ci_post_W);
        res.total_Ci_Cs_dB = static_cast<float>(total_Ci_dBW - static_cast<double>(res.unjammed_Cs_dBW));
    } else {
        res.total_Ci_Cs_dB = -999.0f;
    }

    // Eq. (9.18) - Effective Carrier-to-Noise Density Ratio (Kaplan p. 560)
    const double unjammed_inv = std::pow(10.0, -static_cast<double>(res.unjammed_CN0_dBHz) / 10.0);
    double eff_CN0_dBHz       = -10.0 * std::log10(unjammed_inv + sum_interference_term);

    // Section 9.2.3.1 - Signal Power Degradation from Pulse Blanking (Kaplan p. 586)
    if (totalBlankedFraction > 0.0) {
        eff_CN0_dBHz += 10.0 * std::log10(1.0 - totalBlankedFraction);
    }

    if (res.isFrontEndSaturated) {
        eff_CN0_dBHz = -50.0;
    }

    res.effective_CN0_dBHz      = static_cast<float>(eff_CN0_dBHz);
    res.effective_CN0_linear_Hz = static_cast<float>(std::pow(10.0, eff_CN0_dBHz / 10.0));

    return res;
}