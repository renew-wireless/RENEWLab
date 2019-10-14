#include "include/comms-lib.h"
#include "include/macros.h"
#include "include/sdr-lib.h"

std::vector<std::complex<float>> RadioConfig::snoopSamples(size_t r, size_t channel, size_t readSize)
{
    struct Radio* bsRadio = &bsRadios[0][r];
    SoapySDR::Device* dev = bsRadio->dev;
    std::vector<uint32_t> samps_int = dev->readRegisters("RX_SNOOPER", channel, readSize);
    std::vector<std::complex<float>> samps;
    for (size_t s = 0; s < samps_int.size(); s++) {
        float real = ((int16_t)(samps_int[s] >> 16)) / 32768.0;
        float imag = ((int16_t)(samps_int[s] && 0xFFFF)) / 32768.0;
        samps.push_back(std::complex<float>(real, imag));
    }
    return samps;
}

std::vector<std::complex<float>> RadioConfig::snoopSamples(SoapySDR::Device* dev, size_t channel, size_t readSize)
{
    std::vector<uint32_t> samps_int = dev->readRegisters("RX_SNOOPER", channel, readSize);
    std::vector<std::complex<float>> samps;
    for (size_t s = 0; s < samps_int.size(); s++) {
        float real = ((int16_t)(samps_int[s] >> 16)) / 32768.0;
        float imag = ((int16_t)(samps_int[s] && 0xFFFF)) / 32768.0;
        samps.push_back(std::complex<float>(real, imag));
    }
    return samps;
}

void RadioConfig::adjustCalibrationGains(std::vector<SoapySDR::Device*> rxDevs, SoapySDR::Device* txDev, size_t channel, double fftBin)
{
    double targetLevel = -10;
    double attnMax = -18;
    size_t N = 1024;
    size_t rxDevsSize = rxDevs.size();
    auto win = CommsLib::hannWindowFunction(N);
    const auto windowGain = CommsLib::windowFunctionPower(win);
    // reset all gains
    for (size_t ch = 0; ch < 2; ch++) {
        txDev->setGain(SOAPY_SDR_TX, ch, "PA2", 0);
        txDev->setGain(SOAPY_SDR_TX, ch, "PAD", 0);
        txDev->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);
        txDev->setGain(SOAPY_SDR_TX, ch, "ATTN", attnMax);
    }
    for (size_t r = 0; r < rxDevsSize; r++) {
        for (size_t ch = 0; ch < 2; ch++) {
            rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "LNA", 0);
            rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "PGA", 0);
            rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "TIA", 0);
            rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "ATTN", attnMax);
        }
    }
    txDev->setGain(SOAPY_SDR_TX, channel, "PAD", 40);
    usleep(1000);

    float maxToneLevel = -200;
    std::vector<bool> adjustedRadios(rxDevsSize, 0);
    std::vector<float> toneLevels(rxDevsSize, 0);
    size_t remainingRadios = adjustedRadios.size();
    for (size_t r = 0; r < rxDevsSize; r++) {
        const auto samps = RadioConfig::snoopSamples(rxDevs[r], channel, N);
        auto toneLevel = CommsLib::measureTone(samps, win, windowGain, fftBin, N);
        if (toneLevel >= targetLevel) {
            adjustedRadios[r] = true;
            remainingRadios--;
        }
        toneLevels[r] = toneLevel;
        if (toneLevel > maxToneLevel)
            maxToneLevel = toneLevel;
    }

    std::string nextGainStage;
    if (remainingRadios == rxDevsSize) {
        // if all need adjustment, try bumping up tx gain first
        txDev->setGain(SOAPY_SDR_TX, channel, "ATTN", (targetLevel - maxToneLevel) + attnMax);
        nextGainStage = "ATTN";
    } else {
        for (size_t r = 0; r < rxDevsSize; r++) {
            if (adjustedRadios[r])
                continue;
            rxDevs[r]->setGain(SOAPY_SDR_RX, channel, "ATTN", (targetLevel - toneLevels[r]) + attnMax);
        }
        nextGainStage = "LNA";
    }
    usleep(1000);

    for (size_t r = 0; r < rxDevsSize; r++) {
        if (adjustedRadios[r])
            continue;
        const auto samps = RadioConfig::snoopSamples(rxDevs[r], channel, N);
        float toneLevel = CommsLib::measureTone(samps, win, windowGain, fftBin, N);
        if (toneLevel >= targetLevel) {
            adjustedRadios[r] = true;
            remainingRadios--;
        }
        toneLevels[r] = toneLevel;
    }

    if (remainingRadios == 0)
        return;
    double minGain = 0;
    if (nextGainStage == "ATTN")
        minGain = attnMax;

    // adjust next gain stage
    for (size_t r = 0; r < rxDevsSize; r++) {
        if (adjustedRadios[r])
            continue;
        rxDevs[r]->setGain(SOAPY_SDR_RX, channel, nextGainStage, (targetLevel - toneLevels[r]) + minGain);
    }
    usleep(1000);

    for (size_t r = 0; r < rxDevsSize; r++) {
        if (adjustedRadios[r])
            continue;
        const auto samps = RadioConfig::snoopSamples(rxDevs[r], channel, N);
        auto toneLevel = CommsLib::measureTone(samps, win, windowGain, fftBin, N);
        if (toneLevel > targetLevel) {
            adjustedRadios[r] = true;
            remainingRadios--;
        }
        toneLevels[r] = toneLevel;
    }

    if (remainingRadios == 0 || nextGainStage == "LNA")
        return;

    // adjust next gain stage
    for (size_t r = 0; r < rxDevsSize; r++) {
        if (adjustedRadios[r])
            continue;
        rxDevs[r]->setGain(SOAPY_SDR_RX, channel, "LNA", (targetLevel - toneLevels[r]));
    }
    usleep(1000);

    for (size_t r = 0; r < rxDevsSize; r++) {
        if (adjustedRadios[r])
            continue;
        auto samps = RadioConfig::snoopSamples(rxDevs[r], channel, N);
        float toneLevel = CommsLib::measureTone(samps, win, windowGain, fftBin, N);
        if (toneLevel > targetLevel) {
            adjustedRadios[r] = true;
            remainingRadios--;
        }
        toneLevels[r] = toneLevel;
    }

    std::cout << rxDevsSize - 1 - remainingRadios << " radios reached target level" << std::endl;
}

void RadioConfig::setIQBalance(SoapySDR::Device* dev, size_t channel, int direction, int gcorr, int iqcorr)
{
    auto gcorri = (gcorr < 0) ? 2047 - std::abs(gcorr) : 2047;
    auto gcorrq = (gcorr > 0) ? 2047 - std::abs(gcorr) : 2047;
    double gainIQ = double(gcorrq) / double(gcorri);
    double phaseIQ = 2 * std::atan(iqcorr / 2047.0);
    std::complex<double> IQcorr = gainIQ * std::exp(std::complex<double>(0, phaseIQ));
    dev->setIQBalance(direction, channel, IQcorr);
}

void RadioConfig::dciqMinimize(SoapySDR::Device* targetDev, SoapySDR::Device* refDev, size_t channel, int direction, double rxCenterTone, double txCenterTone)
{
    size_t N = 1024;
    std::vector<float> win = CommsLib::hannWindowFunction(N);
    const auto windowGain = CommsLib::windowFunctionPower(win);

    targetDev->setIQBalance(direction, channel, 0.0);
    targetDev->setDCOffset(direction, channel, 0.0);

    //match the internal fixed point representation for DC correction
    const int fixedScale = (direction == SOAPY_SDR_RX) ? 64 : 128;

    //measure initial
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(GAIN_SETTLE_TIME_MS));
        const auto samps = snoopSamples(refDev, channel, N);
        const auto measDCLevel = CommsLib::measureTone(samps, win, windowGain, rxCenterTone, N);
        const auto measImbalanceLevel = CommsLib::measureTone(samps, win, windowGain, rxCenterTone - txCenterTone, N);
        const auto desiredToneLevel = CommsLib::measureTone(samps, win, windowGain, rxCenterTone + txCenterTone, N);
        std::cout << "dciqMinimize initial: dcLvl=" << measDCLevel << " dB, imLvl=" << measImbalanceLevel << " dB, toneLevel=" << desiredToneLevel << "dB" << std::endl;
    }

    //look through each correction arm twice
    float minDcLevel(0);
    std::complex<double> bestDcCorr(0.0, 0.0);
    for (size_t iter = 0; iter < 4; iter++) {
        int start = -fixedScale, stop = +fixedScale, step = 8;
        if (iter == 2)
            minDcLevel = 0; //restart with finer search
        if (iter > 1) //narrow in for the final iteration set
        {
            const int center = int((((iter % 2) == 0) ? bestDcCorr.imag() : bestDcCorr.real()) * fixedScale);
            start = std::max<int>(start, center - 8), stop = std::min<int>(stop, center + 8), step = 1;
        }
        for (int i = start; i < stop; i += step) {
            //try I or Q arm based on iteration
            const auto dcCorr = ((iter % 2) == 0) ? std::complex<double>(bestDcCorr.real(), double(i) / fixedScale) : std::complex<double>(double(i) / fixedScale, bestDcCorr.imag());
            targetDev->setDCOffset(direction, channel, dcCorr);

            //measure the efficacy
            std::this_thread::sleep_for(std::chrono::milliseconds(GAIN_SETTLE_TIME_MS));
            const auto samps = snoopSamples(refDev, channel, N);
            const auto measDcLevel = CommsLib::measureTone(samps, win, windowGain, rxCenterTone, N);

            //save desired results
            if (measDcLevel < minDcLevel) {
                minDcLevel = measDcLevel;
                bestDcCorr = dcCorr;
            }
        }
    }

    targetDev->setDCOffset(direction, channel, bestDcCorr);

    //correct IQ imbalance
    float minImbalanceLevel(0);
    int bestgcorr = 0, bestiqcorr = 0;
    for (size_t iter = 0; iter < 4; iter++) {
        int start = -512, stop = 512, step = 8;
        if (iter == 2)
            minImbalanceLevel = 0; //restart with finer search
        if (iter > 1) {
            const int center = ((iter % 2) == 0) ? bestgcorr : bestiqcorr;
            start = std::max<int>(start, center - 8);
            stop = std::min<int>(stop, center + 8), step = 1;
        }
        //SoapySDR::logf(debugLogLevel, "start=%d, stop=%d, step=%d", start, stop, step);
        for (int i = start; i < stop; i += step) {
            const int gcorr = ((iter % 2) == 0) ? i : bestgcorr;
            const int iqcorr = ((iter % 2) == 1) ? i : bestiqcorr;
            RadioConfig::setIQBalance(targetDev, direction, channel, gcorr, iqcorr);

            //measure the efficacy
            std::this_thread::sleep_for(std::chrono::milliseconds(GAIN_SETTLE_TIME_MS));
            const auto samps = snoopSamples(refDev, channel, N);
            const auto measImbalanceLevel = CommsLib::measureTone(samps, win, windowGain, rxCenterTone - txCenterTone, N);

            //save desired results
            if (measImbalanceLevel < minImbalanceLevel) {
                minImbalanceLevel = measImbalanceLevel;
                bestgcorr = gcorr;
                bestiqcorr = iqcorr;
            }
        }
    }

    //apply the ideal correction
    RadioConfig::setIQBalance(targetDev, direction, channel, bestgcorr, bestiqcorr);

    //measure corrections
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(GAIN_SETTLE_TIME_MS));
        const auto samps = snoopSamples(refDev, channel, N);
        const auto measDCLevel = CommsLib::measureTone(samps, win, windowGain, rxCenterTone, N);
        const auto measImbalanceLevel = CommsLib::measureTone(samps, win, windowGain, rxCenterTone - txCenterTone, N);
        const auto desiredToneLevel = CommsLib::measureTone(samps, win, windowGain, rxCenterTone + txCenterTone, N);
        std::cout << "dciqMinimize final: dcLvl=" << measDCLevel << " dB, imLvl=" << measImbalanceLevel << " dB, toneLevel=" << desiredToneLevel << "dB" << std::endl;
    }
}

void RadioConfig::dciqCalibrationProc(size_t channel)
{
    double sampleRate = _cfg->rate;
    double centerRfFreq = _cfg->freq;
    double toneBBFreq = sampleRate / 7;
    size_t radioSize = _cfg->nBsSdrs[0];

    size_t referenceRadio = radioSize / 2;
    struct Radio* refRadio = &bsRadios[0][referenceRadio];
    SoapySDR::Device* refDev = refRadio->dev;

    /* 
     * Start with calibrating the rx paths on all radios using a reference radio
     */
    refDev->setFrequency(SOAPY_SDR_TX, channel, "RF", centerRfFreq + toneBBFreq);
    refDev->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
    refDev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", std::to_string(1 << 14));
    std::vector<SoapySDR::Device*> allButRefDevs;
    for (size_t r = 0; r < radioSize; r++) {
        if (r == referenceRadio)
            continue;
        struct Radio* bsRadio = &bsRadios[0][r];
        SoapySDR::Device* dev = bsRadio->dev;
        dev->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);
        dev->setDCOffsetMode(SOAPY_SDR_RX, channel, false);
        allButRefDevs.push_back(dev);
    }

    // Tune rx gains for calibration on all radios except reference radio
    // Tune tx gain on reference radio
    RadioConfig::adjustCalibrationGains(allButRefDevs, refDev, channel, toneBBFreq / sampleRate);

    // Minimize Rx DC offset and IQ Imbalance on all receiving radios
    for (size_t r = 0; r < radioSize - 1; r++) {
        RadioConfig::dciqMinimize(allButRefDevs[r], allButRefDevs[r], channel, SOAPY_SDR_RX, 0.0, toneBBFreq / sampleRate);
    }

    /* 
     * Calibrate the rx path of the reference radio
     */
    std::vector<SoapySDR::Device*> refDevContainer;
    refDevContainer.push_back(refDev);
    SoapySDR::Device* refRefDev = allButRefDevs[referenceRadio - 1];

    refRefDev->setFrequency(SOAPY_SDR_TX, channel, "RF", centerRfFreq + toneBBFreq);
    refRefDev->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
    refRefDev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", std::to_string(1 << 14));
    refDev->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);
    refDev->setDCOffsetMode(SOAPY_SDR_RX, channel, false);

    // Tune rx gain for calibraion on reference radio
    // Tune tx gain on neighboring radio to reference radio
    RadioConfig::adjustCalibrationGains(refDevContainer, refRefDev, channel, toneBBFreq / sampleRate);
    RadioConfig::dciqMinimize(refDev, refDev, channel, SOAPY_SDR_RX, 0.0, toneBBFreq / sampleRate);

    /* 
     * Calibrate the tx path of the reference radio
     */
    double txToneBBFreq = sampleRate / 21;
    std::vector<SoapySDR::Device*> refRefDevContainer;
    refRefDevContainer.push_back(refRefDev);

    refRefDev->setFrequency(SOAPY_SDR_RX, channel, "RF", centerRfFreq - toneBBFreq);
    refRefDev->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);
    refDev->setFrequency(SOAPY_SDR_TX, channel, "BB", txToneBBFreq);
    refDev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", std::to_string(1 << 14));

    // Tune tx gain for calibraion on reference antenna
    // Tune rx gain on neighboring radio to reference radio
    RadioConfig::adjustCalibrationGains(refRefDevContainer, refDev, channel, (toneBBFreq + txToneBBFreq) / sampleRate);
    RadioConfig::dciqMinimize(refDev, refRefDev, channel, SOAPY_SDR_TX, toneBBFreq / sampleRate, txToneBBFreq / sampleRate);

    // kill TX on ref at the end
    refDev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", "NONE");
    refDev->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);

    /* 
     * Now calibrate the tx paths on all other radios using the reference radio
     */
    refDev->setFrequency(SOAPY_SDR_RX, channel, "RF", centerRfFreq - toneBBFreq);
    refDev->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);
    for (size_t r = 0; r < radioSize - 1; r++) {
        allButRefDevs[r]->setFrequency(SOAPY_SDR_TX, channel, "BB", txToneBBFreq);
        allButRefDevs[r]->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", std::to_string(1 << 14));
        // Tune tx gain for calibraion of the current radio
        // Tune rx gain on the reference radio
        RadioConfig::adjustCalibrationGains(refDevContainer, allButRefDevs[r], channel, (toneBBFreq + txToneBBFreq) / sampleRate);
        RadioConfig::dciqMinimize(allButRefDevs[r], refDev, channel, SOAPY_SDR_TX, toneBBFreq / sampleRate, txToneBBFreq / sampleRate);
        allButRefDevs[r]->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", "NONE");
        allButRefDevs[r]->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
    }
}

void RadioConfig::collectCSI(bool& adjust)
{
    int R = bsRadios[0].size();
    if (R < 2) {
        std::cout << "No need to sample calibrate with one Iris! skipping ..." << std::endl;
        return;
    }
    std::vector<std::vector<double>> pilot;
    //std::vector<std::complex<float>> pilot_cf32;
    std::vector<std::complex<int16_t>> pilot_cint16;
    int type = CommsLib::LTS_SEQ;
    int seqLen = 160; // Sequence length
    pilot = CommsLib::getSequence(seqLen, type);
    // double array to complex 32-bit float vector
    double max_abs = 0;
    for (int i = 0; i < seqLen; i++) {
        std::complex<double> samp(pilot[0][i], pilot[1][i]);
        max_abs = max_abs > std::abs(samp) ? max_abs : std::abs(samp);
    }

    pilot_cint16.resize(seqLen);
    for (int i = 0; i < seqLen; i++) {
        auto re = 0.25 * pilot[0][i] / max_abs * 32767;
        auto im = 0.25 * pilot[1][i] / max_abs * 32767;
        pilot_cint16[i] = std::complex<int16_t>((int16_t)re, (int16_t)im);
    }

    // Prepend/Append vectors with prefix/postfix number of null samples
    std::vector<std::complex<int16_t>> prefix_vec(_cfg->prefix, 0);
    std::vector<std::complex<int16_t>> postfix_vec(_cfg->sampsPerSymbol - _cfg->prefix - seqLen, 0);
    pilot_cint16.insert(pilot_cint16.begin(), prefix_vec.begin(), prefix_vec.end());
    pilot_cint16.insert(pilot_cint16.end(), postfix_vec.begin(), postfix_vec.end());
    // Transmitting from only one chain, create a null vector for chainB
    std::vector<std::complex<int16_t>> dummy_cint16(pilot_cint16.size(), 0);

    int ch = (_cfg->bsChannel == "B") ? 1 : 0;
    std::vector<void*> txbuff(2);
    if (ch == 0) {
        txbuff[0] = pilot_cint16.data(); //pilot_cf32.data();
        txbuff[1] = dummy_cint16.data(); //zero_vec.data();
    } else {
        txbuff[0] = dummy_cint16.data(); //zero_vec.data();
        txbuff[1] = pilot_cint16.data(); //pilot_cf32.data();
    }

    //std::vector<std::vector<std::complex<float>>> buff;
    std::vector<std::vector<std::complex<int16_t>>> buff;
    //int ant = _cfg->bsChannel.length();
    //int M = bsRadios[0].size() * ant;
    buff.resize(R * R);
    for (int i = 0; i < R; i++) {
        for (int j = 0; j < R; j++) {
            buff[i * R + j].resize(_cfg->sampsPerSymbol);
        }
    }

    std::vector<std::complex<int16_t>> dummyBuff0(_cfg->sampsPerSymbol);
    std::vector<std::complex<int16_t>> dummyBuff1(_cfg->sampsPerSymbol);
    std::vector<void*> dummybuffs(2);
    dummybuffs[0] = dummyBuff0.data();
    dummybuffs[1] = dummyBuff1.data();

    for (int i = 0; i < R; i++) {
        struct Radio* bsRadio = &bsRadios[0][i];
        SoapySDR::Device* dev = bsRadio->dev;
        RadioConfig::drain_buffers(dev, bsRadio->rxs, dummybuffs, _cfg->sampsPerSymbol);
    }

    for (int i = 0; i < R; i++) {
        struct Radio* bsRadio = &bsRadios[0][i];
        SoapySDR::Device* dev = bsRadio->dev;
        dev->setGain(SOAPY_SDR_TX, ch, "PAD", _cfg->calTxGain[ch]);
        dev->writeSetting("TDD_CONFIG", "{\"tdd_enabled\":false}");
        dev->writeSetting("TDD_MODE", "false");
        dev->activateStream(bsRadio->txs);
    }

    long long txTime(0);
    long long rxTime(0);
    for (int i = 0; i < R; i++) {
        // All write, or prepare to receive.
        for (int j = 0; j < R; j++) {
            struct Radio* bsRadio = &bsRadios[0][j];
            SoapySDR::Device* dev = bsRadio->dev;
            int flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
            if (j == i) {
                int ret = dev->writeStream(bsRadio->txs, txbuff.data(),
                    _cfg->sampsPerSymbol, flags, txTime, 1000000);
                if (ret < 0)
                    std::cout << "bad write" << std::endl;
            } else {
                int ret = dev->activateStream(bsRadio->rxs, flags, rxTime,
                    _cfg->sampsPerSymbol);
                if (ret < 0)
                    std::cout << "bad activate at node " << j << std::endl;
            }
        }

        radioTrigger();

        // All but one receive.
        int flags = 0;
        for (int j = 0; j < R; j++) {
            struct Radio* bsRadio = &bsRadios[0][j];
            SoapySDR::Device* dev = bsRadio->dev;
            if (j == i)
                continue;
            std::vector<void*> rxbuff(2);
            rxbuff[0] = buff[(i * R + j)].data();
            //rxbuff[1] = ant == 2 ? buff[(i*M+j)*ant+1].data() : dummyBuff.data();
            rxbuff[1] = dummyBuff0.data();
            int ret = dev->readStream(bsRadio->rxs, rxbuff.data(),
                _cfg->sampsPerSymbol, flags, rxTime, 1000000);
            if (ret < 0)
                std::cout << "bad read at node " << j << std::endl;
        }
    }

    int ref_ant = 0;
    int ref_offset = ref_ant == 0 ? 1 : 0;
    std::vector<int> offset(R);

    bool good_csi = true;
    for (int i = 0; i < R; i++) {
        std::vector<std::complex<double>> rx(_cfg->sampsPerSymbol);
        int k = ((i == ref_ant) ? ref_offset : ref_ant) * R + i;
        std::transform(buff[k].begin(), buff[k].end(), rx.begin(),
            [](std::complex<int16_t> cf) {
                return std::complex<double>(cf.real() / 32768.0, cf.imag() / 32768.0);
            });
        int peak = CommsLib::findLTS(rx, seqLen);
        offset[i] = peak < 128 ? 0 : peak - 128;
        //std::cout << i << " " << offset[i] << std::endl;
        if (offset[i] == 0)
            good_csi = false;

#if DEBUG_PLOT
        std::vector<double> rx_I(_cfg->sampsPerSymbol);
        std::transform(rx.begin(), rx.end(), rx_I.begin(),
            [](std::complex<double> cf) {
                return cf.real();
            });
        plt::figure_size(1200, 780);
        plt::plot(rx_I);
        plt::xlim(0, _cfg->sampsPerSymbol);
        plt::ylim(-1, 1);
        plt::title("Sample figure");
        plt::legend();
        plt::save(std::to_string(i) + ".png");
#endif
    }

    // adjusting trigger delays based on lts peak index
    adjust &= good_csi;
    if (adjust) {
        for (int i = 0; i < R; i++) {
            struct Radio* bsRadio = &bsRadios[0][i];
            SoapySDR::Device* dev = bsRadio->dev;
            // if offset[i] == 0, then good_csi is false and we never get here???
            int delta = (offset[i] == 0) ? 0 : offset[ref_offset] - offset[i];
            std::cout << "adjusting delay of node " << i << " by " << delta << std::endl;
            while (delta < 0) {
                dev->writeSetting("ADJUST_DELAYS", "-1");
                ++delta;
            }
            while (delta > 0) {
                dev->writeSetting("ADJUST_DELAYS", "1");
                --delta;
            }
        }
    }

    for (int i = 0; i < R; i++) {
        struct Radio* bsRadio = &bsRadios[0][i];
        SoapySDR::Device* dev = bsRadio->dev;
        dev->deactivateStream(bsRadio->txs);
        dev->deactivateStream(bsRadio->rxs);
        dev->setGain(SOAPY_SDR_TX, ch, "PAD", _cfg->txgain[ch]); //[0,30]
        RadioConfig::drain_buffers(dev, bsRadio->rxs, dummybuffs, _cfg->sampsPerSymbol);
    }
}
