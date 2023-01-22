/*
 Original code copyright Skylark Wireless LLC.
 Copyright (c) 2018-2022, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
 Performs IQ imbalance and DC offset calibration of Radios in massive-MIMO base station.

 This procedure is adapted from original code that calibrate radio by itself in FDD mode
 To work in TDD mode, this code uses a reference radio to calibrate the whole array.
 Note: Because of OTA calibration, the code is (cannot be) parallelized so this a slow procedure.
---------------------------------------------------------------------
*/

#include "include/BaseRadioSet.h"
#include "include/Radio.h"
#include "include/comms-lib.h"
#include "include/macros.h"
#include "include/utils.h"

static std::vector<std::complex<float>> snoopSamples(SoapySDR::Device* dev,
                                                     size_t channel,
                                                     size_t readSize) {
  std::vector<uint32_t> samps_int =
      dev->readRegisters("RX_SNOOPER", channel, readSize);
  std::vector<std::complex<float>> samps =
      Utils::uint32tocfloat(samps_int, "IQ");
  return samps;
}

static void adjustCalibrationGains(std::vector<SoapySDR::Device*> rxDevs,
                                   SoapySDR::Device* txDev, size_t channel,
                                   double fftBin) {
  using std::cout;
  using std::endl;
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
      rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "LNA2", 14.0);
    }
  }

  txDev->setGain(SOAPY_SDR_TX, channel, "PAD", 40);
  std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

  float maxToneLevel = -200;
  std::vector<bool> adjustedRadios(rxDevsSize, 0);
  std::vector<float> toneLevels(rxDevsSize, 0);
  size_t remainingRadios = adjustedRadios.size();
  for (size_t r = 0; r < rxDevsSize; r++) {
    const auto samps = snoopSamples(rxDevs[r], channel, N);
    auto toneLevel = CommsLib::measureTone(samps, win, windowGain, fftBin, N);
    if (toneLevel >= targetLevel) {
      adjustedRadios[r] = true;
      remainingRadios--;
    }
    toneLevels[r] = toneLevel;
    if (toneLevel > maxToneLevel) maxToneLevel = toneLevel;
    cout << "Node " << r << ": toneLevel0=" << toneLevel << endl;
  }

  std::string nextGainStage;
  if (remainingRadios == rxDevsSize) {
    // if all need adjustment, try bumping up tx gain first
    txDev->setGain(SOAPY_SDR_TX, channel, "ATTN",
                   std::min((targetLevel - maxToneLevel) + attnMax, -6.0));
    cout << "Increasing TX gain level (ATTN) to "
         << std::min((targetLevel - maxToneLevel) + attnMax, -6.0) << endl;
    nextGainStage = "ATTN";
  } else {
    for (size_t r = 0; r < rxDevsSize; r++) {
      if (adjustedRadios[r]) continue;
      rxDevs[r]->setGain(
          SOAPY_SDR_RX, channel, "ATTN",
          std::min((targetLevel - toneLevels[r]) + attnMax, 0.0));
      cout << "Increasing RX gain level (ATTN) to "
           << std::min((targetLevel - maxToneLevel) + attnMax, 0.0) << endl;
    }
    nextGainStage = "LNA";
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

  for (size_t r = 0; r < rxDevsSize; r++) {
    if (adjustedRadios[r]) continue;
    const auto samps = snoopSamples(rxDevs[r], channel, N);
    float toneLevel = CommsLib::measureTone(samps, win, windowGain, fftBin, N);
    if (toneLevel >= targetLevel) {
      adjustedRadios[r] = true;
      remainingRadios--;
    }
    toneLevels[r] = toneLevel;
    cout << "Node " << r << ": toneLevel1=" << toneLevel << endl;
  }

  if (remainingRadios == 0) return;
  double minGain = 0;
  double maxGain = 30;
  if (nextGainStage == "ATTN") {
    minGain = attnMax;
    maxGain = 0;
  }

  // adjust next gain stage
  for (size_t r = 0; r < rxDevsSize; r++) {
    if (adjustedRadios[r]) continue;
    rxDevs[r]->setGain(
        SOAPY_SDR_RX, channel, nextGainStage,
        std::min((targetLevel - toneLevels[r]) + minGain, maxGain));
    cout << "Increasing RX gain level (" << nextGainStage << ") to "
         << std::min((targetLevel - toneLevels[r]) + minGain, maxGain) << endl;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

  for (size_t r = 0; r < rxDevsSize; r++) {
    if (adjustedRadios[r]) continue;
    const auto samps = snoopSamples(rxDevs[r], channel, N);
    auto toneLevel = CommsLib::measureTone(samps, win, windowGain, fftBin, N);
    if (toneLevel > targetLevel) {
      adjustedRadios[r] = true;
      remainingRadios--;
    }
    toneLevels[r] = toneLevel;
    cout << "Node " << r << ": toneLevel2=" << toneLevel << endl;
  }

  if (remainingRadios == 0 || nextGainStage == "LNA") return;

  // adjust next gain stage
  minGain = 0;
  maxGain = 30;
  for (size_t r = 0; r < rxDevsSize; r++) {
    if (adjustedRadios[r]) continue;
    rxDevs[r]->setGain(SOAPY_SDR_RX, channel, "LNA",
                       std::min((targetLevel - toneLevels[r]), maxGain));
    cout << "Increasing RX gain level (LNA) to "
         << std::min((targetLevel - toneLevels[r]) + minGain, maxGain) << endl;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

  for (size_t r = 0; r < rxDevsSize; r++) {
    if (adjustedRadios[r]) continue;
    auto samps = snoopSamples(rxDevs[r], channel, N);
    float toneLevel = CommsLib::measureTone(samps, win, windowGain, fftBin, N);
    if (toneLevel > targetLevel) {
      adjustedRadios[r] = true;
      remainingRadios--;
    }
    toneLevels[r] = toneLevel;
    cout << "Node " << r << ": toneLevel3=" << toneLevel << endl;
  }

  std::cout << rxDevsSize - remainingRadios << " radios reached target level"
            << std::endl;
}

static void setIQBalance(SoapySDR::Device* dev, int direction, size_t channel,
                         int gcorr, int iqcorr) {
  auto gcorri = (gcorr < 0) ? 2047 - std::abs(gcorr) : 2047;
  auto gcorrq = (gcorr > 0) ? 2047 - std::abs(gcorr) : 2047;
  double gainIQ = double(gcorrq) / double(gcorri);
  double phaseIQ = 2 * std::atan(iqcorr / 2047.0);
  std::complex<double> IQcorr =
      gainIQ * std::exp(std::complex<double>(0, phaseIQ));
  dev->setIQBalance(direction, channel, IQcorr);
}

static void dciqMinimize(SoapySDR::Device* targetDev, SoapySDR::Device* refDev,
                         int direction, size_t channel, double rxCenterTone,
                         double txCenterTone) {
  size_t N = 1024;
  std::vector<float> win = CommsLib::hannWindowFunction(N);
  const auto windowGain = CommsLib::windowFunctionPower(win);

  targetDev->setIQBalance(direction, channel, 0.0);
  targetDev->setDCOffset(direction, channel, 0.0);

  //match the internal fixed point representation for DC correction
  const int fixedScale = (direction == SOAPY_SDR_RX) ? 64 : 128;

  //measure initial
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
    const auto samps = snoopSamples(refDev, channel, N);
    const auto measDCLevel =
        CommsLib::measureTone(samps, win, windowGain, rxCenterTone, N);
    const auto measImbalanceLevel = CommsLib::measureTone(
        samps, win, windowGain, rxCenterTone - txCenterTone, N);
    const auto desiredToneLevel = CommsLib::measureTone(
        samps, win, windowGain, rxCenterTone + txCenterTone, N);
    std::cout << "dciqMinimize initial: dcLvl=" << measDCLevel
              << " dB, imLvl=" << measImbalanceLevel
              << " dB, toneLevel=" << desiredToneLevel << "dB" << std::endl;
  }

  //look through each correction arm twice
  float minDcLevel(0);
  std::complex<double> bestDcCorr(0.0, 0.0);
  for (size_t iter = 0; iter < 4; iter++) {
    int start = -fixedScale, stop = +fixedScale, step = 8;
    if (iter == 2) minDcLevel = 0;  //restart with finer search
    if (iter > 1)                   //narrow in for the final iteration set
    {
      const int center =
          int((((iter % 2) == 0) ? bestDcCorr.imag() : bestDcCorr.real()) *
              fixedScale);
      start = std::max<int>(start, center - 8),
      stop = std::min<int>(stop, center + 8), step = 1;
    }
    for (int i = start; i < stop; i += step) {
      //try I or Q arm based on iteration
      const auto dcCorr =
          ((iter % 2) == 0)
              ? std::complex<double>(bestDcCorr.real(), double(i) / fixedScale)
              : std::complex<double>(double(i) / fixedScale, bestDcCorr.imag());
      targetDev->setDCOffset(direction, channel, dcCorr);

      //measure the efficacy
      std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
      const auto samps = snoopSamples(refDev, channel, N);
      const auto measDcLevel =
          CommsLib::measureTone(samps, win, windowGain, rxCenterTone, N);

      //save desired results
      if (measDcLevel < minDcLevel) {
        minDcLevel = measDcLevel;
        bestDcCorr = dcCorr;
      }
    }
  }

  targetDev->setDCOffset(direction, channel, bestDcCorr);
  if (direction == SOAPY_SDR_TX) {
    long dccorri = std::lround(bestDcCorr.real() * 128);
    long dccorrq = std::lround(bestDcCorr.imag() * 128);
    std::cout << "Optimized TX DC Offset: (" << dccorri << "," << dccorrq
              << ")\n";
  } else {
    long dcoffi = std::lround(bestDcCorr.real() * 64);
    if (dcoffi < 0) dcoffi = (1 << 6) | std::abs(dcoffi);

    long dcoffq = std::lround(bestDcCorr.imag() * 64);
    if (dcoffq < 0) dcoffq = (1 << 6) | std::abs(dcoffq);
    std::cout << "Optimized RX DC Offset: (" << dcoffi << "," << dcoffq
              << ")\n";
  }

  //correct IQ imbalance
  float minImbalanceLevel(0);
  int bestgcorr = 0, bestiqcorr = 0;
  for (size_t iter = 0; iter < 4; iter++) {
    int start = -512, stop = 512, step = 8;
    if (iter == 2) minImbalanceLevel = 0;  //restart with finer search
    if (iter > 1) {
      const int center = ((iter % 2) == 0) ? bestgcorr : bestiqcorr;
      start = std::max<int>(start, center - 8);
      stop = std::min<int>(stop, center + 8), step = 1;
    }
    //SoapySDR::logf(debugLogLevel, "start=%d, stop=%d, step=%d", start, stop, step);
    for (int i = start; i < stop; i += step) {
      const int gcorr = ((iter % 2) == 0) ? i : bestgcorr;
      const int iqcorr = ((iter % 2) == 1) ? i : bestiqcorr;
      setIQBalance(targetDev, direction, channel, gcorr, iqcorr);

      //measure the efficacy
      std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
      const auto samps = snoopSamples(refDev, channel, N);
      const auto measImbalanceLevel = CommsLib::measureTone(
          samps, win, windowGain, rxCenterTone - txCenterTone, N);

      //save desired results
      if (measImbalanceLevel < minImbalanceLevel) {
        minImbalanceLevel = measImbalanceLevel;
        bestgcorr = gcorr;
        bestiqcorr = iqcorr;
      }
    }
  }

  //apply the ideal correction
  setIQBalance(targetDev, direction, channel, bestgcorr, bestiqcorr);
  auto gcorri = (bestgcorr < 0) ? 2047 - std::abs(bestgcorr) : 2047;
  auto gcorrq = (bestgcorr > 0) ? 2047 - std::abs(bestgcorr) : 2047;
  std::cout << "Optimized IQ Imbalance Setting: GCorr (" << gcorri << ","
            << gcorrq << "), iqcorr=" << bestiqcorr << "\n";

  //measure corrections
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
    const auto samps = snoopSamples(refDev, channel, N);
    const auto measDCLevel =
        CommsLib::measureTone(samps, win, windowGain, rxCenterTone, N);
    const auto measImbalanceLevel = CommsLib::measureTone(
        samps, win, windowGain, rxCenterTone - txCenterTone, N);
    const auto desiredToneLevel = CommsLib::measureTone(
        samps, win, windowGain, rxCenterTone + txCenterTone, N);
    std::cout << "dciqMinimize final: dcLvl=" << measDCLevel
              << " dB, imLvl=" << measImbalanceLevel
              << " dB, toneLevel=" << desiredToneLevel << "dB" << std::endl;
  }
}

void BaseRadioSet::dciqCalibrationProc(size_t channel) {
  std::cout << "****************************************************\n";
  std::cout << "   DC Offset and IQ Imbalance Calibration: Ch " << channel
            << std::endl;
  std::cout << "****************************************************\n";
  double sampleRate = _cfg->rate();
  double centerRfFreq = _cfg->radio_rf_freq();
  double toneBBFreq = sampleRate / 7;
  size_t radioSize = _cfg->n_bs_sdrs().at(0);

  size_t referenceRadio = _cfg->cal_ref_sdr_id();  //radioSize / 2;
  Radio* refRadio = bsRadios[0][referenceRadio];
  auto* refDev = refRadio->RawDev();

  /* 
     * Start with calibrating the rx paths on all radios using the reference radio
     */
  std::cout << "Calibrating Rx Channels with Tx Reference Radio\n";
  refDev->setFrequency(SOAPY_SDR_TX, channel, "RF", centerRfFreq + toneBBFreq);
  refDev->setFrequency(SOAPY_SDR_RX, channel, "RF", centerRfFreq);
  refDev->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
  std::vector<SoapySDR::Device*> allButRefDevs;
  for (size_t r = 0; r < radioSize; r++) {
    if (r == referenceRadio) continue;
    Radio* bsRadio = bsRadios[0][r];
    auto* dev = bsRadio->RawDev();
    // must set TX "RF" Freq to make sure, we continue using the same LO for rx cal
    dev->setFrequency(SOAPY_SDR_TX, channel, "RF", centerRfFreq);
    dev->setFrequency(SOAPY_SDR_RX, channel, "RF", centerRfFreq);
    dev->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);
    dev->setDCOffsetMode(SOAPY_SDR_RX, channel, false);
    allButRefDevs.push_back(dev);
  }
  refDev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST",
                       std::to_string(1 << 14));
  refDev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "true");

  // Tune rx gains for calibration on all radios except reference radio
  // Tune tx gain on reference radio
  adjustCalibrationGains(allButRefDevs, refDev, channel,
                         toneBBFreq / sampleRate);

  // Minimize Rx DC offset and IQ Imbalance on all receiving radios
  // TODO: Parallelize this loop
  for (size_t r = 0; r < radioSize - 1; r++) {
    dciqMinimize(allButRefDevs[r], allButRefDevs[r], SOAPY_SDR_RX, channel, 0.0,
                 toneBBFreq / sampleRate);
  }

  refDev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", "NONE");
  refDev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "false");

  /* 
     * Calibrate the rx path of the reference radio
     */
  std::cout << "Calibrating Rx Channel of the Reference Radio\n";
  std::vector<SoapySDR::Device*> refDevContainer;
  refDevContainer.push_back(refDev);
  SoapySDR::Device* refRefDev = allButRefDevs[referenceRadio - 1];

  refRefDev->setFrequency(SOAPY_SDR_TX, channel, "RF",
                          centerRfFreq + toneBBFreq);
  refRefDev->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
  // must set TX "RF" Freq to make sure, we continue using the same LO for rx cal
  refDev->setFrequency(SOAPY_SDR_TX, channel, "RF", centerRfFreq);
  refDev->setFrequency(SOAPY_SDR_RX, channel, "RF", centerRfFreq);
  refDev->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);
  refDev->setDCOffsetMode(SOAPY_SDR_RX, channel, false);

  refRefDev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST",
                          std::to_string(1 << 14));
  refRefDev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "true");

  // Tune rx gain for calibraion on reference radio
  // Tune tx gain on neighboring radio to reference radio
  adjustCalibrationGains(refDevContainer, refRefDev, channel,
                         toneBBFreq / sampleRate);
  dciqMinimize(refDev, refDev, SOAPY_SDR_RX, channel, 0.0,
               toneBBFreq / sampleRate);

  refRefDev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", "NONE");
  refRefDev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "false");

  /* 
     * Calibrate the tx path of the reference radio
     */
  std::cout << "Calibrating Tx Channels with Rx Reference Radio\n";
  double txToneBBFreq = sampleRate / 21;
  std::vector<SoapySDR::Device*> refRefDevContainer;
  refRefDevContainer.push_back(refRefDev);

  // must set TX "RF" Freq to make sure, we continue using the same LO for rx cal
  refRefDev->setFrequency(SOAPY_SDR_TX, channel, "RF", centerRfFreq);
  refRefDev->setFrequency(SOAPY_SDR_RX, channel, "RF", centerRfFreq);
  refRefDev->setFrequency(
      SOAPY_SDR_RX, channel, "BB",
      -toneBBFreq);  // Should this be nagative if we need centerRfFreq-toneBBFreq at true center?
  refDev->setFrequency(SOAPY_SDR_TX, channel, "RF", centerRfFreq);
  refDev->setFrequency(SOAPY_SDR_TX, channel, "BB", txToneBBFreq);
  refDev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST",
                       std::to_string(1 << 14));
  refDev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "true");

  // Tune tx gain for calibraion on reference antenna
  // Tune rx gain on neighboring radio to reference radio
  adjustCalibrationGains(refRefDevContainer, refDev, channel,
                         (toneBBFreq + txToneBBFreq) / sampleRate);
  dciqMinimize(refDev, refRefDev, SOAPY_SDR_TX, channel,
               toneBBFreq / sampleRate, txToneBBFreq / sampleRate);

  // kill TX on ref at the end
  refDev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", "NONE");
  refDev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "false");
  refDev->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
  refRefDev->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);

  /* 
     * Now calibrate the tx paths on all other radios using the reference radio
     */
  std::cout << "Calibrating Tx Channel of the Reference Radio\n";
  // refDev->setFrequency(SOAPY_SDR_RX, channel, "RF", centerRfFreq);
  refDev->setFrequency(
      SOAPY_SDR_RX, channel, "BB",
      -toneBBFreq);  // Should this be nagative if we need centerRfFreq-toneBBFreq at true center?
  for (size_t r = 0; r < radioSize - 1; r++) {
    allButRefDevs[r]->setFrequency(SOAPY_SDR_TX, channel, "RF", centerRfFreq);
    allButRefDevs[r]->setFrequency(SOAPY_SDR_TX, channel, "BB", txToneBBFreq);
    allButRefDevs[r]->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST",
                                   std::to_string(1 << 14));
    allButRefDevs[r]->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE",
                                   "true");
    // Tune tx gain for calibraion of the current radio
    // Tune rx gain on the reference radio
    adjustCalibrationGains(refDevContainer, allButRefDevs[r], channel,
                           (toneBBFreq + txToneBBFreq) / sampleRate);
    dciqMinimize(allButRefDevs[r], refDev, SOAPY_SDR_TX, channel,
                 toneBBFreq / sampleRate, txToneBBFreq / sampleRate);
    allButRefDevs[r]->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE",
                                   "false");
    allButRefDevs[r]->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST",
                                   "NONE");
    allButRefDevs[r]->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
  }
  std::cout << "****************************************************\n";
  std::cout << "   Ending DC Offset and IQ Imbalance Calibration\n";
  std::cout << "****************************************************\n";
}
