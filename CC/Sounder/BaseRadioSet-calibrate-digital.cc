/*
 Original code copyright Skylark Wireless LLC.
 Copyright (c) 2018-2022, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
 Performs sample offset calibration of radios in massive-MIMO base station.
---------------------------------------------------------------------
*/

#include "include/BaseRadioSet.h"
#include "include/Radio.h"
#include "include/comms-lib.h"
#include "include/macros.h"
#include "include/utils.h"

int BaseRadioSet::syncTimeOffset() {
  int num_radios = _cfg->n_bs_sdrs().at(0) - 1;
  if (num_radios < 1) {
    std::cout << "No need to sample calibrate with one Iris! skipping ..."
              << std::endl;
    return -1;
  }
  int seqLen = 160;  // Sequence length
  std::vector<std::vector<float>> pilot =
      CommsLib::getSequence(CommsLib::LTS_SEQ, seqLen);
  auto pilot_cint16 = Utils::float_to_cint16(pilot);

  // Prepend/Append vectors with prefix/postfix number of null samples
  std::vector<std::complex<int16_t>> prefix_vec(_cfg->prefix(), 0);
  std::vector<std::complex<int16_t>> postfix_vec(
      _cfg->samps_per_slot() - _cfg->prefix() - seqLen, 0);
  pilot_cint16.insert(pilot_cint16.begin(), prefix_vec.begin(),
                      prefix_vec.end());
  pilot_cint16.insert(pilot_cint16.end(), postfix_vec.begin(),
                      postfix_vec.end());
  // Transmitting from only one chain, create a null vector for chainB
  std::vector<std::complex<int16_t>> dummy_cint16(pilot_cint16.size(), 0);
  size_t num_samps = _cfg->samps_per_slot();

  std::vector<void*> txbuff(2);
  txbuff[0] = pilot_cint16.data();
  if (_cfg->bs_sdr_ch() == 2) txbuff[1] = dummy_cint16.data();

  std::vector<std::complex<int16_t>> dummyBuff0(num_samps);
  std::vector<std::complex<int16_t>> dummyBuff1(num_samps);
  std::vector<void*> dummybuffs(2);
  dummybuffs[0] = dummyBuff0.data();
  dummybuffs[1] = dummyBuff1.data();

  long long txTime(0);
  long long rxTime(0);

  Radio* ref_radio = bsRadios.at(0).back();
  auto* ref_dev = ref_radio->RawDev();

  int offset_diff = num_samps;
  std::vector<std::vector<std::complex<int16_t>>> buff;
  buff.resize(num_radios);
  for (int i = 0; i < num_radios; i++) {
    buff[i].resize(num_samps);
  }

  ref_dev->setGain(SOAPY_SDR_TX, 0, "PAD", _cfg->cal_tx_gain().at(0));
  for (int i = 0; i < num_radios; i++)
    bsRadios.at(0).at(i)->drain_buffers(dummybuffs, num_samps);

  ref_radio->activateXmit();
  int ret = ref_radio->xmit(txbuff.data(), num_samps, 3, txTime);
  if (ret < 0) std::cout << "bad write" << std::endl;

  // All write, or prepare to receive.
  for (int j = 0; j < num_radios; j++) {
    ret = bsRadios.at(0).at(j)->activateRecv(rxTime, num_samps, 3);
    if (ret < 0) std::cout << "bad activate at node " << j << std::endl;
  }

  radioTrigger();

  for (int i = 0; i < num_radios; i++) {
    std::vector<void*> rxbuff(2);
    rxbuff[0] = buff[i].data();
    //rxbuff[1] = ant == 2 ? buff[(i*M+j)*ant+1].data() : dummyBuff.data();
    if (_cfg->bs_sdr_ch() == 2) rxbuff[1] = dummyBuff0.data();
    int ret = bsRadios.at(0).at(i)->recv(rxbuff.data(), num_samps, rxTime);
    if (ret < 0) std::cout << "bad read at node " << i << std::endl;
  }

  int min_offset = num_samps;
  int max_offset = 0;
  std::vector<int> offset(num_radios, 0);

  for (int i = 0; i < num_radios; i++) {
    // std::cout << "s" << i << "=[";
    // for (size_t s = 0; s < num_samps; s++)
    //     std::cout << buff[i].at(s).real() << "+1j*" << buff[i].at(s).imag()
    //               << " ";
    // std::cout << "];" << std::endl;
    auto rx = Utils::cint16_to_cfloat(buff[i]);
    int peak = CommsLib::findLTS(rx, seqLen);
    offset[i] = peak < seqLen ? 0 : peak - seqLen;
    //std::cout << i << " " << offset[i] << std::endl;
    if (offset[i] != 0) {
      min_offset = std::min(offset[i], min_offset);
      max_offset = std::max(offset[i], max_offset);
    }
  }
  std::cout << "Min/Max offset is " << min_offset << "/" << max_offset << "."
            << std::endl;
  const std::string filename = "files/iris_samp_offsets.dat";
  Utils::WriteVector(filename, "", offset);
  offset_diff = max_offset - min_offset;

  ref_radio->deactivateXmit();
  ref_dev->setGain(SOAPY_SDR_TX, 0, "PAD", _cfg->tx_gain().at(0));
  for (int i = 0; i < num_radios; i++) {
    Radio* bsRadio = bsRadios.at(0).at(i);
    bsRadio->deactivateRecv();
    bsRadio->drain_buffers(dummybuffs, num_samps);
  }
  return offset_diff;
}
