/*
 Copyright (c) 2018-2019, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
 
---------------------------------------------------------------------
 Reads configuration parameters from file 
---------------------------------------------------------------------
*/

#include "include/config.h"
#include "include/comms-lib.h"
#include "include/macros.h"
#include "include/utils.h"

Config::Config(const std::string& jsonfile)
{
    std::string conf;
    Utils::loadTDDConfig(jsonfile, conf);
    const auto jConf = json::parse(conf);
    std::stringstream ss;
    json tddConf;
    ss << jConf.value("BaseStations", tddConf);
    tddConf = json::parse(ss);
    bsPresent = (!tddConf.empty());
    if (bsPresent)
        std::cout << tddConf << "\n\n";
    ss.clear();

    json tddConfCl;
    ss << jConf.value("Clients", tddConfCl);
    tddConfCl = json::parse(ss);
    clPresent = (!tddConfCl.empty());
    if (clPresent)
        std::cout << tddConfCl << "\n\n";
    ss.clear();

    // common (BaseStation config overrides these)
    if (bsPresent) {
        freq = tddConf.value("frequency", 3.6e9);
        rate = tddConf.value("rate", 5e6);
        nco = tddConf.value("nco_frequency", 0.75 * rate);
        bwFilter = rate + 2 * nco;
        radioRfFreq = freq - nco;
        subframeSize = tddConf.value("subframe_size", 0);
        prefix = tddConf.value("prefix", 0);
        postfix = tddConf.value("postfix", 0);
        sampsPerSymbol = subframeSize + prefix + postfix;
        fftSize = tddConf.value("fft_size", 0);
        cpSize = tddConf.value("cp_size", 0);
        tx_scale = tddConf.value("tx_scale", 0.5);
        beacon_seq = tddConf.value("beacon_seq", "gold_ifft");
        pilot_seq = tddConf.value("pilot_seq", "lts-half");

        // BS
        if (!kUseUHD)
            hub_file = tddConf.value("hub_id", "hub_serials.txt");
        json sdr_id_files = tddConf.value("sdr_id", json::array());
        nCells = sdr_id_files.size();
        bs_sdr_file.assign(sdr_id_files.begin(), sdr_id_files.end());
        bsChannel = tddConf.value("channel", "A");
        if (bsChannel != "A" && bsChannel != "B" && bsChannel != "AB")
            throw std::invalid_argument("error channel config: not any of A/B/AB!\n");
        auto jBsFrames = tddConf.value("frame_schedule", json::array());
        frames.assign(jBsFrames.begin(), jBsFrames.end());
        single_gain = tddConf.value("single_gain", true);
        txgain[0] = tddConf.value("txgainA", 20);
        rxgain[0] = tddConf.value("rxgainA", 20);
        txgain[1] = tddConf.value("txgainB", 20);
        rxgain[1] = tddConf.value("rxgainB", 20);
        calTxGain[0] = tddConf.value("calTxGainA", 10);
        calTxGain[1] = tddConf.value("calTxGainB", 10);
        sampleCalEn = tddConf.value("sample_calibrate", false);
        imbalanceCalEn = tddConf.value("imbalance_calibrate", false);
        beamsweep = tddConf.value("beamsweep", false);
        beacon_ant = tddConf.value("beacon_antenna", 0);
        max_frame = tddConf.value("max_frame", 0);

        bs_sdr_ids.resize(nCells);
        nBsSdrs.resize(nCells);
        nBsAntennas.resize(nCells);
        for (size_t i = 0; i < nCells; i++) {
            Utils::loadDevices(bs_sdr_file[i], bs_sdr_ids[i]);
            nBsSdrs[i] = bs_sdr_ids[i].size();
            nBsAntennas[i] = bsChannel.length() * nBsSdrs[i];
        }
        if (!kUseUHD)
            Utils::loadDevices(hub_file, hub_ids);
        symbolsPerFrame = frames.at(0).size();

        //std::vector<std::vector<size_t>> pilotSymbols = Utils::loadSymbols(frames, 'P');
        //std::vector<std::vector<size_t>> ULSymbols = Utils::loadSymbols(frames, 'T');
        //std::vector<std::vector<size_t>> DLSymbols = Utils::loadSymbols(frames, 'R');

        pilotSymbols.resize(frames.size());
        ULSymbols.resize(frames.size());
        DLSymbols.resize(frames.size());
        for (auto fr = frames.begin(); fr != frames.end(); ++fr) {
            size_t f = fr - frames.begin();
            for (int g = 0; g < symbolsPerFrame; g++) {
                switch ((*fr)[g]) {
                case 'P':
                    pilotSymbols[f].push_back(g);
                    break;
                case 'U':
                    ULSymbols[f].push_back(g);
                    break;
                case 'D':
                    DLSymbols[f].push_back(g);
                    break;
                default:
                    break;
                }
            }
        }
        pilotSymsPerFrame = pilotSymbols[0].size();
        ulSymsPerFrame = ULSymbols[0].size();
        dlSymsPerFrame = DLSymbols[0].size();
        // read commons from Client json config
        if (!clPresent) {
            nClSdrs = nClAntennas = std::count(frames.at(0).begin(), frames.at(0).end(), 'P');
            clDataMod = tddConf.value("modulation", "QPSK");
        }
    }

    // Clients
    if (clPresent) {
        auto jClSdrs = tddConfCl.value("sdr_id", json::array());
        // auto jClSdrs = tddConfCl.value("sdr_ip", json::array());
        nClSdrs = jClSdrs.size();
        cl_sdr_ids.assign(jClSdrs.begin(), jClSdrs.end());
        // cl_sdr_ips.assign(jClSdrs.begin(), jClSdrs.end());
        clChannel = tddConfCl.value("channel", "A");
        if (clChannel != "A" && clChannel != "B" && clChannel != "AB")
            throw std::invalid_argument("error channel config: not any of A/B/AB!\n");
        clSdrCh = (clChannel == "AB") ? 2 : 1;
        nClAntennas = nClSdrs * clSdrCh;
        clAgcEn = tddConfCl.value("agc_en", false);
        clAgcGainInit = tddConfCl.value("agc_gain_init", 70); // 0 to 108
        clDataMod = tddConfCl.value("modulation", "QPSK");
        frame_mode = tddConfCl.value("frame_mode", "continuous_resync");
        hw_framer = tddConfCl.value("hw_framer", true);
        txAdvance = tddConfCl.value("tx_advance", 250); // 250

        auto jClTxgainA_vec = tddConfCl.value("txgainA", json::array());
        clTxgain_vec[0].assign(jClTxgainA_vec.begin(), jClTxgainA_vec.end());
        auto jClRxgainA_vec = tddConfCl.value("rxgainA", json::array());
        clRxgain_vec[0].assign(jClRxgainA_vec.begin(), jClRxgainA_vec.end());
        auto jClTxgainB_vec = tddConfCl.value("txgainB", json::array());
        clTxgain_vec[1].assign(jClTxgainB_vec.begin(), jClTxgainB_vec.end());
        auto jClRxgainB_vec = tddConfCl.value("rxgainB", json::array());
        clRxgain_vec[1].assign(jClRxgainB_vec.begin(), jClRxgainB_vec.end());

        auto jClFrames = tddConfCl.value("frame_schedule", json::array());
        assert(jClSdrs.size() == jClFrames.size());
        clFrames.assign(jClFrames.begin(), jClFrames.end());
        clPilotSymbols = Utils::loadSymbols(clFrames, 'P');
        clULSymbols = Utils::loadSymbols(clFrames, 'U');
        clDLSymbols = Utils::loadSymbols(clFrames, 'D');

        // read commons from Client json config
        if (!bsPresent) {
            freq = tddConfCl.value("frequency", 3.6e9);
            rate = tddConfCl.value("rate", 5e6);
            nco = tddConfCl.value("nco_frequency", 0.75 * rate);
            bwFilter = rate + 2 * nco;
            radioRfFreq = freq - nco;
            subframeSize = tddConfCl.value("subframe_size", 0);
            prefix = tddConfCl.value("prefix", 0);
            postfix = tddConfCl.value("postfix", 0);
            sampsPerSymbol = subframeSize + prefix + postfix;
            fftSize = tddConfCl.value("fft_size", 0);
            cpSize = tddConfCl.value("cp_size", 0);
            tx_scale = tddConfCl.value("tx_scale", 0.5);
            beacon_seq = tddConfCl.value("beacon_seq", "gold_ifft");
            pilot_seq = tddConfCl.value("pilot_seq", "lts-half");
            symbolsPerFrame = clFrames.at(0).size();
            single_gain = tddConfCl.value("single_gain", true);
        }
    }

    ulDataSymPresent = (bsPresent && !ULSymbols[0].empty())
        || (clPresent && !clULSymbols[0].empty());

    // compose Beacon subframe:
    // STS Sequence (for AGC) + GOLD Sequence (for Sync)
    // 15reps of STS(16) + 2reps of gold_ifft(128)
    srand(time(NULL));
    const int seqLen = 128;
    std::vector<std::vector<double>> gold_ifft = CommsLib::getSequence(seqLen, CommsLib::GOLD_IFFT);
    std::vector<std::complex<int16_t>> gold_ifft_ci16 = Utils::double_to_cint16(gold_ifft);
    for (size_t i = 0; i < seqLen; i++) {
        gold_cf32.push_back(std::complex<float>(gold_ifft[0][i], gold_ifft[1][i]));
    }

    std::vector<std::vector<double>> sts_seq = CommsLib::getSequence(0, CommsLib::STS_SEQ);
    std::vector<std::complex<int16_t>> sts_seq_ci16 = Utils::double_to_cint16(sts_seq);

    // Populate STS (stsReps repetitions)
    int stsReps = 15;
    for (int i = 0; i < stsReps; i++) {
        beacon_ci16.insert(beacon_ci16.end(), sts_seq_ci16.begin(), sts_seq_ci16.end());
    }

    // Populate gold sequence (two reps, 128 each)
    int goldReps = 2;
    for (int i = 0; i < goldReps; i++) {
        beacon_ci16.insert(beacon_ci16.end(), gold_ifft_ci16.begin(), gold_ifft_ci16.end());
    }

    beaconSize = beacon_ci16.size();

    if (sampsPerSymbol < beaconSize + prefix + postfix) {
        std::string msg = "Minimum supported subframe_size is ";
        msg += std::to_string(beaconSize);
        throw std::invalid_argument(msg);
    }

    beacon = Utils::cint16_to_uint32(beacon_ci16, false, "QI");
    coeffs = Utils::cint16_to_uint32(gold_ifft_ci16, true, "QI");

    int fracBeacon = subframeSize % beaconSize;
    std::vector<std::complex<int16_t>> preBeacon(prefix, 0);
    std::vector<std::complex<int16_t>> postBeacon(postfix + fracBeacon, 0);
    beacon_ci16.insert(beacon_ci16.begin(), preBeacon.begin(), preBeacon.end());
    beacon_ci16.insert(beacon_ci16.end(), postBeacon.begin(), postBeacon.end());
    beacon = Utils::cint16_to_uint32(beacon_ci16, false, "QI");

    // compose pilot subframe
    if (fftSize != 64) {
        fftSize = 64;
        std::cout << "Unsupported fftSize! Setting fftSize to 64..." << std::endl;
    }

    if (cpSize != 16 && cpSize != 0) {
        cpSize = ulDataSymPresent ? 16 : 0;
        std::cout << "Invalid cpSize! Setting cpSize to " << cpSize << "..." << std::endl;
    }
    int pilotSeqLen = fftSize + cpSize;

    int pilotReps = subframeSize / pilotSeqLen;
    int frac = subframeSize % pilotSeqLen;
    std::vector<std::complex<int16_t>> pre(prefix, 0);
    std::vector<std::complex<int16_t>> post(postfix + frac, 0);

    pilotSym = CommsLib::getSequence(pilotSeqLen, CommsLib::LTS_SEQ);
    std::vector<std::complex<int16_t>> lts_ci16 = Utils::double_to_cint16(pilotSym);
    pilot_ci16.insert(pilot_ci16.begin(), pre.begin(), pre.end());

    for (int i = 0; i < pilotReps; i++)
        pilot_ci16.insert(pilot_ci16.end(), lts_ci16.begin(), lts_ci16.end());

    pilot_ci16.insert(pilot_ci16.end(), post.begin(), post.end());

    pilot = Utils::cint16_to_uint32(pilot_ci16, false, "QI");
    pilot_cf32 = Utils::uint32tocfloat(pilot, "QI");
    size_t remain_size = 4096 - pilot.size(); // 4096 is the size of TX_RAM in the FPGA
    for (size_t j = 0; j < remain_size; j++)
        pilot.push_back(0);
#if DEBUG_PRINT
    for (size_t j = 0; j < pilot_ci16.size(); j++) {
        std::cout << "Pilot[" << j << "]: \t " << pilot_ci16[j] << std::endl;
    }
#endif

    // compose data subframe
    if (ulDataSymPresent) {
        int mod_type = clDataMod == "64QAM" ? CommsLib::QAM64 : (clDataMod == "16QAM" ? CommsLib::QAM16 : CommsLib::QPSK);
        std::cout << mod_type << std::endl;
        int mod_order = 1 << mod_type;
        std::cout << mod_order << std::endl;

        data_ind = CommsLib::getDataSc(fftSize);
        pilot_sc = CommsLib::getPilotSc(fftSize);
        int ofdmSize = fftSize + cpSize;
        int nDataScs = data_ind.size();
        int syms = subframeSize / ofdmSize;
        std::vector<std::complex<float>> pre1(prefix, 0);
        std::vector<std::complex<float>> post1(subframeSize % ofdmSize + postfix, 0);
        for (unsigned int i = 0; i < nClAntennas; i++) {
            std::vector<std::complex<float>> data_cf;
            std::vector<std::complex<float>> data_freq_dom;
            data_cf.insert(data_cf.begin(), pre1.begin(), pre1.end());
            std::vector<std::vector<int>> dataBits;
            dataBits.resize(syms);
            for (int s = 0; s < syms; s++) {
                for (int c = 0; c < nDataScs; c++)
                    dataBits[s].push_back(rand() % mod_order);
                std::vector<std::complex<float>> mod_data = CommsLib::modulate(dataBits[s], mod_type);
#if DEBUG_PRINT
                std::cout << "Modulation output: " << mod_data[0] << " " << mod_data[1] << std::endl;
#endif
                std::vector<std::complex<float>> ofdmSym(fftSize);
                int sc = 0;
                for (int c = 0; c < nDataScs; c++) {
                    sc = data_ind[c];
                    ofdmSym[sc] = mod_data[c];
                }
#if DEBUG_PRINT
                std::cout << "Data symbol: " << ofdmSym[sc - 2] << " " << ofdmSym[sc - 1] << std::endl;
#endif
                for (size_t c = 0; c < pilot_sc[0].size(); c++) {
                    sc = pilot_sc[0][c];
                    ofdmSym[sc] = pilot_sc[1][c];
                }
#if DEBUG_PRINT
                std::cout << "Pilot symbol: " << ofdmSym[pilot_sc[0][0]] << " " << ofdmSym[pilot_sc[0][1]] << std::endl;
#endif
                std::vector<std::complex<float>> txSym = CommsLib::IFFT(ofdmSym, fftSize);
                for (auto p = txSym.begin(); p != txSym.end(); ++p)
                    *p *= tx_scale;
                txSym.insert(txSym.begin(), txSym.end() - cpSize, txSym.end()); // add CP
#if DEBUG_PRINT
                std::cout << "IFFT output: " << txSym[0] << " " << txSym[64] << std::endl;
#endif
                data_cf.insert(data_cf.end(), txSym.begin(), txSym.end());
                data_freq_dom.insert(data_freq_dom.end(), ofdmSym.begin(), ofdmSym.end());
            }
            data_cf.insert(data_cf.end(), post1.begin(), post1.end());
            txdata.push_back(data_cf);
            txdata_time_dom.push_back(data_cf);
            txdata_freq_dom.push_back(data_freq_dom);
        }
#if DEBUG_PRINT
        for (size_t i = 0; i < txdata.size(); i++) {
            for (size_t j = 0; j < txdata[i].size(); j++) {
                std::cout << "Values[" << i << "][" << j << "]: \t " << txdata[i][j] << std::endl;
            }
        }
        for (size_t i = 0; i < txdata_freq_dom.size(); i++) {
            for (size_t j = 0; j < txdata_freq_dom[i].size(); j++) {
                std::cout << "FREQ DOMAIN Values[" << i << "][" << j << "]: \t " << txdata_freq_dom[i][j] << std::endl;
            }
        }
#endif
    }

    if (bsPresent) {
        // set trace file path
        time_t now = time(0);
        tm* ltm = localtime(&now);
        int cell_num = nCells;
        int ant_num = getNumAntennas();
        std::string ulPresentStr = (ulDataSymPresent ? "uplink-" : "");
        std::string filename = "logs/trace-" + ulPresentStr + std::to_string(1900 + ltm->tm_year) + "-" + std::to_string(1 + ltm->tm_mon) + "-" + std::to_string(ltm->tm_mday) + "-" + std::to_string(ltm->tm_hour) + "-" + std::to_string(ltm->tm_min) + "-" + std::to_string(ltm->tm_sec) + "_" + std::to_string(cell_num) + "x" + std::to_string(ant_num) + "x" + std::to_string(pilotSymsPerFrame) + ".hdf5";
        trace_file = tddConf.value("trace_file", filename);
    }

    // Multi-threading settings
    unsigned nCores = this->getCoreCount();
    core_alloc = nCores > RX_THREAD_NUM;
    if (bsPresent && (pilotSymsPerFrame + ulSymsPerFrame > 0)) {
        rx_thread_num = (nCores >= 2 * RX_THREAD_NUM && nBsSdrs[0] >= RX_THREAD_NUM) ? RX_THREAD_NUM : 1;
        task_thread_num = TASK_THREAD_NUM;
        if (clPresent && nCores < 1 + task_thread_num + rx_thread_num + nClSdrs)
            core_alloc = false;
    } else {
        rx_thread_num = 0;
        task_thread_num = 0;
        if (clPresent && nCores <= 1 + nClSdrs)
            core_alloc = false;
    }

    if (bsPresent && core_alloc) {
        printf("allocating %d cores to receive threads ... \n", rx_thread_num);
        printf("allocating %d cores to record threads ... \n", task_thread_num);
    }

    if (clPresent && core_alloc)
        printf("allocating %zu cores to client threads ... \n", nClSdrs);

    running = true;

    std::cout << "Configuration file was successfully parsed!" << std::endl;
}

size_t Config::getNumAntennas()
{
    if (!bsPresent)
        return 1;
    return nBsSdrs[0] * bsChannel.length();
}

Config::~Config() {}

int Config::getClientId(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % frames.size();
    it = find(pilotSymbols[fid].begin(), pilotSymbols[fid].end(), symbol_id);
    if (it != pilotSymbols[fid].end()) {
        return it - pilotSymbols[fid].begin();
    }
    return -1;
}

int Config::getUlSFIndex(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % frames.size();
    it = find(ULSymbols[fid].begin(), ULSymbols[fid].end(), symbol_id);
    if (it != ULSymbols[fid].end()) {
        return it - ULSymbols[fid].begin();
    }
    return -1;
}

int Config::getDlSFIndex(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % frames.size();
    it = find(DLSymbols[fid].begin(), DLSymbols[fid].end(), symbol_id);
    if (it != DLSymbols[fid].end())
        return it - DLSymbols[fid].begin();
    return -1;
}

bool Config::isPilot(int frame_id, int symbol_id)
{
    try {
        return frames[frame_id % frames.size()].at(symbol_id) == 'P';
    } catch (const std::out_of_range&) {
        return false;
    }
}

bool Config::isData(int frame_id, int symbol_id)
{
    try {
        return frames[frame_id % frames.size()].at(symbol_id) == 'U';
    } catch (const std::out_of_range&) {
        return false;
    }
}

unsigned Config::getCoreCount()
{
    unsigned nCores = std::thread::hardware_concurrency();
#if DEBUG_PRINT
    std::cout << "number of CPU cores " << std::to_string(nCores) << std::endl;
#endif
    return nCores;
}
