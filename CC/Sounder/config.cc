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

Config::Config(std::string jsonfile)
{
    std::string conf;
    Utils::loadTDDConfig(jsonfile, conf);
    const auto jConf = json::parse(conf);
    std::stringstream ss;
    json tddConf;
    ss << jConf.value("BaseStations", tddConf);
    tddConf = json::parse(ss);
    bsPresent = (!tddConf.empty());
    if (bsPresent) std::cout << tddConf << "\n\n";
    ss.clear();

    json tddConfCl;
    ss << jConf.value("Clients", tddConfCl);
    tddConfCl = json::parse(ss);
    clPresent = (!tddConfCl.empty());
    if (clPresent) std::cout << tddConfCl << "\n\n";
    ss.clear();

    // common (BaseStation config overrides these)
    if (bsPresent)
    {
        freq = tddConf.value("frequency", 3.6e9);
        rate = tddConf.value("rate", 5e6);
        int samps = tddConf.value("subframe_size", 0);
        prefix = tddConf.value("prefix", 0);
        postfix = tddConf.value("postfix", 0);
        sampsPerSymbol = samps + prefix + postfix;
        fftSize = tddConf.value("fft_size", 0);
        cpSize = tddConf.value("cp_size", 0);
        beacon_seq = tddConf.value("beacon_seq", "gold_ifft");
        pilot_seq = tddConf.value("pilot_seq", "lts");

        // BS 
        hub_file = tddConf.value("hub_id", "hub_serials.txt");
        json sdr_id_files = tddConf.value("sdr_id", json::array());
        nCells = sdr_id_files.size();
        for (int i = 0; i < nCells; i++) bs_sdr_file.push_back(sdr_id_files.at(i).get<std::string>());
        bsChannel = tddConf.value("channel", "A");
	if (bsChannel != "A" && bsChannel != "B" && bsChannel != "AB")
	    throw std::invalid_argument( "error channel config: not any of A/B/AB!\n");
        bsSdrCh = (bsChannel == "AB") ? 2 : 1;
        auto jBsFrames = tddConf.value("frame_schedule", json::array());
        framePeriod = jBsFrames.size();
        for(int f = 0; f < framePeriod; f++) frames.push_back(jBsFrames.at(f).get<std::string>());
        txgainA = tddConf.value("txgainA", 20);
        rxgainA = tddConf.value("rxgainA", 20);
        txgainB = tddConf.value("txgainB", 20);
        rxgainB = tddConf.value("rxgainB", 20);
        beamsweep = tddConf.value("beamsweep", false);
        beacon_ant = tddConf.value("beacon_antenna", 0);
        max_frame = tddConf.value("max_frame", 0);
    
        bs_sdr_ids.resize(nCells);
        nBsSdrs.resize(nCells);
        nBsAntennas.resize(nCells);
        for (int i = 0; i < nCells; i++)
        {
            Utils::loadDevices(bs_sdr_file[i], bs_sdr_ids[i]);
            nBsSdrs[i] = bs_sdr_ids[i].size();
            nBsAntennas[i] = bsSdrCh * nBsSdrs[i];
        }
        Utils::loadDevices(hub_file, hub_ids);
        symbolsPerFrame = frames.at(0).size();

        //std::vector<std::vector<size_t>> pilotSymbols = Utils::loadSymbols(frames, 'P');
        //std::vector<std::vector<size_t>> ULSymbols = Utils::loadSymbols(frames, 'T');
        //std::vector<std::vector<size_t>> DLSymbols = Utils::loadSymbols(frames, 'R');

        pilotSymbols.resize(framePeriod);
        for(int f = 0; f < framePeriod; f++)
        {
            std::string fr = frames[f]; 
            for (int g = 0; g < symbolsPerFrame; g++)
            {
                if (fr[g] == 'P')
                    pilotSymbols[f].push_back(g);
            }
        }
        ULSymbols.resize(framePeriod);
        for(int f = 0; f < framePeriod; f++)
        {
            std::string fr = frames[f]; 
            for (int g = 0; g < symbolsPerFrame; g++)
            {
                if (fr[g] == 'U')
                    ULSymbols[f].push_back(g);
            }
        }
        DLSymbols.resize(framePeriod);
        for(int f = 0; f < framePeriod; f++)
        {
            std::string fr = frames[f]; 
            for (int g = 0; g < symbolsPerFrame; g++)
            {
                if (fr[g] == 'D')
                    DLSymbols[f].push_back(g);
            }
        }
        pilotSymsPerFrame = pilotSymbols[0].size();
        ulSymsPerFrame = ULSymbols[0].size();
        dlSymsPerFrame = DLSymbols[0].size();
        // read commons from Client json config
        if (!clPresent)
        {
            nClSdrs = std::count(frames.at(0).begin(), frames.at(0).end(), 'P');
	    clDataMod = tddConf.value("modulation", "QPSK");
        }
    }

    // Clients
    if (clPresent)
    {
        auto jClSdrs = tddConfCl.value("sdr_id", json::array());
        nClSdrs = jClSdrs.size();
        for (int i = 0; i < nClSdrs; i++) cl_sdr_ids.push_back(jClSdrs.at(i).get<std::string>());
        clChannel = tddConfCl.value("channel", "A");
	if (clChannel != "A" && clChannel != "B" && clChannel != "AB")
	    throw std::invalid_argument( "error channel config: not any of A/B/AB!\n");
        clSdrCh = (clChannel == "AB") ? 2 : 1;
	clAgcEn = tddConfCl.value("agc_en", false);
	clDataMod = tddConfCl.value("modulation", "QPSK");
        frame_mode = tddConfCl.value("frame_mode", "continuous_resync");

	auto jClTxgainA_vec = tddConfCl.value("txgainA", json::array());
	auto jClRxgainA_vec = tddConfCl.value("rxgainA", json::array());
	auto jClTxgainB_vec = tddConfCl.value("txgainB", json::array());
	auto jClRxgainB_vec = tddConfCl.value("rxgainB", json::array());
        for(int f = 0; f < nClSdrs; f++) {
	    clTxgainA_vec.push_back(jClTxgainA_vec.at(f).get<double>());
	    clRxgainA_vec.push_back(jClRxgainA_vec.at(f).get<double>());
	    clTxgainB_vec.push_back(jClTxgainB_vec.at(f).get<double>());
	    clRxgainB_vec.push_back(jClRxgainB_vec.at(f).get<double>());
	}

        auto jClFrames = tddConfCl.value("frame_schedule", json::array());
        assert((size_t)nClSdrs == jClFrames.size());
        for(int f = 0; f < nClSdrs; f++) clFrames.push_back(jClFrames.at(f).get<std::string>());
        clPilotSymbols = Utils::loadSymbols(clFrames, 'P');
        clULSymbols = Utils::loadSymbols(clFrames, 'U');
        clDLSymbols = Utils::loadSymbols(clFrames, 'D');

        // read commons from Client json config
        if (!bsPresent)
        {
            freq = tddConfCl.value("frequency", 3.6e9);
            rate = tddConfCl.value("rate", 5e6);
            int samps = tddConfCl.value("subframe_size", 0);
            prefix = tddConfCl.value("prefix", 0);
            postfix = tddConfCl.value("postfix", 0);
            sampsPerSymbol = samps + prefix + postfix;
            fftSize = tddConfCl.value("fft_size", 0);
            cpSize = tddConfCl.value("cp_size", 0);
            beacon_seq = tddConfCl.value("beacon_seq", "gold_ifft");
            pilot_seq = tddConfCl.value("pilot_seq", "lts");
            symbolsPerFrame = clFrames.at(0).size();
        }
    }

    if (bsPresent or clPresent)
    {
        /**** Signal Generation ****/
	if (fftSize != 64) fftSize = 64;
	data_ind = CommsLib::getDataSc(fftSize);
        pilot_sc = CommsLib::getPilotSc(fftSize);
	if (pilot_seq.compare("lts") == 0) {
	    pilot_double = CommsLib::getSequence(160, CommsLib::LTS_SEQ);
	} else {
	    throw std::invalid_argument( "Only LTS pilots currently supported" );
	}
	std::vector<std::vector<std::complex<float>>> txdata_freq_dom_conf = txdata_freq_dom;

        srand(time(NULL));
        std::vector<std::vector<double>> gold_ifft = CommsLib::getSequence(128, CommsLib::GOLD_IFFT);
        beacon_ci16.resize(256); 
        for (int i = 0; i < 128; i++)
        {
            beacon_ci16[i] = std::complex<int16_t>( (int16_t)(gold_ifft[0][i]*32768), (int16_t)(gold_ifft[1][i]*32768) );
            beacon_ci16[i+128] = beacon_ci16[i];
        }
        std::vector<std::complex<int16_t>> pre0(prefix, 0);
        std::vector<std::complex<int16_t>> post0(sampsPerSymbol-256-prefix, 0);
        beacon_ci16.insert(beacon_ci16.begin(),pre0.begin(),pre0.end());
        beacon_ci16.insert(beacon_ci16.end(),post0.begin(),post0.end());

        beacon = Utils::cint16_to_uint32(beacon_ci16, false, "IQ"); 

        std::vector<std::complex<int16_t>> pre(prefix, 0);
        std::vector<std::complex<int16_t>> post(postfix, 0);

        coeffs_ci16 = Utils::double_to_int16(gold_ifft, "IQ"); 
#ifdef NEWCORR
        coeffs = Utils::cint16_to_uint32(coeffs_ci16, true, "IQ");
#else
        coeffs = Utils::cint16_to_uint32(coeffs_ci16, true, "QI");
#endif
        // compose pilot subframe
        std::vector<std::vector<double>> lts = CommsLib::getSequence(160, CommsLib::LTS_SEQ);
        std::vector<std::complex<int16_t>> lts_ci16 = Utils::double_to_int16(lts, "IQ"); 
        int nSamps = sampsPerSymbol - prefix - postfix;
        int rep = nSamps / 160;
        int frac = nSamps % 160;
        pilot_ci16.insert(pilot_ci16.begin(), pre.begin(), pre.end());

        for (int i = 0 ; i < rep; i++)
            pilot_ci16.insert(pilot_ci16.end(), lts_ci16.begin(), lts_ci16.end());

        pilot_ci16.insert(pilot_ci16.end(), lts_ci16.begin(), lts_ci16.begin()+frac);
        pilot_ci16.insert(pilot_ci16.end(), post.begin(), post.end());

        pilot = Utils::cint16_to_uint32(pilot_ci16, false, "IQ");
#if DEBUG_PRINT
        for (int j = 0; j < pilot.size(); j++)
        {
            std::cout << "Pilot[" << j << "]: \t " << pilot_ci16[j] << std::endl;
        }
#endif

        // compose data subframe
        if ((bsPresent and ULSymbols[0].size() > 0) or (clPresent and clULSymbols[0].size() > 0))
        {
            int fftSize = fftSize; 
            int cpSize = cpSize; 
            int mod_type = clDataMod == "64QAM" ? CommsLib::QAM64 : (clDataMod == "16QAM" ? CommsLib::QAM16 : CommsLib::QPSK); 
            std::cout << mod_type << std::endl;
            int mod_order = (int)pow(2, mod_type); 
            std::cout << mod_order << std::endl;
            if (fftSize != 64) fftSize = 64;
            if (cpSize != 16) cpSize = 16;
            std::vector<int> data_ind = CommsLib::getDataSc(fftSize);
            pilot_sc = CommsLib::getPilotSc(fftSize);
            int ofdmSize = fftSize + cpSize;
            int nDataScs = data_ind.size(); 
            int syms = nSamps / ofdmSize;
            std::vector<std::complex<float>> pre1(prefix, 0);
            std::vector<std::complex<float>> post1(nSamps % ofdmSize + postfix, 0);
            for (int i = 0; i < nClSdrs; i++)
            {
                std::vector<std::complex<float>> data_cf;
		std::vector<std::complex<float>> data_freq_dom;
                data_cf.insert(data_cf.begin(), pre1.begin(), pre1.end()); 
                std::vector<std::vector<int>> dataBits;
                dataBits.resize(syms);
                for (int s = 0; s < syms; s++)
                {
                    for (int c = 0; c < nDataScs; c++) dataBits[s].push_back(rand() % mod_order);
                    std::vector<std::complex<float>> mod_data = CommsLib::modulate(dataBits[s], mod_type);
#if DEBUG_PRINT
                    std::cout << "Modulation output: "<< mod_data[0] << " " << mod_data[1] << std::endl;
#endif
                    std::vector<std::complex<float>> ofdmSym(fftSize);
                    int sc = 0;
                    for (int c = 0; c < nDataScs; c++)
                    {
                        sc = data_ind[c];
                        ofdmSym[sc] = mod_data[c];
                    }
#if DEBUG_PRINT
                    std::cout << "Data symbol: " << ofdmSym[sc-2] << " " << ofdmSym[sc-1] << std::endl;
#endif
                    for (size_t c = 0; c < pilot_sc[0].size(); c++)
                    {
                        sc = pilot_sc[0][c];
                        ofdmSym[sc] = pilot_sc[1][c];
                    }
#if DEBUG_PRINT
                    std::cout << "Pilot symbol: " << ofdmSym[pilot_sc[0][0]] << " " << ofdmSym[pilot_sc[0][1]] << std::endl;
#endif
                    std::vector<std::complex<float>> txSym = CommsLib::IFFT(ofdmSym, fftSize);
                    txSym.insert(txSym.begin(), txSym.end()-cpSize, txSym.end()); // add CP
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
            for (int i = 0; i < txdata.size(); i++)
            {
                for (int j = 0; j < txdata[i].size(); j++){
            	    std::cout << "Values["<< i <<"][" << j << "]: \t " << txdata[i][j] << std::endl;
                }
            }
            for (int i = 0; i < txdata_freq_dom.size(); i++)
            {
                for (int j = 0; j < txdata_freq_dom[i].size(); j++){
                    std::cout << "FREQ DOMAIN Values["<< i <<"][" << j << "]: \t " << txdata_freq_dom[i][j] << std::endl;
                }
            }
#endif
        }
        /**** End Signal Generation ****/
    }

    std::cout << "Configuration file was successfully parsed!" << std::endl;
}

int Config::getNumAntennas() 
{
    if (!bsPresent) 
        return 1;
    else
        return nBsSdrs[0]*bsSdrCh; 
}

Config::~Config(){}

int Config::getClientId(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % framePeriod;
    it = find(pilotSymbols[fid].begin(), pilotSymbols[fid].end(), symbol_id);
    if (it != pilotSymbols[fid].end()) 
    {
#if DEBUG_PRINT
        printf("getClientId(%d, %d) = %d\n",frame_id, symbol_id, it-pilotSymbols[fid].begin());
#endif
        return it-pilotSymbols[fid].begin();
    }else 
        return -1;
}

int Config::getUlSFIndex(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % framePeriod;
    it = find(ULSymbols[fid].begin(), ULSymbols[fid].end(), symbol_id);
    if (it != ULSymbols[fid].end()) 
    {
#if DEBUG_PRINT
        printf("getUlSFIndexId(%d, %d) = %d\n",frame_id, symbol_id, it-ULSymbols[fid].begin());
#endif
        return it-ULSymbols[fid].begin();
    }else 
        return -1;
}

int Config::getDlSFIndex(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % framePeriod;
    it = find(DLSymbols[fid].begin(), DLSymbols[fid].end(), symbol_id);
    if (it != DLSymbols[fid].end()) 
        return it-DLSymbols[fid].begin();
    else 
        return -1;
}

bool Config::isPilot(int frame_id, int symbol_id) 
{
    int fid = frame_id % framePeriod;
#if DEBUG_PRINT
    printf("isPilot(%d, %d) = %c\n",frame_id, symbol_id, frames[fid].at(symbol_id));
#endif
    return frames[fid].at(symbol_id) == 'P' ? true : false;
} 

bool Config::isData(int frame_id, int symbol_id) 
{
    int fid = frame_id % framePeriod;
#if DEBUG_PRINT
    printf("isData(%d, %d) = %c\n",frame_id, symbol_id, frames[fid].at(symbol_id));
#endif
    return frames[fid].at(symbol_id) == 'U' ? true : false;
} 


