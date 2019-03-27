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
//freq(0),rate(0),prefix(0),postfix(0),sampsPerSymbol(0),fftSize(0),cpSize(0)
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
        json bs_sdr_ids = tddConf.value("sdr_id", json::array());
        nCells = bs_sdr_ids.size();
        for (int i = 0; i < nCells; i++) bs_sdr_file.push_back(bs_sdr_ids.at(i).get<std::string>());
        bsSdrCh = tddConf.value("polarization", "single") == "single"? 1 : 2;
        auto jBsFrames = tddConf.value("frame_schedule", json::array());
        framePeriod = jBsFrames.size();
        for(int f = 0; f < framePeriod; f++) frames.push_back(jBsFrames.at(f).get<std::string>());
        txgainA = tddConf.value("txgainA", 20);
        rxgainA = tddConf.value("rxgainA", 20);
        txgainB = tddConf.value("txgainB", 20);
        rxgainB = tddConf.value("rxgainB", 20);
        beamsweep = tddConf.value("beamsweep", false);
        beacon_ant = tddConf.value("beacon_antenna", 0);
    }

    // Clients
    if (clPresent)
    {
        auto jClSdrs = tddConfCl.value("sdr_id", json::array());
        nClSdrs = jClSdrs.size();
        for (int i = 0; i < nClSdrs; i++) cl_sdr_ids.push_back(jClSdrs.at(i).get<std::string>());
	clSdrCh = tddConfCl.value("polarization", "single") == "single"? 1 : 2;
	clAgcEn = tddConfCl.value("agc_en", false);
	clDataMod = tddConfCl.value("modulation", "QPSK");
        clTxgainA = tddConfCl.value("txgainA", 20);
        clRxgainA = tddConfCl.value("rxgainA", 20);
        clTxgainB = tddConfCl.value("txgainB", 20);
        clRxgainB = tddConfCl.value("rxgainB", 20);
        auto jClFrames = tddConfCl.value("frame_schedule", json::array());
        assert(nClSdrs == jClFrame.size());
        for(int f = 0; f < nClSdrs; f++) clFrames.push_back(jClFrames.at(f).get<std::string>());

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
        }

	// ****** OBCH *******
	if (fftSize != 64) fftSize = 64;
	data_ind = CommsLib::getDataSc(fftSize);
        pilot_sc = CommsLib::getPilotSc(fftSize);
	if (pilot_seq.compare("lts") == 0) {
	    pilot_double = CommsLib::getSequence(160, CommsLib::LTS_SEQ);
	} else {
	    throw std::invalid_argument( "Only LTS pilots currently supported" );
	}
	std::vector<std::vector<std::complex<float>>> txdata_freq_dom_conf = txdata_freq_dom;
	// ****** END OBCH *******

    }

    if (!clPresent) nClSdrs = std::count(frames.at(0).begin(), frames.at(0).end(), 'P');

    if (bsPresent)
    {
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
    }
    else if (clPresent)
        symbolsPerFrame = clFrames.at(0).size();

    std::cout << "Configuration file was successfully parsed!" << std::endl;
}

int Config::getNumAntennas() 
{ 
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


