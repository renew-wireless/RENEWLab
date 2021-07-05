/*
 Copyright (c) 2018-2021, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
 
---------------------------------------------------------------------
 main function
 - initializes all Clients
 - Brings up Recorder and the BaseStation
---------------------------------------------------------------------
*/

#include "include/data_generator.h"
#include "include/recorder.h"
#include "include/signalHandler.hpp"
#include "include/version_config.h"
#include <gflags/gflags.h>

DEFINE_bool(gen_ul_bits, false,
    "Generate random bits for uplink transmissions, otherwise read from file!");
DEFINE_string(conf, "files/conf.json", "JSON configuration file name");
DEFINE_string(storepath, "logs", "Dataset store path");
DEFINE_bool(bs_only, false, "Run BS only");
DEFINE_bool(client_only, false, "Run client only");

int main(int argc, char* argv[])
{
    gflags::SetVersionString(GetSounderProjectVersion());
    gflags::SetUsageMessage(
        "sounder Options: -bs_only -client_only -conf -gen_ul_bits -storepath");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    Config config(
        FLAGS_conf, FLAGS_storepath, FLAGS_bs_only, FLAGS_client_only);
    int ret = EXIT_SUCCESS;
    if (FLAGS_gen_ul_bits) {
        DataGenerator dg(&config);
        dg.GenerateData(FLAGS_storepath);
    } else {
        try {
            SignalHandler signalHandler;

            // Register signal handler to handle kill signal
            signalHandler.setupSignalHandlers();
            config.loadULData(FLAGS_storepath);
            Sounder::Recorder dr(&config);
            dr.do_it();
            ret = EXIT_SUCCESS;
        } catch (SignalException& e) {
            std::cerr << "SignalException: " << e.what() << std::endl;
            ret = EXIT_FAILURE;
        } catch (const std::exception& exc) {
            std::cerr << "Program terminated Exception: " << exc.what()
                      << std::endl;
            ret = EXIT_FAILURE;
        }
    }
    gflags::ShutDownCommandLineFlags();
    return ret;
}
