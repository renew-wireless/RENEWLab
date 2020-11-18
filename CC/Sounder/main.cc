/*
 Copyright (c) 2018-2020, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
 
---------------------------------------------------------------------
 main function
 - initializes all Clients
 - Brings up Recorder and the BaseStation
---------------------------------------------------------------------
*/

#include "include/recorder.h"
#include "include/signalHandler.hpp"

int main(int argc, char const* argv[])
{
    int ret;
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " CONFIG_FILE_NAME" << std::endl;
        return EXIT_FAILURE;
    }
    if (strcmp(argv[1], "--help") == 0 or strcmp(argv[1], "-h") == 0) {
        std::cerr << "Usage: " << argv[0] << " CONFIG_FILE_NAME" << std::endl;
        return EXIT_SUCCESS;
    }

    Config config(argv[1]);

    try {
        SignalHandler signalHandler;

        // Register signal handler to handle kill signal
        signalHandler.setupSignalHandlers();
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
    return ret;
}
