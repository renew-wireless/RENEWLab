#! usr/bin/python3
"""
Generates CSV file containing information on Iris power output using various transmission settings.

Make sure the correct serial is used at beginning of main()

Ensure that SISO_TX.py is running properly before starting this code.

Currently relying on having the Spike software running on a PC wired to this one.

The Spectrum Analyzer (SA) uses SCPI communication protocol, and the pyVisa/pyVisa-py modules support this, but
the Signal Hound SA44B we are using has quite limited functionality, so the only functions we can really utilize are
the purest read_raw()/write() methods to interact with it, and the strings passed to it are simply SCPI format.

See the Spike SCPI Programming Manual for more info.

Usage example:

Author: Tristan Mansfield
"""


from help_funcs import *

def main():
    ###long=True if you want avg, max, min, and raw ch. power, False for only raw ch. power (False results in much faster runtime)
    long = True

    sdr = SoapySDR.Device(dict(serial="RF3C000064"))

    ###Initializes SA device, see docstring for details
    rm = visa.ResourceManager()

    ###The IP address here must match the Windows machine running Spike.
    device = rm.open_resource("TCPIP::192.168.0.5::5025::SOCKET")

    ###Allows the program to interpret the end of data returned, so read_raw() works properly
    device.read_termination = '\n'
    device.write_termination = '\n'

    if long:
        big_data = pd.DataFrame(columns=['freq', 'rate', 'ant', 'TSP_const', 'PAD', 'IAMP', 'PA1', 'PA2', 'PA3', 'LMS7', 'ZYNQ', 'Frontendtemp', 'Raw C&W Channel Power', 'Max Ch. Power', 'Min Ch. Power', 'Avg. Ch. Power'])
        ###Note that since less initialization actually happens in initialize2, more must happen in the function.
        initialize2(device)
        for freq in freqlst:
            ###Update this here only once to avoid re-writing same freq multiple times.
            device.write(":sense:freq:center " + str(freq))
            for rate in ratelst:
                for ant in antlst:
                    ###Default here is 0.5
                    for TSP in TSP_TSG_CONST_lst:
                        for PAD in PAD_lst:
                            for IAMP in IAMP_lst:
                                #TODO: Don't set the list to PA*lst until Oscar explicitly says to
                                for PA1 in [0]:
                                    for PA2 in [0]:
                                        for PA3 in [0]:
                                            # With all of these settings, want to update the transmission, then write to the
                                            # SA, ensure that everything has updated, then take measurement
                                            change(rate=rate, ampl=TSP, ant=ant, gain=PAD, freq=freq, PA2=PA2, IAMP=IAMP)

                                            # The below section simply writes the SCPI strings to the SA to read from traces 1, 2, 3, 6 and
                                            # record their values.
                                            ###Raw C&W
                                            device.write(":sense:chpower:trace 1")
                                            one_sweep(device)
                                            device.write(":sense:chpower:chpower? 0")
                                            cwchpower = float(device.read_raw())

                                            # So that Max/Min/Avg have higher sample size, the larger the range, the more sweeps that will be
                                            # considered, but increases runtime substantially.
                                            for num in range(5):
                                                one_sweep(device)

                                            ###Min
                                            device.write(":sense:chpower:trace 2")
                                            one_sweep(device)
                                            device.write(":sense:chpower:chpower? 0")
                                            minchpower = float(device.read_raw())

                                            ###Max
                                            device.write(":sense:chpower:trace 3")
                                            one_sweep(device)
                                            device.write(":sense:chpower:chpower? 0")
                                            maxchpower = float(device.read_raw())

                                            ###Avg
                                            device.write(":sense:chpower:trace 6")
                                            one_sweep(device)
                                            device.write(":sense:chpower:chpower? 0")
                                            avgchpower = float(device.read_raw())

                                            ###Clearing all traces for next setting iteration
                                            device.write(":trace:clear:all")

                                            ###Take temp. measurements
                                            LMS7 = sdr.readSensor("LMS7_TEMP")
                                            ZYNQ = sdr.readSensor("ZYNQ_TEMP")
                                            front = sdr.readSensor("FE_TEMP")

                                            add = pd.Series(
                                                [freq, rate, ant, TSP, PAD, IAMP, PA1, PA2, PA3, LMS7, ZYNQ, front,
                                                 cwchpower, maxchpower, minchpower, avgchpower],
                                                index=['freq', 'rate', 'ant', 'TSP_const', 'PAD', 'IAMP', 'PA1', 'PA2', 'PA3', 'LMS7', 'ZYNQ', 'Frontendtemp', 'Raw C&W Channel Power', 'Max Ch. Power', 'Min Ch. Power', 'Avg. Ch. Power'])
                                            big_data = big_data.append(add, ignore_index=True, sort=False)
    else:
        big_data = pd.DataFrame(columns=['freq', 'rate', 'ant', 'TSP_const', 'PAD', 'IAMP', 'PA1', 'PA2', 'PA3', 'LMS7', 'ZYNQ', 'Frontendtemp', 'Channel Power'])
        initialize(device)
        for freq in freqlst:
            ###Update this here only once to avoid re-writing same freq multiple times.
            device.write(":sense:freq:center "+ str(freq))
            for rate in ratelst:
                for ant in antlst:
                    ###Default here is 0.5
                    for TSP in TSP_TSG_CONST_lst:
                        for PAD in PAD_lst:
                            for IAMP in IAMP_lst:
                                #TODO: Don't set the list to PA*lst until Oscar explicitly says to
                                for PA1 in [0]:
                                    for PA2 in [0]:
                                        for PA3 in [0]:
                                            # With all of these settings, want to update the transmission, then write to the
                                            # SA, ensure that everything has updated, then take measurement
                                            change(rate=rate, ampl=TSP, ant=ant, gain=PAD, freq=freq, PA2=PA2, IAMP=IAMP)

                                            ###Updates the trace and takes the measurement on maxhold (trace 6, as specified by initialize())
                                            one_sweep(device)
                                            device.write(":sense:chpower:chpower? 0")
                                            chpower = device.read_raw()

                                            ###Take temp. measurements
                                            LMS7=sdr.readSensor("LMS7_TEMP")
                                            ZYNQ=sdr.readSensor("ZYNQ_TEMP")
                                            front=sdr.readSensor("FE_TEMP")

                                            # print(freq, rate, ant, TSP, PAD, IAMP, PA1, PA2, PA3, LMS7, ZYNQ, front, chpower)

                                            add = pd.Series([freq, rate, ant, TSP, PAD, IAMP, PA1, PA2, PA3, LMS7, ZYNQ, front, chpower],
                                                            index=['freq', 'rate', 'ant', 'TSP_const', 'PAD', 'IAMP', 'PA1', 'PA2', 'PA3', 'LMS7', 'ZYNQ', 'Frontendtemp', 'Channel Power'])
                                            big_data=big_data.append(add, ignore_index=True, sort=False)

    big_data.to_csv("big_data_out.csv")

if __name__ == '__main__':
    main()
