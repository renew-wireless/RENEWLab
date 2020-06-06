#include "comms-lib.h"
#include "macros.h"
#include "utils.h"
#include <atomic>
#include <random>
#include <unistd.h>
int main(int argc, char const* argv[])
{

    /*
     * test abs2_avx
     */
    std::vector<std::complex<float>> testDataCF32;
    testDataCF32.push_back(std::complex<float>(.1, .2));
    testDataCF32.push_back(std::complex<float>(.2, .3));
    testDataCF32.push_back(std::complex<float>(.3, -.4));
    testDataCF32.push_back(std::complex<float>(.4, .5));
    testDataCF32.push_back(std::complex<float>(.5, .6));
    testDataCF32.push_back(std::complex<float>(-.6, .7));
    testDataCF32.push_back(std::complex<float>(.7, .8));
    testDataCF32.push_back(std::complex<float>(.8, .9));

    testDataCF32.push_back(std::complex<float>(.9, .0));
    testDataCF32.push_back(std::complex<float>(.1, .2));
    testDataCF32.push_back(std::complex<float>(.2, .3));
    testDataCF32.push_back(std::complex<float>(.3, -.4));
    testDataCF32.push_back(std::complex<float>(.4, .5));
    testDataCF32.push_back(std::complex<float>(.5, .6));
    testDataCF32.push_back(std::complex<float>(-.6, .7));
    testDataCF32.push_back(std::complex<float>(.7, .8));

    testDataCF32.push_back(std::complex<float>(.8, .9));
    testDataCF32.push_back(std::complex<float>(.9, .0));
    testDataCF32.push_back(std::complex<float>(.1, .2));
    testDataCF32.push_back(std::complex<float>(.2, .3));
    testDataCF32.push_back(std::complex<float>(.3, -.4));
    testDataCF32.push_back(std::complex<float>(.4, .5));
    testDataCF32.push_back(std::complex<float>(.5, .6));
    testDataCF32.push_back(std::complex<float>(-.6, .7));

    testDataCF32.push_back(std::complex<float>(.7, .8));
    testDataCF32.push_back(std::complex<float>(.8, .9));
    testDataCF32.push_back(std::complex<float>(.9, .0));
    std::vector<std::complex<int16_t>> testDataCS16;
    for (int i = 0; i < testDataCF32.size(); i++)
        testDataCS16.push_back(std::complex<int16_t>((int16_t)(testDataCF32[i].real() * 32768), (int16_t)(testDataCF32[i].imag() * 32768)));

    auto testAbs0 = CommsLib::abs2_avx(testDataCF32);
    auto testAbs1 = CommsLib::abs2_avx(testDataCS16);
    bool pass = true;
    std::cout << "\nTesting abs_avx (floating-point):\n";
    for (size_t i = 0; i < testAbs0.size(); i++) {
	std::complex<float> gt = (testDataCF32[i] * std::conj(testDataCF32[i]));
        //std::cout << "ABS Float RESULT: " << testAbs0[i] << ", GT: " << gt << std::endl;
	if (std::abs(testAbs0[i] - gt) > 0.01)
            pass = false;
    }
    std::cout << (pass ? "PASSED" : "FAILED") << std::endl;

    pass = true;
    std::cout << "\nTesting abs_avx (fixed-point):\n";
    for (size_t i = 0; i < testAbs0.size(); i++) {
	std::complex<float> gt = (testDataCF32[i] * std::conj(testDataCF32[i]));
	std::complex<double> gt_double(gt.real(), gt.imag());
        //std::cout << "ABS INT RESULT: " << testAbs1[i] / (std::pow(2.0, 30)) << ", GT: " << gt << std::endl;
	if (std::abs((testAbs1[i] / (std::pow(2.0, 30))) - gt_double) > 0.01)
            pass = false;
    }
    std::cout << (pass ? "PASSED" : "FAILED") << std::endl;

    /*
     * test complex_mult_cs16
     */
    std::cout << "\nTesting Complex Mult" << std::endl;
    auto testCS2 = CommsLib::complex_mult_avx(testDataCS16, testDataCS16, true);
    auto testCF2 = CommsLib::complex_mult_avx(testDataCF32, testDataCF32, true);
    for (int i = 0; i < testCS2.size(); i++)
        std::cout << "Conj True - CS Result: (" << testCS2[i].real() / 32768.0 << "," << testCS2[i].imag() / 32768.0 << "), CF Result: " << testCF2[i] << std::endl;

    testCS2 = CommsLib::complex_mult_avx(testDataCS16, testDataCS16, false);
    testCF2 = CommsLib::complex_mult_avx(testDataCF32, testDataCF32, false);
    for (int i = 0; i < testCS2.size(); i++)
        std::cout << "Conj False - CS Result: (" << testCS2[i].real() / 32768.0 << "," << testCS2[i].imag() / 32768.0 << "), CF Result: " << testCF2[i] << std::endl;

    /*
     * test mult_avx
     */
    int dly = 0;
    bool conj = true;
    auto testMultRes = CommsLib::auto_corr_mult_avx(testDataCF32, dly, conj);
    auto testMultResCS = CommsLib::auto_corr_mult_avx(testDataCS16, dly, conj);
    std::cout << "\nTesting auto_corr_mult_avx (dly = " << dly << ", conj = " << (conj ? "true" : "false")
              << "):\n";
    for (size_t i = 0; i < testMultRes.size(); i++) {
        if (i < dly)
            std::cout << "Float Op Result: " << testMultRes[i] << ", INT Op Result: (" << testMultResCS[i].real() / 32768.0 << "," << testMultResCS[i].imag() / 32668.0 << "), GT: (0, 0)" << std::endl;
        else
            std::cout << "Float Op Result: " << testMultRes[i] << ", INT Op Result: (" << testMultResCS[i].real() / 32768.0 << "," << testMultResCS[i].imag() / 32668.0 << "), GT:" << testDataCF32[i] * (conj ? std::conj(testDataCF32[i - dly]) : testDataCF32[i - dly]) << std::endl;
    }

    conj = false;
    testMultRes = CommsLib::auto_corr_mult_avx(testDataCF32, dly, conj);
    testMultResCS = CommsLib::auto_corr_mult_avx(testDataCS16, dly, conj);
    std::cout << "\nTesting auto_corr_mult_avx (dly = " << dly << ", conj = " << (conj ? "true" : "false")
              << "):\n";
    for (size_t i = 0; i < testMultRes.size(); i++) {
        if (i < dly)
            std::cout << "Float Op Result: " << testMultRes[i] << ", INT Op Result: (" << testMultResCS[i].real() / 32768.0 << "," << testMultResCS[i].imag() / 32668.0 << "), GT: (0, 0)" << std::endl;
        else
            std::cout << "Float Op Result: " << testMultRes[i] << ", INT Op Result: (" << testMultResCS[i].real() / 32768.0 << "," << testMultResCS[i].imag() / 32668.0 << "), GT:" << testDataCF32[i] * (conj ? std::conj(testDataCF32[i - dly]) : testDataCF32[i - dly]) << std::endl;
    }

    dly = 1;
    conj = true;
    testMultRes = CommsLib::auto_corr_mult_avx(testDataCF32, dly, conj);
    testMultResCS = CommsLib::auto_corr_mult_avx(testDataCS16, dly, conj);
    std::cout << "\nTesting auto_corr_mult_avx (dly = " << dly << ", conj = " << (conj ? "true" : "false")
              << "):\n";
    for (size_t i = 0; i < testMultRes.size(); i++) {
        if (i < dly)
            std::cout << "Float Op Result: " << testMultRes[i] << ", INT Op Result: (" << testMultResCS[i].real() / 32768.0 << "," << testMultResCS[i].imag() / 32668.0 << "), GT: (0, 0)" << std::endl;
        else
            std::cout << "Float Op Result: " << testMultRes[i] << ", INT Op Result: (" << testMultResCS[i].real() / 32768.0 << "," << testMultResCS[i].imag() / 32668.0 << "), GT:" << testDataCF32[i] * (conj ? std::conj(testDataCF32[i - dly]) : testDataCF32[i - dly]) << std::endl;
    }

    dly = 2;
    conj = false;
    testMultRes = CommsLib::auto_corr_mult_avx(testDataCF32, dly, conj);
    testMultResCS = CommsLib::auto_corr_mult_avx(testDataCS16, dly, conj);
    std::cout << "\nTesting auto_corr_mult_avx (dly = " << dly << ", conj = " << (conj ? "true" : "false")
              << "):\n";
    for (size_t i = 0; i < testMultRes.size(); i++) {
        if (i < dly)
            std::cout << "Float Op Result: " << testMultRes[i] << ", INT Op Result: (" << testMultResCS[i].real() / 32768.0 << "," << testMultResCS[i].imag() / 32668.0 << "), GT: (0, 0)" << std::endl;
        else
            std::cout << "Float Op Result: " << testMultRes[i] << ", INT Op Result: (" << testMultResCS[i].real() / 32768.0 << "," << testMultResCS[i].imag() / 32668.0 << "), GT:" << testDataCF32[i] * (conj ? std::conj(testDataCF32[i - dly]) : testDataCF32[i - dly]) << std::endl;
    }

    /*
     * test correlate
     */
    std::cout << "\nTesting Complex Correlate:\n";
    std::vector<std::complex<float>> testCorrSeqCF32;
    testCorrSeqCF32.push_back(std::complex<float>(.3, -.3));
    testCorrSeqCF32.push_back(std::complex<float>(.4, .2));
    testCorrSeqCF32.push_back(std::complex<float>(.5, -.2));
    testCorrSeqCF32.push_back(std::complex<float>(.1, .1));
    testCorrSeqCF32.push_back(std::complex<float>(.3, -.3));
    testCorrSeqCF32.push_back(std::complex<float>(.4, .2));
    testCorrSeqCF32.push_back(std::complex<float>(.5, -.2));
    testCorrSeqCF32.push_back(std::complex<float>(.1, .1));

    std::vector<std::complex<int16_t>> testCorrSeqCS16;
    for (int i = 0; i < testCorrSeqCF32.size(); i++)
        testCorrSeqCS16.push_back(std::complex<int16_t>((int16_t)(testCorrSeqCF32[i].real() * 32768), (int16_t)(testCorrSeqCF32[i].imag() * 32768)));

    auto testCorrResCF32 = CommsLib::correlate_avx(testDataCF32, testCorrSeqCF32);
    auto testCorrResCS16 = CommsLib::correlate_avx(testDataCS16, testCorrSeqCS16);

    //std::vector<std::complex<double>> testCorrSeq2;
    //std::vector<std::complex<double>> iq;
    //for (size_t i = 0; i < testCorrSeqCF32.size(); i++)
    //    testCorrSeq2.push_back(std::conj(testCorrSeqCF32[testCorrSeqCF32.size() - i]));
    //for (size_t i = 0; i < testDataCF32.size(); i++) {
    //    iq.push_back(std::complex<double>(testDataCF32[i].real(), testDataCF32[i].imag()));
    //}
    //auto testCorrRef = CommsLib::convolve(iq, testCorrSeq2);
    std::vector<std::complex<float>> testCorrRef;
    std::vector<std::complex<float>> testDataExtend(testCorrSeqCF32.size() - 1, 0);
    testDataExtend.insert(testDataExtend.end(), testDataCF32.begin(), testDataCF32.end());
    for (size_t i = 0; i < testDataExtend.size(); i++) {
        std::complex<float> acc(0);
        for (size_t j = 0; j < testCorrSeqCF32.size(); j++) {
            if (i + j < testDataExtend.size())
                acc += testDataExtend[i + j] * std::conj(testCorrSeqCF32[j]);
            else
                break;
        }
        testCorrRef.push_back(acc);
    }
    for (size_t i = 0; i < testCorrResCF32.size(); i++) {
        //std::cout << testCorrResCF32[i] << ", GT:" << testCorrRef[i] << std::endl;
        std::cout << "Float Op Result: " << testCorrResCF32[i] << ", INT Op Result: (" << testCorrResCS16[i].real() / 32768.0 << "," << testCorrResCS16[i].imag() / 32768.0 << "), GT:" << testCorrRef[i] << std::endl;
    }

    std::cout << "\nTesting Real Correlate:\n";
    std::vector<float> testCorrSeqFloat(7, 1.0);
    std::vector<int16_t> testCorrSeqInt(7, 1);
    std::vector<int16_t> testAbsInt;
    for (size_t i = 0; i < testAbs0.size(); i++)
        testAbsInt.push_back(std::abs((int16_t)(testAbs0[i] * 32768)));
    auto testCorrResFloat = CommsLib::correlate_avx_s(testAbs0, testCorrSeqFloat);
    auto testCorrResInt = CommsLib::correlate_avx_si(testAbsInt, testCorrSeqInt);
    std::vector<float> testCorrRefReal;
    std::vector<float> testDataExtend1(testCorrSeqFloat.size() - 1, 0);
    testDataExtend1.insert(testDataExtend1.end(), testAbs0.begin(), testAbs0.end());
    for (size_t i = 0; i < testDataExtend1.size(); i++) {
        float acc(0);
        for (size_t j = 0; j < testCorrSeqFloat.size(); j++) {
            if (i + j < testDataExtend1.size())
                acc += testDataExtend1[i + j] * testCorrSeqFloat[j];
            else
                break;
        }
        testCorrRefReal.push_back(acc);
    }
    for (size_t i = 0; i < testCorrResFloat.size(); i++) {
        std::cout << "Float Op Result: " << testCorrResFloat[i] << ", INT Op Result: (" << testCorrResInt[i] / 32768.0 << "), GT:" << testCorrRefReal[i] << std::endl;
    }
    exit(0);
    /*
     * test findBeacon
     */
    std::cout << "Testing findBeaconGold:\n";

    // generate beacon
    std::vector<std::vector<double>> gold_ifft = CommsLib::getSequence(128, CommsLib::GOLD_IFFT);
    std::vector<std::complex<int16_t>> gold_ifft_ci16 = Utils::double_to_cint16(gold_ifft);

    int goldReps = 2;
    std::vector<std::complex<int16_t>> beacon_ci16;
    for (int i = 0; i < goldReps; i++) {
        beacon_ci16.insert(beacon_ci16.end(), gold_ifft_ci16.begin(), gold_ifft_ci16.end());
    }

    size_t beaconSize = beacon_ci16.size();

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(.0, 0.002);

    size_t symbolsPerFrame = 20;
    size_t sampsPerSymbol = 1024 + 150;
    size_t SYNC_NUM_SAMPS = sampsPerSymbol * symbolsPerFrame;
    std::vector<std::complex<float>> buffs(SYNC_NUM_SAMPS, 0);

    //FILE* fpi = fopen("input_vector_i.m", "w+");
    //FILE* fpq = fopen("input_vector_q.m", "w+");
    //fprintf(fpi, "rx_sig_i=[");
    //fprintf(fpq, "rx_sig_q=[");

    size_t prefix = 501;
    for (size_t i = 0; i < prefix; i++)
        buffs[i] = std::complex<float>((float)(distribution(generator)),
            (float)(distribution(generator)));
    for (size_t i = 0; i < beaconSize; i++)
        buffs[i + prefix] = std::complex<float>((float)(beacon_ci16[i].real() / 32768.0 / 2 + distribution(generator)),
            (float)(beacon_ci16[i].imag() / 32768.0 / 2 + distribution(generator)));
    for (size_t i = prefix + beaconSize; i < SYNC_NUM_SAMPS; i++)
        buffs[i] = std::complex<float>((float)(distribution(generator)),
            (float)(distribution(generator)));

    //for (size_t i = 0; i < SYNC_NUM_SAMPS; i++) {
    //    fprintf(fpi, "%1.4f ", buffs[i].real());
    //    fprintf(fpq, "%1.4f ", buffs[i].imag());
    //}
    //fprintf(fpi, "];");
    //fprintf(fpq, "];");
    //fclose(fpi);
    //fclose(fpq);

    struct timespec tv, tv2;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    int sync_index = CommsLib::findBeaconGold(buffs);
    clock_gettime(CLOCK_MONOTONIC, &tv2);
    double diff = ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec)) / 1e3;
    std::cout << "SYNC Found at index " << sync_index << std::endl;
    std::cout << "Correlation took " << diff << " usec" << std::endl;
    return 0;
}
