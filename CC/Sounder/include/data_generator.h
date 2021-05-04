/**
 * @file data_generator.h
 * @brief Implementation file for the Data generator class to generate binary
 * files as inputs to Agora, sender and correctness tests
 */
#ifndef DATA_GENERATOR_H_
#define DATA_GENERATOR_H_

#include "config.h"
#include <random>
#include <string>

class DataGenerator {
public:
    // The profile of the input information bits
    explicit DataGenerator(Config* cfg)
        : cfg_(cfg)
    {
    }

    void GenerateData(const std::string& directory);

private:
    Config* cfg_;
};
#endif
