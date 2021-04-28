#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <sstream> // std::stringstream
#include <vector>
#include <utility>   // std::pair
#include <stdexcept> // std::exceptions
#include "ProbabilityEstimator.h"

std::vector<std::vector<double>> ProbabilityEstimator::getEmpricialDistributions(std::vector<int> &clientsIDs)
{
    std::vector<std::vector<double>> resultDistros;
    //std::cout << "resultDistros " << resultDistros.size() << "\n";
    if (clientsIDs.size() < 1)
    {
        throw std::invalid_argument("ProbabilityEstimator::getEmpricialDistributions() --> Client IDs list cannot be empty!");
    }
    std::vector<double> current_distro;
    for (const auto &id : clientsIDs)
    {
        try
        {
            //LookUp an Id (i.e. id - 1 because clients starts from 1-index while vector is 0-index)
            current_distro = this->currCollDateVector.at((id - 1));
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            throw std::runtime_error("ProbabilityEstimator::getEmpricialDistributions() -> Could not find a probability distribution for client-id: " + std::to_string(id) + " --- Current Index conversion (0-index to 1-index) is: " + std::to_string(id - 1) + " --- Current Dist-Vector Size is: " + std::to_string(this->currCollDateVector.size()));
        }
        resultDistros.push_back(current_distro);
        current_distro.clear();
    }
    return resultDistros;
}

void ProbabilityEstimator::readDistributions(const std::string &fname)
{

    std::cout << "Reading distributions file "<< fname << " \n";
    // Clear the current distributions (i.e. distributions of the previous date)
    this->currCollDateVector.clear();

    // file format is: <order_id, given_demand, cDist...>
    // in case of 'gaussian', cDist = [mean, variance]
    // in case of 'discrete', cDist = [...] corresponding to a binning of 0..13.7 into given nr of bins
    unsigned int order_id;
    double given_demand;
    std::vector<double> cDist;
    
    int i = 0;
    std::ifstream fin;
    std::string line, word;
    // Open an existing file of a specific date.
    fin.open(fname, std::ios::in);
    if (fin.is_open())
    {
        // read an entire row and store it in a string variable 'line'
        while (std::getline(fin, line))
        {
            // used for streaming the line into words
            std::stringstream cstr(line);
            
            // read first two entries
            std::getline(cstr, word, ',');
            order_id = stoul(word);
            std::getline(cstr, word, ',');
            given_demand = stod(word);
            
            // TODO SILIA, ensure that in problem object, stop[i] has 'order_id' and 'given_demand' matching!!!
            unsigned int p_order_id = order_id; // p.smth[i].smth
            double p_given_demand = given_demand; // p.smth[i].smth
            if ((order_id != p_order_id) or (given_demand != p_given_demand)) {
                // if not: we are exporting data differently, and any results are meaningless
                throw std::runtime_error("Mismatch in data detected for customer "+std::to_string(i));
            }
            
            
            // read rest into cdist 
            while (std::getline(cstr, word, ','))
            {
                try
                {
                    cDist.push_back(std::stod(word));
                }
                catch (const std::exception &e)
                {
                    throw std::invalid_argument("ProbabilityEstimator.readDistributions() -> Value is not converted to double or does not exist in the file --> Value: " + word);
                }
            }
            this->currCollDateVector.push_back(cDist);
            i = i + 1;
            //Clear the vector to store new values
            cDist.clear();
        }
    }
    else
    {
        throw std::runtime_error("ProbabilityEstimator.readDistributions() -> Failed to read file '"+fname+"'\n");
    }
    fin.close();
}
