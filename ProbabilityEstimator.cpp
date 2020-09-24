#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <sstream> // std::stringstream
#include <vector>
#include <vector>
#include <utility>   // std::pair
#include <stdexcept> // std::exceptions
#include <map>

#include "ProbabilityEstimator.h"

inline double ProbabilityEstimator::convolution(int v, const std::vector<double> &probabDistOne, const std::vector<double> &probabDistTwo)
{
    double convSum = 0.0;
    for (int w = 1; w < v; w++)
    {
        /*
        v: can be a value between 1..b
        For example with b = 10, proba_d2[0] = P(D2=1.37)
        Lists are 0-indexed, each index (i.e. proba_d2[w - 1]) represent the probability of that interval w.
        */
        convSum += probabDistTwo[w - 1] * probabDistOne[v - w - 1];
    }

    return convSum;
}

inline std::vector<double> ProbabilityEstimator::cdf(const std::vector<double> &probabDistOne, const std::vector<double> &probabDistTwo)
{
    if (probabDistOne.size() != probabDistTwo.size())
    {
        throw std::invalid_argument(
            "ProbabilityEstimator::cdf() -> The probability distributions are not equal in size! " + std::to_string(probabDistOne.size()) + " != " + std::to_string(probabDistTwo.size()));
    }
    /*
    - P(v = 0) = 0, and start from v = 1 ... b = v + 1
    - The range of v represented here in i as the 0-index, therefore start from index 1
    - (i.e. index 1 means the conceptual b = 2)
    - b: nbinsHisto is the number of bins in histogram which is equal to the length of the client distribution.
    - The +1 in nbinsHisto, is to make sure that the last value of v = b (which will be also shifted by -1 in Cdf).
    */
    int nbinsHisto = probabDistTwo.size() + 1;
    std::vector<double> jointDist;
    for (int i = 1; i < nbinsHisto; i++)
    {
        double currentCdf = this->convolution(i, probabDistOne, probabDistTwo);
        jointDist.push_back(currentCdf);
    }
    return jointDist;
}

inline double ProbabilityEstimator::accumulateResult(const std::vector<double> &probabDist)
{
    double sum_of_elems = 0.0;
    // std::for_each(probabDist.begin(), probabDist.end(), [&](double n) { sum_of_elems += n; });
    for (auto &n : probabDist){
        sum_of_elems += n;
    }
    return 1 - sum_of_elems;
}

std::vector<double> ProbabilityEstimator::jointCDF(const std::vector<std::vector<double>> &emplDists)
{
    // Should be equal to the number of clients in the current route.
    int nrOfDists = emplDists.size();
    if (nrOfDists < 1)
    {
        throw std::invalid_argument(" ProbabilityEstimator::jointCDF() -> Client distributions list cannot be empty!");
    }
    // Vector represent the accumulative result of the joint CDF .
    std::vector<double> accuJoRes;
    std::vector<double> prevDist(emplDists[0]);
    accuJoRes.push_back(this->accumulateResult(prevDist));
    for (int i = 1; i < nrOfDists; i++)
    {
        std::vector<double> currCdfDist = this->cdf(prevDist, emplDists[i]);
        accuJoRes.push_back(this->accumulateResult(currCdfDist));
        //Vector assigment is a deep-copy in case of vectors, not like arrays (i.e. not passed by reference).
        prevDist = currCdfDist;
    }
    return accuJoRes;
}

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
            throw std::runtime_error("ProbabilityEstimator::getEmpricialDistributions() -> Could not find a probability distribution for client-id: " + std::to_string(id) + " --- Current Dist-Vector Size is: " + std::to_string(this->currCollDateVector.size()));
        }
        resultDistros.push_back(current_distro);
        current_distro.clear();
    }
    return resultDistros;
}
//will replace the above...
void ProbabilityEstimator::readDistributions(const std::string &collection_date, const std::string &probabilityResolution)
{

    std::cout << "Read distributions (will replace the above) \n";
    // Clear the current distributions (i.e. distributions of the previous date)
    this->currCollDateVector.clear();

    std::ifstream fin;
    std::vector<double> cDist;
    std::string line, key, value;
    // Open an existing file of a specific date.
    fin.open(collection_date + "_" + probabilityResolution + ".csv", std::ios::in);
    if (fin.is_open())
    {
        // read an entire row and store it in a string variable 'line'
        while (std::getline(fin, line))
        {
            // used for streaming the line into words
            std::stringstream cstr(line);
            //Read the first column of the line
            //This represents the client id
            //TODO: remove order_number and change the way files are generated.
            std::getline(cstr, key, ',');
            while (std::getline(cstr, value, ','))
            {
                try
                {
                    cDist.push_back(std::stod(value));
                }
                catch (const std::exception &e)
                {
                    throw std::invalid_argument("ProbabilityEstimator.readDistributions() -> Value is not converted to double or does not exist in the file --> Value: " + value);
                }
            }
            this->currCollDateVector.push_back(cDist);
            //Clear the vector to store new values
            cDist.clear();
        }
    }
    else
    {
        throw std::runtime_error("Failed to read file '" + collection_date + "_" + probabilityResolution + ".csv" + "'\n");
    }
    fin.close();
}
