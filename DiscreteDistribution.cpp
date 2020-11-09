#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <sstream> // std::stringstream
#include <vector>
#include <utility>   // std::pair
#include <stdexcept> // std::exceptions
#include "ProbabilityEstimator.h"
#include "DiscreteDistribution.h"

inline double DiscreteDistribution::accumulateResult(const std::vector<double> &dist)
{
    double sum_of_elems = 0.0;
    for (auto &n : dist)
    {
        sum_of_elems += n;
    }
    //std::max(double,double): to fix floating point precision issues.
    return std::max(0.0, (1 - sum_of_elems));
}
inline double DiscreteDistribution::convolution(int v, const std::vector<double> &probabDistOne, const std::vector<double> &probabDistTwo)
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
inline std::vector<double> DiscreteDistribution::cdf(const std::vector<double> &probabDistOne, const std::vector<double> &probabDistTwo)
{
    if (probabDistOne.size() != probabDistTwo.size())
    {
        throw std::invalid_argument(
            "DiscreteDistribution::cdf() -> The probability distributions are not equal in size! " + std::to_string(probabDistOne.size()) + " != " + std::to_string(probabDistTwo.size()));
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
std::vector<double> DiscreteDistribution::jointCDF(const std::vector<std::vector<double>> &empricialDists)
{
    // Should be equal to the number of clients in the current route.
    int nrOfDists = empricialDists.size();
    if (nrOfDists < 1)
    {
        throw std::invalid_argument(" DiscreteDistribution::jointCDF() -> Client distributions list cannot be empty!");
    }
    // Vector represent the accumulative result of the joint CDF .
    std::vector<double> accuJoRes;
    std::vector<double> prevDist(empricialDists[0]);
    accuJoRes.push_back(this->accumulateResult(prevDist));
    for (int i = 1; i < nrOfDists; i++)
    {
        std::vector<double> currCdfDist = this->cdf(prevDist, empricialDists[i]);
        accuJoRes.push_back(this->accumulateResult(currCdfDist));
        //Vector assigment is a deep-copy in case of vectors, not like arrays (i.e. not passed by reference).
        prevDist = currCdfDist;
    }
    return accuJoRes;
}
std::string DiscreteDistribution::classname()
{
    return "DiscreteDistribution";
}