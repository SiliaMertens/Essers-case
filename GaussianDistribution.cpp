#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <sstream> // std::stringstream
#include <vector>
#include <utility>   // std::pair
#include <stdexcept> // std::exceptions
#include "ProbabilityEstimator.h"
#include "GaussianDistribution.h"

#include <cmath>
#include <iomanip>
// default constructor
GaussianDistribution::GaussianDistribution() { this->loadMeter = 13.7; }
// overloaded constructor (i.e. in case different Load meter value is provided.)
GaussianDistribution::GaussianDistribution(double lm) { this->loadMeter = lm; }
inline double GaussianDistribution::cdf(double x, double mu, double sigma)
{
    return 0.5 * (1 + std::erf((x - mu) / (sigma * std::sqrt(2))));
}
inline double GaussianDistribution::accumulateResult(const std::vector<double> &dist)
{
    //std::max(double,double): to fix floating point precision issues.
    return std::max(0.0, (1 - this->cdf(this->loadMeter, dist[0], dist[1])));
}
std::vector<double> GaussianDistribution::jointGaussian(const std::vector<double> &distOne, const std::vector<double> &distTwo)
{
    double jointVariance = distOne[2] + distTwo[2];
    double jointStdv = sqrt(jointVariance);
    double jointMu = distOne[0] + distTwo[0];
    std::vector<double> result{jointMu, jointStdv, jointVariance};
    return result;
}
std::vector<double> GaussianDistribution::jointCDF(const std::vector<std::vector<double>> &empricialDists)
{
    // Should be equal to the number of clients in the current route.
    int nrOfDists = empricialDists.size();
    if (nrOfDists < 1)
    {
        throw std::invalid_argument(" GaussianDistribution::jointCDF() -> Client distributions list cannot be empty!");
    }
    // Vector represent the accumulative result of the joint CDF .
    std::vector<double> accuJoRes;
    std::vector<double> prevDist(empricialDists[0]);
    accuJoRes.push_back(this->accumulateResult(prevDist));
    for (int i = 1; i < nrOfDists; i++)
    {
        std::vector<double> currJointDist = this->jointGaussian(prevDist, empricialDists[i]);
        accuJoRes.push_back(this->accumulateResult(currJointDist));
        //Vector assigment is a deep-copy in case of vectors, not like arrays (i.e. not passed by reference).
        prevDist = currJointDist;
    }
    return accuJoRes;
}
std::string GaussianDistribution::classname()
{
    return "GaussianDistribution";
}