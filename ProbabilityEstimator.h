// To Avoid adding/defining multiple headers of the same concept.
#ifndef PROBABILITY_OF_FAILURE_H
#define PROBABILITY_OF_FAILURE_H
#include <map>

class ProbabilityEstimator
{
private:
    std::vector<std::vector<double>> currCollDateVector;
    double accumulateResult(const std::vector<double> &probabDist);
    double convolution(int v, const std::vector<double> &probabDistOne, const std::vector<double> &probabDistTwo);
    std::vector<double> cdf(const std::vector<double> &probabDistOne, const std::vector<double> &probabDistTwo);

public:
    ProbabilityEstimator(){};
    std::vector<double> jointCDF(const std::vector<std::vector<double>> &empricialDistros);
    std::vector<std::vector<double>> getEmpricialDistributions(std::vector<int> &clientsIDs);
    void readDistributions(const std::string &collection_date, const std::string &probabilityResolution);
};

#endif /* PROBABILITY_OF_FAILURE_H */
