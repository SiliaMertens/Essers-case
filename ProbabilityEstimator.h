// To Avoid adding/defining multiple headers of the same concept.
#ifndef PROBABILITY_OF_FAILURE_H
#define PROBABILITY_OF_FAILURE_H

class ProbabilityEstimator
{
protected:
    std::vector<std::vector<double>> currCollDateVector;

public:
    virtual ~ProbabilityEstimator() {};
    virtual std::vector<double> jointCDF(const std::vector<std::vector<double>> &empricialDists) = 0;
    std::vector<std::vector<double>> getEmpricialDistributions(std::vector<int> &clientsIDs);
    void readDistributions(const std::string &collection_date, const std::string &resolution);
    virtual std::string classname() = 0;
};

#endif /* PROBABILITY_OF_FAILURE_H */
