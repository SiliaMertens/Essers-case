// To Avoid adding/defining multiple headers of the same concept.
#ifndef DISCRETE_DISTRIBUTION_H
#define DISCRETE_DISTRIBUTION_H

class DiscreteDistribution : public ProbabilityEstimator
{
private:
    double convolution(int v, const std::vector<double> &probabDistOne, const std::vector<double> &probabDistTwo);
    std::vector<double> cdf(const std::vector<double> &probabDistOne, const std::vector<double> &probabDistTwo);

protected:
    inline double accumulateResult(const std::vector<double> &dist) override;

public:
    DiscreteDistribution(){};
    ~DiscreteDistribution(){};
    std::vector<double> jointCDF(const std::vector<std::vector<double>> &empricialDists) override;
    std::string classname() override;
};

#endif /* DISCRETE_DISTRIBUTION_H */
