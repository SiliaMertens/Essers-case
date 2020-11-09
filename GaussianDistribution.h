// To Avoid adding/defining multiple headers of the same concept.
#ifndef GAUSSIAN_DISTRIBUTION_H
#define GAUSSIAN_DISTRIBUTION_H

class GaussianDistribution : public ProbabilityEstimator
{
private:
    double loadMeter;
    inline double cdf(double x, double mu, double sigma);
    std::vector<double> jointGaussian(const std::vector<double> &distOne, const std::vector<double> &distTwo);
protected:
    inline double accumulateResult(const std::vector<double> &dist) override;
public:
    // default constructor
    GaussianDistribution();
    // overloaded constructor (i.e. in case different Load meter value is provided.)
    GaussianDistribution(double lm);
    ~GaussianDistribution() {};
    std::vector<double> jointCDF(const std::vector<std::vector<double>> &empricialDists) override;
    std::string classname() override;
};

#endif /* GAUSSIAN_DISTRIBUTION_H */
