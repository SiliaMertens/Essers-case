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
inline double DiscreteDistribution::convolution(int v, const std::vector<double> &probabDistOne, const std::vector<double> &probabDistTwo, const int l2, const int u2)
{
    // probabilities of bins reaching unit value 'v':
    // 0+v, 1+(v-1), ..., v+0
    // but index '0' is value '1'!
    // so skip 0+v and v+0: 1+(v-1), 2+(v-2), ..., (v-1)+1   and then from value to index, another -1
    double convSum = 0.0;
    // now, we assume 'Two' is the shortest, and we range only from min/max of that
    for (int w = std::max(1,l2+1); w <= std::min(v-1,u2+1); w++)
    {
        /*
        v: can be a value between 1..b
        For example with b = 10, proba_d2[0] = P(D2='1 unit') = P(D2=1.37)
        Lists are 0-indexed, each index (i.e. proba_d2[w - 1]) represent the probability of that interval w.
        */
        convSum += probabDistTwo[(w)-1] * probabDistOne[(v - w)-1];
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
    - P(v = 0) = 0, and so the probab array starts from v = 1 ... b
    - so, at the '0'th index, it represents the probability of 1 unit, at the kth index, the probability of k-1 units
    */
    int s = probabDistOne.size();
    
    // min and max non-zero value
    //int l1 = 0; int u1 = 0;
    int l2 = 0; int u2 = 0;
    for (int i=0; i < s; i++) {
        /*
        if (probabDistOne[i] > 0) {
            u1 = i; // non-zero
        } else if (u1 == 0) { // no non-zero yet
            l1 = i;
        }
        */
        if (probabDistTwo[i] > 0) {
            u2 = i; // non-zero
        } else if (u2 == 0) { // no non-zero yet
            l2 = i;
        }            
    }
    //std::cout <<" l2,u2 "<<l2<<","<<u2<<"\n";
    
    std::vector<double> jointDist(s, 0.0); // always 0..b-1
    for (int v = 2; v <= s; v++) // v=1 impossible in combo because both are non-zero
    {
        jointDist[v-1] = this->convolution(v, probabDistOne, probabDistTwo, l2,u2);
        //std::cout << "debug conv"<<v<<", One: "<<probabDistOne[v-1] << " Two: " <<probabDistTwo[v-1] << " conv: "<<jointDist[v-1]<<" prev:"<<jointDist[v-2]<<"\n";
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
    // first distr
    accuJoRes.push_back(this->accumulateResult(empricialDists[0]));
    
    // if more than one
    if (nrOfDists > 1) {
        std::vector<double> prevDist(empricialDists[0]);
        for (int i = 1; i < nrOfDists; i++)
        {
            std::vector<double> currCdfDist = this->cdf(prevDist, empricialDists[i]);
            accuJoRes.push_back(this->accumulateResult(currCdfDist));
            //Vector assigment is a deep-copy in case of vectors, not like arrays (i.e. not passed by reference).
            prevDist = currCdfDist;
        }
    }
    return accuJoRes;
}
std::string DiscreteDistribution::classname()
{
    return "DiscreteDistribution";
}
