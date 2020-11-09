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

void ProbabilityEstimator::readDistributions(const std::string &collection_date, const std::string &resolution)
{

    std::cout << "Read distributions of a "<< this->classname() << " \n";
    // Clear the current distributions (i.e. distributions of the previous date)
    this->currCollDateVector.clear();

    std::ifstream fin;
    std::vector<double> cDist;
    std::string line, key, value;
    // Open an existing file of a specific date.
    fin.open(collection_date + "_" + resolution + ".csv", std::ios::in);
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
        throw std::runtime_error("ProbabilityEstimator.readDistributions() -> Failed to read file '" + collection_date + "_" + resolution + ".csv" + "'\n");
    }
    fin.close();
}
