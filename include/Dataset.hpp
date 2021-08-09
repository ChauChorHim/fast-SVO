#ifndef DATASET_H
#define DATASET_H

#include <vector>
#include <string>
#include <Eigen/Core>

namespace fast_SVO
{

class Dataset {
public:
    Dataset(const std::string &strPathToSequence, const std::string &sequenceNo);
    int getImagesAmount() const;

private:
    friend class System;
    std::vector<std::string> strImageLeft_;
    std::vector<std::string> strImageRight_;
    std::vector<double> timestamps_;
    std::vector<Eigen::Matrix<double, 3, 4>> posesGroundTruth_;
    void loadImages(const std::string &strPathToSequence);
    void loadTruePoses(const std::string &strPathToSequence, const std::string &sequenceNo);
};
}

#endif //DATASET_H