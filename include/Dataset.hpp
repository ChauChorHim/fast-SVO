#ifndef DATASET_H
#define DATASET_H

#include <vector>
#include <string>
#include <Eigen/Core>

namespace fast_SVO
{

class Dataset{
public:
    Dataset(const std::string &strPathToSequence, const std::string &sequenceNo = "-1");
    int getImagesAmount() const;
    const std::string getStrImage(const int i, const bool isLeft) const;
    const double getTimestamps(const int i) const;
    const Eigen::Matrix<double, 3, 4> getPoseGroundTruth(const int i) const;

private:
    std::vector<std::string> strImageLeft_;
    std::vector<std::string> strImageRight_;
    std::vector<double> timestamps_;
    std::vector<Eigen::Matrix<double, 3, 4>> posesGroundTruth_;
    void loadImages(const std::string &strPathToSequence);
    void loadTruePoses(const std::string &strPathToSequence, const std::string &sequenceNo);
};
}

#endif //DATASET_H