#ifndef DATASET_H
#define DATASET_H

#include <vector>
#include <string>

namespace fast_SVO
{

class Dataset {
public:
    Dataset(const std::string &strPathToSequence, const std::string &sequenceNo);
    int getImagesNum();

private:
    friend class System;
    std::vector<std::string> vstrImageLeft_;
    std::vector<std::string> vstrImageRight_;
    std::vector<double> vTimestamps_;
    std::string sequenceNo_;
    std::vector<std::string> posesGroundTruth_;
    void loadImages(const std::string &strPathToSequence);
    void loadTruePoses(const std::string &strPathToSequence, const std::string &sequenceNo);
};
}

#endif //DATASET_H