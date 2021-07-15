#ifndef DATASET_H
#define DATASET_H

#include <vector>
#include <string>

namespace fast_SVO
{

class Dataset {
public:
    Dataset(const std::string &strPathToSequence);
    int getImagesNum();

private:
    friend class System;
    std::vector<std::string> vstrImageLeft_;
    std::vector<std::string> vstrImageRight_;
    std::vector<double> vTimestamps_;
    void loadImages(const std::string &strPathToSequence);
};
}

#endif //DATASET_H