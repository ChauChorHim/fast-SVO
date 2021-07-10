#ifndef DATASET_H
#define DATASET_H

#include <vector>
#include <string>

using std::vector;
using std::string;

namespace fast_SVO
{

class Dataset {
public:
    Dataset(const string &strPathToSequence);
    int getImagesNum();

private:
    friend class System;
    vector<string> vstrImageLeft_;
    vector<string> vstrImageRight_;
    vector<double> vTimestamps_;
    void loadImages(const string &strPathToSequence);
};
}

#endif //DATASET_H