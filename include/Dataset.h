#ifndef DATASET_H
#define DATASET_H

#include <vector>
#include <string>
#include <tuple>
#include <opencv2/imgcodecs.hpp>

using std::vector;
using std::string;
using std::tuple;

namespace fast_SVO
{

class Dataset {
public:
    Dataset(const string &strPathToSequence);
    int getImagesNum();
    tuple<cv::Mat, cv::Mat, double> getImages(const int ni);

private:
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    void loadImages(const string &strPathToSequence);
};
}

#endif //DATASET_H