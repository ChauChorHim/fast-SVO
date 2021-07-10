#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <tuple>
#include <opencv2/core/core.hpp>
#include <Dataset.h>

using std::string;
using std::tuple;

namespace fast_SVO
{
class System
{
public:
    // Input dataset
    enum DatasetType {
        MyDataset = 0,
        KITTI = 1
    };

private:
    const DatasetType datasetType_;
    const Dataset* pDataset_;
    cv::Mat curImLeft_; 
    cv::Mat curImRight_;
    double curTimestamp_ = 0;

public:
    System(const Dataset *pDataset, const string &strSetting, const DatasetType datasetType);
    
    double UpdateImages(const int ni);

    cv::Mat TrackStereo(const int ni);

    void SaveTrajecotry(const string &filename);
};



}// namespace fast_SVO

#endif//SYSTEM_H