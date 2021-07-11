#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <tuple>
#include <opencv2/core/core.hpp>

#include <Dataset.h>
#include <Tracking.h>

using std::string;
using std::tuple;

namespace fast_SVO
{

class Tracking;

class System
{
public:
    // Input dataset
    enum DatasetType {
        MyDataset = 0,
        KITTI = 1
    };

private:
    const DatasetType mDatasetType_;
    const Dataset* mpDataset_;
    cv::Mat mCurImLeft_; 
    cv::Mat mCurImRight_;
    double mCurTimestamp_ = 0;

    // Tracker. It receives a frame and computes the associated camera pose.
    Tracking* mpTracker_;

public:
    System(const Dataset *mpDataset, const string &strSettingFile, const DatasetType mDatasetType);
    
    double UpdateImages(const int ni);

    cv::Mat TrackStereo(const int ni);

    void SaveTrajecotry(const string &filename);
};



}// namespace fast_SVO

#endif//SYSTEM_H