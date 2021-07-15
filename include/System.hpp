#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <tuple>

#include "Dataset.hpp"
#include "Tracking.hpp"

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
    const DatasetType datasetType_;
    const Dataset* dataset_;
    cv::Mat curImLeft_; 
    cv::Mat curImRight_;
    double curTimestamp_ = 0;

    // Tracker. It receives a frame and computes the associated camera pose.
    Tracking* tracker_;

public:
    System(const Dataset *dataset, const std::string &strSettingFile, const DatasetType datasetType);
    
    double updateImages(const int ni);

    cv::Mat trackStereo(const int ni);

    void saveTrajecotry(const std::string &filename);
};



}// namespace fast_SVO

#endif//SYSTEM_H