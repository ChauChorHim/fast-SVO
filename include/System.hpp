/*
 * 
 */
/*
 * 
 */
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

    //std::vector<cv::KeyPoint> curLeftKeypoints_;
    //cv::OutputArray curLeftDescriptors_;
    //std::vector<cv::KeyPoint> curRightKeypoints_;
    //cv::OutputArray curRightDescriptors_;

    //std::vector<cv::KeyPoint> preLeftKeypoints_;
    //cv::OutputArray preLeftDescriptors_;
    //std::vector<cv::KeyPoint> preRightKeypoints_;
    //cv::OutputArray preRightDescriptors_;

    // Tracker. It receives a frame and computes the associated camera pose.
    Tracking* tracker_;

public:
    System(const Dataset *dataset, const std::string &strSettingFile, const DatasetType datasetType);
    
    double updateImages(const int i);

    void trackStereo();

    void saveTrajecotry(const std::string &filename);
};



}// namespace fast_SVO

#endif//SYSTEM_H