/*
 * 
 */
/*
 * 
 */
#include <iostream> // cerr
#include <tuple>

#include "System.hpp"

#include <opencv2/imgcodecs.hpp>

namespace fast_SVO
{

System::System(const Dataset *dataset, const std::string &strSettingFile, const DatasetType datasetType) : datasetType_(datasetType), dataset_(dataset) {

    //Check settings file
    cv::FileStorage fsSettings(strSettingFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {
        std::cerr << "Failed to open settings file at: " << strSettingFile << std::endl;
        exit(-1);
    }

    tracker_ = new Tracking(this, strSettingFile);
}

/**
 * @description: This function update the left and right image for the System by reading the corresponding images path
 * @param {const int} ni (No. of images pair)
 * @return {double} curTimestamp_ (current time stamp)
 */
double System::updateImages(const int i) {
    curTimestamp_ = dataset_->vTimestamps_[i];
    curImLeft_ = cv::imread(dataset_->vstrImageLeft_[i], cv::IMREAD_UNCHANGED);
    curImRight_ = cv::imread(dataset_->vstrImageRight_[i], cv::IMREAD_UNCHANGED);
    if(curImLeft_.empty()) {
        std::cerr << std::endl << "Failed to load image at: " 
             << std::string(dataset_->vstrImageLeft_[i]) << std::endl; 
    }
    return curTimestamp_; // used for checking the current time stamp
}

void System::trackStereo() {
    tracker_->updateImagesFeatures(curImLeft_, curImRight_, curTimestamp_);
    tracker_->matchStereoFeaturesNaive();
    tracker_->showMatches(curImLeft_, curImRight_);
    tracker_->matchFeaturesNaive();
    tracker_->getTranform(R_, T_);
    combineTransform();
}

void System::combineTransform() {

}

}