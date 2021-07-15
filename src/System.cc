#include <iostream> // cerr

#include "System.h"

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
 * @param {const int} ni: No. of images pair
 * @return {*}
 */
double System::updateImages(const int ni) {
    curTimestamp_ = dataset_->vTimestamps_[ni];
    curImLeft_ = cv::imread(dataset_->vstrImageLeft_[ni], cv::IMREAD_UNCHANGED);
    curImRight_ = cv::imread(dataset_->vstrImageRight_[ni], cv::IMREAD_UNCHANGED);
    if(curImLeft_.empty()) {
        std::cerr << std::endl << "Failed to load image at: " 
             << std::string(dataset_->vstrImageLeft_[ni]) << std::endl; 
    }
    return curTimestamp_; // used for checking the current time stamp
}

cv::Mat System::trackStereo(const int ni) {

}

}