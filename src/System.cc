#include "System.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <iostream> // cerr

using std::cerr;
using std::endl;
using std::make_tuple;

namespace fast_SVO
{

System::System(const Dataset *pDataset, const string &strSetting, const DatasetType datasetType) : datasetType_(datasetType), pDataset_(pDataset) {}

double System::UpdateImages(const int ni) {
    curTimestamp_ = pDataset_->vTimestamps_[ni];
    curImLeft_ = cv::imread(pDataset_->vstrImageLeft_[ni], CV_LOAD_IMAGE_UNCHANGED);
    curImRight_ = cv::imread(pDataset_->vstrImageRight_[ni], CV_LOAD_IMAGE_UNCHANGED);
    if(curImLeft_.empty()) {
        cerr << endl << "Failed to load image at: " 
             << string(pDataset_->vstrImageLeft_[ni]) << endl; 
    }
    return curTimestamp_; // used for checking the current time stamp
}

cv::Mat System::TrackStereo(const int ni) {

}

}