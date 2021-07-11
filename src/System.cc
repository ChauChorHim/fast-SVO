#include <System.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <iostream> // cerr

using std::cerr;
using std::endl;
using std::make_tuple;
using std::exit;

namespace fast_SVO
{

System::System(const Dataset *mpDataset, const string &strSettingFile, const DatasetType mDatasetType) : mDatasetType_(mDatasetType), mpDataset_(mpDataset) {

    //Check settings file
    cv::FileStorage fsSettings(strSettingFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {
        cerr << "Failed to open settings file at: " << strSettingFile << endl;
        exit(-1);
    }

    mpTracker_ = new Tracking(this, strSettingFile);
}

double System::UpdateImages(const int ni) {
    mCurTimestamp_ = mpDataset_->vTimestamps_[ni];
    mCurImLeft_ = cv::imread(mpDataset_->vstrImageLeft_[ni], CV_LOAD_IMAGE_UNCHANGED);
    mCurImRight_ = cv::imread(mpDataset_->vstrImageRight_[ni], CV_LOAD_IMAGE_UNCHANGED);
    if(mCurImLeft_.empty()) {
        cerr << endl << "Failed to load image at: " 
             << string(mpDataset_->vstrImageLeft_[ni]) << endl; 
    }
    return mCurTimestamp_; // used for checking the current time stamp
}

cv::Mat System::TrackStereo(const int ni) {

}

}