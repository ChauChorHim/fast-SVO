#include <iostream> // cerr
#include <tuple>
#include <chrono>
#include <vector>

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

    tracker_ = new Tracking(strSettingFile);
    curEstPose_ << 1, 0, 0, 0,
                   0, 1, 0, 0, 
                   0, 0, 1, 0,
                   0, 0, 0, 1;
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
    // Initializing keypoints and descriptors for each frame
    std::vector<cv::KeyPoint> leftKeypoints;
    std::vector<cv::KeyPoint> rightKeypoints;
    cv::Mat leftDescriptors;
    cv::Mat rightDescriptors;

    // Initializing the timeset
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    double ttrack = std::chrono::duration_cast<std::chrono::duration<double>> (std::chrono::steady_clock::now() - t0).count();
    t0 = std::chrono::steady_clock::now();

    tracker_->updateImagesFeatures(curImLeft_, curImRight_, curTimestamp_, leftKeypoints, rightKeypoints, leftDescriptors, rightDescriptors);

    std::vector<cv::DMatch> matches;
    Eigen::Matrix4Xd points3d;
    tracker_->matchStereoFeaturesNaive(leftKeypoints, rightKeypoints, leftDescriptors, rightDescriptors, matches, points3d);
    tracker_->showMatches(curImLeft_, curImRight_, leftKeypoints, rightKeypoints, matches);

    Eigen::Matrix3Xd points2d;
    
    tracker_->matchFeaturesNaive(leftKeypoints, leftDescriptors, matches, points2d);

    tracker_->getTranform(R_, T_, points2d); // prePoints3d is at tracker_
//std::cout << "R_:\n" << R_ << "\nT_:\n" << T_ << std::endl;
    tracker_->updatePreFeatures(leftKeypoints, leftDescriptors, points3d);

}

void System::calculateCurPose(const size_t i) {
    Eigen::Matrix<double, 3, 4> curEstPose_inhomo;
    Eigen::Matrix4d curEstPose_homo = Eigen::Matrix4d::Zero();
    curEstPose_inhomo << R_.transpose(), -R_.transpose()*T_;
    curEstPose_homo.topRows(3) = curEstPose_inhomo;
    curEstPose_homo(3, 3) = 1;

//std::cout << "curEstPose_homo: \n" << curEstPose_homo << std::endl;
//std::cout << "curEstPose_: \n" << curEstPose_ << std::endl;
    curEstPose_ = curEstPose_ * curEstPose_homo;
    estPoses_.push_back(curEstPose_);

    //Eigen::Matrix4d curRealPose;
    //for (int j = 0; j < 12; ++j) {
    //    curRealPose << stof(dataset_->posesGroundTruth_[12 * i + j]);
    //}
    //curRealPose_ = curRealPose + curRealPose_;
}

void System::showTrajectory(const std::string &windowName, cv::Mat &whiteboard) {
//std::cout << "Current estimated pose: \n" << curEstPose_ << std::endl;
    cv::Point2d pointEst(curEstPose_(0, 3) + 500, curEstPose_(1, 3) + 500);
    //cv::Point2d pointGT(curRealPose_(0, 3) + 500, curRealPose_(1, 3) + 500);
//std::cout << "point: \n" << "(" << pointEst.x - 500 << ", " << pointEst.y - 500 << ")" << std::endl;
//std::cout << "real point: \n" << "(" << pointGT.x << ", " << pointGT.y << ")" << std::endl;
    cv::circle(whiteboard, pointEst, 1, cv::Scalar(0, 0, 255), -1);
    //cv::circle(whiteboard, pointGT, 1, cv::Scalar(255, 0, 0), -1);
    cv::imshow(windowName, whiteboard);
    cv::waitKey(5);
}

}