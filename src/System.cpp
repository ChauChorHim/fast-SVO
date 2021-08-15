#include <iostream> // cerr
#include <chrono>
#include <vector>
#include <thread>
#include <fstream>

#include "System.hpp"

#include <opencv2/imgcodecs.hpp>

namespace fast_SVO
{

System::System(const Dataset *dataset, const std::string &strSettingFile, const DatasetType datasetType) : 
                datasetType_{datasetType}, 
                dataset_{dataset}, 
                loopTimers_{LoopTimer("updateCurFeatures"), LoopTimer("matchStereoFeaturesNaive"), LoopTimer("matchFeaturesNaive"), LoopTimer("getTransform"), LoopTimer("updateFeatures")},
                tracker_{Tracking(strSettingFile)} {

    //Check settings file path
    cv::FileStorage fsSettings(strSettingFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {
        std::cerr << "Failed to open settings file at: " << strSettingFile << std::endl;
        exit(-1);
    }

    curEstPose_ << 1, 0, 0, 0,
                   0, 1, 0, 0, 
                   0, 0, 1, 0,
                   0, 0, 0, 1;
    curRealPose_ << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0;
    estPoses_.reserve(dataset_->getImagesAmount() + 1);
    
}

System::~System() {}

/**
 * @description: This function update the left and right image for the System by reading the corresponding images path
 * @param {const int} ni (No. of images pair)
 * @return {double} curTimestamp_ (current time stamp)
 */
double System::updateImages(const int i) {
    curTimestamp_ = dataset_->getTimestamps(i);
    curImLeft_ = cv::imread(dataset_->getStrImage(i, 1), cv::IMREAD_UNCHANGED);
    curImRight_ = cv::imread(dataset_->getStrImage(i, 0), cv::IMREAD_UNCHANGED);
    if(curImLeft_.empty()) {
        std::cerr << std::endl << "Failed to load image at: " 
             << std::string(dataset_->getStrImage(i, 1)) << std::endl; 
    }
    curImageNo_ = i;
    return curTimestamp_; // used for checking the current time stamp
}


void System::resetAllTimers() {
    for (auto timer : loopTimers_) 
        timer.reset();
}

void System::trackStereo() {
    // Initializing keypoints and descriptors for each frame
    std::vector<cv::KeyPoint> leftKeypoints;
    std::vector<cv::KeyPoint> rightKeypoints;
    cv::Mat leftDescriptors;
    cv::Mat rightDescriptors;

    loopTimers_[0].start();
    std::thread leftThread(&Tracking::updateCurFeatures, &tracker_, true, std::ref(curImLeft_), std::ref(leftKeypoints), std::ref(leftDescriptors));
    std::thread rightThread(&Tracking::updateCurFeatures, &tracker_, false, std::ref(curImRight_), std::ref(rightKeypoints), std::ref(rightDescriptors));
    leftThread.join();
    rightThread.join();
    loopTimers_[0].pause();

    std::vector<cv::DMatch> matches;
    Eigen::Matrix4Xd points3d;

    loopTimers_[1].start();
    tracker_.matchStereoFeaturesNaive(leftKeypoints, rightKeypoints, leftDescriptors, rightDescriptors, matches, points3d);
    loopTimers_[1].pause();

    //tracker_.showMatches(curImLeft_, curImRight_, leftKeypoints, rightKeypoints, matches);

    Eigen::Matrix3Xd points2d;
    
    loopTimers_[2].start();
    tracker_.matchFeaturesNaive(leftKeypoints, leftDescriptors, matches, points2d);
    loopTimers_[2].pause();

    loopTimers_[3].start();
    tracker_.getTranform(R_, T_, points2d); // prePoints3d is at tracker_
    loopTimers_[3].pause();

    loopTimers_[4].start();
    tracker_.updatePreFeatures(leftKeypoints, leftDescriptors, points3d);
    loopTimers_[4].pause();

}

void System::calculateCurPose() {
    Eigen::Matrix<double, 3, 4> curEstPose_inhomo;
    Eigen::Matrix4d curEstPose_homo = Eigen::Matrix4d::Zero();
    curEstPose_inhomo << R_.transpose(), -R_.transpose() * T_;
    curEstPose_homo.topRows(3) = curEstPose_inhomo;
    curEstPose_homo(3, 3) = 1;

    curEstPose_ = curEstPose_ * curEstPose_homo;
    estPoses_.push_back(curEstPose_);

}

void System::showTrajectory(const std::string &windowName, cv::Mat &whiteboard) {
    curRealPose_ = dataset_->getPoseGroundTruth(curImageNo_);
    cv::Point2d pointEst(curEstPose_(2, 3) + 500, curEstPose_(0, 3) + 500);
    cv::Point2d pointGT(curRealPose_(2, 3) + 500, curRealPose_(0, 3) + 500);
    cv::circle(whiteboard, pointEst, 1, cv::Scalar(0, 0, 255), -1);
    cv::circle(whiteboard, pointGT, 1, cv::Scalar(255, 0, 0), -1);
    cv::imshow(windowName, whiteboard);
    cv::waitKey(1);
}

void System::saveTrajectory(const std::string &pathToResult, const std::string &filename) {
    std::ofstream fout;
    fout.open(pathToResult + "/" + filename);
    for (auto matrix : estPoses_) {
        for (int i = 0; i < matrix.rows(); ++i)
            fout << matrix.row(i) << " ";
        fout << std::endl;
    }
    fout.close();
}


}