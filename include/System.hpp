#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>

#include "Dataset.hpp"
#include "Tracking.hpp"
#include "Module.hpp"
#include "Timer.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

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
    const Dataset* dataset_;

    size_t curImageNo;

    cv::Mat curImLeft_, curImRight_;
    double curTimestamp_ = 0;

    //-- Tracker. It receives a frame and computes the associated camera pose.
    Tracking* tracker_;

    //-- Current 3D-2D estimated Rotation matrix and translation vector
    Eigen::Matrix3d R_; 
    Eigen::Vector3d T_;

    //-- Current estimated pose
    Eigen::Matrix4d curEstPose_;

    //-- All estimated poses
    std::vector<Eigen::Matrix4d> estPoses_;

    //-- Current real pose
    Eigen::Matrix<double, 3, 4> curRealPose_;

    //-- looptimer for timing the process to improve efficiency
    LoopTimer loopTimer;

    //-- Record used time of some time-comsuing process
public:
    System(const Dataset *dataset, const std::string &strSettingFile, const DatasetType datasetType);

    ~System();
    
    double updateImages(const int i);

    void trackStereo();

    void calculateCurPose();

    void showTrajectory(const std::string &windowName, cv::Mat &whiteboard);

    void logInfo();

};

}// namespace fast_SVO

#endif//SYSTEM_H