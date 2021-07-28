#ifndef TRACKING_H
#define TRACKING_H

#include <string>

#include "Solver.hpp"

#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <opencv2/core/mat.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

namespace fast_SVO
{

class System;

class Tracking {

public:
    Tracking(System* system, const std::string &strSettingFile);

    void updateImagesFeatures(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp);

    void matchStereoFeaturesNaive();

    void showMatches(const cv::Mat &image1, const cv::Mat &image2);

    void matchFeaturesNaive();

    void getTranform(Eigen::Matrix3d &R, Eigen::Vector3d &T);

    enum trackingState {
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

private:
    // tracking state
    trackingState state;

    // Calibration matrix
    Eigen::Matrix3d K_;
    Eigen::Vector4d distCoef_;
    float baseline_;

    // Projection matrix
    cv::Matx34d P1_;
    cv::Matx34d P2_;

    // Color order (true RGB, false BGR, ignored if grayscale)
    bool rgbOrder_;

    // System pointer
    System* system_;

    // ORB features extractor
    cv::Ptr<cv::ORB> ORBextractorLeft_, ORBextractorRight_;

    // features keypoints
    std::vector<cv::KeyPoint> leftKeypoints_, rightKeypoints_, preLeftKeypoints_;

    // features descriptors
    cv::Mat leftDescriptors_, rightDescriptors_, preLeftDescriptors_;

    // features matcher
    cv::Ptr<cv::DescriptorMatcher> matcher_;

    // features matches, used for visualizing the matching
    std::vector<cv::DMatch> matches_;

    // triangulated 3D points
    Eigen::Matrix<double, 4, Eigen::Dynamic> points3d_, prePoints3d_;

    // 2D points in left camera image frame
    Eigen::Matrix<double, 3, Eigen::Dynamic> points2d_;

    // p3p solver
    Solver* p3pSolver_;
};
}

#endif//TRACKING_H