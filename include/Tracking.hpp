/*
 * 
 */
/*
 * 
 */
#ifndef TRACKING_H
#define TRACKING_H

#include <string>

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

    void showMatches(const cv::Mat &imRectLeft, const cv::Mat &imRectRight);

private:

    // Calibration matrix
    cv::Mat K_;
    cv::Mat distCoef_;
    float baseline_;

    // Color order (true RGB, false BGR, ignored if grayscale)
    bool rgbOrder_;

    // ORB features extractor
    cv::Ptr<cv::ORB> ORBextractorLeft_, ORBextractorRight_;

    // features keypoints
    std::vector<cv::KeyPoint> leftKeypoints_, rightKeypoints_;

    // features descriptors
    cv::Mat leftDescriptors_, rightDescriptors_;

    // features matcher
    cv::Ptr<cv::DescriptorMatcher> matcher_;

    // features matches
    std::vector<cv::DMatch> matches_;

    // System pointer
    System* system_;


};
}

#endif//TRACKING_H