/*
 * @Author: your name
 * @Date: 2021-07-11 20:57:55
 * @LastEditTime: 2021-07-15 10:46:03
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /fast-SVO/include/Tracking.h
 */
#ifndef TRACKING_H
#define TRACKING_H

#include <string>

#include "ORBextractor.h"
#include "System.h"

#include <opencv2/core/mat.hpp>

namespace fast_SVO
{

class System;

class Tracking {

public:
    Tracking(System* system, const std::string &strSettingFile);

    // Preprocess the input and call Track(). 
    // Extract features and performs stereo matching.
    cv::Mat grabImagesStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp);

private:

    // Calibration matrix
    cv::Mat K_;
    cv::Mat distCoef_;
    float baseline_;

    // Color order (true RGB, false BGR, ignored if grayscale)
    bool rgbOrder_;

    // ORB features extractor
    ORBextractor* ORBextractorLeft_, *ORBextractorRight_;

    // System pointer
    System* system_;
};
}

#endif//TRACKING_H