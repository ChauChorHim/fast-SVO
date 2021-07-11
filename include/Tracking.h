#ifndef TRACKING_H
#define TRACKING_H

#include <string>
#include "ORBextractor.h"
#include "System.h"

using std::string;


namespace fast_SVO
{

class System;

class Tracking {

public:
    Tracking(System* pSys, const string &strSettingFile);

    // Preprocess the input and call Track(). 
    // Extract features and performs stereo matching.
    cv::Mat GrabImagesStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp);

private:

    // Calibration matrix
    cv::Mat mK_;
    cv::Mat mDistCoef_;
    float mbf_;

    // Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB_;

    // ORB features extractor
    ORBextractor* mpORBextractorLeft_, *mpORBextractorRight_;

    // System pointer
    System* mpSystem_;
};
}

#endif//TRACKING_H