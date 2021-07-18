/*
 * 
 */
/*
 * 
 */
#include <iostream>

#include "Tracking.hpp"

#include <opencv2/calib3d.hpp>

namespace fast_SVO
{

Tracking::Tracking(System* system, const std::string &strSettingFile) : system_(system) {
    cv::FileStorage fSettings(strSettingFile, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(K_);

    cv::Mat P = cv::Mat::zeros(cv::Size_<float>(3, 4), CV_32F);
    P.at<float>(0, 0) = fx;
    P.at<float>(0, 2) = cx;
    P.at<float>(1, 1) = fy;
    P.at<float>(1, 2) = cy;
    P.at<float>(2, 2) = 1.;
    P.copyTo(P1_);

    baseline_ = fSettings["Camera.bf"];
    P.at<float>(0, 3) = -baseline_;
    P.copyTo(P2_);

    cv::Mat distCoef(4, 1, CV_32F);
    distCoef.at<float>(0) = fSettings["Camera.k1"];
    distCoef.at<float>(1) = fSettings["Camera.k2"];
    distCoef.at<float>(2) = fSettings["Camera.p1"];
    distCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0) {
        distCoef.resize(5);
        distCoef.at<float>(4) = k3;
    }
    distCoef.copyTo(distCoef_);

    float fps = fSettings["Camera.fps"];
    if (fps == 0)
        fps = 30;

    std::cout << std::endl << "Camera Parameters: " << std::endl;
    std::cout << "- fx: " << fx << std::endl;
    std::cout << "- fy: " << fy << std::endl;
    std::cout << "- cx: " << cx << std::endl;
    std::cout << "- cy: " << cy << std::endl;
    std::cout << "- k1: " << distCoef.at<float>(0) << std::endl;
    std::cout << "- k2: " << distCoef.at<float>(1) << std::endl;
    if(distCoef.rows==5)
        std::cout << "- k3: " << distCoef.at<float>(4) << std::endl;
    std::cout << "- p1: " << distCoef.at<float>(2) << std::endl;
    std::cout << "- p2: " << distCoef.at<float>(3) << std::endl;
    std::cout << "- fps: " << fps << std::endl;

    int rgbOrder = fSettings["Camera.RGB"];
    rgbOrder_ = rgbOrder;
    if(rgbOrder_)
        std::cout << "- color order: RGB (ignored if grayscale)" << std::endl;
    else
        std::cout << "- color order: BGR (ignored if grayscale)" << std::endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float scaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];

    ORBextractorLeft_ = cv::ORB::create(nFeatures,scaleFactor,nLevels);
    ORBextractorRight_ = cv::ORB::create(nFeatures,scaleFactor,nLevels);

    std::cout << std::endl  << "ORB Extractor Parameters: " << std::endl;
    std::cout << "- Number of Features: " << nFeatures << std::endl;
    std::cout << "- Scale Levels: " << nLevels << std::endl;
    std::cout << "- Scale Factor: " << scaleFactor << std::endl;

    matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
}


void Tracking::updateImagesFeatures(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp) {
    ORBextractorLeft_->detectAndCompute(imRectLeft, cv::noArray(), leftKeypoints_, leftDescriptors_);    
    ORBextractorRight_->detectAndCompute(imRectRight, cv::noArray(), rightKeypoints_, rightDescriptors_);    
}

void Tracking::matchStereoFeaturesNaive() {
    std::vector<std::vector<cv::DMatch>> knnMatches_;
    //matcher_->match(leftDescriptors_, rightDescriptors_, matches_);
    matcher_->knnMatch(leftDescriptors_, rightDescriptors_, knnMatches_, 2);

    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.8f;
    std::vector<cv::KeyPoint> goodLeftKeypoints_, goodRightKeypoints_;
    size_t j = 0;
    for (size_t i = 0; i < knnMatches_.size(); i++)
    {
        if (knnMatches_[i][0].distance < ratio_thresh * knnMatches_[i][1].distance) {
            goodMatches_.push_back(knnMatches_[i][0]);
            leftKeypoints_[j] = leftKeypoints_[i];
            rightKeypoints_[j] = rightKeypoints_[i];
            j++;
        }
    }

    //-- Triangulate 3D points
    cv::Mat pnts3D(4,leftKeypoints_.size(),CV_64F);
    cv::triangulatePoints(P1_, P2_, leftKeypoints_, rightKeypoints_, pnts3D);
    pnts3D.copyTo(points3d_);
}

void Tracking::showMatches(const cv::Mat &imRectLeft, const cv::Mat &imRectRight) {
    cv::Mat imMatches;
    cv::drawMatches(imRectLeft, leftKeypoints_, imRectRight, rightKeypoints_, goodMatches_, imMatches);
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE); // create window
    cv::imshow("Display Image", imMatches); // show the image
    cv::waitKey(50);
    goodMatches_.clear();
}

}