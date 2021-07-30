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

class Tracking {

public:
    Tracking(const std::string &strSettingFile);

    void updateImagesFeatures(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, 
                              std::vector<cv::KeyPoint> &leftKeypoints, std::vector<cv::KeyPoint> &rightKeypoints,
                              cv::Mat &leftDescriptors, cv::Mat &rightDescriptors);

    void matchStereoFeaturesNaive(std::vector<cv::KeyPoint> &leftKeypoints, std::vector<cv::KeyPoint> &rightKeypoints,
                                  cv::Mat &leftDescriptors, cv::Mat &rightDescriptors, std::vector<cv::DMatch> &matches,
                                  Eigen::Matrix<double, 4, Eigen::Dynamic> &points3d);

    void showMatches(const cv::Mat &image1, const cv::Mat &image2,
                     const std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2,
                     const std::vector<cv::DMatch> &matches);

    void matchFeaturesNaive(std::vector<cv::KeyPoint> &leftKeypoints, cv::Mat &leftDescriptors, std::vector<cv::DMatch> &matches,
                            Eigen::Matrix<double, 3, Eigen::Dynamic> &points2d);

    void getTranform(Eigen::Matrix3d &R, Eigen::Vector3d &T, const Eigen::Matrix3Xd &points2d);

    void updatePreFeatures(std::vector<cv::KeyPoint> &leftKeypoints, cv::Mat &leftDescriptors, Eigen::Matrix4Xd &points3d);

    void checkValidProj(const Eigen::Matrix3Xd &points2d, const Eigen::Matrix4Xd &points3d);

    enum trackingState {
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

private:
    // tracking state
    trackingState state_;

    // Calibration matrix
    Eigen::Matrix3d K_;
    Eigen::Vector4d distCoef_;
    double baseline_;

    // Projection matrix
    cv::Matx34d P1_;
    cv::Matx34d P2_;

    // Color order (true RGB, false BGR, ignored if grayscale)
    bool rgbOrder_;

    // ORB features extractor
    cv::Ptr<cv::ORB> ORBextractorLeft_, ORBextractorRight_;

    // previous features keypoints
    std::vector<cv::KeyPoint> preLeftKeypoints_;

    // previous features descriptors
    cv::Mat preLeftDescriptors_;

    // features matcher
    cv::Ptr<cv::DescriptorMatcher> matcher_;

    // previous triangulated 3D points
    Eigen::Matrix4Xd prePoints3d_;

    // p3p solver
    Solver* p3pSolver_;


    
};
}

#endif//TRACKING_H