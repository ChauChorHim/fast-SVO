/*
 * 
 */
/*
 * 
 */
#include <iostream>
#include <thread>

#include "Tracking.hpp"

#include <opencv2/calib3d.hpp>

namespace fast_SVO
{

Tracking::Tracking(const std::string &strSettingFile) {
    state_ = NOT_INITIALIZED;
    cv::FileStorage fSettings(strSettingFile, cv::FileStorage::READ);
    double fx = fSettings["Camera.fx"];
    double fy = fSettings["Camera.fy"];
    double cx = fSettings["Camera.cx"];
    double cy = fSettings["Camera.cy"];

    K_ << fx, 0, cx,
          0, fy, cy,
          0, 0, 1;

    cv::Matx34d P = cv::Matx34d(K_(0, 0), K_(0, 1), K_(0, 2), 0,
                                K_(1, 0), K_(1, 1), K_(1, 2), 0,
                                K_(2, 0), K_(2, 1), K_(2, 2), 0);
    P1_ = P;

    baseline_ = fSettings["Camera.bf"];
    P(0, 3) = -baseline_;
    P2_ = P;

    distCoef_(0) = fSettings["Camera.k1"];
    distCoef_(1) = fSettings["Camera.k2"];
    distCoef_(2) = fSettings["Camera.p1"];
    distCoef_(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0) {
        distCoef_.resize(5);
        distCoef_(4) = k3;
    }

    float fps = fSettings["Camera.fps"];
    if (fps == 0)
        fps = 30;

    std::cout << std::endl << "Camera Parameters: " << std::endl;
    std::cout << "- fx: " << fx << std::endl;
    std::cout << "- fy: " << fy << std::endl;
    std::cout << "- cx: " << cx << std::endl;
    std::cout << "- cy: " << cy << std::endl;
    std::cout << "- k1: " << distCoef_(0) << std::endl;
    std::cout << "- k2: " << distCoef_(1) << std::endl;
    if(distCoef_.rows()==5)
        std::cout << "- k3: " << distCoef_(4) << std::endl;
    std::cout << "- p1: " << distCoef_(2) << std::endl;
    std::cout << "- p2: " << distCoef_(3) << std::endl;
    std::cout << "- fps: " << fps << std::endl;

    int rgbOrder = fSettings["Camera.RGB"];
    rgbOrder_ = rgbOrder;
    if(rgbOrder_)
        std::cout << "- color order: RGB (ignored if grayscale)" << std::endl;
    else
        std::cout << "- color order: BGR (ignored if grayscale)" << std::endl;

    // Load ORB parameters
    const int nFeatures = fSettings["ORBextractor.nFeatures"];
    const float scaleFactor = fSettings["ORBextractor.scaleFactor"];
    const int nLevels = fSettings["ORBextractor.nLevels"];
    ORBextractorLeft_ = cv::ORB::create(nFeatures,scaleFactor,nLevels);
    ORBextractorRight_ = cv::ORB::create(nFeatures,scaleFactor,nLevels);

    std::cout << std::endl  << "ORB Extractor Parameters: " << std::endl;
    std::cout << "- Number of Features: " << nFeatures << std::endl;
    std::cout << "- Scale Levels: " << nLevels << std::endl;
    std::cout << "- Scale Factor: " << scaleFactor << std::endl;

    matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    // Load RANSAC parameters
    const float confidence = fSettings["RANSAC.confidence"];
    const float probability = fSettings["RANSAC.probability"];
    const size_t numIter = ceil(log(1. - confidence) / log(1. - pow(probability, 4)));
    const float epsilon = K_(0, 2) * 0.02;
    std::cout << std::endl  << "RANSAC Parameters: " << std::endl;
    std::cout << "- RANSAC confidence: " << confidence << std::endl;
    std::cout << "- RANSAC probability: " << probability << std::endl;
    std::cout << "- RANSAC Number of Iteration: " << numIter << std::endl;
    std::cout << "- RANSAC epsilon: " << epsilon << std::endl;
    p3pSolver_ = new Solver(numIter, epsilon, K_);
}

void Tracking::updateImagesFeatures(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, 
                                    std::vector<cv::KeyPoint> &leftKeypoints, std::vector<cv::KeyPoint> &rightKeypoints, 
                                    cv::Mat &leftDescriptors, cv::Mat &rightDescriptors) {
    ORBextractorLeft_->detectAndCompute(imRectLeft, cv::noArray(), leftKeypoints, leftDescriptors);    
    ORBextractorRight_->detectAndCompute(imRectRight, cv::noArray(), rightKeypoints, rightDescriptors);    
}

void Tracking::matchStereoFeaturesNaive(std::vector<cv::KeyPoint> &leftKeypoints, std::vector<cv::KeyPoint> &rightKeypoints,
                                        cv::Mat &leftDescriptors, cv::Mat &rightDescriptors, std::vector<cv::DMatch> &matches,
                                        Eigen::Matrix<double, 4, Eigen::Dynamic> &points3d) {
    std::vector<std::vector<cv::DMatch>> knnMatches;
    matcher_->knnMatch(leftDescriptors, rightDescriptors, knnMatches, 2);

    std::vector<cv::Point2f> leftKeypointsCoor, rightKeypointsCoor;
    cv::Mat goodLeftDescriptors, goodRightDescriptors;
    std::vector<cv::KeyPoint> goodLeftKeypoints, goodRightKeypoints;
    std::vector<cv::DMatch> goodMatches;

    //-- Filter matches using the Lowe's ratio test and epipolar constraint
    const float ratio_thresh = 0.8f;
    size_t j = 0;
    for (size_t i = 0; i < knnMatches.size(); i++)
    {
        size_t leftIndex = knnMatches[i][0].queryIdx;
        size_t rightIndex = knnMatches[i][0].trainIdx;
        if (knnMatches[i][0].distance < ratio_thresh * knnMatches[i][1].distance && abs(leftKeypoints[leftIndex].pt.y - rightKeypoints[rightIndex].pt.y) < 0.1) {
            knnMatches[i][0].queryIdx = j;
            knnMatches[i][0].trainIdx = j;
            goodMatches.push_back(knnMatches[i][0]);

            leftKeypointsCoor.push_back(leftKeypoints[leftIndex].pt); // leftKeypointsCoor and rightKeypointsCoor are used in cv::triangulatePoints()
            rightKeypointsCoor.push_back(rightKeypoints[rightIndex].pt);

            goodLeftKeypoints.push_back(leftKeypoints[leftIndex]);
            goodRightKeypoints.push_back(rightKeypoints[rightIndex]);

            goodLeftDescriptors.push_back(leftDescriptors.row(leftIndex));
            goodRightDescriptors.push_back(rightDescriptors.row(rightIndex));
            j++;
        }
    }

    //-- Move the goodMatches to matches_
    matches = std::move(goodMatches);

    //-- Move the goodLeftKeypoints to leftKeypoints_
    leftKeypoints = std::move(goodLeftKeypoints);
    rightKeypoints = std::move(goodRightKeypoints);

    //-- Move the goodLeftDescriptors to leftDescriptors_
    leftDescriptors = std::move(goodLeftDescriptors);

    //-- Triangulate 3D points with qualified matches
    if (leftKeypointsCoor.size() >= 4) { // P3P needs at least 4 points
        cv::Mat pnts3D(4, leftKeypointsCoor.size(), CV_64F);
        cv::triangulatePoints(P1_, P2_, leftKeypointsCoor, rightKeypointsCoor, pnts3D);
        size_t colsNum = pnts3D.cols;
        points3d.conservativeResize(Eigen::NoChange, colsNum);
        cv::cv2eigen(pnts3D, points3d);
        size_t lastRowNum = points3d.rows() - 1;
        points3d.array().rowwise() /= points3d.row(lastRowNum).array();
    }

    // Check the projection
    //Eigen::Matrix3Xd points2d;
    //points2d.conservativeResize(Eigen::NoChange, leftKeypoints.size());
    //Eigen::Vector3d point2d;
    //for (int i = 0; i < leftKeypoints.size(); ++i) {
    //    point2d << leftKeypoints[i].pt.x, leftKeypoints[i].pt.y, 1;
    //    points2d.col(i) = point2d; 
    //}
    //checkValidProj(points2d, points3d);

}

void Tracking::showMatches(const cv::Mat &image1, const cv::Mat &image2, 
                           const std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2,
                           const std::vector<cv::DMatch> &matches) {
    if (matches.size() < 4) { // cv::drawMatches can't receive empty input array
        std::cout << "Dump this frame. There are only " << matches.size() << " matches" << std::endl;
        return;
    }
    cv::Mat imMatches;
    cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, imMatches);
    cv::namedWindow("Stereo matching features", cv::WINDOW_AUTOSIZE); // create window
    cv::imshow("Stereo matching features", imMatches); // show the image
    cv::waitKey(1);
}

void Tracking::matchFeaturesNaive(const std::vector<cv::KeyPoint> &leftKeypoints, const cv::Mat &leftDescriptors, std::vector<cv::DMatch> &matches, 
                                  Eigen::Matrix<double, 3, Eigen::Dynamic> &points2d) {
    if (state_ == OK) {
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher_->knnMatch(leftDescriptors, preLeftDescriptors_, knnMatches, 2);
        std::vector<cv::DMatch> goodMatches;
        Eigen::Matrix<double, 4, Eigen::Dynamic> prePoints3d;
        
        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.8f;
        size_t j = 0;

        for (size_t i = 0; i < knnMatches.size(); i++)
        {
            size_t curIndex = knnMatches[i][0].queryIdx;
            size_t preIndex = knnMatches[i][0].trainIdx;
            if (knnMatches[i][0].distance < ratio_thresh * knnMatches[i][1].distance) {
                knnMatches[i][0].queryIdx = j;
                knnMatches[i][0].trainIdx = j;
                goodMatches.push_back(knnMatches[i][0]);

                prePoints3d.conservativeResize(Eigen::NoChange, j + 1);
                prePoints3d.col(j) = prePoints3d_.col(preIndex);

                points2d.conservativeResize(Eigen::NoChange, j + 1);
                Eigen::Vector3d point2d;
                point2d << leftKeypoints[curIndex].pt.x, leftKeypoints[curIndex].pt.y, 1;
                points2d.col(j) = point2d;

                j++;
            }
        }
        matches = std::move(goodMatches);
        prePoints3d_ = std::move(prePoints3d);

    } else if (state_ == NOT_INITIALIZED) {
        state_ = OK;
    }

}

void Tracking::getTranform(Eigen::Matrix3d &R, Eigen::Vector3d &T, const Eigen::Matrix3Xd &points2d) {
    if (points2d.cols()) {
        p3pSolver_->p3pRansac(R, T, prePoints3d_, points2d);
    }
}

void Tracking::updatePreFeatures(std::vector<cv::KeyPoint> &leftKeypoints, cv::Mat &leftDescriptors, Eigen::Matrix4Xd &points3d) {
    //-- update the 3D-2D features
    prePoints3d_ = std::move(points3d);
    preLeftKeypoints_ = std::move(leftKeypoints);
    preLeftDescriptors_ = std::move(leftDescriptors);
}

void Tracking::checkValidProj(const Eigen::Matrix3Xd &points2d, const Eigen::Matrix4Xd &points3d) {
    if (points2d.cols()) {
        std::cout << "num of points3d: " << points3d.cols() << ", num of points2d: " << points2d.cols() << std::endl;
        Eigen::Matrix3Xd points2d_proj;
        Eigen::Matrix<double, 3, 4> P1;
        cv::cv2eigen(P1_, P1);
        points2d_proj = P1 * points3d;
        points2d_proj.array().rowwise() /= points2d_proj.row(2).array();
        double meanError = 0;
        Eigen::Matrix3Xd points2d_err;
        points2d_err = points2d_proj - points2d;
        for (int i = 0; i < points2d.cols(); ++i) 
            meanError += points2d_err.col(i).norm();
    
        std::cout << "meanError: " << meanError / points2d.cols() << std::endl;
    }
}

}