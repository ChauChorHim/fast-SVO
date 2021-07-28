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
    state = NOT_INITIALIZED;
    cv::FileStorage fSettings(strSettingFile, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

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


void Tracking::updateImagesFeatures(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp) {
    ORBextractorLeft_->detectAndCompute(imRectLeft, cv::noArray(), leftKeypoints_, leftDescriptors_);    
    ORBextractorRight_->detectAndCompute(imRectRight, cv::noArray(), rightKeypoints_, rightDescriptors_);    
}

void Tracking::matchStereoFeaturesNaive() {
    std::vector<std::vector<cv::DMatch>> knnMatches;
    matcher_->knnMatch(leftDescriptors_, rightDescriptors_, knnMatches, 2);

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
        if (knnMatches[i][0].distance < ratio_thresh * knnMatches[i][1].distance && abs(leftKeypoints_[leftIndex].pt.y - rightKeypoints_[rightIndex].pt.y) < 0.1) {
            knnMatches[i][0].queryIdx = j;
            knnMatches[i][0].trainIdx = j;
            goodMatches.push_back(knnMatches[i][0]);

            leftKeypointsCoor.push_back(leftKeypoints_[leftIndex].pt); // leftKeypointsCoor and rightKeypointsCoor are used in cv::triangulatePoints()
            rightKeypointsCoor.push_back(rightKeypoints_[rightIndex].pt);

            goodLeftKeypoints.push_back(leftKeypoints_[leftIndex]);
            goodRightKeypoints.push_back(rightKeypoints_[rightIndex]);

            goodLeftDescriptors.push_back(leftDescriptors_.row(leftIndex));
            goodRightDescriptors.push_back(rightDescriptors_.row(rightIndex));
            j++;
        }
    }

    //-- Move the goodMatches to matches_
    matches_ = std::move(goodMatches);

    //-- Move the goodLeftKeypoints and goodRightKeypoints to leftKeypoints_ and rightKeypoints_
    leftKeypoints_ = std::move(goodLeftKeypoints);
    rightKeypoints_ = std::move(goodRightKeypoints);

    //-- Move the goodLeftDescriptors and goodRightDescriptors to leftDescriptors_ and rightDescriptors_
    leftDescriptors_ = std::move(goodLeftDescriptors);
    rightDescriptors_ = std::move(goodRightDescriptors);

    //-- Triangulate 3D points with qualified matches
    if (leftKeypointsCoor.size() >= 4) { // P3P needs at least 4 points
        cv::Mat pnts3D(4, leftKeypointsCoor.size(), CV_64F);
        cv::triangulatePoints(P1_, P2_, leftKeypointsCoor, rightKeypointsCoor, pnts3D);
        size_t colsNum = pnts3D.cols;
        points3d_.conservativeResize(Eigen::NoChange, colsNum);
        cv::cv2eigen(pnts3D, points3d_);
        size_t lastRowNum = points3d_.rows() - 1;
        points3d_.array().rowwise() /= points3d_.row(lastRowNum).array();
    }
}

void Tracking::showMatches(const cv::Mat &imRectLeft, const cv::Mat &imRectRight) {
    if (matches_.size() < 4) { // cv::drawMatches can't receive empty input array
        std::cout << "Dump this frame. There are only " << matches_.size() << " matches" << std::endl;
        return;
    }
    cv::Mat imMatches;
    cv::drawMatches(imRectLeft, leftKeypoints_, imRectRight, rightKeypoints_, matches_, imMatches);
    cv::namedWindow("Stereo matching features", cv::WINDOW_AUTOSIZE); // create window
    cv::imshow("Stereo matching features", imMatches); // show the image
    cv::waitKey(1);
}

void Tracking::matchFeaturesNaive() {
    if (state == OK) {
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher_->knnMatch(leftDescriptors_, preLeftDescriptors_, knnMatches, 2);
        std::vector<cv::DMatch> goodMatches;
        std::vector<cv::KeyPoint> goodLeftKeypoints;
        cv::Mat goodLeftDescriptors;
        Eigen::Matrix<double, 4, Eigen::Dynamic> prePoints3d;
        Eigen::Matrix<double, 3, Eigen::Dynamic> points2d;
        
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

                goodLeftKeypoints.push_back(leftKeypoints_[curIndex]);
                goodLeftDescriptors.push_back(leftDescriptors_.row(curIndex));

                const int curCols3D = prePoints3d.cols();
                prePoints3d.conservativeResize(Eigen::NoChange, curCols3D + 1);
                prePoints3d.col(curCols3D) = prePoints3d_.col(preIndex);

                Eigen::Vector3d point2d;
                const int curCols2D = points2d.cols();
                point2d << leftKeypoints_[curIndex].pt.x, leftKeypoints_[curIndex].pt.y, 1;
                points2d.conservativeResize(Eigen::NoChange, curCols2D + 1);
                points2d.col(curCols3D) = point2d;

                j++;
            }
        }
        leftKeypoints_ = std::move(goodLeftKeypoints);
        leftDescriptors_ = std::move(goodLeftDescriptors);
        prePoints3d_ = std::move(prePoints3d);
        points2d_ = std::move(points2d);
        
    } else if (state == NOT_INITIALIZED) {
        prePoints3d_ = points3d_;
        preLeftKeypoints_ = leftKeypoints_;
        preLeftDescriptors_ = leftDescriptors_;
        state = OK;
    }

}

void Tracking::getTranform(Eigen::Matrix3d &R, Eigen::Vector3d &T) {
    if (points2d_.cols()) {
        p3pSolver_->p3pRansac(R, T, prePoints3d_, points2d_);
    }
    //-- update the 3D-2D features
    Eigen::Matrix4Xd prePoints3d = points3d_;
    prePoints3d_ = std::move(prePoints3d);
    preLeftKeypoints_ = leftKeypoints_;
    preLeftDescriptors_ = leftDescriptors_;
}

}