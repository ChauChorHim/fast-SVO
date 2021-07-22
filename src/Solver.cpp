#include <cmath>
#include <iostream>

#include "Solver.hpp"

#include <opencv2/core.hpp>

namespace fast_SVO
{
    
Solver::Solver(const float confidence, const float probability, const cv::Mat K) : numIter_{ceil(log(1. - confidence) / log(1. - pow(probability, 4)))}, K_{K}, epsilon_{K_.at<float>(0, 2) * 0.02} {
    if(cv::invert(K, invK_) == 0) {
        std::cout << "K is singular" << std::endl;
    } 
}

void Solver::p3pRansac(cv::Mat &R, cv::Mat &T, const cv::Mat &points3D, std::vector<cv::KeyPoint> &keypoints2D) {
    // Get 2D points in camera frame
}

} // namespace fast_SVO
