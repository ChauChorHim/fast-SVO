#include <cmath>
#include <iostream>

#include "Solver.hpp"

#include <opencv2/core.hpp>

namespace fast_SVO
{
    
//Solver::Solver(const float confidence, const float probability, const cv::Mat K) : numIter_{ceil(log(1. - confidence) / log(1. - pow(probability, 4)))}, K_{K}, epsilon_{K_.at<float>(0, 2) * 0.02} {
Solver::Solver(const size_t numIter, const float epsilon, const cv::Mat K) : numIter_{numIter}, epsilon_{epsilon}, K_{K} {
    int success = cv::invert(K, invK_);
    if(0 == success) {
        std::cout << "K is singular" << std::endl;
    } 
}

void Solver::p3pRansac(cv::Mat &R, cv::Mat &T, const cv::Mat &points3D, const cv::Mat &points2d) {
    // Get 2D points in camera frame
    points2D_ = invK_ * points2d;
    points2D_ /= points2D_.at<float>(2);
}

} // namespace fast_SVO
