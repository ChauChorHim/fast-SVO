#ifndef _SOLVER_H
#define _SOLVER_H


#include <opencv2/core/mat.hpp>

namespace fast_SVO
{
class Tracking;

class Solver {
public:
    Solver(const float confidence, const float probability, const cv::Mat K);
    void p3pRansac(cv::Mat &R, cv::Mat &T, const cv::Mat &points3D, std::vector<cv::KeyPoint> &keypoints2D);

private:
    const std::size_t numIter_;
    const float epsilon_;
    const cv::Mat K_;
    const cv::Mat invK_;

    cv::Mat points2D_;
};
    
} // namespace fast_SVO


#endif