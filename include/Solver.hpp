#ifndef _SOLVER_H
#define _SOLVER_H


#include <opencv2/core/mat.hpp>

namespace fast_SVO
{
class Tracking;

class Solver {
public:
    Solver(const size_t numIter, const float epsilon, const cv::Mat K);
    void p3pRansac(cv::Mat &R, cv::Mat &T, const cv::Mat &points3D, const cv::Mat &points2d);

private:
    const size_t numIter_;
    const float epsilon_;
    const cv::Mat K_;
    cv::Mat invK_;

    // 2D points in left camera frame
    cv::Mat points2D_;
};
    
} // namespace fast_SVO


#endif