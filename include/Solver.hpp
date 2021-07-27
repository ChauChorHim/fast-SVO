#ifndef _SOLVER_H
#define _SOLVER_H

#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/eigen.hpp>

namespace fast_SVO
{
class Tracking;

class Solver {
public:
    Solver(const size_t numIter, const float epsilon, const Eigen::Matrix3d K);
    void p3pRansac(Eigen::Matrix3d &R, Eigen::Vector3d &T, const Eigen::Matrix<double, 4, Eigen::Dynamic> &points3D, const Eigen::Matrix<double, 3, Eigen::Dynamic> &points2d);

private:
    void p3p(const Eigen::Matrix4d &worldPoints, const Eigen::Matrix3d &imageVectors);

    const size_t numIter_;
    const float epsilon_;
    const Eigen::Matrix3d K_;
    Eigen::Matrix3d invK_;

};
    
} // namespace fast_SVO


#endif