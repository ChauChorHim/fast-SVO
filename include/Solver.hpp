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
    void p3p(const Eigen::IndexedView<const Eigen::Matrix4Xd, Eigen::internal::AllRange<4>, std::vector<int>> &worldPoints, const Eigen::IndexedView<Eigen::Matrix3Xd, Eigen::internal::AllRange<3>, std::vector<int>> &imageVectors);
    const size_t numIter_;
    const float epsilon_;
    const Eigen::Matrix3d K_;
    Eigen::Matrix3d invK_;

};
    
} // namespace fast_SVO


#endif