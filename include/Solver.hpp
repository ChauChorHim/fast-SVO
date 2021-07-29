#ifndef _SOLVER_H
#define _SOLVER_H

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <complex>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/eigen.hpp>

namespace fast_SVO
{
class Tracking;

class Solver {
public:
    Solver(const size_t numIter, const float epsilon, const Eigen::Matrix3d K);
    void p3pRansac(Eigen::Matrix3d &R, 
                   Eigen::Vector3d &T, 
                   const Eigen::Matrix<double, 4, Eigen::Dynamic> &points3d, 
                   const Eigen::Matrix<double, 3, Eigen::Dynamic> &points2d);

private:
    void p3p(const Eigen::Matrix4d &worldPoints, 
             const Eigen::Matrix<double, 3, 4> &imageVectors, 
             Eigen::Matrix<double, 3, 16> &poses);
    void roots4thOrder(const std::vector<double> &factors);
    bool uniqueSolution(const Eigen::Matrix<double, 3, 16> &poses, Eigen::Matrix3d &R_est, Eigen::Vector3d &T_est, 
                        const Eigen::Matrix4d &worldPoints, const Eigen::Matrix<double, 3, 4> &imageVectors);
    const size_t numIter_;
    const float epsilon_;
    const Eigen::Matrix3d K_;
    Eigen::Matrix3d invK_;
    std::vector<double> roots_;
};
    
} // namespace fast_SVO


#endif