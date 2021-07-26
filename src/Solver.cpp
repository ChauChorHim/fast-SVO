#include <cmath>
#include <iostream>
#include <random>

#include "Solver.hpp"

#include <opencv2/core.hpp>

namespace fast_SVO
{
    
Solver::Solver(const size_t numIter, const float epsilon, const Eigen::Matrix3d K) : numIter_{numIter}, epsilon_{epsilon}, K_{K} {
    invK_ = K.inverse();
}

void Solver::p3pRansac(Eigen::Matrix3d &R, Eigen::Vector3d &T, const Eigen::Matrix<double, 4, Eigen::Dynamic> &points3D, const Eigen::Matrix<double, 3, Eigen::Dynamic> &points2d) {
    // Get 2D points in camera frame
    Eigen::Matrix<double, 3, Eigen::Dynamic> points2D;
    points2D = invK_ * points2d;
    size_t lastRowNum = points2D.rows() - 1;
    std::cout << points2D << std::endl;
    points2D.array().rowwise() /= points2D.row(lastRowNum).array();
    std::cout << points2D << std::endl;

    long int numPoints{points3D.cols()};
    constexpr int p3pNumPoints{4};
    std::random_device randomDevice;
    std::mt19937 eng(randomDevice());
    std::uniform_int_distribution<std::mt19937::result_type> randomDistribution(0, numPoints);
    std::vector<int> randomNumbers(4, 0);

    for (size_t i = 0; i < numIter_; ++i) {
        // Get 4 random points for p3p 
        for (size_t i = 0; i < randomNumbers.size(); ++i) {
            randomNumbers[i] = randomDistribution(eng);
        }   
        

    }
}

} // namespace fast_SVO
