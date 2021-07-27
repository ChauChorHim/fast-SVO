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
    points2D.array().rowwise() /= points2D.row(lastRowNum).array();

    // Normalize the 2D points
    for (int i = 0; i < points2D.cols(); ++i)
        points2D.col(i).normalize();

    // Random machine setup
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
        const auto worldPoints = points3D(Eigen::all, randomNumbers);
        const auto imageVectors = points2D(Eigen::all, randomNumbers);
        p3p(worldPoints, imageVectors);
    }
}

void Solver::p3p(const Eigen::IndexedView<const Eigen::Matrix4Xd, Eigen::internal::AllRange<4>, std::vector<int>> &worldPoints, 
                 const Eigen::IndexedView<Eigen::Matrix3Xd, Eigen::internal::AllRange<3>, std::vector<int>> &imageVectors) {
    // Derive world points
    Eigen::Vector4d worldPoint1 = worldPoints(Eigen::all, 0);
    Eigen::Vector4d worldPoint2 = worldPoints(Eigen::all, 1);
    Eigen::Vector4d worldPoint3 = worldPoints(Eigen::all, 2);
    std::cout << worldPoint1 << std::endl << worldPoint2 << std::endl;

    // Vectors between world points
    Eigen::Vector4d vect1 = worldPoint2 - worldPoint1;
    Eigen::Vector4d vect2 = worldPoint3 - worldPoint1;
/*
    if (vect1.cross(vect2).norm()) {
        std::cerr << "The three points used in p3p must be non-collinear" << std::endl;
    }
    */
}


} // namespace fast_SVO
