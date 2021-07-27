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
    size_t lastRowNum = points2D.rows() - 1; // May delete?
    points2D.array().rowwise() /= points2D.row(lastRowNum).array(); // May delete?

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
    Eigen::Vector3d worldPoint1 = worldPoints(Eigen::all, 0).topRows(3);
    Eigen::Vector3d worldPoint2 = worldPoints(Eigen::all, 1).topRows(3);
    Eigen::Vector3d worldPoint3 = worldPoints(Eigen::all, 2).topRows(3);

    // Vectors between world points
    Eigen::Vector3d vect1 = worldPoint2 - worldPoint1;
    Eigen::Vector3d vect2 = worldPoint3 - worldPoint1;

    if (vect1.cross(vect2).norm() == 0) {
        std::cerr << "The three points used in p3p must be non-collinear" << std::endl;
        return;
    }

    // Derive image vectors
    Eigen::Vector3d imageVector1 = imageVectors(Eigen::all, 0);
    Eigen::Vector3d imageVector2 = imageVectors(Eigen::all, 1);
    Eigen::Vector3d imageVector3 = imageVectors(Eigen::all, 2);

    // Compute an orthogonal basis for the tau frame and then invert it
    // The resulting matrix T1 converts from frame nu to frame tau
    Eigen::Vector3d xTau = imageVector1;
    Eigen::Vector3d zTau = imageVector1.cross(imageVector2);
    zTau /= zTau.norm();
    Eigen::Vector3d yTau = zTau.cross(xTau);
    Eigen::Matrix3d T;
    T << xTau, yTau, zTau;
    
    // Transform imageVector3 from nu to tau using T
    Eigen::Vector3d imageVector3_Tau = T * imageVector3;

    // If z-comp of imageVector3_Tau > 0, inverse worldPoint1 and worldPoint2, imageVector1 and imageVector2
    if (imageVector3_Tau(2) > 0) {
        Eigen::Vector3d worldPoint1 = worldPoints(Eigen::all, 1).topRows(3);
        Eigen::Vector3d worldPoint2 = worldPoints(Eigen::all, 0).topRows(3); 
        imageVector1 = imageVectors(Eigen::all, 1);
        imageVector2 = imageVectors(Eigen::all, 0);
        // Recompute the basis of tau frame
        xTau = imageVector1;
        zTau = imageVector1.cross(imageVector2);
        zTau /= zTau.norm();
        yTau = zTau.cross(xTau);
        Eigen::Matrix3d T_tmp;
        T_tmp << xTau, yTau, zTau;
        T = std::move(T_tmp);
    }

    // Compute an orthogonal basis for the eta frame and then invert it
    // The resulting matrix N converts from the world frame to frame nu
    Eigen::Vector3d xEta = worldPoint2 - worldPoint1;
    xEta /= xEta.norm();
    Eigen::Vector3d zEta = worldPoint3 - worldPoint1;
    zEta = xEta.cross(zEta) / xEta.cross(zEta).norm();
    Eigen::Vector3d yEta = zEta.cross(xEta);
    Eigen::Matrix3d N;
    N << xEta, yEta, zEta;

    // Convert worldPoint3 from world frame to nu frame
    Eigen::Vector3d worldPoint3_Eta = N * (worldPoint3 - worldPoint1);
    double p1 = worldPoint3_Eta(0);
    double p2 = worldPoint3_Eta(1);

    // Length of vector worldPoint2 - worldPoint1
    double d12 = (worldPoint2 - worldPoint1).norm();

    // Define phi
    double phi1 = imageVector3_Tau(0) / imageVector3_Tau(2);
    double phi2 = imageVector3_Tau(1) / imageVector3_Tau(2);

    // Define b = cot(beta)
    double cosBeta = imageVector1.transpose() * imageVector2;
    double b = 1 / (1 - pow(cosBeta, 2)) - 1;
    b = cosBeta < 0 ? -sqrt(b) : sqrt(b);

    // Define auxiliary variables that are helpful to type less
    double phi1_pw2 = pow(phi1, 2);
    double phi2_pw2 = pow(phi2, 2);
    double p1_pw2 = pow(p1, 2);
    double p1_pw3 = p1_pw2 * p1;
    double p1_pw4 = p1_pw2 * p1;
    double p2_pw2 = pow(p2, 2);
    double p2_pw3 = p2_pw2 * p1;
    double p2_pw4 = p2_pw2 * p1;
    double d12_pw2 = pow(d12, 2);
    double b_pw2 = pow(b, 2);

    // Define the factors of 4th degree polynomial
    double factor4 = -phi2_pw2 * p2_pw4 
                    - p2_pw4 * phi1_pw2 
                    - p2_pw4;
    double factor3 = 2 * p2_pw3 * d12 * b 
                    + 2 * phi2_pw2 * p2_pw3 * d12 * b 
                    - 2 * phi2 * p2_pw3 * phi1 * d12;
    double factor2 = -phi2_pw2 * p2_pw2 * p1_pw2 
                    - phi2_pw2 * p2_pw2 * d12_pw2 * b_pw2 
                    - phi2_pw2 * p2_pw2 * d12_pw2 
                    + phi2_pw2 * p2_pw4
                    + p2_pw4 * phi1_pw2
                    + 2 * p1 * p2_pw2 * d12
                    + 2 * phi1 * phi2 * p1 * p2_pw2 * d12 * b
                    - p2_pw2 * p1_pw2 * phi1_pw2
                    + 2 * p1 * p2_pw2 * phi2_pw2 * d12
                    - p2_pw2 * d12_pw2 * b_pw2
                    - 2 * p1_pw2 * p2_pw2; 
    double factor1 = 2 * p1_pw2 * p2 * d12 * b
                    + 2 * phi2 * p2_pw3 * phi1 * d12
                    - 2 * phi2_pw2 * p2_pw3 * d12 * b
                    - 2 * p1 * p2 * d12_pw2 * b;
    double factor0 = -2 * phi2 * p2_pw2 * phi1 * p1 * d12 * b
                    + phi2_pw2 * p2_pw2 * d12_pw2
                    + 2 * p1_pw3 * d12
                    - p1_pw2 * d12_pw2
                    + phi2_pw2 * p2_pw2 * p1_pw2
                    - p1_pw4
                    - 2 * phi2_pw2 * p2_pw2 * p1 * d12
                    + p2_pw2 * phi1_pw2 * p1_pw2
                    + phi2_pw2 * p2_pw2 * d12_pw2 * b_pw2;
    std::vector<double> factors {factor4, factor3, factor2, factor1, factor0};
}


} // namespace fast_SVO
