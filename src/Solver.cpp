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

void Solver::p3pRansac(Eigen::Matrix3d &R, Eigen::Vector3d &T, 
                       const Eigen::Matrix4Xd &prePoints3d, const Eigen::Matrix3Xd &points2d,
                       std::vector<Eigen::Matrix<double, 4, 4>> worldPoints,
                       std::vector<Eigen::Matrix<double, 3, 4>> imagesVectors) {
    // Get 2D vectors in camera frame
    Eigen::Matrix<double, 3, Eigen::Dynamic> vectors2d;
    vectors2d = invK_ * points2d;
    size_t lastRowNum = vectors2d.rows() - 1; // May delete?
    vectors2d.array().rowwise() /= vectors2d.row(lastRowNum).array(); // May delete?

    // Normalize the 2D vectors
    for (int i = 0; i < vectors2d.cols(); ++i)
        vectors2d.col(i).normalize();

    // Random machine setup
    long int numPoints{prePoints3d.cols()};
    std::random_device randomDevice;
    std::mt19937 eng(randomDevice());
    std::uniform_int_distribution<std::mt19937::result_type> randomDistribution(0, numPoints);
    std::vector<int> randomNumbers(4, 0);

    // potential poses
    Eigen::Matrix<double, 3, 16> poses;

    // temp estimated R and T
    Eigen::Matrix3d R_est;
    Eigen::Vector3d T_est;

    // Best inliers number
    size_t bestInliersNum = 0;

    // Best mean error
    double meanError = 99999;

    static int i = 0;
    std::cout << worldPoints[i] << std::endl;
    p3p(worldPoints[i], imagesVectors[i], poses);
    bool validEstimate = uniqueSolution(poses, R_est, T_est, worldPoints[i], imagesVectors[i]);
    std::cout << "R_est:\n" << R_est << "\nT_est:\n" << T_est << std::endl;
    i++;
/*
    for (size_t i = 0; i < numIter_; ++i) {
        // Get 4 random points for p3p 
        for (size_t i = 0; i < randomNumbers.size(); ++i) {
            randomNumbers[i] = randomDistribution(eng);
        }   
        const Eigen::Matrix<double, 4, 4> worldPoints = prePoints3d(Eigen::all, randomNumbers);
        const Eigen::Matrix<double, 3, 4> imageVectors = vectors2d(Eigen::all, randomNumbers);

        // get potential poses (R and T)
        p3p(worldPoints, imageVectors, poses);
        
        // disambiguate the potential poses and get the best R_est and T_est
        bool validEstimate = uniqueSolution(poses, R_est, T_est, worldPoints, imageVectors);

        if(validEstimate) {
            // Construct the estimated projection matrix P_est
            Eigen::Matrix<double, 3, 4> P_est;
            P_est << R_est, T_est;
            P_est = K_ * P_est;
            // Reproject 3D points to the image plane
            Eigen::Matrix3Xd points2d_est = P_est * prePoints3d;
            points2d_est.array().rowwise() /= points2d_est.row(2).array();
            // Check consensus
            size_t inliersNum = 0;
            double meanError_tmp = 0;

            for (int i = 0; i < points2d.cols(); ++i) {
                meanError_tmp += (points2d_est.col(i)-points2d.col(i)).norm();
                if ((points2d.col(i) - points2d_est.col(i)).norm() < epsilon_)
                    ++inliersNum;
            }
            if (inliersNum > bestInliersNum) {
                R = R_est;
                T = T_est;
                bestInliersNum = inliersNum;
                meanError = meanError_tmp / points2d.cols(); 
            }
        }
    }
*/
//std::cout << "best mean error: " << meanError << ", bestInliersum = " << bestInliersNum << ", points2d.cols() = " << points2d.cols() << std::endl;
}

void Solver::p3p(const Eigen::Matrix<double, 4, 4> &worldPoints, 
                 const Eigen::Matrix<double, 3, 4> &imageVectors,
                 Eigen::Matrix<double, 3, 16> &poses) {
    // Derive world points
    Eigen::Vector3d worldPoint1 = worldPoints(Eigen::all, 0).topRows(3);
    Eigen::Vector3d worldPoint2 = worldPoints(Eigen::all, 1).topRows(3);
    Eigen::Vector3d worldPoint3 = worldPoints(Eigen::all, 2).topRows(3);

    // Vectors between world points
    Eigen::Vector3d vect1 = worldPoint2 - worldPoint1;
    Eigen::Vector3d vect2 = worldPoint3 - worldPoint1;

    if (vect1.cross(vect2).norm() == 0) {
        //std::cerr << "The three points used in p3p must be non-collinear" << std::endl;
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
    T.transposeInPlace();
    
    // Transform imageVector3 from nu to tau using T
    Eigen::Vector3d imageVector3_Tau = T * imageVector3;

    // If z-comp of imageVector3_Tau > 0, inverse worldPoint1 and worldPoint2, imageVector1 and imageVector2
    if (imageVector3_Tau(2) > 0) {
        worldPoint1 = worldPoints(Eigen::all, 1).topRows(3);
        worldPoint2 = worldPoints(Eigen::all, 0).topRows(3); 
        imageVector1 = imageVectors(Eigen::all, 1);
        imageVector2 = imageVectors(Eigen::all, 0);
        // Recompute the basis of tau frame
        xTau = imageVector1;
        zTau = imageVector1.cross(imageVector2);
        zTau /= zTau.norm();
        yTau = zTau.cross(xTau);
        Eigen::Matrix3d T_tmp;
        T_tmp << xTau, yTau, zTau;
        T_tmp.transposeInPlace();
        T = std::move(T_tmp);
        imageVector3_Tau = T * imageVector3;
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
    N.transposeInPlace();

    // Convert worldPoint3 from world frame to nu frame
    Eigen::Vector3d worldPoint3_Nu = N * (worldPoint3 - worldPoint1);
    double p1 = worldPoint3_Nu(0);
    double p2 = worldPoint3_Nu(1);

    // Length of vector worldPoint2 - worldPoint1
    double d12 = (worldPoint2 - worldPoint1).norm();

    // Define phi
    double phi1 = imageVector3_Tau(0) / imageVector3_Tau(2);
    double phi2 = imageVector3_Tau(1) / imageVector3_Tau(2);

    // Define b = cot(beta)
    double cosBeta = imageVector1.transpose() * imageVector2;
    double b = 1 / (1 - std::pow(cosBeta, 2)) - 1;
    b = cosBeta < 0 ? -sqrt(b) : sqrt(b);


    // Define auxiliary variables that are helpful to type less
    double phi1_pw2 = std::pow(phi1, 2);
    double phi2_pw2 = std::pow(phi2, 2);
    double p1_pw2 = std::pow(p1, 2);
    double p1_pw3 = p1_pw2 * p1;
    double p1_pw4 = p1_pw3 * p1;
    double p2_pw2 = std::pow(p2, 2);
    double p2_pw3 = p2_pw2 * p2;
    double p2_pw4 = p2_pw3 * p2;
    double d12_pw2 = std::pow(d12, 2);
    double b_pw2 = std::pow(b, 2);

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

    std::vector<double> factors {factor0, factor1, factor2, factor3, factor4};
    // Solve the fourth order equation
    roots4thOrder(factors);

    // Backsubstitute solutions in other equations
    for (int i = 0; i < roots_.size(); ++i) {
        double cotAlpha = (-phi1 * p1 / phi2 - roots_[i] * p2 + d12 * b) / (-phi1 * roots_[i] * p2 / phi2 + p1 - d12);
        double cosTheta = roots_[i];
            
        double sinTheta = std::sqrt(1 - pow(roots_[i], 2));
        double sinAlpha = std::sqrt(1 / (pow(cotAlpha, 2) + 1));
        double cosAlpha = std::sqrt(1 - pow(sinAlpha, 2));

        if (cotAlpha < 0) {
            cosAlpha = -cosAlpha;
        }

//std::cout << sinTheta << ", " << cosTheta << ", " << sinAlpha << ", " << cosAlpha << std::endl << std::endl;

        // Build C_nu
        Eigen::Vector3d C_nu;
        C_nu << d12 * cosAlpha * (sinAlpha * b + cosAlpha),
                cosTheta * d12 * sinAlpha * (sinAlpha * b + cosAlpha),
                sinTheta * d12 * sinAlpha * (sinAlpha * b + cosAlpha);
        
        // Compute C
        Eigen::Vector3d C = worldPoint1 + N.transpose() * C_nu;

        // Build Q
        Eigen::Matrix3d Q;
        Q << -cosAlpha, -sinAlpha * cosTheta, -sinAlpha * sinTheta,
             sinAlpha, -cosAlpha * cosTheta, -cosAlpha * sinTheta,
             0, -sinTheta, cosTheta;

        // Compute R
        Eigen::Matrix3d R = N.transpose() * Q.transpose() * T;

        poses.col(4 * i) = C;
        poses.middleCols(4 * i + 1, 3) = R;
    }
}


void Solver::roots4thOrder(const std::vector<double> &factors) {
    Eigen::Matrix4d matrixAdjoint; 
    matrixAdjoint << 0, 0, 0, -factors[0]/factors[4],
                     1, 0, 0, -factors[1]/factors[4],
                     0, 1, 0, -factors[2]/factors[4],
                     0, 0, 1, -factors[3]/factors[4];
    Eigen::Matrix<std::complex<double>, 4, 1> matrixEigen;
    matrixEigen = matrixAdjoint.eigenvalues();
//std::cout << matrixEigen << std::endl;
    std::vector<double> allRealRoots;
    for(int i = 0; i < 4; ++i) {
        if (matrixEigen[i].imag() == 0)
            allRealRoots.push_back(matrixEigen[i].real());
    }
    roots_ = std::move(allRealRoots);
}

bool Solver::uniqueSolution(const Eigen::Matrix<double, 3, 16> &poses, Eigen::Matrix3d &R_est, Eigen::Vector3d &T_est, 
                            const Eigen::Matrix<double, 4, 4> &worldPoints, const Eigen::Matrix<double, 3, 4> &imageVectors) {
    try
    {
        std::vector<Eigen::Matrix3d> validR;
        std::vector<Eigen::Vector3d> validT;
        for (int i = 0; i < int(poses.cols() / 4); ++i) {
//std::cout << "poses: " << poses << std::endl << std::endl;
            Eigen::Matrix3d R_tmp = poses.middleCols(4 * i + 1, 3).transpose();
//std::cout << "R_tmp: " << R_tmp << "\n\n";
            Eigen::Vector3d T_tmp = -R_tmp * poses.col(4 * i); 
            Eigen::Matrix3d T_tmp_3;
            T_tmp_3 << T_tmp.transpose(),
                       T_tmp.transpose(),
                       T_tmp.transpose();
            Eigen::Matrix3d proj = R_tmp * worldPoints.topLeftCorner(3, 3) + T_tmp_3.transpose();
//std::cout << proj.row(2) << std::endl;
            if (proj.row(2).minCoeff() > 0) {
                validR.push_back(R_tmp);
                validT.push_back(T_tmp);
            }
        }
        int extrinsicIndex = -1;
        double bestError = 99999;
        for (int i = 0; i < validT.size(); ++i) {
            Eigen::Vector3d proj = validR[i] * worldPoints.topRightCorner(3, 1) + validT[i];
            proj /= proj[2];
            double curError = (proj.topRows(2) - imageVectors.topRightCorner(2, 1)).norm();
//std::cout << curError << std::endl;
            if (curError < bestError) {
                extrinsicIndex = i;
                bestError = curError;
            }
        }
//std::cout << extrinsicIndex << std::endl;
        if (-1 == extrinsicIndex) {
            R_est = Eigen::Matrix3d::Zero();
            T_est = Eigen::Vector3d::Zero();
            return false;
        } else {
            R_est = validR[extrinsicIndex];
            T_est = validT[extrinsicIndex];
            return true;
        }
    }
    catch(const std::exception& e)
    {
        R_est = Eigen::Matrix3d::Zero();
        T_est = Eigen::Vector3d::Zero();
        std::cerr << e.what() << '\n';
        return false;
    }
    
}

void Solver::checkValidProj(const Eigen::Matrix3Xd &points2d, const Eigen::Matrix4Xd &points3d) {
    if (points2d.cols()) {
        std::cout << "num of points3d: " << points3d.cols() << ", num of points2d: " << points2d.cols() << std::endl;
        Eigen::Matrix3Xd points2d_proj;
        Eigen::Matrix<double, 3, 4> P1;
        Eigen::Vector3d T1;
        T1 << 0, 0, 1;
        P1 << K_, T1;
        points2d_proj = P1 * points3d;
        points2d_proj.array().rowwise() /= points2d_proj.row(2).array();
        double meanError = 0;
        Eigen::Matrix3Xd points2d_err;
        points2d_err = points2d_proj - points2d;
        for (int i = 0; i < points2d.cols(); ++i) 
            meanError += points2d_err.col(i).norm();
    
        std::cout << "meanError: " << meanError / points2d.cols() << std::endl;
    }
}

} // namespace fast_SVO
