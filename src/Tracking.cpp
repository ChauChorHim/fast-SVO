/*
 * 
 */
/*
 * 
 */
#include <iostream>
#include <thread>

#include "Tracking.hpp"

#include <opencv2/calib3d.hpp>

namespace fast_SVO
{

Tracking::Tracking(const std::string &strSettingFile) {
    state_ = NOT_INITIALIZED;
    cv::FileStorage fSettings(strSettingFile, cv::FileStorage::READ);
    double fx = fSettings["Camera.fx"];
    double fy = fSettings["Camera.fy"];
    double cx = fSettings["Camera.cx"];
    double cy = fSettings["Camera.cy"];

    K_ << fx, 0, cx,
          0, fy, cy,
          0, 0, 1;

    cv::Matx34d P = cv::Matx34d(K_(0, 0), K_(0, 1), K_(0, 2), 0,
                                K_(1, 0), K_(1, 1), K_(1, 2), 0,
                                K_(2, 0), K_(2, 1), K_(2, 2), 0);
    P1_ = P;

    baseline_ = fSettings["Camera.bf"];
    P(0, 3) = -baseline_;
    P2_ = P;

    distCoef_(0) = fSettings["Camera.k1"];
    distCoef_(1) = fSettings["Camera.k2"];
    distCoef_(2) = fSettings["Camera.p1"];
    distCoef_(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0) {
        distCoef_.resize(5);
        distCoef_(4) = k3;
    }

    float fps = fSettings["Camera.fps"];
    if (fps == 0)
        fps = 30;

    std::cout << std::endl << "Camera Parameters: " << std::endl;
    std::cout << "- fx: " << fx << std::endl;
    std::cout << "- fy: " << fy << std::endl;
    std::cout << "- cx: " << cx << std::endl;
    std::cout << "- cy: " << cy << std::endl;
    std::cout << "- k1: " << distCoef_(0) << std::endl;
    std::cout << "- k2: " << distCoef_(1) << std::endl;
    if(distCoef_.rows()==5)
        std::cout << "- k3: " << distCoef_(4) << std::endl;
    std::cout << "- p1: " << distCoef_(2) << std::endl;
    std::cout << "- p2: " << distCoef_(3) << std::endl;
    std::cout << "- fps: " << fps << std::endl;

    int rgbOrder = fSettings["Camera.RGB"];
    rgbOrder_ = rgbOrder;
    if(rgbOrder_)
        std::cout << "- color order: RGB (ignored if grayscale)" << std::endl;
    else
        std::cout << "- color order: BGR (ignored if grayscale)" << std::endl;

    // Load ORB parameters
    const int nFeatures = fSettings["ORBextractor.nFeatures"];
    const float scaleFactor = fSettings["ORBextractor.scaleFactor"];
    const int nLevels = fSettings["ORBextractor.nLevels"];
    ORBextractorLeft_ = cv::ORB::create(nFeatures,scaleFactor,nLevels);
    ORBextractorRight_ = cv::ORB::create(nFeatures,scaleFactor,nLevels);

    std::cout << std::endl  << "ORB Extractor Parameters: " << std::endl;
    std::cout << "- Number of Features: " << nFeatures << std::endl;
    std::cout << "- Scale Levels: " << nLevels << std::endl;
    std::cout << "- Scale Factor: " << scaleFactor << std::endl;

    matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    // Load RANSAC parameters
    const float confidence = fSettings["RANSAC.confidence"];
    const float probability = fSettings["RANSAC.probability"];
    const size_t numIter = ceil(log(1. - confidence) / log(1. - pow(probability, 4)));
    const float epsilon = K_(0, 2) * 0.02;
    std::cout << std::endl  << "RANSAC Parameters: " << std::endl;
    std::cout << "- RANSAC confidence: " << confidence << std::endl;
    std::cout << "- RANSAC probability: " << probability << std::endl;
    std::cout << "- RANSAC Number of Iteration: " << numIter << std::endl;
    std::cout << "- RANSAC epsilon: " << epsilon << std::endl;
    p3pSolver_ = new Solver(numIter, epsilon, K_);
}

void Tracking::updateImagesFeatures(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, 
                                    std::vector<cv::KeyPoint> &leftKeypoints, std::vector<cv::KeyPoint> &rightKeypoints, 
                                    cv::Mat &leftDescriptors, cv::Mat &rightDescriptors) {
    ORBextractorLeft_->detectAndCompute(imRectLeft, cv::noArray(), leftKeypoints, leftDescriptors);    
    ORBextractorRight_->detectAndCompute(imRectRight, cv::noArray(), rightKeypoints, rightDescriptors);    
}

void Tracking::matchStereoFeaturesNaive(std::vector<cv::KeyPoint> &leftKeypoints, std::vector<cv::KeyPoint> &rightKeypoints,
                                        cv::Mat &leftDescriptors, cv::Mat &rightDescriptors, std::vector<cv::DMatch> &matches,
                                        Eigen::Matrix<double, 4, Eigen::Dynamic> &points3d) {
    std::vector<std::vector<cv::DMatch>> knnMatches;
    matcher_->knnMatch(leftDescriptors, rightDescriptors, knnMatches, 2);

    std::vector<cv::Point2f> leftKeypointsCoor, rightKeypointsCoor;
    cv::Mat goodLeftDescriptors, goodRightDescriptors;
    std::vector<cv::KeyPoint> goodLeftKeypoints, goodRightKeypoints;
    std::vector<cv::DMatch> goodMatches;

    //-- Filter matches using the Lowe's ratio test and epipolar constraint
    const float ratio_thresh = 0.8f;
    size_t j = 0;
    for (size_t i = 0; i < knnMatches.size(); i++)
    {
        size_t leftIndex = knnMatches[i][0].queryIdx;
        size_t rightIndex = knnMatches[i][0].trainIdx;
        if (knnMatches[i][0].distance < ratio_thresh * knnMatches[i][1].distance && abs(leftKeypoints[leftIndex].pt.y - rightKeypoints[rightIndex].pt.y) < 0.1) {
            knnMatches[i][0].queryIdx = j;
            knnMatches[i][0].trainIdx = j;
            goodMatches.push_back(knnMatches[i][0]);

            leftKeypointsCoor.push_back(leftKeypoints[leftIndex].pt); // leftKeypointsCoor and rightKeypointsCoor are used in cv::triangulatePoints()
            rightKeypointsCoor.push_back(rightKeypoints[rightIndex].pt);

            goodLeftKeypoints.push_back(leftKeypoints[leftIndex]);
            goodRightKeypoints.push_back(rightKeypoints[rightIndex]);

            goodLeftDescriptors.push_back(leftDescriptors.row(leftIndex));
            goodRightDescriptors.push_back(rightDescriptors.row(rightIndex));
            j++;
        }
    }

    //-- Move the goodMatches to matches_
    matches = std::move(goodMatches);

    //-- Move the goodLeftKeypoints to leftKeypoints_
    leftKeypoints = std::move(goodLeftKeypoints);
    rightKeypoints = std::move(goodRightKeypoints);

    //-- Move the goodLeftDescriptors to leftDescriptors_
    leftDescriptors = std::move(goodLeftDescriptors);

    //-- Triangulate 3D points with qualified matches
    if (leftKeypointsCoor.size() >= 4) { // P3P needs at least 4 points
        cv::Mat pnts3D(4, leftKeypointsCoor.size(), CV_64F);
        cv::triangulatePoints(P1_, P2_, leftKeypointsCoor, rightKeypointsCoor, pnts3D);
        size_t colsNum = pnts3D.cols;
        points3d.conservativeResize(Eigen::NoChange, colsNum);
        cv::cv2eigen(pnts3D, points3d);
        size_t lastRowNum = points3d.rows() - 1;
        points3d.array().rowwise() /= points3d.row(lastRowNum).array();
    }

    // Check the projection
    //Eigen::Matrix3Xd points2d;
    //points2d.conservativeResize(Eigen::NoChange, leftKeypoints.size());
    //Eigen::Vector3d point2d;
    //for (int i = 0; i < leftKeypoints.size(); ++i) {
    //    point2d << leftKeypoints[i].pt.x, leftKeypoints[i].pt.y, 1;
    //    points2d.col(i) = point2d; 
    //}
    //checkValidProj(points2d, points3d);

}

void Tracking::showMatches(const cv::Mat &image1, const cv::Mat &image2, 
                           const std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2,
                           const std::vector<cv::DMatch> &matches) {
    if (matches.size() < 4) { // cv::drawMatches can't receive empty input array
        std::cout << "Dump this frame. There are only " << matches.size() << " matches" << std::endl;
        return;
    }
    cv::Mat imMatches;
    cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, imMatches);
    cv::namedWindow("Stereo matching features", cv::WINDOW_AUTOSIZE); // create window
    cv::imshow("Stereo matching features", imMatches); // show the image
    cv::waitKey(1);
}

void Tracking::matchFeaturesNaive(const std::vector<cv::KeyPoint> &leftKeypoints, const cv::Mat &leftDescriptors, std::vector<cv::DMatch> &matches, 
                                  Eigen::Matrix<double, 3, Eigen::Dynamic> &points2d) {
    if (state_ == OK) {
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher_->knnMatch(leftDescriptors, preLeftDescriptors_, knnMatches, 2);
        std::vector<cv::DMatch> goodMatches;
        Eigen::Matrix<double, 4, Eigen::Dynamic> prePoints3d;
        
        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.8f;
        size_t j = 0;

        for (size_t i = 0; i < knnMatches.size(); i++)
        {
            size_t curIndex = knnMatches[i][0].queryIdx;
            size_t preIndex = knnMatches[i][0].trainIdx;
            if (knnMatches[i][0].distance < ratio_thresh * knnMatches[i][1].distance) {
                knnMatches[i][0].queryIdx = j;
                knnMatches[i][0].trainIdx = j;
                goodMatches.push_back(knnMatches[i][0]);

                prePoints3d.conservativeResize(Eigen::NoChange, j + 1);
                prePoints3d.col(j) = prePoints3d_.col(preIndex);

                points2d.conservativeResize(Eigen::NoChange, j + 1);
                Eigen::Vector3d point2d;
                point2d << leftKeypoints[curIndex].pt.x, leftKeypoints[curIndex].pt.y, 1;
                points2d.col(j) = point2d;

                j++;
            }
        }
        matches = std::move(goodMatches);
        prePoints3d_ = std::move(prePoints3d);

    } else if (state_ == NOT_INITIALIZED) {
        state_ = OK;
    }

}

void Tracking::getTranform(Eigen::Matrix3d &R, Eigen::Vector3d &T, const Eigen::Matrix3Xd &points2d) {
    if (points2d.cols()) {
        std::vector<Eigen::Matrix<double, 4, 4>> worldPoints(8);
        std::vector<Eigen::Matrix<double, 3, 4>> imagesVectors(8);
worldPoints[0] << -65.13568878173828,-105.75692749023438,43.20112228393555,-15.890262603759766,
-3.924161672592163,1.8327897787094116,-19.655210494995117,-1.4293158054351807,
183.1659698486328,152.6389617919922,316.50885009765625,24.347074508666992,
1.0,1.0,1.0,1.0;

imagesVectors[0] << -0.3495787754600492,-0.5829332676314201,0.11912015591505079,-0.5615021430083297,
-0.020067761630534513,0.009755510608632289,-0.06086916614101028,-0.049891012756984085,
0.9366920330028393,0.8124614670870983,0.9910122769512354,0.825969872479097;




worldPoints[1] << 13.970573425292969,-18.129749298095703,2.961895704269409,-1.9180668592453003,
-3.516700506210327,-2.0953123569488525,1.402607798576355,-6.082356929779053,
29.21649932861328,26.166664123535156,47.47681427001953,127.2000503540039,
1.0,1.0,1.0,1.0;

imagesVectors[1] << 0.411086788049169,-0.5872605406823471,0.041119719320198235,-0.036128803283047586,
-0.10767769128262698,-0.06695640571723117,0.029505146850385345,-0.04773149900281933,
0.9052144317737434,0.8066237642735966,0.9987184863575748,0.9982065986439275;




worldPoints[2] << -47.57573318481445,-7.504974842071533,48.10525131225586,30.679122924804688,
-3.396549940109253,-5.5206193923950195,-8.624061584472656,-12.133024215698242,
127.19874572753906,94.95362854003906,189.90725708007812,152.63755798339844,
1.0,1.0,1.0,1.0;

imagesVectors[2] << -0.37544815164314765,-0.10708412750424762,0.219306028572773,0.16979519534690712,
-0.028649492790105445,-0.05826797444135037,-0.04563627119931936,-0.07808856397874883,
0.92640050301725,0.9925410987919626,0.974588218984131,0.982380663395225;




worldPoints[3] << -2.135514259338379,-85.57598114013672,-13.714941024780273,6.744651794433594,
-4.889528751373291,-5.353606224060059,-2.5719826221466064,-10.504088401794434,
105.99895477294922,219.8004608154297,22.895851135253906,189.90725708007812,
1.0,1.0,1.0,1.0;

imagesVectors[3] << -0.06051099346921309,-0.3955896864566439,-0.5451102975448399,0.0015709752369166814,
-0.04599464891198967,-0.02460545559859196,-0.09840890761572454,-0.05804207816998464,
0.99710727203287,0.9180976917103756,0.8325685860110601,0.9983129014484959;




worldPoints[4] << -12.324196815490723,6.865288069109514e+17,-7.405273914337158,10.569441795349121,
-9.429786682128906,-3.6526930761927885e+17,-3.381866216659546,-3.6300301551818848,
189.90725708007812,3.981763987124519e+18,158.25442504882812,26.166677474975586,
1.0,1.0,1.0,1.0;

imagesVectors[4] << -0.10240511134775618,0.13020880971873205,-0.0854897289202376,0.33812321631577347,
-0.05073434648851053,-0.09057425266125096,-0.022976334326284504,-0.13199842176075954,
0.9934481462342346,0.9873408583799655,0.9960740907734079,0.9317988555696693;




worldPoints[5] << -1.6064803589859574e+18,-37.00172805786133,-1.7521591186523438,-7.995590686798096,
6.698791084899697e+17,-5.687343597412109,-9.295648574829102,-1.3901387453079224,
-1.0922434161117495e+19,152.63943481445312,183.1659698486328,152.6413116455078,
1.0,1.0,1.0,1.0;

imagesVectors[5] << 0.10571108889760501,-0.27671661949367216,-0.052241246154710395,-0.0907122767590967,
-0.06588097597072474,-0.03578021507347411,-0.05119883691887366,-0.009069301440185164,
0.9922121056957667,0.9602852121663076,0.9973211776044653,0.9958358452157505;




worldPoints[6] << -8.410284042358398,-19.75961685180664,-91.25357055664062,-23.021133422851562,
-6.705440044403076,-10.63332462310791,-6.761606693267822,-6.73090124130249,
109.90023040771484,158.25442504882812,152.6385040283203,105.50474548339844,
1.0,1.0,1.0,1.0;

imagesVectors[6] << -0.12179862136555437,-0.1678616558023033,-0.5510922676198612,-0.2562941703526485,
-0.06285880874324624,-0.06775035723728907,-0.039855297787484145,-0.06317130903678382,
0.9905623988405932,0.9834797169263475,0.833491972251605,0.9645323654278413;




worldPoints[7] << -25.902385711669922,16.12737274169922,-17.329463958740234,5.018422603607178,
-6.460633277893066,-17.275466918945312,-9.832795143127441,-4.288181781768799,
94.95362854003906,263.77081298828125,183.1659698486328,109.8982925415039,
1.0,1.0,1.0,1.0;

imagesVectors[7] << -0.3108839135730395,0.012234828606890654,-0.14275634545830798,-0.005650956104789901,
-0.06852208454832788,-0.06939434143079015,-0.055945576524900724,-0.04142712427752577,
0.9479746390123857,0.9975142777656604,0.9881754491479156,0.9991255477011867;

        p3pSolver_->p3pRansac(R, T, prePoints3d_, points2d, worldPoints, imagesVectors);
    }
}

void Tracking::updatePreFeatures(std::vector<cv::KeyPoint> &leftKeypoints, cv::Mat &leftDescriptors, Eigen::Matrix4Xd &points3d) {
    //-- update the 3D-2D features
    prePoints3d_ = std::move(points3d);
    preLeftKeypoints_ = std::move(leftKeypoints);
    preLeftDescriptors_ = std::move(leftDescriptors);
}

void Tracking::checkValidProj(const Eigen::Matrix3Xd &points2d, const Eigen::Matrix4Xd &points3d) {
    if (points2d.cols()) {
        std::cout << "num of points3d: " << points3d.cols() << ", num of points2d: " << points2d.cols() << std::endl;
        Eigen::Matrix3Xd points2d_proj;
        Eigen::Matrix<double, 3, 4> P1;
        cv::cv2eigen(P1_, P1);
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

}