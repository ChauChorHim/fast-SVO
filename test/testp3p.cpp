#include <iostream>
#include <vector>
#include <Eigen/Dense>

std::vector<double> roots4thOrder(const std::vector<double> &factors) {
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
    return allRealRoots;
}

bool uniqueSolution(const Eigen::Matrix<double, 3, 16> &poses, Eigen::Matrix3d &R_est, Eigen::Vector3d &T_est, 
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

void p3p(Eigen::Matrix<double, 4, 4> &worldPoints, 
         Eigen::Matrix<double, 3, 4> &imageVectors,
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
    std::vector<double> roots_ = roots4thOrder(factors);

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
void calculateCurPose(Eigen::Matrix4d &curEstPose, Eigen::Matrix3d R, Eigen::Vector3d T) {
    Eigen::Matrix<double, 3, 4> curEstPose_inhomo;
    Eigen::Matrix4d curEstPose_homo = Eigen::Matrix4d::Zero();
    curEstPose_inhomo << R.transpose(), -R.transpose()*T;
    curEstPose_homo.topRows(3) = curEstPose_inhomo;
    curEstPose_homo(3, 3) = 1;

    curEstPose = curEstPose * curEstPose_homo;
    std::cout << "\ncurEstPose: \n" << curEstPose << std::endl;
}
int main() {
    int i = 5;
    std::vector<Eigen::Matrix<double, 4, 4>> worldPoints(i);
    std::vector<Eigen::Matrix<double, 3, 4>> imagesVectors(i);
    std::vector<Eigen::Matrix<double, 3, 4>> imagesPoints(i);
    Eigen::Matrix4d curEstPose;
    curEstPose << 1, 0, 0, 0,
                  0, 1, 0, 0, 
                  0, 0, 1, 0,
                  0, 0, 0, 1;

    worldPoints[0] << -17.881362915039062,-4.821349620819092,-44.5218620300293,27.40815544128418,
    -1.6092126369476318,2.093446969985962,-6.224493980407715,-9.18687915802002,
    27.47499656677246,106.00076293945312,152.63943481445312,106.00076293945312,
    1.0,1.0,1.0,1.0;
    imagesVectors[0] << -0.5603520082060583,-0.060563334446343625,-0.2952431824248775,0.23098562652307483,
    -0.04842793943581502,0.019709261499204354,-0.03892887107431865,-0.08400937626963213,
    0.8268375666244482,0.9979697528141303,0.9546287269030352,0.9693235089682547;
    imagesPoints[0] << -0.6777050666598119,-0.06068654312975297,-0.3092754011108513,0.23829570250383725,
    -0.05857007638576625,0.01974935757684748,-0.04077906936721881,-0.08666804786262894,
    1.0,1.0,1.0,1.0;

    worldPoints[1] << 13.970573425292969,-18.129749298095703,2.961895704269409,-1.9180668592453003,
    -3.516700506210327,-2.0953123569488525,1.402607798576355,-6.082356929779053,
    29.21649932861328,26.166664123535156,47.47681427001953,127.2000503540039,
    1.0,1.0,1.0,1.0;
    imagesVectors[1] << 0.411086788049169,-0.5872605406823471,0.041119719320198235,-0.036128803283047586,
    -0.10767769128262698,-0.06695640571723117,0.029505146850385345,-0.04773149900281933,
    0.9052144317737434,0.8066237642735966,0.9987184863575748,0.9982065986439275;
    imagesPoints[1] << 0.45413194224450826,-0.7280476557880778,0.04117248241810956,-0.03619371313726927,
    -0.11895268955404903,-0.0830082235148733,0.029543006616402523,-0.04781725453194058,
    1.0,1.0,1.0,1.0;

    worldPoints[2] << -1.5215195417404175,2.1572506427764893,-1.0620871782302856,97.82195281982422,
    -1.6680313348770142,1.9099235534667969,-11.903233528137207,-24.768232345581055,
    8.791999816894531,54.259212493896484,263.7596130371094,379.81451416015625,
    1.0,1.0,1.0,1.0;
    imagesVectors[2] << -0.1997014750864377,0.012879170476122198,-0.030453428486171254,0.22169159042681041,
    -0.18830951679047964,0.03376367388439974,-0.04709138690962982,-0.06482406105566606,
    0.9615918295901005,0.9993468573491762,0.99842625665212,0.9729597524267294;
    imagesPoints[2] << -0.20767800738444797,0.012887587909452103,-0.030501429908590727,0.22785278617524862,
    -0.19583102829685095,0.033785740792701136,-0.047165613480092794,-0.06662563471303276,
    1.0,1.0,1.0,1.0;

    worldPoints[3] << 24.769466400146484,-56.17012023925781,-14.568060874938965,41.75687026977539,
    -24.231081008911133,-4.470845699310303,-2.668896436691284,-15.741430282592773,
    379.81451416015625,127.19874572753906,24.347097396850586,183.1659698486328,
    1.0,1.0,1.0,1.0;
    imagesVectors[3] << 0.031188410222662957,-0.4352523027823327,-0.5439724534993808,0.18857975231468474,
    -0.06644591031189065,-0.03541899110297804,-0.09702236543607923,-0.0869361480830528,
    0.9973024737112643,0.899611542829417,0.8334750328828403,0.9782023222081482;
    imagesPoints[3] << 0.03127276934007939,-0.48382249677832845,-0.6526559669314603,0.19278195117038122,
    -0.06662563471303276,-0.039371428018342064,-0.11640704473233746,-0.08887338141541845,
    1.0,1.0,1.0,1.0;

    worldPoints[4] << 26.704116821289062,3.4070496559143066,13.904589653015137,-1.142388939857483,
    -10.63359546661377,-5.362561225891113,1.2544305324554443,1.4495086669921875,
    158.25845336914062,109.90023040771484,25.439802169799805,54.259212493896484,
    1.0,1.0,1.0,1.0;
    imagesVectors[4] << 0.12832769939250985,-0.008088879192217742,0.44693357593281,-0.06052467900030185,
    -0.06815729879960908,-0.05116982081106765,0.04029227046954372,0.023835438893180478,
    0.9893869739333373,0.998657208190867,0.8936592816304881,0.9978820747386337;
    imagesPoints[4] << 0.12970425402139596,-0.008099755477528947,0.5001163028457292,-0.060653137813057256,
    -0.06888841332592818,-0.05123862361516934,0.04508683711764305,0.023886027714671076,
    1.0,1.0,1.0,1.0;

    for (int i = 0; i < worldPoints.size(); ++i) {
        Eigen::Matrix<double, 3, 16> poses;
        p3p(worldPoints[i], imagesVectors[i], poses);
        Eigen::Matrix3d R_est;
        Eigen::Vector3d T_est;
        bool dummy = uniqueSolution(poses, R_est, T_est, worldPoints[i], imagesPoints[i]);
        //std::cout << "\n-------------\nR_est:\n" << R_est << "\nT_est:\n" << T_est << std::endl;
        calculateCurPose(curEstPose, R_est, T_est);
    }
}