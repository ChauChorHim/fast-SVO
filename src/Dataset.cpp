#include <fstream> // ifstream
#include <sstream> // stringstream
#include <iomanip> // setfill, stew
#include <iostream>
#include "Dataset.hpp"


namespace fast_SVO
{

Dataset::Dataset(const std::string &strPathToSequence, const std::string &sequenceNo) {
    loadImages(strPathToSequence + sequenceNo);
    loadTruePoses(strPathToSequence, sequenceNo); 
}

int Dataset::getImagesAmount() const {
    return strImageLeft_.size();
}

// i: image No. isLeft: return left image file path if isLeft == 1
const std::string Dataset::getStrImage(const int i, const bool isLeft) const {
    return isLeft ? strImageLeft_[i] : strImageRight_[i];
}

const double Dataset::getTimestamps(const int i) const {
    return timestamps_[i];
}

const Eigen::Matrix<double, 3, 4> Dataset::getPoseGroundTruth(const int i) const {
    return posesGroundTruth_[i];
}

void Dataset::loadImages(const std::string &strPathToSequence) {
    std::ifstream fTimes;
    std::string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof()) {
        std::string s;
        getline(fTimes, s);
        if(!s.empty()) {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            timestamps_.push_back(t);
        }
    }
    std::string strPrefixLeft = strPathToSequence + "/image_0/";
    std::string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = timestamps_.size();
    strImageLeft_.resize(nTimes);
    strImageRight_.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        strImageLeft_[i] = strPrefixLeft + ss.str() + ".png";
        strImageRight_[i] = strPrefixRight + ss.str() + ".png";
    }
}


void Dataset::loadTruePoses(const std::string &strPathToSequence, const std::string &sequenceNo) {
    std::string pathToFile = strPathToSequence + std::string("/poses/") + sequenceNo + std::string(".txt");
    std::cout << pathToFile << std::endl;
    std::ifstream fin(pathToFile);
    std::string tmp;
    std::vector<std::string> posesGT;
    while(fin >> tmp) {
        posesGT.push_back(tmp.c_str());
    }
    for (int i = 0; i < getImagesAmount(); ++i) {
        std::vector<double> pose;
        pose.reserve(12);
        for (int j = 0; j < 12; ++j) {
            pose.push_back(std::stod(posesGT[12 * i + j]));
        }
        Eigen::Matrix<double, 3, 4> curRealPose;
        curRealPose << pose[0], pose[1], pose[2], pose[3],
                       pose[4], pose[5], pose[6], pose[7],
                       pose[8], pose[9], pose[10], pose[11];
        posesGroundTruth_.push_back(curRealPose);
    }
    
}

}