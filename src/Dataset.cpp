#include <fstream> // ifstream
#include <sstream> // stringstream
#include <iomanip> // setfill, stew
#include <iostream>
#include "Dataset.hpp"


namespace fast_SVO
{

Dataset::Dataset(const std::string &strPathToSequence, const std::string &sequenceNo) : sequenceNo_(sequenceNo) {
    loadImages(strPathToSequence + sequenceNo_);
    loadTruePoses(strPathToSequence, sequenceNo_); 
}

int Dataset::getImagesNum() const {
    return vstrImageLeft_.size();
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
            vTimestamps_.push_back(t);
        }
    }
    std::string strPrefixLeft = strPathToSequence + "/image_0/";
    std::string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps_.size();
    vstrImageLeft_.resize(nTimes);
    vstrImageRight_.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        vstrImageLeft_[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight_[i] = strPrefixRight + ss.str() + ".png";
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
    for (int i = 0; i < getImagesNum(); ++i) {
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