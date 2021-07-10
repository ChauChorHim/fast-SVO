#include <fstream> // ifstream
#include <sstream> // stringstream
#include <iomanip> // setfill, stew
#include "Dataset.h"

using std::endl;

namespace fast_SVO
{

Dataset::Dataset(const string &strPathToSequence) {
    Dataset::loadImages(strPathToSequence);
}

int Dataset::getImagesNum() {
    return vstrImageLeft_.size();
}

void Dataset::loadImages(const string &strPathToSequence) {
    std::ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if(!s.empty()) {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps_.push_back(t);
        }
    }
    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

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
}