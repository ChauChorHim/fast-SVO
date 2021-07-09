#include <fstream> // ifstream
#include <sstream> // stringstream
#include <iomanip> // setfill, stew
#include <iostream> // cerr
#include "Dataset.h"
#include <opencv2/imgcodecs/imgcodecs_c.h>

using std::make_tuple;
using std::cerr;
using std::endl;

namespace fast_SVO
{

Dataset::Dataset(const string &strPathToSequence) {
    Dataset::loadImages(strPathToSequence);
}

int Dataset::getImagesNum() {
    return vstrImageLeft.size();
}

tuple<cv::Mat, cv::Mat, double> Dataset::getImages(const int ni) {
    cv::Mat imLeft = cv::imread(vstrImageLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat imRight = cv::imread(vstrImageRight[ni], CV_LOAD_IMAGE_UNCHANGED);
    if(imLeft.empty()) {
        cerr << endl << "Failed to load image at: " 
             << string(vstrImageLeft[ni]) << endl; 
    }
    return make_tuple(imLeft, imRight, vTimestamps[ni]);
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
            vTimestamps.push_back(t);
        }
    }
    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
}