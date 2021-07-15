/*
 * @Author: your name
 * @Date: 2021-07-10 16:26:18
 * @LastEditTime: 2021-07-15 10:46:49
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /fast-SVO/include/System.h
 */
#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <tuple>

#include "Dataset.h"
#include "Tracking.h"

#include <opencv2/imgcodecs.hpp>

namespace fast_SVO
{

class Tracking;

class System
{
public:
    // Input dataset
    enum DatasetType {
        MyDataset = 0,
        KITTI = 1
    };

private:
    const DatasetType datasetType_;
    const Dataset* dataset_;
    cv::Mat curImLeft_; 
    cv::Mat curImRight_;
    double curTimestamp_ = 0;

    // Tracker. It receives a frame and computes the associated camera pose.
    Tracking* tracker_;

public:
    System(const Dataset *dataset, const std::string &strSettingFile, const DatasetType datasetType);
    
    double updateImages(const int ni);

    cv::Mat trackStereo(const int ni);

    void saveTrajecotry(const std::string &filename);
};



}// namespace fast_SVO

#endif//SYSTEM_H