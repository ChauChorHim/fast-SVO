#include <iostream>
#include <string>

#include "Dataset.hpp"
#include "System.hpp"

int main(int argc, char **argv) {
    if (argc != 4) {
        std::cerr << std::endl << "Usage: ./stereo_kitti path_to_settings path_to_sequence sequence_no" << std::endl;
        return 1;
    }

    // Retrive sequence path to dataset
    // The constructor use private LoadImages
    fast_SVO::Dataset KITTI = fast_SVO::Dataset(std::string(argv[2]), std::string(argv[3])); // path_to_sequence sequence_no

    const int imagesNum = KITTI.getImagesNum();

    fast_SVO::System SVO(&KITTI, argv[1], fast_SVO::System::KITTI);

    std::cout << std::endl << "--------" << std::endl;
    std::cout << "Start processing sequence ..."  << std::endl;
    std::cout << "Images in the sequence: " << imagesNum << std::endl << std::endl;


    // Record the true frame time
    double tframe = 0;

    // Set the whiteboard for visualizing the plots
    std::string windowName = "x-y Trajectory"; 
    cv::namedWindow(windowName);
    cv::Mat whiteboard = cv::Mat::zeros(cv::Size(1000, 1000), CV_8UC3);
    whiteboard.setTo(255);

    // Main loop
    for (int i = 0; i < imagesNum; ++i) {
    //for (int i = 0; i < 2; ++i) {
        tframe = SVO.updateImages(i); // update the images pair to No. ni

        std::cout << std::endl << "----------------frame " << i << "----------------" << std::endl << std::endl;
        SVO.trackStereo();
        if (i != 0) {
            SVO.calculateCurPose(i); // combine current R and T to the previous Rs and Ts
            SVO.showTrajectory(windowName, whiteboard);
        }
    }

    return 0;
}