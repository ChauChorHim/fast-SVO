#include <iostream>
#include <string>

#include "Dataset.hpp"
#include "System.hpp"
#include "Timer.hpp"

int main(int argc, char **argv) {
    if (argc != 4) {
        std::cerr << std::endl << "Usage: ./stereo_kitti path_to_settings path_to_sequence sequence_no" << std::endl;
        return 1;
    }
    std::string sequenceNo = "-1";
    if (std::string(argv[3]).size() < 2) {
        sequenceNo = "0" + std::string(argv[3]);
    }
    else {
        sequenceNo = std::string(argv[3]);
    }

    // Retrive sequence path to dataset
    // The constructor use private LoadImages
    fast_SVO::Dataset KITTI = fast_SVO::Dataset(std::string(argv[2]), sequenceNo); // path_to_sequence sequence_no

    const int imagesAmount = KITTI.getImagesAmount();

    fast_SVO::System SVO(&KITTI, argv[1], fast_SVO::System::KITTI);

    std::cout << std::endl << "--------" << std::endl;
    std::cout << "Start processing sequence ... "  << sequenceNo << std::endl;
    std::cout << "Images amount in the sequence: " << imagesAmount << std::endl << std::endl;


    // Record the true frame time
    double tframe = 0;

    // Set the whiteboard for visualizing the plots
    std::string windowName = "x-y Trajectory"; 
    cv::namedWindow(windowName);
    cv::Mat whiteboard = cv::Mat::zeros(cv::Size(1000, 1000), CV_8UC3);
    whiteboard.setTo(255);

    fast_SVO::LoopTimer mainTimer = fast_SVO::LoopTimer("\n\nMAIN TIMER");
    mainTimer.reset();


    // Main loop
    for (int i = 0; i < imagesAmount; ++i) {
        tframe = SVO.updateImages(i); // update the images pair to No. ni
        //if (i % 10 == 0) {
        //    std::cout << std::endl << "----------------frame " << i << "----------------" << std::endl << std::endl;
        //    std::cout << "TIMESTAMP: " << tframe << std::endl;
        //}
        mainTimer.start();
        if (i == 0)
            SVO.resetAllTimers();
        
        SVO.trackStereo();
        mainTimer.pause();
        if (i != 0) {
            SVO.calculateCurPose(); // combine current R and T to the previous Rs and Ts
            //SVO.showTrajectory(windowName, whiteboard);
        }
        //if (i % 10 == 0)
        //    mainTimer.showTiming(mainTimer.getStartTime(), mainTimer.getPauseTime(), "mainTimer");
    }
    // Save the results to evaluate
    const std::string pathToResult {"."};
    const std::string filename {sequenceNo + "_fast-SVO.txt"};
    SVO.saveTrajectory(pathToResult, filename);
    return 0;
}