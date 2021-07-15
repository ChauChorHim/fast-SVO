#include <iostream>
#include <chrono>

#include "Dataset.hpp"
#include "System.hpp"

#include <opencv2/highgui.hpp>

void TEMP_show_frame(cv::Mat& imLeft) {
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE); // create window
    cv::imshow("Display Image", imLeft); // show the image
    cv::waitKey(50);
}

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << std::endl << "Usage: ./stereo_kitti path_to_settings path_to_sequence" << std::endl;
        return 1;
    }

    // Retrive sequence path to dataset
    // The constructor use private LoadImages
    fast_SVO::Dataset KITTI {std::string(argv[2])};

    const int nImages = KITTI.getImagesNum();

    fast_SVO::System SVO(&KITTI, argv[1], fast_SVO::System::KITTI);

    std::cout << std::endl << "--------" << std::endl;
    std::cout << "Start processing sequence ..."  << std::endl;
    std::cout << "Images in the sequence: " << nImages << std::endl << std::endl;

    // Main loop
    double tframe = 0;
    for (int ni = 0; ni < nImages; ++ni) {
        tframe = SVO.updateImages(ni); // update the images pair to No. ni

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        SVO.trackStereo(ni);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>> (t2 - t1).count();
    }
    return 0;
}