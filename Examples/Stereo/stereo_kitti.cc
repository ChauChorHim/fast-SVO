#include <iostream>
#include <chrono>

#include <opencv2/highgui.hpp>

#include <Dataset.h>
#include <System.h>

using std::cerr;
using std::endl;
using std::cout;
using std::chrono::steady_clock;

void TEMP_show_frame(cv::Mat& imLeft) {
    cv::namedWindow("Display Image", CV_WINDOW_AUTOSIZE); // create window
    cv::imshow("Display Image", imLeft); // show the image
    cv::waitKey(50);
}

int main(int argc, char **argv) {
    if (argc != 3) {
        cerr << endl << "Usage: ./stereo_kitti path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrive sequence path to dataset
    // The constructor use private LoadImages
    fast_SVO::Dataset KITTI {string(argv[2])};

    const int nImages = KITTI.getImagesNum();

    fast_SVO::System SVO(&KITTI, argv[1], fast_SVO::System::KITTI);

    cout << endl << "--------" << endl;
    cout << "Start processing sequence ..."  << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    double tframe = 0;
    for (int ni = 0; ni < nImages; ++ni) {
        tframe = SVO.UpdateImages(ni); // update the images pair to No. ni

        steady_clock::time_point t1 = steady_clock::now();

        SVO.TrackStereo(ni);

        steady_clock::time_point t2 = steady_clock::now();
    }
    return 0;
}