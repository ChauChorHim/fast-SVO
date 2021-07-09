#include <iostream>
#include <opencv2/core/core.hpp>

#include <opencv2/highgui.hpp>

#include <Dataset.h>

using std::cerr;
using std::endl;
using std::cout;
using std::tie;

void show_frame_temp(cv::Mat& imLeft) {
    cv::namedWindow("Display Image", CV_WINDOW_AUTOSIZE);//创建窗口
    cv::imshow("Display Image", imLeft);//显示
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

    cout << endl << "--------" << endl;
    cout << "Start processing sequence ..."  << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imLeft, imRight;
    double tframe = 0;
    for (int ni = 0; ni < nImages; ++ni) {
        tie(imLeft, imRight, tframe) = KITTI.getImages(ni);
        cout << tframe << endl;
        show_frame_temp(imLeft);
    }
    return 0;
}