#include <opencv2/opencv.hpp>
#include <string>

void test_local_camera() {
    cv::VideoCapture capture(0);

    while(true) {
        cv::Mat frame;
        capture >> frame;
        cv::imshow("video stream", frame);
        cv::waitKey(30);
    }
}

void test_ip_camera(const std::string &cameraIP) {
    cv::VideoCapture camera(cameraIP);
    cv::Mat frame;
    if (!camera.isOpened()) {
        std::cerr << "open camera error.\n";
    } else {
        camera >> frame;
        while (!frame.empty()) {
            cv::imshow("IP camera stream", frame);
            cv::waitKey(30);
            camera >> frame;
        }
    }
}

int main() {
    const std::string cameraIP {"rtsp://admin:9797@10.16.170.125:8554/live"};
    test_ip_camera(cameraIP);
    return 0;
}