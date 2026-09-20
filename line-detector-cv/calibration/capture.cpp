#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    std::string camPath = "/dev/v4l/by-id/usb-046d_081b_AE54AA40-video-index0";

    cv::VideoCapture cap;
    if (argc > 1) {
        cap.open(std::atoi(argv[1]));
    } else {
        cap.open(camPath, cv::CAP_V4L2);
    }

    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera." << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    std::cout << "Camera opened. Actual resolution: "
              << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cout << "Press 's' to save a frame, 'q' to quit." << std::endl;

    cv::Mat frame;
    int count = 0;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Warning: grabbed empty frame, skipping." << std::endl;
            continue;
        }

        cv::imshow("Calibration Capture", frame);
        int key = cv::waitKey(1);

        if (key == 's') {
            std::string filename = "calib_" + std::to_string(count++) + ".jpg";
            cv::imwrite(filename, frame);
            std::cout << "Saved " << filename << std::endl;
        } else if (key == 'q') {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}