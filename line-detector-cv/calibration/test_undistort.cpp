#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    // Load calibration
    cv::FileStorage fs("camera_calib.yml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open camera_calib.yml. Run this from the "
                     "calibration/ directory." << std::endl;
        return -1;
    }

    cv::Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["dist_coeffs"] >> distCoeffs;
    fs.release();

    std::cout << "Loaded camera matrix:\n" << cameraMatrix << std::endl;
    std::cout << "Loaded distortion coefficients:\n" << distCoeffs << std::endl;

    // Open the same webcam used for calibration
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

    // Must match calibration resolution exactly
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    std::cout << "Actual resolution: "
              << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cout << "Showing raw (left) vs undistorted (right). Press 'q' to quit."
              << std::endl;

    cv::Mat frame, undistorted, sideBySide;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Warning: empty frame, skipping." << std::endl;
            continue;
        }

        cv::undistort(frame, undistorted, cameraMatrix, distCoeffs);

        // Draw a reference grid on both so distortion correction is easy to see
        cv::Mat frameGrid = frame.clone();
        cv::Mat undistGrid = undistorted.clone();
        for (int x = 0; x < frameGrid.cols; x += 40) {
            cv::line(frameGrid, cv::Point(x, 0), cv::Point(x, frameGrid.rows),
                      cv::Scalar(0, 255, 0), 1);
            cv::line(undistGrid, cv::Point(x, 0), cv::Point(x, undistGrid.rows),
                      cv::Scalar(0, 255, 0), 1);
        }
        for (int y = 0; y < frameGrid.rows; y += 40) {
            cv::line(frameGrid, cv::Point(0, y), cv::Point(frameGrid.cols, y),
                      cv::Scalar(0, 255, 0), 1);
            cv::line(undistGrid, cv::Point(0, y), cv::Point(undistGrid.cols, y),
                      cv::Scalar(0, 255, 0), 1);
        }

        cv::hconcat(frameGrid, undistGrid, sideBySide);
        cv::putText(sideBySide, "RAW", cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        cv::putText(sideBySide, "UNDISTORTED", cv::Point(650, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

        cv::imshow("Calibration check: raw vs undistorted", sideBySide);

        int key = cv::waitKey(1);
        if (key == 'q') {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
