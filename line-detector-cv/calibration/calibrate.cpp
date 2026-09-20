#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

int main() {
    // Checkerboard = INNER corners, not squares. Adjust to your actual board.
    cv::Size checkerboardSize(7, 4);
    float squareSize = 4.8f; // cm — your actual measured square size

    // Prepare object points (3D real-world points, scaled by actual square size)
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < checkerboardSize.height; i++)
        for (int j = 0; j < checkerboardSize.width; j++)
            objp.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));

    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f>> imgpoints;

    std::vector<cv::String> images;
    cv::glob("calib_images/calib_*.jpg", images);

    cv::Size imageSize;

    for (const auto& fname : images) {
        cv::Mat img = cv::imread(fname);
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        imageSize = gray.size();

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, checkerboardSize, corners,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
            objpoints.push_back(objp);
            imgpoints.push_back(corners);
            std::cout << "Found corners in: " << fname << std::endl;
        } else {
            std::cout << "FAILED on: " << fname << " (discarded)" << std::endl;
        }
    }

    if (objpoints.size() < 10) {
        std::cerr << "Only " << objpoints.size() << " valid images — aim for 15-20+ for reliable calibration." << std::endl;
    }

    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(objpoints, imgpoints, imageSize,
                                       cameraMatrix, distCoeffs, rvecs, tvecs);

    std::cout << "RMS re-projection error: " << rms << std::endl;
    std::cout << "Camera matrix:\n" << cameraMatrix << std::endl;
    std::cout << "Distortion coefficients:\n" << distCoeffs << std::endl;

    // Save for reuse
    cv::FileStorage fs("camera_calib.yml", cv::FileStorage::WRITE);
    fs << "camera_matrix" << cameraMatrix;
    fs << "dist_coeffs" << distCoeffs;
    fs.release();

    return 0;
}