#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>

// Standalone line detector (no ROS dependency yet).
// Pipeline: downscale -> ROI crop (bottom band) -> blur -> adaptive threshold
//           (+ optional HSV color mask) -> largest contour -> centroid -> offset
//
// Prints offset (-1.0 .. +1.0) and detection status to the console each frame,
// and draws an overlay so you can SEE what the threshold/contour is doing --
// crucial for tuning block size / C against your actual course lighting.
//
// Once this is tuned and working, port the same logic into a ROS 2 node
// (subscribe to /camera/image_raw instead of cv::VideoCapture, publish
// /line_offset and /line_detected instead of printing) -- that's the version
// that goes in ros2_ws/src.

struct LineDetectorParams {
    int downscale_width = 320;
    int downscale_height = 240;

    double roi_top_fraction = 0.55;   // ignore top 55% of frame
    int blur_kernel_size = 5;         // must be odd

    int adaptive_block_size = 25;     // must be odd, > 1 -- TUNE THIS FIRST
    double adaptive_c = 7.0;          // TUNE THIS SECOND

    bool use_color_mask = false;      // turn on if your tape has a strong, consistent color
    int hsv_h_min = 0, hsv_h_max = 179;
    int hsv_s_min = 0, hsv_s_max = 255;
    int hsv_v_min = 0, hsv_v_max = 255;

    double min_contour_area = 150.0;  // in downscaled-pixel units
    int missed_frames_threshold = 4;  // consecutive misses before declaring "lost"
    bool invert_offset_sign = false;  // flip if positive should mean "line to the left"
};

class LineDetector {
public:
    explicit LineDetector(const LineDetectorParams& params) : params_(params) {}

    // Update tunable params without resetting missed-frame/offset state.
    void setParams(const LineDetectorParams& params) { params_ = params; }

    // Returns true if a line offset is currently valid (detected, or within the
    // missed-frame grace window). Writes the offset into `offset_out`.
    // `debug_frame` (optional) gets the ROI + overlay drawn on it for imshow.
    bool process(const cv::Mat& bgr_frame, double& offset_out, cv::Mat* debug_frame = nullptr) {
        cv::Mat frame;
        cv::resize(bgr_frame, frame, cv::Size(params_.downscale_width, params_.downscale_height),
                   0, 0, cv::INTER_AREA);

        int roi_y = static_cast<int>(params_.downscale_height * params_.roi_top_fraction);
        roi_y = std::clamp(roi_y, 0, params_.downscale_height - 1);
        cv::Rect roi(0, roi_y, params_.downscale_width, params_.downscale_height - roi_y);
        cv::Mat roi_frame = frame(roi);

        cv::Mat gray;
        cv::cvtColor(roi_frame, gray, cv::COLOR_BGR2GRAY);

        int k = params_.blur_kernel_size;
        if (k % 2 == 0) k += 1;
        cv::GaussianBlur(gray, gray, cv::Size(k, k), 0);

        int block_size = params_.adaptive_block_size;
        if (block_size % 2 == 0) block_size += 1;
        if (block_size < 3) block_size = 3;

        cv::Mat thresh;
        cv::adaptiveThreshold(gray, thresh, 255,
                               cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV,
                               block_size, params_.adaptive_c);

        if (params_.use_color_mask) {
            cv::Mat hsv, color_mask;
            cv::cvtColor(roi_frame, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv,
                        cv::Scalar(params_.hsv_h_min, params_.hsv_s_min, params_.hsv_v_min),
                        cv::Scalar(params_.hsv_h_max, params_.hsv_s_max, params_.hsv_v_max),
                        color_mask);
            cv::bitwise_and(thresh, color_mask, thresh);
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double best_area = 0.0;
        int best_idx = -1;
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area > best_area) {
                best_area = area;
                best_idx = static_cast<int>(i);
            }
        }

        bool detected_this_frame = (best_idx >= 0 && best_area >= params_.min_contour_area);
        double cx = -1.0;

        if (detected_this_frame) {
            missed_frames_ = 0;
            cv::Moments m = cv::moments(contours[best_idx]);
            cx = m.m10 / (m.m00 + 1e-6);

            double frame_center_x = params_.downscale_width / 2.0;
            double offset = (cx - frame_center_x) / (params_.downscale_width / 2.0);
            offset = std::clamp(offset, -1.0, 1.0);
            if (params_.invert_offset_sign) offset = -offset;

            last_offset_ = offset;
            have_valid_offset_ = true;
        } else {
            missed_frames_ += 1;
            if (missed_frames_ >= params_.missed_frames_threshold) {
                have_valid_offset_ = false;
            }
        }

        if (debug_frame) {
            roi_frame.copyTo(*debug_frame);
            cv::Mat thresh_bgr;
            cv::cvtColor(thresh, thresh_bgr, cv::COLOR_GRAY2BGR);
            cv::hconcat(*debug_frame, thresh_bgr, *debug_frame);

            if (detected_this_frame) {
                cv::drawContours(*debug_frame, contours, best_idx, cv::Scalar(0, 255, 0), 2);
                cv::circle(*debug_frame, cv::Point(static_cast<int>(cx), debug_frame->rows / 2),
                           5, cv::Scalar(0, 0, 255), -1);
            }
            cv::line(*debug_frame, cv::Point(params_.downscale_width / 2, 0),
                     cv::Point(params_.downscale_width / 2, debug_frame->rows),
                     cv::Scalar(255, 0, 0), 1);
        }

        offset_out = last_offset_;
        return have_valid_offset_;
    }

private:
    LineDetectorParams params_;
    int missed_frames_ = 0;
    double last_offset_ = 0.0;
    bool have_valid_offset_ = false;
};

int main(int argc, char** argv) {
    cv::VideoCapture cap;

    if (argc > 1) {
        // Video file path passed as an argument
        cap.open(argv[1]);
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open video file: " << argv[1] << std::endl;
            return -1;
        }
        std::cout << "Reading from video file: " << argv[1] << std::endl;
    } else {
        // No argument given -- fall back to live webcam
        cap.open(0);
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open camera." << std::endl;
            return -1;
        }
        std::cout << "Reading from webcam (no video file argument given)." << std::endl;
    }

    LineDetectorParams params;
    LineDetector detector(params);

    cv::namedWindow("Line Detector", cv::WINDOW_AUTOSIZE);
    int block_size_trackbar = params.adaptive_block_size;
    int c_trackbar = static_cast<int>(params.adaptive_c);
    cv::createTrackbar("Block Size (odd)", "Line Detector", &block_size_trackbar, 99);
    cv::createTrackbar("C", "Line Detector", &c_trackbar, 50);

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            if (argc > 1) {
                // End of video file -- loop back to the start instead of exiting
                cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                cap >> frame;
                if (frame.empty()) break; // genuinely empty/corrupt file
            } else {
                break;
            }
        }

        params.adaptive_block_size = std::max(3, block_size_trackbar);
        params.adaptive_c = c_trackbar;
        detector.setParams(params);

        double offset = 0.0;
        cv::Mat debug_frame;
        bool detected = detector.process(frame, offset, &debug_frame);

        std::cout << "offset=" << offset << "  detected=" << (detected ? "true" : "false")
                  << std::endl;

        if (!debug_frame.empty()) {
            cv::imshow("Line Detector", debug_frame);
        }

        // Slow playback slightly so you can actually watch it -- press 'q' to quit
        if (cv::waitKey(30) == 'q') break;
    }

    return 0;
}