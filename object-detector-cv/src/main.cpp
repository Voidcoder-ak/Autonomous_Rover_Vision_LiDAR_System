// object-detector-cv — Step 1: YOLOv8n object detection on a webcam, in C++.
//
// What this program does, in plain words:
//   1. Opens the camera.
//   2. Loads the YOLOv8n "brain" (an .onnx file) with ONNX Runtime.
//   3. For every frame: shrink it to 640x640, ask the model "what do you see?",
//      clean up the answer, and draw boxes + labels on the frame.
//
// Why ONNX Runtime and not OpenCV's DNN module?
//   Ubuntu 24.04 ships OpenCV 4.6, which is too old to load YOLOv8 models.
//   ONNX Runtime is a dedicated "model runner" that handles them fine.
//   OpenCV still does everything else: camera, resizing, NMS and drawing.
//
// Usage:  ./object_detector [model_path] [camera_index]
//         ./object_detector yolov8n.onnx 0

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>          // still used for blobFromImage + NMSBoxes
#include <onnxruntime_cxx_api.h>

#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// ---------- Settings you can tune ----------
const float INPUT_SIZE     = 640.0f; // YOLOv8n was trained on 640x640 images
const float CONF_THRESHOLD = 0.45f;  // ignore guesses the model is less than 45% sure about
const float NMS_THRESHOLD  = 0.50f;  // how much two boxes can overlap before one is thrown away

// The 80 things YOLOv8n (COCO dataset) knows how to recognise, in order.
const std::vector<std::string> CLASS_NAMES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"};

// One finished detection: what it is, how sure the model is, and where it is.
struct Detection {
    int classId;
    float confidence;
    cv::Rect box;
};

// Holds the loaded model. Think of it as the "brain in a box":
// you load it once at the start, then ask it questions every frame.
struct YoloModel {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "rover-yolo"};
    Ort::Session session{nullptr};
    std::string inputName;
    std::string outputName;

    explicit YoloModel(const std::string& path) {
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(4); // use 4 CPU cores
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session = Ort::Session(env, path.c_str(), opts);

        // Ask the model what its input/output are called (for YOLOv8: "images" / "output0").
        Ort::AllocatorWithDefaultOptions alloc;
        inputName  = session.GetInputNameAllocated(0, alloc).get();
        outputName = session.GetOutputNameAllocated(0, alloc).get();
    }
};

// Runs the model on one frame and returns the cleaned-up list of detections.
std::vector<Detection> detect(YoloModel& model, const cv::Mat& frame) {
    // 1) Prepare the image the way the model expects:
    //    resize to 640x640, scale pixels from 0..255 to 0..1, and swap BGR -> RGB.
    cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0 / 255.0,
                                          cv::Size((int)INPUT_SIZE, (int)INPUT_SIZE),
                                          cv::Scalar(), /*swapRB=*/true, /*crop=*/false);

    // Hand the blob's numbers to ONNX Runtime (no copying — it just points at them).
    const std::array<int64_t, 4> inputShape{1, 3, (int64_t)INPUT_SIZE, (int64_t)INPUT_SIZE};
    Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memInfo, blob.ptr<float>(), blob.total(), inputShape.data(), inputShape.size());

    // 2) Ask the model. YOLOv8 gives back ONE block of numbers shaped [1, 84, 8400]:
    //    8400 possible boxes, and for each box 84 numbers:
    //      [cx, cy, w, h, score_class0, score_class1, ... score_class79]
    const char* inNames[]  = {model.inputName.c_str()};
    const char* outNames[] = {model.outputName.c_str()};
    std::vector<Ort::Value> outputs =
        model.session.Run(Ort::RunOptions{nullptr}, inNames, &inputTensor, 1, outNames, 1);

    std::vector<int64_t> outShape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    const int numValues = (int)outShape[1]; // 84
    const int numBoxes  = (int)outShape[2]; // 8400
    float* outData = outputs[0].GetTensorMutableData<float>();

    // The numbers come "sideways" (84 rows x 8400 columns).
    // Transpose so each ROW is one box (8400 rows x 84 columns) — much easier to read.
    cv::Mat boxesSideways(numValues, numBoxes, CV_32F, outData);
    cv::Mat rows = boxesSideways.t();

    // The model worked on a 640x640 picture, but our real frame is a different size.
    // These factors stretch the boxes back to the real frame size.
    const float xFactor = frame.cols / INPUT_SIZE;
    const float yFactor = frame.rows / INPUT_SIZE;

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    // 3) Go through all 8400 guesses and keep only the confident ones.
    for (int i = 0; i < numBoxes; ++i) {
        const float* row = rows.ptr<float>(i);

        // Find which of the 80 classes has the highest score for this box.
        cv::Mat scores(1, numValues - 4, CV_32F, (void*)(row + 4));
        cv::Point bestClass;
        double bestScore;
        cv::minMaxLoc(scores, nullptr, &bestScore, nullptr, &bestClass);

        if (bestScore < CONF_THRESHOLD) continue; // not sure enough -> skip

        // The box comes as centre-x, centre-y, width, height (in 640x640 space).
        // OpenCV wants top-left corner + width + height (in real frame space).
        const float cx = row[0], cy = row[1], w = row[2], h = row[3];
        const int left   = (int)((cx - w / 2) * xFactor);
        const int top    = (int)((cy - h / 2) * yFactor);
        const int width  = (int)(w * xFactor);
        const int height = (int)(h * yFactor);

        boxes.emplace_back(left, top, width, height);
        confidences.push_back((float)bestScore);
        classIds.push_back(bestClass.x);
    }

    // 4) Non-Maximum Suppression (NMS): the model often draws several boxes
    //    around the same object. NMS keeps the best one and removes the copies.
    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD, keep);

    std::vector<Detection> detections;
    for (int idx : keep) {
        detections.push_back({classIds[idx], confidences[idx], boxes[idx]});
    }
    return detections;
}

// Draws one box with its label, e.g. "person 0.87".
void drawDetection(cv::Mat& frame, const Detection& d) {
    const cv::Scalar color(0, 255, 0); // green
    cv::rectangle(frame, d.box, color, 2);

    std::string name = (d.classId >= 0 && d.classId < (int)CLASS_NAMES.size())
                           ? CLASS_NAMES[d.classId]
                           : "unknown";
    std::string label = name + " " + cv::format("%.2f", d.confidence);

    int baseline = 0;
    cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    int labelTop = std::max(d.box.y, textSize.height + 4);
    cv::rectangle(frame, cv::Point(d.box.x, labelTop - textSize.height - 4),
                  cv::Point(d.box.x + textSize.width, labelTop), color, cv::FILLED);
    cv::putText(frame, label, cv::Point(d.box.x, labelTop - 2),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
}

int main(int argc, char** argv) {
    const std::string modelPath = (argc > 1) ? argv[1] : "yolov8n.onnx";
    const int cameraIndex       = (argc > 2) ? std::stoi(argv[2]) : 0;

    // Load the model.
    std::unique_ptr<YoloModel> model;
    try {
        model = std::make_unique<YoloModel>(modelPath);
    } catch (const Ort::Exception& e) {
        std::cerr << "Could not load model '" << modelPath << "': " << e.what() << "\n";
        return 1;
    }

    // Open the camera.
    cv::VideoCapture cap(cameraIndex);
    if (!cap.isOpened()) {
        std::cerr << "Could not open camera " << cameraIndex << "\n";
        return 1;
    }

    std::cout << "Running. Press 'q' or ESC to quit.\n";

    cv::Mat frame;
    while (true) {
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "Lost the camera feed.\n";
            break;
        }

        // Time how long detection takes, so we know our FPS.
        const int64 start = cv::getTickCount();
        std::vector<Detection> detections = detect(*model, frame);
        const double seconds = (cv::getTickCount() - start) / cv::getTickFrequency();

        for (const auto& d : detections) drawDetection(frame, d);

        cv::putText(frame, cv::format("FPS: %.1f  objects: %zu", 1.0 / seconds, detections.size()),
                    cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

        cv::imshow("Rover object detector", frame);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
