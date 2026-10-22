# object-detector-cv

Step 1 of camera object detection for the rover: YOLOv8n running on a webcam, in C++.
It shows the camera feed with green boxes and labels (e.g. `person 0.87`) and the FPS.

- **OpenCV** handles the camera, image prep, NMS (removing duplicate boxes) and drawing.
- **ONNX Runtime** runs the YOLOv8n model. (Ubuntu 24.04's OpenCV 4.6 is too old to load YOLOv8 models itself.)

## Setup (Ubuntu 24.04, one time)

**1. Install OpenCV and build tools**

```bash
sudo apt install libopencv-dev cmake g++
```

**2. Download ONNX Runtime** (prebuilt, no compiling) into `third_party/`

```bash
cd object-detector-cv
mkdir -p third_party && cd third_party
wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-x64-1.20.1.tgz
tar xzf onnxruntime-linux-x64-1.20.1.tgz
mv onnxruntime-linux-x64-1.20.1 onnxruntime
cd ..
```

**3. Get the YOLOv8n model as an ONNX file**

```bash
pip install ultralytics          # use a venv if you like
yolo export model=yolov8n.pt format=onnx opset=12 imgsz=640
mkdir -p models && mv yolov8n.onnx models/
```

## Build and run

```bash
cmake -S . -B build
cmake --build build
./build/object_detector models/yolov8n.onnx 0     # 0 = first camera
```

Press `q` or `ESC` to quit. If the wrong camera opens, try `1` instead of `0`.

## Settings to tune (top of `src/main.cpp`)

| Setting | Default | What it does |
|---|---|---|
| `CONF_THRESHOLD` | 0.45 | Ignore detections the model is less sure about than this |
| `NMS_THRESHOLD` | 0.50 | How much two boxes may overlap before the weaker one is removed |

## Next steps

- Step 2: "danger zone" — only care about objects in the middle of the frame and big enough to be close
- Step 3: ROS 2 node that publishes "obstacle ahead" / "slow down" for the drive logic
