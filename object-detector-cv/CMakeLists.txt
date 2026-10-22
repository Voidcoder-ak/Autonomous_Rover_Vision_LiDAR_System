cmake_minimum_required(VERSION 3.16)
project(object_detector_cv CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)  # Release = much faster detection
endif()

# OpenCV: camera, image prep, NMS, drawing (apt: libopencv-dev)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio dnn)

# ONNX Runtime: runs the YOLOv8 model.
# Default location is third_party/onnxruntime (see README); override with
#   cmake -B build -DONNXRUNTIME_DIR=/path/to/onnxruntime-linux-x64-1.20.1
set(ONNXRUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime"
    CACHE PATH "Folder of the extracted ONNX Runtime release")
if(NOT EXISTS "${ONNXRUNTIME_DIR}/include/onnxruntime_cxx_api.h")
  message(FATAL_ERROR "ONNX Runtime not found in ${ONNXRUNTIME_DIR} — see README.md, step 2")
endif()

add_executable(object_detector src/main.cpp)
target_include_directories(object_detector PRIVATE "${ONNXRUNTIME_DIR}/include")
target_link_libraries(object_detector
  ${OpenCV_LIBS}
  "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")

# Remember where libonnxruntime.so is, so the program runs without setting LD_LIBRARY_PATH.
set_target_properties(object_detector PROPERTIES BUILD_RPATH "${ONNXRUNTIME_DIR}/lib")
