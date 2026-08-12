// ============================================================================
// yolo_engine.hpp — YOLO NPU 目标检测 RAII 封装
//
// 模型: yolo26n_split.rknn (INT8, 640x640, COCO 80类)
// API:  rknn_init → rknn_inputs_set → rknn_run → rknn_outputs_get → NMS
// ============================================================================
#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct Detection {
    std::string class_name;
    float confidence;
    int x, y, w, h;
};

class YoloEngine {
public:
    struct Config {
        std::string model_path;
        float conf_threshold = 0.25f;
        float iou_threshold = 0.45f;
    };

    YoloEngine() = default;
    ~YoloEngine();

    YoloEngine(const YoloEngine&) = delete;
    YoloEngine& operator=(const YoloEngine&) = delete;

    bool Init(const Config& config);
    bool IsReady() const { return ctx_ != 0; }

    /// 对 RGB 888 图像做检测（内部缩放到 640x640）
    /// @param rgb    RGB 888 像素数据
    /// @param w, h   图像宽高
    std::vector<Detection> Detect(const uint8_t* rgb, int w, int h);

private:
    void* ctx_ = nullptr;                // rknn_context (uint64_t on aarch64)
    int model_w_ = 640, model_h_ = 640;
    float conf_thres_ = 0.25f;
    float iou_thres_ = 0.45f;
};
