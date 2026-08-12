// ============================================================================
// yolo_engine.cpp — RK3588 NPU YOLO 推理引擎
//
// 模型架构: yolo26n split — bbox[4,8400] + score[80,8400] 双输出
// 输入: RGB uint8 NHWC 640x640
// ============================================================================
#include "yolo/yolo_engine.hpp"
#include "rknn_api.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <memory>
#include <vector>

// ---- COCO 80 类名 ----
static const char* kCocoNames[] = {
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
    "toothbrush"
};

// ---- 简单双线性缩放 (RGB 888 输入) ----
static void resizeRgb(const uint8_t* src, int sw, int sh,
                      uint8_t* dst, int dw, int dh) {
    for (int y = 0; y < dh; ++y) {
        float sy = (y + 0.5f) * sh / dh - 0.5f;
        if (sy < 0) sy = 0;
        int iy = static_cast<int>(sy);
        float dy = sy - iy;
        if (iy >= sh - 1) { iy = sh - 2; dy = 1.0f; }
        for (int x = 0; x < dw; ++x) {
            float sx = (x + 0.5f) * sw / dw - 0.5f;
            if (sx < 0) sx = 0;
            int ix = static_cast<int>(sx);
            float dx = sx - ix;
            if (ix >= sw - 1) { ix = sw - 2; dx = 1.0f; }

            for (int c = 0; c < 3; ++c) {
                float v = src[(iy * sw + ix) * 3 + c] * (1 - dx) * (1 - dy)
                        + src[(iy * sw + ix + 1) * 3 + c] * dx * (1 - dy)
                        + src[((iy+1) * sw + ix) * 3 + c] * (1 - dx) * dy
                        + src[((iy+1) * sw + ix + 1) * 3 + c] * dx * dy;
                dst[(y * dw + x) * 3 + c] = static_cast<uint8_t>(v + 0.5f);
            }
        }
    }
}

// ---- IOU ----
static float IoU(float ax1, float ay1, float ax2, float ay2,
                 float bx1, float by1, float bx2, float by2) {
    float x1 = std::max(ax1, bx1), y1 = std::max(ay1, by1);
    float x2 = std::min(ax2, bx2), y2 = std::min(ay2, by2);
    if (x1 >= x2 || y1 >= y2) return 0;
    float inter = (x2 - x1) * (y2 - y1);
    float area_a = (ax2 - ax1) * (ay2 - ay1);
    float area_b = (bx2 - bx1) * (by2 - by1);
    return inter / (area_a + area_b - inter + 1e-6f);
}

// ---- NMS (拆分版: bbox[4,8400] + score[80,8400]) ----
static std::vector<Detection> postprocess(const float* bbox, const float* score,
                                          float conf_thres, float iou_thres) {
    constexpr int NC = 80, NA = 8400, W = 640, H = 640;
    struct Raw { float x1,y1,x2,y2,conf; int cls; };
    std::vector<Raw> raw; raw.reserve(256);

    for (int a = 0; a < NA; ++a) {
        float best = 0; int best_c = 0;
        for (int c = 0; c < NC; ++c) {
            float s = score[c * NA + a];
            if (s > best) { best = s; best_c = c; }
        }
        if (best < conf_thres) continue;

        float cx = bbox[0*NA+a], cy = bbox[1*NA+a];
        float w  = bbox[2*NA+a], h  = bbox[3*NA+a];
        float x1 = std::max(0.f, std::min(cx - w/2, (float)W));
        float y1 = std::max(0.f, std::min(cy - h/2, (float)H));
        float x2 = std::max(0.f, std::min(cx + w/2, (float)W));
        float y2 = std::max(0.f, std::min(cy + h/2, (float)H));
        if (x2-x1 < 2 || y2-y1 < 2) continue;
        raw.push_back({x1,y1,x2,y2,best,best_c});
    }

    std::sort(raw.begin(), raw.end(),
              [](const Raw& a, const Raw& b) { return a.conf > b.conf; });

    std::vector<bool> keep(raw.size(), true);
    std::vector<Detection> dets;
    for (size_t i = 0; i < raw.size(); ++i) {
        if (!keep[i]) continue;
        auto& r = raw[i];
        dets.push_back({kCocoNames[r.cls], r.conf,
                        (int)r.x1, (int)r.y1, (int)(r.x2-r.x1), (int)(r.y2-r.y1)});
        for (size_t j = i+1; j < raw.size(); ++j)
            if (keep[j] && r.cls == raw[j].cls && IoU(r.x1,r.y1,r.x2,r.y2, raw[j].x1,raw[j].y1,raw[j].x2,raw[j].y2) > iou_thres)
                keep[j] = false;
    }
    return dets;
}

// ============================================================================
// YoloEngine
// ============================================================================
YoloEngine::~YoloEngine() {
    if (ctx_) rknn_destroy(reinterpret_cast<uintptr_t>(ctx_));
}

bool YoloEngine::Init(const Config& cfg) {
    conf_thres_ = cfg.conf_threshold;
    iou_thres_ = cfg.iou_threshold;

    // 读模型文件
    std::ifstream f(cfg.model_path, std::ios::binary | std::ios::ate);
    if (!f) return false;
    size_t sz = f.tellg(); f.seekg(0);
    auto model = std::make_unique<uint8_t[]>(sz);
    f.read(reinterpret_cast<char*>(model.get()), sz);

    // 初始化
    rknn_context raw_ctx = 0;
    if (rknn_init(&raw_ctx, model.get(), sz, 0, nullptr) < 0) return false;
    ctx_ = reinterpret_cast<void*>(static_cast<uintptr_t>(raw_ctx));

    // 查询输入属性
    rknn_input_output_num io;
    rknn_query(raw_ctx, RKNN_QUERY_IN_OUT_NUM, &io, sizeof(io));
    rknn_tensor_attr in_attr{};
    in_attr.index = 0;
    rknn_query(raw_ctx, RKNN_QUERY_INPUT_ATTR, &in_attr, sizeof(in_attr));
    model_w_ = in_attr.dims[1];
    model_h_ = in_attr.dims[2];
    return true;
}

std::vector<Detection> YoloEngine::Detect(const uint8_t* rgb, int w, int h) {
    if (!ctx_) return {};

    // 缩放到模型输入尺寸
    std::vector<uint8_t> resized(model_w_ * model_h_ * 3);
    resizeRgb(rgb, w, h, resized.data(), model_w_, model_h_);

    // 构造输入
    rknn_input in{};
    in.index = 0;
    in.buf = resized.data();
    in.size = resized.size();
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;

    auto raw_ctx = static_cast<rknn_context>(reinterpret_cast<uintptr_t>(ctx_));
    if (rknn_inputs_set(raw_ctx, 1, &in) < 0) return {};
    if (rknn_run(raw_ctx, nullptr) < 0) return {};

    // 取两个输出: bbox[4,8400] + score[80,8400]
    rknn_output outs[2]{};
    outs[0].want_float = 1; outs[0].index = 0;
    outs[1].want_float = 1; outs[1].index = 1;
    if (rknn_outputs_get(raw_ctx, 2, outs, nullptr) < 0) return {};

    auto dets = postprocess((float*)outs[0].buf, (float*)outs[1].buf,
                            conf_thres_, iou_thres_);
    rknn_outputs_release(raw_ctx, 2, outs);
    return dets;
}
