// camera_capture.h — V4L2 极简摄像头采集 (ponytail: 10 行接口)
#pragma once
#include <cstdint>
#include <string>
#include <linux/videodev2.h>

struct CameraFrame {
    uint8_t* data;   // mmap buffer
    size_t   size;
};

class CameraCapture {
public:
    CameraCapture() = default;
    ~CameraCapture();
    CameraCapture(const CameraCapture&) = delete;
    CameraCapture& operator=(const CameraCapture&) = delete;

    /// 打开设备，设置 UYVY 格式 @ w×h，返回 true 成功
    bool Open(const std::string& device, int w, int h);
    bool IsOpen() const { return fd_ >= 0; }
    /// 开始数据流
    bool Start();
    /// 抓一帧（阻塞），成功返回有效 data，失败返回 data==nullptr
    CameraFrame Capture();
    /// 停止 + 关闭
    void Close();

private:
    int fd_ = -1;
    int w_ = 0, h_ = 0;
    struct Buffer { void* start; size_t length; };
    Buffer* buffers_ = nullptr;
    int num_bufs_ = 0;
    bool streaming_ = false;

    bool Xioctl(int req, void* arg);
};
