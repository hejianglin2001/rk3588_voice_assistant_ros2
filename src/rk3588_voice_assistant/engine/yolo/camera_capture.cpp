// camera_capture.cpp — V4L2 mmap 采集
#include "yolo/camera_capture.h"
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

bool CameraCapture::Xioctl(int req, void* arg) {
    for (int retries = 0; retries < 3; retries++) {
        if (ioctl(fd_, req, arg) == 0) return true;
        if (errno != EINTR) break;
        usleep(1000);
    }
    return false;
}

CameraCapture::~CameraCapture() { Close(); }

bool CameraCapture::Open(const std::string& device, int w, int h) {
    fd_ = open(device.c_str(), O_RDWR);
    if (fd_ < 0) { perror("open camera"); return false; }

    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_UYVY;
    fmt.fmt.pix_mp.width = w;
    fmt.fmt.pix_mp.height = h;
    if (!Xioctl(VIDIOC_S_FMT, &fmt)) { perror("VIDIOC_S_FMT"); return false; }
    w_ = fmt.fmt.pix_mp.width; h_ = fmt.fmt.pix_mp.height;

    return true;
}

bool CameraCapture::Start() {
    struct v4l2_requestbuffers req = {};
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    req.count = 4;
    if (!Xioctl(VIDIOC_REQBUFS, &req)) return false;

    num_bufs_ = req.count;
    buffers_ = new Buffer[num_bufs_];
    for (int i = 0; i < num_bufs_; i++) {
        struct v4l2_buffer buf = {};
        struct v4l2_plane plane = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = &plane;
        buf.length = 1;
        if (!Xioctl(VIDIOC_QUERYBUF, &buf)) return false;
        buffers_[i].length = plane.length;
        buffers_[i].start = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, fd_, plane.m.mem_offset);
        if (buffers_[i].start == MAP_FAILED) return false;
    }

    for (int i = 0; i < num_bufs_; i++) {
        struct v4l2_buffer buf = {};
        struct v4l2_plane plane = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = &plane;
        buf.length = 1;
        if (!Xioctl(VIDIOC_QBUF, &buf)) return false;
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (!Xioctl(VIDIOC_STREAMON, &type)) return false;
    streaming_ = true;
    return true;
}

CameraFrame CameraCapture::Capture() {
    CameraFrame f = {nullptr, 0};
    struct v4l2_buffer buf = {};
    struct v4l2_plane plane = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.m.planes = &plane;
    buf.length = 1;

    if (!Xioctl(VIDIOC_DQBUF, &buf)) return f;
    f.data = static_cast<uint8_t*>(buffers_[buf.index].start);
    f.size = plane.bytesused;

    // 用完放回队列
    Xioctl(VIDIOC_QBUF, &buf);
    return f;
}

void CameraCapture::Close() {
    if (streaming_) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        Xioctl(VIDIOC_STREAMOFF, &type);
        streaming_ = false;
    }
    if (buffers_) {
        for (int i = 0; i < num_bufs_; i++)
            if (buffers_[i].start) munmap(buffers_[i].start, buffers_[i].length);
        delete[] buffers_; buffers_ = nullptr;
    }
    if (fd_ >= 0) { close(fd_); fd_ = -1; }
}
