// rga_convert.h — RGA 硬件 UYVY→RGB resize（无 OpenCV）
// ponytail: 单函数，rk3588 硬件 1ms 内完成
#pragma once
#include <cstdint>
#include <cstdlib>
#include <sys/mman.h>
#include "im2d.h"
#include "rga.h"

/// UYVY(src_w×src_h) → RGB(dst_w×dst_h)，硬件 resize
/// 返回 malloc 的 RGB buffer (dst_w*dst_h*3)，调用者 free
/// 失败返回 nullptr
inline uint8_t* rga_uyvy_to_rgb(const void* src, int src_w, int src_h,
                                 int dst_w, int dst_h) {
    uint8_t* dst = static_cast<uint8_t*>(malloc(dst_w * dst_h * 3));
    if (!dst) return nullptr;

    int s_stride = ((src_w * 2) + 15) & ~15;   // UYVY: 2 bytes/px, 16 对齐
    int d_stride = ((dst_w * 3) + 15) & ~15;   // RGB:  3 bytes/px, 16 对齐

    mlock(src, s_stride * src_h);               // 锁定 DMA buf
    mlock(dst, d_stride * dst_h);

    rga_buffer_t rga_src = wrapbuffer_virtualaddr(
        (void*)src, src_w, src_h, RK_FORMAT_UYVY_422);
    rga_buffer_t rga_dst = wrapbuffer_virtualaddr(
        dst, dst_w, dst_h, RK_FORMAT_RGB_888);

    IM_STATUS ret = imresize(rga_src, rga_dst);

    munlock(src, s_stride * src_h);
    munlock(dst, d_stride * dst_h);

    if (ret != IM_STATUS_SUCCESS) {
        free(dst);
        return nullptr;
    }
    return dst;
}
