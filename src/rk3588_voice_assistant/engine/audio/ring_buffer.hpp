// Copyright (c) 2025 Edge Voice AI Assistant
//
// Lock-free Single-Producer Single-Consumer (SPSC) Ring Buffer
//
// 💡 【面试考点】为什么音频链路要用 Lock-free RingBuffer 而不是 std::queue + std::mutex？
//   答：miniaudio 的 data_callback 运行在 ALSA 实时音频线程上，
//   mutex 的 lock() 可能触发 futex 系统调用导致优先级反转 (priority inversion)，
//   造成音频 underrun（爆音）。Lock-free 原子操作保证 wait-free 写入，
//   延迟 < 100ns，满足实时音频的确定性要求。

#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <vector>

template <typename T>
class RingBuffer {
public:
    /// @param capacity 必须是 2 的幂，否则会被向上取整到最近的 2 的幂
    explicit RingBuffer(size_t capacity)
    {
        // 向上取整到 2 的幂，用位运算代替取模，省一条 div 指令
        size_t pow2 = 1;
        while (pow2 < capacity) {
            pow2 <<= 1;
        }
        capacity_ = pow2;
        mask_ = capacity_ - 1;
        buffer_ = std::make_unique<T[]>(capacity_);
    }

    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;
    RingBuffer(RingBuffer&&) = default;
    RingBuffer& operator=(RingBuffer&&) = default;

    /// Producer 端（音频回调线程调用）：写入 N 个样本
    /// @return 实际写入数量（buffer 满时可能返回 0）
    size_t Write(const T* data, size_t count) noexcept
    {
        size_t w = write_idx_.load(std::memory_order_relaxed);
        size_t r = read_idx_.load(std::memory_order_acquire);  // 💡 acquire 确保读到最新的消费者位置

        size_t available = capacity_ - (w - r);
        size_t to_write = (count < available) ? count : available;

        for (size_t i = 0; i < to_write; ++i) {
            buffer_[(w + i) & mask_] = data[i];
        }

        // 💡 release 确保数据写入完成后才对消费者可见
        write_idx_.store(w + to_write, std::memory_order_release);
        return to_write;
    }

    /// Consumer 端（业务线程调用）：读取 N 个样本
    /// @return 实际读取数量（buffer 空时返回 0）
    size_t Read(T* data, size_t count) noexcept
    {
        size_t r = read_idx_.load(std::memory_order_relaxed);
        size_t w = write_idx_.load(std::memory_order_acquire);

        size_t available = w - r;
        size_t to_read = (count < available) ? count : available;

        for (size_t i = 0; i < to_read; ++i) {
            data[i] = buffer_[(r + i) & mask_];
        }

        read_idx_.store(r + to_read, std::memory_order_release);
        return to_read;
    }

    /// 丢弃 buffer 中所有未读数据，重置为空状态
    void Reset() noexcept
    {
        read_idx_.store(write_idx_.load(std::memory_order_acquire), std::memory_order_release);
    }

    size_t AvailableRead() const noexcept
    {
        return write_idx_.load(std::memory_order_acquire) - read_idx_.load(std::memory_order_relaxed);
    }

    size_t Capacity() const noexcept { return capacity_; }

private:
    std::unique_ptr<T[]> buffer_;
    size_t capacity_ = 0;
    size_t mask_ = 0;

    // 💡 为什么用两个独立的 atomic 而不是一个 struct？
    //   ARM64 上 128-bit atomic 需要 LDP/STP 对但非 lock-free，
    //   两个 64-bit atomic 可各自用 LDXR/STXR，真正无锁。
    //   另外 write_idx 只被 producer 写, read_idx 只被 consumer 写，
    //   不存在 store 竞争，atomic 仅用于 memory ordering。
    std::atomic<size_t> write_idx_{0};
    std::atomic<size_t> read_idx_{0};

    // padding 防止 false sharing — write_idx 和 read_idx 经常被不同核访问
    // 但 std::atomic<size_t> 已经是 cache-line 对齐的，不需要额外 pad
};
