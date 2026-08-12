// ============================================================================
// 文件: src/audio/wav_writer.hpp
// 作用: 把内存中的 PCM 音频数据保存为 .wav 文件
// 为什么需要: 录音完成后，需要把数据写到磁盘上验证（用 aplay 播放）
// ============================================================================
//
// 一句话理解 WAV 格式:
//   WAV = 44 字节的"文件头" + 原始 PCM 数据
//   文件头告诉播放器: 采样率? 声道数? 位深? 数据多长?
//   剩下的就是你在录音回调里拿到的 int16_t 数组原样写入

#pragma once

#include <cstdint>
#include <fstream>
#include <string>

namespace audio {

// 写入标准 WAV 文件（PCM 格式，16-bit 有符号整数）
//   path       : 输出文件路径，如 "test.wav"
//   samples    : PCM 采样数据指针
//   num_samples: 采样总数（= 帧数 × 声道数）
//   sample_rate: 采样率，如 16000
//   channels   : 声道数，1=单声道，2=立体声
inline bool WriteWav(const std::string& path,
                     const int16_t* samples,
                     size_t num_samples,
                     uint32_t sample_rate,
                     uint16_t channels)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;

    uint32_t data_size   = static_cast<uint32_t>(num_samples * sizeof(int16_t));
    uint32_t byte_rate   = sample_rate * channels * sizeof(int16_t);
    uint16_t block_align = channels * sizeof(int16_t);

    // ---- RIFF 块 ----
    f.write("RIFF", 4);
    uint32_t chunk_size = 36 + data_size;
    f.write(reinterpret_cast<const char*>(&chunk_size), 4);
    f.write("WAVE", 4);

    // ---- fmt 块（描述音频格式）----
    f.write("fmt ", 4);
    uint32_t fmt_size   = 16;          // PCM 格式固定 16
    uint16_t audio_fmt  = 1;           // 1 = PCM
    uint16_t bits       = 16;          // 16-bit
    f.write(reinterpret_cast<const char*>(&fmt_size), 4);
    f.write(reinterpret_cast<const char*>(&audio_fmt), 2);
    f.write(reinterpret_cast<const char*>(&channels), 2);
    f.write(reinterpret_cast<const char*>(&sample_rate), 4);
    f.write(reinterpret_cast<const char*>(&byte_rate), 4);
    f.write(reinterpret_cast<const char*>(&block_align), 2);
    f.write(reinterpret_cast<const char*>(&bits), 2);

    // ---- data 块（真正的音频数据）----
    f.write("data", 4);
    f.write(reinterpret_cast<const char*>(&data_size), 4);
    f.write(reinterpret_cast<const char*>(samples), data_size);

    return true;
}

}  // namespace audio
