# AudioChunk.msg — VAD 断句后的一段语音
# 采样率 16kHz, 单声道, S16_LE

std_msgs/Header header    # 时间戳等信息

int16[] samples            # PCM 采样数据，16kHz mono
int32 sample_rate          # 采样率，通常 16000
