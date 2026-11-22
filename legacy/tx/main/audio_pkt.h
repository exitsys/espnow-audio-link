#pragma once
#include <stdint.h>

/* 4ms 一帧：48kHz 下 192 采样，IMA-ADPCM 4bit 压到 96 字节/声道。
 * 整包 204 字节，卡在 ESP-NOW 250 字节上限内。 */
#define AUDIO_SAMPLE_RATE   48000
#define AUDIO_FRAME_MS      4
#define AUDIO_SAMPLES       (AUDIO_SAMPLE_RATE * AUDIO_FRAME_MS / 1000)  /* 192 */
#define AUDIO_ADPCM_BYTES   (AUDIO_SAMPLES / 2)                          /* 96  */
#define AUDIO_CHANNELS      2
#define AUDIO_PKT_MAGIC     0xA5

typedef struct {
    uint8_t  magic;
    uint8_t  channels;
    uint16_t samples;
    uint32_t seq;       /* 帧序号，接收端用来定位环形缓冲槽位 */
    uint32_t ts_us;     /* 采集时刻，esp_timer 微秒（低 32 位，溢出靠无符号减法自然回绕） */
    uint8_t  payload[AUDIO_CHANNELS * AUDIO_ADPCM_BYTES];  /* 前 96 左声道，后 96 右声道 */
} __attribute__((packed)) audio_pkt_t;
