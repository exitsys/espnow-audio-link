#pragma once

/* 必须与发射端一致 */
#define WIFI_CHANNEL        6

/* I2S 输出到 DAC（PCM5102A）。
 * 引脚与 alt_audio_link / rx_s3 保持一致（已实测过的接法），换固件不用改线。 */
#define I2S_OUT_BCLK_GPIO   10
#define I2S_OUT_WS_GPIO     11
#define I2S_OUT_DOUT_GPIO   12

/* 环形缓冲深度与预缓冲帧数。
 * PREBUF 决定固定延迟：10 帧 × 4ms = 40ms。
 * 这是"钉死"的值，运行期绝不自适应调整——要改就改这里重新烧。 */
#define RING_SLOTS          32
#define PREBUF_FRAMES       10

/* 延迟统计窗口 */
#define LAT_SAMPLES         1024
