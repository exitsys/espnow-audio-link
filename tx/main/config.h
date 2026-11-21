#pragma once

/* 收发两端必须一致 */
#define WIFI_CHANNEL        6

/* 1 = 不接任何音频硬件，自己发 440Hz 正弦，用来裸测链路抖动。
 * 先把两端都用测试音跑一遍，量出 σ 再决定要不要买 ADC/DAC。 */
#define USE_TEST_TONE       1

/* 同一帧连发两次，抗丢包。带宽翻倍，packet 率 250→500/s，链路差时再开。 */
#define REDUNDANT_SEND      0

/* 0.25dB 步进，80 = 20dBm */
#define TX_POWER_QDBM       80

/* I2S 输入（ADC），仅 USE_TEST_TONE=0 时使用 */
#define I2S_IN_BCLK_GPIO    5
#define I2S_IN_WS_GPIO      4
#define I2S_IN_DIN_GPIO     6
