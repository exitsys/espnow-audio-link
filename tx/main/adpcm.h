#pragma once
#include <stdint.h>

/* IMA ADPCM（4bit），每声道独立状态。
 * 编码/解码都是逐样本递推，状态必须连续，掉一帧后建议重置。 */
typedef struct {
    int32_t valprev;
    int32_t index;
} adpcm_state_t;

void adpcm_reset(adpcm_state_t *st);

/* nsamples 个 int16 压成 nsamples/2 字节（低半字节在前） */
void adpcm_encode_block(const int16_t *in, uint8_t *out, int nsamples, adpcm_state_t *st);

/* nsamples/2 字节还原成 nsamples 个 int16 */
void adpcm_decode_block(const uint8_t *in, int16_t *out, int nsamples, adpcm_state_t *st);
