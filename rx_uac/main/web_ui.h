/* 内网调参面板：ESP32 自建热点，手机/电脑连上后用浏览器调水位参数。
 *
 * 为什么走 SoftAP 而不是连路由器：
 *   ESP-NOW 要求收发双方同信道。RX 一旦连上路由器，信道就跟着路由器走，
 *   TX 那边还停在 11，链路当场断。自建 AP 时信道由我们指定（= ALINK_WIFI_CHANNEL），
 *   ESP-NOW 完全不受影响，代价只是连上热点的手机没有外网。
 *
 * 注意：AP 的 beacon 会占一点空口时间，beacon_interval 设大一些（500ms）
 * 减少对 ESP-NOW 的干扰。
 */
#ifndef WEB_UI_H
#define WEB_UI_H

#include <stdint.h>
#include <stdbool.h>

/* 可在网页上调的参数。全部是运行时变量，改完立刻生效，不重编译不烧录。
 *
 * 不含 I2S_DMA_DESC：DMA 描述符数量是编译期的，改它要重建 I2S 通道，
 * 中途重建必然中断播放。想改只能重编译。
 */
typedef struct {
    int32_t target_q;    /* 目标水位（包），8~40 */
    float   deadband;    /* 死区（包），0.5~8。3.0 是音质底线 */
    float   gain;        /* P 增益，2~32。越大收敛越快但越容易振荡 */
    int32_t adj_limit;   /* 重采样比上限（Q16），50~400 = 0.08%~0.6% */
} alink_tune_t;

/* 状态快照，供网页轮询。每秒由 play_task 刷一次。 */
typedef struct {
    uint32_t pkt, lost, dup, pad, filled, plc, overrun;
    uint32_t drift_drop, i2s_err, underrun, phase_clamp;
    int32_t  queue, q_min, q_max;
    float    ratio_pct;     /* 重采样比偏离 1.0 的百分比 */
    float    out_rate;      /* 实测输出帧率 */
    float    peak_dbfs;
    int32_t  rssi;
    uint32_t zero_run_max;  /* 本秒最长连续零帧 */
    int32_t  dma_gap;
    uint32_t jump_edge, jump_mid;
    int32_t  jump_max;
    int32_t  i2s_rate;
} alink_stats_t;

/* 由 alink_rx.c 实现 */
void alink_get_stats(alink_stats_t *out);
void alink_get_tune(alink_tune_t *out);
bool alink_set_tune(const alink_tune_t *in);   /* 越界值会被夹到合法范围 */
bool alink_rx_restart_stream(void);            /* 丢弃队列重新预缓冲 */

/* 启动 HTTP 服务（AP 已在 alink_rx.c 里配好） */
void web_ui_start(void);

#endif /* WEB_UI_H */
