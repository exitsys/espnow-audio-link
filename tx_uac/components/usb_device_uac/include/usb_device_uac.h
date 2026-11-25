/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*uac_output_cb_t)(uint8_t *buf, size_t len, void *cb_ctx);
typedef esp_err_t (*uac_input_cb_t)(uint8_t *buf, size_t len, size_t *bytes_read, void *cb_ctx);
typedef void (*uac_set_mute_cb_t)(uint32_t mute, void *cb_ctx);
typedef void (*uac_set_volume_cb_t)(uint32_t volume, void *cb_ctx);

/**
 * @brief USB UAC Device Config
 *
 */
typedef struct  {
    bool skip_tinyusb_init;                      /*!< if true, the Tinyusb and usb phy will not be initialized */
    uac_output_cb_t output_cb;                   /*!< callback function for UAC data output, if NULL, output will be disabled */
    uac_input_cb_t input_cb;                     /*!< callback function for UAC data input, if NULL, input will be disabled */
    uac_set_mute_cb_t set_mute_cb;               /*!< callback function for set mute, if NULL, the set mute request will be ignored */
    uac_set_volume_cb_t set_volume_cb;           /*!< callback function for set volume, if NULL, the set volume request will be ignored */
    void *cb_ctx;                                /*!< callback context, for user specific usage */
#if CONFIG_USB_DEVICE_UAC_AS_PART
    int spk_itf_num;                             /*!< If CONFIG_USB_DEVICE_UAC_AS_PART is enabled, you need to provide the speaker interface number */
    int mic_itf_num;                             /*!< If CONFIG_USB_DEVICE_UAC_AS_PART is enabled, you need to provide the mic interface number */
#endif
} uac_device_config_t;

/**
 * @brief Initialize the USB Audio Class (UAC) device.
 *
 * @param config Pointer to the UAC device configuration structure.
 * @return
 *       - ESP_OK on success
 *       - ESP_FAIL on failure
 */
esp_err_t uac_device_init(uac_device_config_t *config);

/* 【本工程补丁】USB 级诊断计数（2026-11-26）。USB 任务写，应用层读，不上锁：
 * 每个字段都是 32 位整数，单读单写是原子的，跨字段不要求严格一致。
 *
 * 起因：2026-11-25 有一段录音里电脑送来的音频少了约 108 ms，接收端缓冲被抽空一次。
 * 只看应用层收到的字节数，分不清少掉的部分是主机没发，还是丢在本机。这里在每一包
 * 进来时记几个数：
 *   - zlp_cnt   零长包：有 rx_done 但没有数据 —— 主机发了空包，或者包在总线上坏了
 *   - skip_cnt  缺帧：有 SOF 却没有那一帧的 rx_done —— 主机没发 OUT 事务，
 *               或者本机没来得及把端点挂好（全速同步传输没有握手，主机不知道）
 *   - late_cnt  SOF 回调间隔超过 1.5ms —— TinyUSB 任务或 USB 中断迟到过，
 *               这种时刻的缺帧就是本机的责任
 *   - wait_cnt  进 rx_done 时上一块数据还没被取走，这次没读 FIFO
 *   - hit_rf_cnt / hit_log_cnt  零长或缺帧发生时，本机 1.5ms 内发过 ESP-NOW /
 *               应用层正在打印的次数（用来排除自身射频干扰和串口阻塞）
 * evt_* 是最近 8 次异常：开机起的毫秒数和种类（1 零长 2 缺帧 3 迟到）。
 * mark_* 三个时刻由应用层写，只用于上面两个「同时」的判断。 */
typedef struct {
    uint32_t sof_cnt;          /* SOF 回调次数，流没开的时候也是每毫秒一次 */
    uint32_t rx_cnt;           /* rx_done 次数 */
    uint32_t rx_bytes;         /* rx_done 收到的字节总数 */
    uint32_t zlp_cnt;
    uint32_t skip_cnt;
    uint32_t late_cnt;
    int32_t  late_max_us;
    int32_t  rx_gap_max_us;    /* 相邻两次 rx_done 的最大间隔 */
    uint32_t flush_cnt;        /* 判成「新的播放」而清 FIFO 的次数 */
    uint32_t wait_cnt;
    uint32_t hit_rf_cnt;
    uint32_t hit_log_cnt;
    volatile int64_t mark_rf_us;
    volatile int64_t mark_log_us;
    volatile int64_t mark_log_end_us;
    volatile uint32_t sof_frame;
    uint32_t evt_ms[8];
    uint8_t  evt_kind[8];
    uint8_t  evt_i;
} uac_diag_t;
extern uac_diag_t g_uac_diag;

#ifdef __cplusplus
}
#endif
