/*
 * SPDX-FileCopyrightText: 2023-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * 【本工程改动】espnow_audio，2026-11。改动一览见仓库根目录 NOTICE.md，
 * 文件内的改动处都标了「本工程补丁」。
 *
 *   1. tud_audio_set_itf_cb：去掉 USB 中断上下文里的 printf。
 *   2. tud_audio_rx_done_isr：反馈值固定为标称速率，不再交给 TinyUSB 的
 *      FIFO_COUNT 算法；只读走 FIFO 半满线以上的数据，水位线按帧长对齐。
 *   3. tud_audio_rx_done_isr：上一块数据没被 usb_spk_task 取走时先不读，
 *      留在 FIFO 里等下一次（原版会覆盖，丢 1ms 音频）。
 *   4. usb_device_uac.h 的 uac_diag_t：每一包进来时记几个数，用来判断
 *      「音频少了」是主机没发还是丢在本机。为了拿到帧号，挂载后打开 SOF 回调。
 */

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_private/usb_phy.h"
#include "esp_timer.h"
#include "tusb.h"
#include "uac_config.h"
#include "usb_device_uac.h"
#include "uac_descriptors.h"

static const char *TAG = "usbd_uac";

const uint32_t sample_rates[] = {DEFAULT_SAMPLE_RATE};

#define N_SAMPLE_RATES  TU_ARRAY_SIZE(sample_rates)

enum {
    VOLUME_CTRL_0_DB = 0,
    VOLUME_CTRL_10_DB = 2560,
    VOLUME_CTRL_20_DB = 5120,
    VOLUME_CTRL_30_DB = 7680,
    VOLUME_CTRL_40_DB = 10240,
    VOLUME_CTRL_50_DB = 12800,
    VOLUME_CTRL_60_DB = 15360,
    VOLUME_CTRL_70_DB = 17920,
    VOLUME_CTRL_80_DB = 20480,
    VOLUME_CTRL_90_DB = 23040,
    VOLUME_CTRL_100_DB = 25600,
    VOLUME_CTRL_SILENCE = 0x8000,
};

// Resolution per format
const uint8_t spk_resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX};
const uint8_t mic_resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_TX};

typedef struct {
    usb_phy_handle_t phy_hdl;
    uac_device_config_t user_cfg;
    int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];         // SPK feature unit (RX = OUT = SPK), +1 for master channel 0
    int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];      // SPK feature unit (RX = OUT = SPK), +1 for master channel 0
    uint8_t mic_buf1[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ];      // Buffer for microphone data
    uint8_t mic_buf2[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ];      // Buffer for microphone data
    uint8_t spk_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ];      // Buffer for speaker data
    uint8_t *mic_buf_write;                                      // Pointer to the buffer to write to
    uint8_t *mic_buf_read;                                       // Pointer to the buffer to read from
    int spk_data_size;                                           // Speaker data size received in the last frame
    int mic_data_size;
    int spk_itf_num;
    int mic_itf_num;
    uint8_t spk_resolution;
    uint8_t mic_resolution;
    uint32_t current_sample_rate;                                // Current resolution, update on format change
    TaskHandle_t mic_task_handle;
    TaskHandle_t spk_task_handle;
    size_t spk_bytes_per_ms;
    size_t mic_bytes_per_ms;
    bool spk_active;
    bool mic_active;
} uac_device_t;

static uac_device_t *s_uac_device = NULL;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
#define UAC_ENTER_CRITICAL()    portENTER_CRITICAL(&s_mux)
#define UAC_EXIT_CRITICAL()     portEXIT_CRITICAL(&s_mux)

/* 【本工程补丁】USB 级诊断计数，字段含义见 usb_device_uac.h 的 uac_diag_t。 */
uac_diag_t g_uac_diag;

/* 记一次异常：时刻（开机起的毫秒数）和种类，只留最近 8 次。
 * 零长和缺帧发生时顺便记下本机当时是不是正在发 ESP-NOW（mark_rf_us 在 1.5ms 内）
 * 或者应用层正在打印 —— 用来排除自身射频干扰和串口阻塞这两种本机原因。 */
static void uac_diag_event(uint8_t kind, int64_t now)
{
    uac_diag_t *d = &g_uac_diag;
    if (kind == 1 || kind == 2) {
        if (d->mark_rf_us != 0 && now - d->mark_rf_us < 1500000) {
            d->hit_rf_cnt++;
        }
        if (d->mark_log_us != 0 && now >= d->mark_log_us && now <= d->mark_log_end_us) {
            d->hit_log_cnt++;
        }
    }
    d->evt_ms[d->evt_i]   = (uint32_t)(now / 1000);
    d->evt_kind[d->evt_i] = kind;
    d->evt_i = (uint8_t)((d->evt_i + 1) % TU_ARRAY_SIZE(d->evt_ms));
}

/* 缺帧统计的累加状态（见 tud_audio_rx_done_isr），流重新开始时清零。 */
static int64_t  s_diag_prev_us;
static uint32_t s_diag_prev_frm;
static uint32_t s_diag_df_acc;
static uint32_t s_diag_rx_n;

/* SOF 回调，跑在 USB 任务里，每毫秒一次。正常情况下相邻两次间隔 1ms；
 * 超过 1.5ms 说明 USB 任务（或它依赖的 USB 中断）被卡住了一帧以上。 */
void tud_sof_cb(uint32_t frame_count)
{
    static int64_t t_prev = 0;
    const int64_t now = esp_timer_get_time();
    g_uac_diag.sof_frame = frame_count;
    g_uac_diag.sof_cnt++;
    if (t_prev) {
        const int64_t gap = now - t_prev;
        if (gap > 1500) {
            g_uac_diag.late_cnt++;
            if (gap > g_uac_diag.late_max_us) {
                g_uac_diag.late_max_us = (int32_t)gap;
            }
            uac_diag_event(3, now);
        }
    }
    t_prev = now;
}

static void usb_phy_init(void)
{
    // Configure USB PHY
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .target = USB_PHY_TARGET_INT,
#if CONFIG_TINYUSB_RHPORT_HS
        .otg_speed = USB_PHY_SPEED_HIGH,
#endif
    };
    usb_new_phy(&phy_conf, &s_uac_device->phy_hdl);
}

static void tusb_device_task(void *arg)
{
    while (1) {
        tud_task();
    }
}

#if !CONFIG_USB_DEVICE_UAC_AS_PART
// Invoked when device is mounted
void tud_mount_cb(void)
{
    s_uac_device->spk_active = false;
    s_uac_device->mic_active = false;
    ESP_LOGI(TAG, "USB mounted");
    /* 【本工程补丁】诊断要用帧号判断「缺帧」，打开给应用层的 SOF 回调。
     * 组件自己用的 FIFO_COUNT 反馈算法不需要 SOF，所以原版没开。 */
    tud_sof_cb_enable(true);
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    ESP_LOGI(TAG, "USB unmounted");
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    s_uac_device->spk_active = false;
    s_uac_device->mic_active = false;
    ESP_LOGI(TAG, "USB suspended");
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    ESP_LOGI(TAG, "USB resumed");
}
#endif

// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        if (request->bRequest == AUDIO_CS_REQ_CUR) {
            TU_LOG1("Clock get current freq %lu\r\n", s_uac_device->current_sample_rate);
            audio_control_cur_4_t curf = { (int32_t) tu_htole32(s_uac_device->current_sample_rate) };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
        } else if (request->bRequest == AUDIO_CS_REQ_RANGE) {
            audio_control_range_4_n_t(N_SAMPLE_RATES) rangef = {
                .wNumSubRanges = tu_htole16(N_SAMPLE_RATES)
            };
            TU_LOG1("Clock get %d freq ranges\r\n", N_SAMPLE_RATES);
            for (uint8_t i = 0; i < N_SAMPLE_RATES; i++) {
                rangef.subrange[i].bMin = (int32_t) sample_rates[i];
                rangef.subrange[i].bMax = (int32_t) sample_rates[i];
                rangef.subrange[i].bRes = 0;
                TU_LOG1("Range %d (%d, %d, %d)\r\n", i, (int)rangef.subrange[i].bMin, (int)rangef.subrange[i].bMax, (int)rangef.subrange[i].bRes);
            }

            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
        }
    } else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID && request->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t cur_valid = {
            .bCur = 1
        };
        TU_LOG1("Clock get is valid %u\r\n", cur_valid.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
    }

    TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
}

// Helper for clock set requests
static bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
    (void)rhport;

    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));

        uint32_t target_sample_rate = (uint32_t)((audio_control_cur_4_t const *)buf)->bCur;
        TU_LOG1("Clock set current freq: %ld\r\n", target_sample_rate);

        if (target_sample_rate != s_uac_device->current_sample_rate) {
            // For now, we only support one sample rate
            return false;
        }

        return true;
    } else {
        TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n",
                request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t* feedback_param)
{
    (void)func_id;
    (void)alt_itf;
    // Set feedback method to fifo counting
    feedback_param->method = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    feedback_param->sample_freq = s_uac_device->current_sample_rate;

    ESP_LOGD(TAG, "Feedback method: %d, sample freq: %"PRIu32"", feedback_param->method, feedback_param->sample_freq);
}

// Helper for feature unit get requests
static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);

    if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && request->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t mute1 = {
            .bCur = s_uac_device->mute[request->bChannelNumber]
        };
        TU_LOG1("Get channel %u mute %d\r\n", request->bChannelNumber, mute1.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &mute1, sizeof(mute1));
    } else if (UAC2_ENTITY_SPK_FEATURE_UNIT && request->bControlSelector == AUDIO_FU_CTRL_VOLUME) {
        if (request->bRequest == AUDIO_CS_REQ_RANGE) {
            audio_control_range_2_n_t(1) range_vol = {
                .wNumSubRanges = tu_htole16(1),
                .subrange[0] = { .bMin = tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(256) }
            };
            TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\r\n", request->bChannelNumber,
                    range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
        } else if (request->bRequest == AUDIO_CS_REQ_CUR) {
            audio_control_cur_2_t cur_vol = {
                .bCur = tu_htole16(s_uac_device->volume[request->bChannelNumber])
            };
            TU_LOG1("Get channel %u volume %d dB\r\n", request->bChannelNumber, cur_vol.bCur / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
        }
    }
    TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
    (void)rhport;

    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO_FU_CTRL_MUTE) {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));
        s_uac_device->mute[request->bChannelNumber] = ((audio_control_cur_1_t const *)buf)->bCur;
        TU_LOG1("Set speaker channel %d Mute: %d\r\n", request->bChannelNumber, s_uac_device->mute[request->bChannelNumber]);
        if (s_uac_device->user_cfg.set_mute_cb) {
            s_uac_device->user_cfg.set_mute_cb(s_uac_device->mute[request->bChannelNumber], s_uac_device->user_cfg.cb_ctx);
        }

        return true;
    } else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME) {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));
        s_uac_device->volume[request->bChannelNumber] = ((audio_control_cur_2_t const *)buf)->bCur;
        int volume_db = s_uac_device->volume[request->bChannelNumber] / 256; // Convert to dB
        int volume = (volume_db + 50) * 2; // Map to range 0 to 100
        TU_LOG1("Set speaker channel %d volume: %d dB (%d)\r\n", request->bChannelNumber, volume_db, volume);
        if (s_uac_device->user_cfg.set_volume_cb) {
            s_uac_device->user_cfg.set_volume_cb(volume, s_uac_device->user_cfg.cb_ctx);
        }
        return true;
    } else {
        TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n",
                request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;

    if (request->bEntityID == UAC2_ENTITY_CLOCK) {
        return tud_audio_clock_get_request(rhport, request);
    }
    if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return tud_audio_feature_unit_get_request(rhport, request);
    } else {
        TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n",
                request->bEntityID, request->bControlSelector, request->bRequest);
    }
    return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;

    if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return tud_audio_feature_unit_set_request(rhport, request, buf);
    }
    if (request->bEntityID == UAC2_ENTITY_CLOCK) {
        return tud_audio_clock_set_request(rhport, request, buf);
    }
    TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;

    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

#if SPEAK_CHANNEL_NUM
    if (s_uac_device->spk_itf_num == itf && alt == 0) {
        TU_LOG2("Speaker interface closed");
        s_uac_device->spk_data_size = 0;
        s_uac_device->spk_active = false;
        s_diag_prev_us = 0;          /*!< 关流到再开流之间的帧不算缺帧 */
    }
#endif

#if MIC_CHANNEL_NUM
    if (s_uac_device->mic_itf_num == itf && alt == 0) {
        TU_LOG2("Microphone interface closed");
        s_uac_device->mic_data_size = 0;
        s_uac_device->mic_active = false;
    }
#endif

    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    TU_LOG2("Set interface %d alt %d\r\n", itf, alt);

#if SPEAK_CHANNEL_NUM
    if (s_uac_device->spk_itf_num == itf && alt != 0) {
        s_uac_device->spk_data_size = 0;
        s_diag_prev_us = 0;          /*!< 新的一次流，缺帧统计从头算 */
        s_diag_df_acc  = 0;
        s_diag_rx_n    = 0;
        s_uac_device->spk_resolution = spk_resolutions_per_format[alt - 1];
        s_uac_device->spk_active = true;
        s_uac_device->spk_bytes_per_ms = s_uac_device->current_sample_rate / 1000 * SPEAK_CHANNEL_NUM * CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX;
        xTaskNotifyGive(s_uac_device->spk_task_handle);
        /* 这里绝对不能 printf：本函数跑在 USB 中断上下文，裸 printf 走的是
         * newlib 的 stdout 锁，和别处的 ESP_LOG 是两套锁。
         * 混用会把 stdout 锁踩坏 —— 表现是日志被截断成 "I (502) esp_"，
         * 严重时 USB 任务卡死，主机超时后重发 SET_INTERFACE，又触发一次
         * printf，形成 15Hz 的重枚举死循环（之前那次 abort() 也是同一个病根）。
         * 需要调试就开 TU_LOG1，别用 printf。 */
        TU_LOG1("Speaker interface %d-%d opened", itf, alt);
    }
#endif

#if MIC_CHANNEL_NUM
    if (s_uac_device->mic_itf_num == itf && alt != 0) {
        s_uac_device->mic_data_size = 0;
        s_uac_device->mic_resolution = mic_resolutions_per_format[alt - 1];
        s_uac_device->mic_active = true;
        s_uac_device->mic_bytes_per_ms = s_uac_device->current_sample_rate / 1000 * MIC_CHANNEL_NUM * CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX;
        // Prefill to the flow-control setpoint (depth/2): the controller only
        // corrects ~0.4% per packet, so from an empty FIFO it climbs for
        // seconds, sending audible zero-length packets on the way. Mirrors
        // the speaker path's buffered start.
        tu_fifo_t *in_ff = tud_audio_get_ep_in_ff();
        if (in_ff != NULL) {
            static const uint8_t zeros[64] = {0};
            uint16_t want = tu_fifo_depth(in_ff) / 2;
            while (want > 0) {
                uint16_t chunk = (want > sizeof(zeros)) ? (uint16_t)sizeof(zeros) : want;
                if (tud_audio_write(zeros, chunk) != chunk) {
                    break;
                }
                want -= chunk;
            }
        }
        xTaskNotifyGive(s_uac_device->mic_task_handle);
        TU_LOG1("Microphone interface %d-%d opened", itf, alt);
    }
#endif

    return true;
}

bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)ep_out;
    (void)cur_alt_setting;

    static bool new_play = false;
    static int64_t last_time = 0;
    int64_t now = esp_timer_get_time();

    /**
     * @brief If no data is received for a certain period, it is considered as the initiation
     *        of a new audio transmission. At this point, the FIFO data is cleared, and a segment
     *        of data is buffered in the I2S.
     */
    if (now - last_time > 100 * CONFIG_UAC_SPK_NEW_PLAY_INTERVAL) {
        new_play = true;
        tud_audio_n_clear_ep_out_ff(func_id);
        g_uac_diag.flush_cnt++;
    }
    last_time = now;

    /* 【本工程补丁】USB 级诊断计数（见 usb_device_uac.h 的 uac_diag_t）。 */
    {
        uac_diag_t *d = &g_uac_diag;
        const uint32_t frm = d->sof_frame;       /* 最近一次 SOF 回调的帧号，14 位回绕 */
        d->rx_cnt++;
        d->rx_bytes += n_bytes_received;
        if (s_diag_prev_us) {
            const int64_t gap = now - s_diag_prev_us;
            if (gap > d->rx_gap_max_us) {
                d->rx_gap_max_us = (int32_t)gap;
            }
            /* 累计算：从这一轮流的第一包算起，「帧号一共走了多少帧」减「收到了多少次 rx_done」，
             * 就是没收到数据的帧数。不逐次判断「相邻帧号差是不是 2」：USB 任务偶尔迟到一帧时，
             * 那一帧的数据会排到下一个 SOF 之后才处理，帧号差先少 1、下一次再多 1，
             * 逐次判断会误报，累计算会抵消（中间结果可能是负的，所以用有符号数）。
             * 帧号会回绕：全速是 11 位（每 2048 毫秒一圈，2026-11-26 上板时按 14 位算，
             * 每圈误报一次），高速是 14 位的微帧号、一帧走 8 步。 */
            const bool hs = (tud_speed_get() == TUSB_SPEED_HIGH);
            const uint32_t df = (frm - s_diag_prev_frm) & (hs ? 0x3FFFu : 0x7FFu);
            s_diag_df_acc += hs ? (df >> 3) : df;
            s_diag_rx_n++;
            const int32_t skip_now = (int32_t)((int64_t)s_diag_df_acc - (int64_t)s_diag_rx_n);
            if (skip_now > 0 && (uint32_t)skip_now > d->skip_cnt) {
                d->skip_cnt = (uint32_t)skip_now;
                uac_diag_event(2, now);
            }
        }
        s_diag_prev_frm = frm;
        s_diag_prev_us  = now;
        if (n_bytes_received == 0) {
            d->zlp_cnt++;
            uac_diag_event(1, now);
        }
    }

    int bytes_remained = tud_audio_n_available(func_id);
    const size_t bytes_per_ms = s_uac_device->spk_bytes_per_ms;

    /*!< 【本工程补丁】反馈值直接钉在标称采样率，不交给 FIFO 水位算法。
     *
     *    TinyUSB 的 FIFO_COUNT 在这个盒子上会失稳：FIFO 一旦被填满，
     *    feedback = nom - (lvl - thr) * rate 会被压到下限 47.01kHz
     *    （min_value = (sample_freq-1)/1000 << 16），主机就按 47kHz 送，
     *    比接收端 I2S 的 48k 慢 2%，于是接收端每秒欠 5~7 个包、持续断续。
     *
     *    主机对这个反馈值是认的（它确实跟着降到了 47k），所以直接把
     *    「48 样本/毫秒」写进去即可。两端晶振 ±50ppm 的长期差由 TX 侧
     *    40ms 的 StreamBuffer 和 RX 侧的水位闭环吸收，不需要 USB 侧精细调速。 */
    {
        const uint32_t frame_div = (tud_speed_get() == TUSB_SPEED_FULL) ? 1000 : 8000;
        /* 这里写的就是标称值本身，不要加预补偿。
         *
         * 这一行曾按注释写成「×64/63，补盒子 63/64 的折扣」，但整型运算
         * (48000/1000)*64/63 = 3072/63 = 48，结果和标称值一模一样，补偿从未生效；
         * 而实测盒子送 47.93k（差 0.15%，不是 1.56%），说明折扣这个结论本身也不成立。
         * 别把它「修对」：真按 48.76 样本/帧发，主机会被催快 1.6%，重采样 ±0.3% 的上限
         * 吃不下，接收端队列会一直溢出。验收仍然看 S3 日志的「UAC=xxxxx B/s」，
         * 目标 192000 B/s。 */
        const uint32_t spm = s_uac_device->current_sample_rate / frame_div;
        tud_audio_n_fb_set(func_id, spm << 16);
    }

    /*!< 水位目标：让 FIFO 停在一半左右，只作为防溢出/防抽干的保护。
     *
     * 必须按帧长对齐（帧长 = 每样本字节 × 声道数，立体声 16bit = 4）。
     * 2156/2 - 192 = 886 不是 4 的倍数：起播后 FIFO 以整帧递增，到 1152 才第一次
     * 读走 266 字节（66 帧多 1 个样本），此后交给应用的字节流余量恒差 2 字节。
     * 应用层按整帧拼包，一次「半包」就会让后面的包全部错开一个样本（左右声道对调）。
     * 向下取整到 884 之后首块是 268 字节，读走的始终是整帧。 */
    const size_t frame_sz  = CFG_TUD_AUDIO_FUNC_1_FORMAT_1_FRAME_SZ_RX;
    const size_t half      = CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 2;
    const size_t level_raw = (half > bytes_per_ms) ? (half - bytes_per_ms) : bytes_per_ms;
    const size_t level_set = level_raw - level_raw % frame_sz;

    if (new_play) {
        if (bytes_remained < (int)level_set) {
            return true;    /*!< 攒到目标水位再起播 */
        }
        new_play = false;
    }

    /*!< ⚠ 本工程补丁（2026-11-19），原版这两行是无条件读走 1ms：
     *      s_uac_device->spk_data_size = tud_audio_n_read(func_id, spk_buf, bytes_per_ms);
     *
     *    问题：TinyUSB 在 audiod_rx_xfer_isr 里的顺序是
     *      audiod_fb_fifo_count_update(tu_fifo_count(&ep_out_ff));  ← 先算反馈
     *      tud_audio_rx_done_isr(...)                               ← 再进这里
     *    每包都把 FIFO 抽到 0，反馈只看到 ~9% 水位（192/2156），于是反馈端点一直在
     *    "数据缺失"区间，主机被逐包催快（每包约 +0.4%），最终爬到 1.7~1.8 倍速率。
     *    下行链路被灌两倍数据量 → 接收端队列长期高于高水位 → 每秒丢几十包。
     *    听感：音调正常（I2S 仍是 48k），但内容一直在跳，即"断续/发碎"。
     *
     *    修法：只读走超出目标水位的部分，把 FIFO 钉在半满，反馈才能稳在 48k。
     *    稳态下每包进 192、读 192，水位恒定在 level_set（884 字节，约 4.6ms）。 */
    /*! 【本工程补丁】上一块还没被 usb_spk_task 取走时先不读。数据留在 USB FIFO 里
     * （放得下 11ms），下一次 rx_done 再读。原版直接读进同一块 spk_buf，上一块没来得及
     * 交给应用就被覆盖，那 1ms 音频无声无息地丢了 —— 应用层看到的「电脑少送了音频」
     * 也可能是这么来的，所以同时计数。 */
    if (s_uac_device->spk_data_size != 0) {
        g_uac_diag.wait_cnt++;
        return true;
    }

    int over = bytes_remained - (int)level_set;
    if (over < (int)bytes_per_ms) {
        return true;        /*!< 还没攒够一包，等下一包。反馈看到水位低会催主机多发，正常 */
    }
    size_t bytes_require = (size_t)over;
    if (bytes_require > level_set) {
        bytes_require = level_set;   /*!< 单次最多读半个 FIFO，避免一次抽干 */
    }

    s_uac_device->spk_data_size = tud_audio_n_read(func_id, s_uac_device->spk_buf, bytes_require);
    xTaskNotifyGive(s_uac_device->spk_task_handle);
    return true;
}

bool tud_audio_tx_done_isr(uint8_t rhport, uint16_t n_bytes_sent, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)n_bytes_sent;
    (void)ep_in;
    (void)cur_alt_setting;
    size_t bytes_require = MIC_INTERVAL_MS * s_uac_device->mic_bytes_per_ms;

    tu_fifo_t *sw_in_fifo = tud_audio_n_get_ep_in_ff(func_id);
    uint16_t fifo_remained = tu_fifo_remaining(sw_in_fifo);
    if (fifo_remained < bytes_require) {
        return true;
    }

    // load data chunk by chunk
    UAC_ENTER_CRITICAL();
    if (s_uac_device->mic_data_size > 0) {
        tud_audio_n_write(func_id, (void *)s_uac_device->mic_buf_read, s_uac_device->mic_data_size);
        s_uac_device->mic_data_size = 0;
    }
    UAC_EXIT_CRITICAL();

    return true;
}

#if SPEAK_CHANNEL_NUM
static void usb_spk_task(void *pvParam)
{
    while (1) {
        if (s_uac_device->spk_active == false) {
            ulTaskNotifyTake(pdFAIL, portMAX_DELAY);
            continue;
        }
        // clear the notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (s_uac_device->spk_data_size == 0) {
            continue;
        }
        // playback the data from the ring buffer chunk by chunk
        if (s_uac_device->user_cfg.output_cb) {
            s_uac_device->user_cfg.output_cb((uint8_t *)s_uac_device->spk_buf, s_uac_device->spk_data_size, s_uac_device->user_cfg.cb_ctx);
        }
        s_uac_device->spk_data_size = 0;
    }
}
#endif

#if MIC_CHANNEL_NUM
static void usb_mic_task(void *pvParam)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        if (s_uac_device->mic_active == false) {
            // clear the notification
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            xLastWakeTime = xTaskGetTickCount();
            continue;
        }
        // clear the notification
        // read data from the microphone chunk by chunk
        size_t bytes_require = MIC_INTERVAL_MS * s_uac_device->mic_bytes_per_ms;
        if (s_uac_device->user_cfg.input_cb) {
            size_t bytes_read = 0;
            esp_err_t ret = s_uac_device->user_cfg.input_cb((uint8_t *)s_uac_device->mic_buf_write, bytes_require, &bytes_read, s_uac_device->user_cfg.cb_ctx);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read data from mic");
                continue;
            }
            uint8_t *tmp_buf = s_uac_device->mic_buf_write;
            UAC_ENTER_CRITICAL();
            s_uac_device->mic_buf_write = s_uac_device->mic_buf_read;
            s_uac_device->mic_buf_read = tmp_buf;
            s_uac_device->mic_data_size = bytes_read;
            UAC_EXIT_CRITICAL();
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MIC_INTERVAL_MS));
    }
}
#endif

esp_err_t uac_device_init(uac_device_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is NULL");
    if (s_uac_device != NULL) {
        ESP_LOGW(TAG, "uac device already initialized");
        return ESP_OK;
    }
    s_uac_device = calloc(1, sizeof(uac_device_t));
    ESP_RETURN_ON_FALSE(s_uac_device != NULL, ESP_ERR_NO_MEM, TAG, "Failed to allocate memory for uac device");
    s_uac_device->user_cfg.output_cb = config->output_cb;
    s_uac_device->user_cfg.input_cb = config->input_cb;
    s_uac_device->user_cfg.cb_ctx = config->cb_ctx;
    s_uac_device->user_cfg.set_mute_cb = config->set_mute_cb;
    s_uac_device->user_cfg.set_volume_cb = config->set_volume_cb;
    s_uac_device->current_sample_rate = DEFAULT_SAMPLE_RATE;
    s_uac_device->mic_buf_write = s_uac_device->mic_buf1;
    s_uac_device->mic_buf_read = s_uac_device->mic_buf2;

#if CONFIG_USB_DEVICE_UAC_AS_PART
    s_uac_device->spk_itf_num = config->spk_itf_num;
    s_uac_device->mic_itf_num = config->mic_itf_num;
#else
#if SPEAK_CHANNEL_NUM
    s_uac_device->spk_itf_num = ITF_NUM_AUDIO_STREAMING_SPK;
#endif
#if MIC_CHANNEL_NUM
    s_uac_device->mic_itf_num = ITF_NUM_AUDIO_STREAMING_MIC;
#endif
#endif

    BaseType_t ret_val;
    if (!config->skip_tinyusb_init) {
        usb_phy_init();
        bool usb_init = tusb_init();
        if (!usb_init) {
            ESP_LOGE(TAG, "USB Device Stack Init Fail");
            return ESP_FAIL;
        }
        ret_val = xTaskCreatePinnedToCore(tusb_device_task, "TinyUSB", 4096, NULL, CONFIG_UAC_TINYUSB_TASK_PRIORITY,
                                          NULL, CONFIG_UAC_TINYUSB_TASK_CORE == -1 ? tskNO_AFFINITY : CONFIG_UAC_TINYUSB_TASK_CORE);
        ESP_RETURN_ON_FALSE(ret_val == pdPASS, ESP_FAIL, TAG, "Failed to create TinyUSB task");
    }

#if MIC_CHANNEL_NUM
    ret_val = xTaskCreatePinnedToCore(usb_mic_task, "usb_mic_task", 4096, NULL, CONFIG_UAC_MIC_TASK_PRIORITY,
                                      &s_uac_device->mic_task_handle, CONFIG_UAC_MIC_TASK_CORE == -1 ? tskNO_AFFINITY : CONFIG_UAC_MIC_TASK_CORE);
    ESP_RETURN_ON_FALSE(ret_val == pdPASS, ESP_FAIL, TAG, "Failed to create usb_mic task");
#endif

#if SPEAK_CHANNEL_NUM
    ret_val = xTaskCreatePinnedToCore(usb_spk_task, "usb_spk_task", 4096, NULL, CONFIG_UAC_SPK_TASK_PRIORITY,
                                      &s_uac_device->spk_task_handle, CONFIG_UAC_SPK_TASK_CORE == -1 ? tskNO_AFFINITY : CONFIG_UAC_SPK_TASK_CORE);
    ESP_RETURN_ON_FALSE(ret_val == pdPASS, ESP_FAIL, TAG, "Failed to create usb_spk task");
#endif

    ESP_LOGI(TAG, "UAC Device Start, Version: %d.%d.%d", USB_DEVICE_UAC_VER_MAJOR, USB_DEVICE_UAC_VER_MINOR, USB_DEVICE_UAC_VER_PATCH);
    return ESP_OK;
}
