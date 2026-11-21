/* ESP-NOW 音频接收端
 *
 * 链路：TX(440Hz 测试音 / I2S 输入) --ADPCM 4:1, 204B/帧, 250帧/s--> RX --I2S--> PCM5102A
 *
 * 和 alt_audio_link(UDP) 相比，这里唯一的变量是传输层：
 *   - 无 IP 栈、无 ARP/DHCP，包到即用
 *   - 每帧硬件 ACK，丢包能立刻知道（UDP 只能靠 seq 事后发现）
 *   - 不进 TCP/IP 队列，省掉一层缓冲带来的抖动
 * 音频算法（ADPCM 编解码）与 alt 版本无关，独立实现。
 */
#include <string.h>
#include <stddef.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/i2s_std.h"

#include "config.h"
#include "audio_pkt.h"
#include "adpcm.h"

static const char *TAG = "RX";

#define FRAME_SAMPLES   (AUDIO_SAMPLES * AUDIO_CHANNELS)      /* 192*2 = 384 */
#define SLOT_MASK       (RING_SLOTS - 1)

/* 环形缓冲：每槽一整帧 PCM。s_filled 标记该槽是否已有数据。 */
static int16_t                s_ring[RING_SLOTS][FRAME_SAMPLES];
static volatile uint8_t       s_filled[RING_SLOTS];

/* wr = 下一个期望写入的 seq；play = 下一个要播的 seq。
 * 用 uint32 自然回绕，差值转 int32 比较。 */
static volatile uint32_t s_wr;
static volatile uint32_t s_play_seq;
static volatile bool     s_started;

static adpcm_state_t s_st[AUDIO_CHANNELS];
static i2s_chan_handle_t s_tx_chan;

/* 统计 */
static volatile uint32_t s_rx_ok, s_lost, s_dup, s_bad;
static volatile uint32_t s_under, s_over, s_skip;

/* ---------- ESP-NOW 收包 ---------- */

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len < (int)offsetof(audio_pkt_t, payload)) { s_bad++; return; }

    const audio_pkt_t *pkt = (const audio_pkt_t *)data;
    if (pkt->magic != AUDIO_PKT_MAGIC ||
        pkt->channels != AUDIO_CHANNELS ||
        pkt->samples  != AUDIO_SAMPLES) {
        s_bad++;
        return;
    }

    uint32_t seq = pkt->seq;

    if (!s_started) {
        /* 起播：预缓冲 PREBUF 帧再开始播，s_play 允许"欠"到 wr 之前 */
        s_wr = seq;
        s_play_seq = seq - PREBUF_FRAMES;
        s_started = true;
        ESP_LOGI(TAG, "首包 seq=%" PRIu32 "，预缓冲 %d 帧", seq, PREBUF_FRAMES);
    }

    int32_t d = (int32_t)(seq - s_wr);

    if (d < 0) {
        /* 迟到包：重复或乱序，丢掉 */
        s_dup++;
        return;
    }

    if (d > 0) {
        /* 中间缺了 d 帧：把空槽标脏，播到那里走 PLC */
        uint32_t n = (d > RING_SLOTS) ? RING_SLOTS : (uint32_t)d;
        for (uint32_t i = 0; i < n; i++) s_filled[(s_wr + i) & SLOT_MASK] = 0;
        s_lost += (uint32_t)d;
        s_wr = seq;
    }

    int16_t *slot = s_ring[seq & SLOT_MASK];
    for (int ch = 0; ch < AUDIO_CHANNELS; ch++) {
        int16_t tmp[AUDIO_SAMPLES];
        adpcm_decode_block(&pkt->payload[ch * AUDIO_ADPCM_BYTES], tmp, AUDIO_SAMPLES, &s_st[ch]);
        for (int i = 0; i < AUDIO_SAMPLES; i++) slot[i * AUDIO_CHANNELS + ch] = tmp[i];
    }

    s_filled[seq & SLOT_MASK] = 1;
    s_wr = seq + 1;
    s_rx_ok++;
}

/* ---------- WiFi ---------- */

static void wifi_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));   /* 省电会带来几十 ms 抖动 */
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));

    /* ACK 也走这个速率，必须和发射端一致 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    esp_err_t r = esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_11M_L);
    if (r != ESP_OK) ESP_LOGW(TAG, "config_espnow_rate: %s", esp_err_to_name(r));
#pragma GCC diagnostic pop

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGW(TAG, "本机 MAC %02X:%02X:%02X:%02X:%02X:%02X  信道 %d",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], WIFI_CHANNEL);
}

/* ---------- I2S 输出 ---------- */

static void i2s_out_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    /* DMA 4 × 192 帧 = 16ms 缓冲，和 alt 版本一致 */
    chan_cfg.dma_desc_num  = 4;
    chan_cfg.dma_frame_num = AUDIO_SAMPLES;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = AUDIO_SAMPLE_RATE,
            .clk_src        = I2S_CLK_SRC_PLL_240M,   /* 240/12.288 整数分频，无抖动 */
            .mclk_multiple  = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_OUT_BCLK_GPIO,
            .ws   = I2S_OUT_WS_GPIO,
            .dout = I2S_OUT_DOUT_GPIO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { false, false, false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));
}

/* ---------- 播放 ---------- */

static void play_task(void *arg)
{
    static int16_t out[FRAME_SAMPLES];
    int16_t last[FRAME_SAMPLES];
    bool have_last = false;
    memset(last, 0, sizeof(last));

    while (1) {
        if (!s_started) {
            vTaskDelay(pdMS_TO_TICKS(AUDIO_FRAME_MS));
            continue;
        }

        int32_t fill = (int32_t)(s_wr - s_play_seq);

        /* 漂移纠偏：两端晶振有 ppm 差，长时间必然积累。
         * 高水位丢一帧、低水位重一帧，只在超阈值时动手，避免频繁切换。 */
        if (fill > PREBUF_FRAMES + 8) {
            s_play_seq++;             /* 丢一帧 4ms */
            s_skip++;
            continue;
        }

        uint32_t slot = s_play_seq & SLOT_MASK;

        if (fill > 0 && s_filled[slot]) {
            memcpy(out, s_ring[slot], sizeof(out));
            s_filled[slot] = 0;
            memcpy(last, out, sizeof(last));
            have_last = true;
        } else {
            /* 欠载：重复上一帧并做 8 帧淡出，避免硬边造成爆音 */
            s_under++;
            for (int i = 0; i < FRAME_SAMPLES; i += AUDIO_CHANNELS) {
                int g = 256 - (i / AUDIO_CHANNELS) * 256 / AUDIO_SAMPLES;
                if (g < 0) g = 0;
                out[i]     = have_last ? (int16_t)((int32_t)last[i] * g >> 8) : 0;
                out[i + 1] = have_last ? (int16_t)((int32_t)last[i + 1] * g >> 8) : 0;
            }
            memset(last, 0, sizeof(last));
            have_last = false;
        }

        size_t written = 0;
        i2s_channel_write(s_tx_chan, out, sizeof(out), &written, portMAX_DELAY);
        s_play_seq++;
    }
}

static void stats_task(void *arg)
{
    uint32_t l_ok = 0, l_lost = 0, l_dup = 0, l_bad = 0, l_under = 0, l_skip = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        uint32_t ok = s_rx_ok, lost = s_lost, dup = s_dup, bad = s_bad;
        uint32_t under = s_under, skip = s_skip;
        uint32_t d_ok = ok - l_ok, d_lost = lost - l_lost;
        int32_t  fill = s_started ? (int32_t)(s_wr - s_play_seq) : -1;

        ESP_LOGI(TAG, "收 %" PRIu32 " 丢 %" PRIu32 " (%.2f%%) 重 %" PRIu32 " 坏 %" PRIu32
                      " | 欠载 %" PRIu32 " 丢弃 %" PRIu32 " | 水位 %" PRId32 "/%d",
                 ok, lost, (d_ok + d_lost) ? 100.0f * d_lost / (d_ok + d_lost) : 0.0f,
                 dup, bad, under, skip, fill, RING_SLOTS);

        l_ok = ok; l_lost = lost; l_dup = dup; l_bad = bad; l_under = under; l_skip = skip;
    }
}

void app_main(void)
{
    wifi_init();
    i2s_out_init();

    adpcm_reset(&s_st[0]);
    adpcm_reset(&s_st[1]);

    /* 播放任务钉在 core 1，尽量不被打断 */
    xTaskCreatePinnedToCore(play_task, "play", 4096, NULL, 12, NULL, 1);
    xTaskCreate(stats_task, "stats", 3072, NULL, 3, NULL);
}
