#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
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

static const char *TAG = "TX";

#if !USE_TEST_TONE
static i2s_chan_handle_t s_i2s_rx = NULL;
#endif
static adpcm_state_t s_st[AUDIO_CHANNELS];
static uint32_t s_seq = 0;
static volatile uint32_t s_sent = 0;
static volatile uint32_t s_send_fail = 0;

/* 广播地址：不用提前知道接收端 MAC，先跑通。
 * 链路稳定后改成接收端实际 MAC（单播有 ACK，丢包率会低不少）。 */
static uint8_t s_peer[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static void on_send(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) s_sent++;
    else s_send_fail++;
}

static void wifi_espnow_init(void)
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

    /* 关省电是低延迟的前提，默认的 modem sleep 会带来几十 ms 抖动 */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(TX_POWER_QDBM));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_send));

    /* 默认 1Mbps：204 字节一帧要占 1.6ms 空口，250 帧/s 就是 40% 占空比。
     * 拉到 11M 后单帧 0.15ms，占空比降到 4%——给同频段别的设备腾地方，
     * 也让每帧的发送时刻更确定。 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    esp_err_t r = esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_11M_L);
    if (r != ESP_OK) ESP_LOGW(TAG, "config_espnow_rate: %s", esp_err_to_name(r));
#pragma GCC diagnostic pop

    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, s_peer, ESP_NOW_ETH_ALEN);
    peer.channel = WIFI_CHANNEL;
    peer.encrypt = false;
    if (!esp_now_is_peer_exist(s_peer)) {
        ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    }

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGW(TAG, "本机 MAC %02X:%02X:%02X:%02X:%02X:%02X  信道 %d",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], WIFI_CHANNEL);
}

#if !USE_TEST_TONE
static void i2s_input_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &s_i2s_rx));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_IN_BCLK_GPIO,
            .ws   = I2S_IN_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_IN_DIN_GPIO,
            .invert_flags = { false, false, false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_rx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_rx));
}
#endif

static void audio_task(void *arg)
{
    static int16_t pcm[AUDIO_SAMPLES * AUDIO_CHANNELS];
    static int16_t ch_buf[AUDIO_SAMPLES];
    static audio_pkt_t pkt;

    float phase = 0.0f;
    const float phase_inc = 2.0f * (float)M_PI * 440.0f / (float)AUDIO_SAMPLE_RATE;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
#if USE_TEST_TONE
        for (int i = 0; i < AUDIO_SAMPLES; i++) {
            int16_t v = (int16_t)(8000.0f * sinf(phase));
            phase += phase_inc;
            if (phase > 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
            pcm[i * 2 + 0] = v;
            pcm[i * 2 + 1] = v;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(AUDIO_FRAME_MS));
#else
        size_t rd = 0;
        i2s_channel_read(s_i2s_rx, pcm, sizeof(pcm), &rd, portMAX_DELAY);
        if (rd != sizeof(pcm)) continue;
#endif

        pkt.magic    = AUDIO_PKT_MAGIC;
        pkt.channels = AUDIO_CHANNELS;
        pkt.samples  = AUDIO_SAMPLES;
        pkt.seq      = s_seq++;
        pkt.ts_us    = (uint32_t)esp_timer_get_time();

        for (int ch = 0; ch < AUDIO_CHANNELS; ch++) {
            for (int i = 0; i < AUDIO_SAMPLES; i++) ch_buf[i] = pcm[i * AUDIO_CHANNELS + ch];
            adpcm_encode_block(ch_buf,
                               &pkt.payload[ch * AUDIO_ADPCM_BYTES],
                               AUDIO_SAMPLES,
                               &s_st[ch]);
        }

        esp_now_send(s_peer, (const uint8_t *)&pkt, sizeof(pkt));
#if REDUNDANT_SEND
        esp_now_send(s_peer, (const uint8_t *)&pkt, sizeof(pkt));
#endif
    }
}

static void stats_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "已发 %" PRIu32 " 帧, 发送失败 %" PRIu32 ", 码率 %d kbps",
                 s_sent, s_send_fail,
                 (int)(AUDIO_SAMPLE_RATE * AUDIO_CHANNELS * 16 / 4 / 1000));
    }
}

void app_main(void)
{
    wifi_espnow_init();

#if !USE_TEST_TONE
    i2s_input_init();
#else
    ESP_LOGW(TAG, "测试音模式：不接音频硬件，循环发 440Hz 正弦，用来裸测链路抖动");
#endif

    adpcm_reset(&s_st[0]);
    adpcm_reset(&s_st[1]);

    /* 优先级高一点，编码+发送不能被别的事情拖后 */
    xTaskCreatePinnedToCore(audio_task, "audio_tx", 4096, NULL, 10, NULL, 1);
    xTaskCreate(stats_task, "stats", 3072, NULL, 3, NULL);
}
