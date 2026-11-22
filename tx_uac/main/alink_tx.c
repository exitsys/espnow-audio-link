/*
 * audio-link 发送端（ESP32-S3）
 *
 * 音源：USB Audio Class。电视机顶盒把 S3 识别成 USB 声卡，音频走 USB 拿到，
 * 再经 WiFi 转发给 C3 接收端。
 *
 * ---------------------------------------------------------------------------
 * 数据通路（2026-11-18 定稿）
 *
 *   盒子 USB ──1ms/包──► usb_device_uac ──output_cb──► StreamBuffer ──► tx_task ──► WiFi
 *                          异步+feedback                    8 包/40ms      攒够一包就发
 *
 * 为什么是「数据驱动」而不是「固定节拍」：
 * UAC 的数据是主机的 USB 时钟推进来的，和 S3 的定时器不同源。按固定 5ms 发，
 * USB 侧缓冲必然缓慢堆积或抽干，而且这个漂移发生在 TX 侧，从 C3 的日志里看不见。
 * 所以 tx_task 只做一件事：阻塞到 StreamBuffer 里攒够一整包，就发出去。
 *
 * usb_device_uac 用的是 ASYNCHRONOUS + feedback 端点（见组件
 * tusb_uac/uac_descriptors.h 的 TUSB_ISO_EP_ATT_ASYNCHRONOUS 和 FB_EP），
 * 也就是说设备这边 FIFO 水位会反向告知主机调速 —— TX 侧的时钟漂移由它兜住，
 * 不需要我们再丢样本补样本。
 *
 * 每包背靠背发两份（ALINK_TX_DUP），接收端按 seq 去重。带宽翻倍到 3.1Mbps，
 * 换掉驱动层丢包和接收 mbox 溢出这类单次事件；持续衰落仍然要靠接收端 PLC 兜。
 *
 * ---------------------------------------------------------------------------
 * 注意：S3 的 USB-OTG 和 USB-Serial-JTAG 共用 GPIO19/20，二选一。
 * 启用 UAC 之后不能再靠 USB 口看 log，必须接 UART（GPIO43/44）用 USB-TTL 看。
 * ---------------------------------------------------------------------------
 */
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "usb_device_uac.h"
#include "alink_common.h"

static const char *TAG = "alink_tx";

/* ---------- 板载 WS2812 熄灭 ----------
 *
 * 这颗 RGB 灯不在固件控制范围内：出厂固件写进去的颜色会一直锁存，
 * 复位时数据线被噪声干扰也会自己亮起来。要熄灭只能主动发一帧 24bit 全黑。
 *
 * 不引入 led_strip / RMT 依赖，用 SPI 模拟 WS2812 时序：
 *   SPI 2.4MHz，每个 WS2812 位用 3 个 SPI 位表示
 *     0 码 = 100 → 高 0.42us / 低 0.83us
 *     1 码 = 110 → 高 0.83us / 低 0.42us     （WS2812 容差 ±150ns，都在窗口内）
 *   全 0 帧 = 72 个 SPI 位 = 9 字节，正好是 92 49 24 循环。
 *
 * 引脚不固定（国产板各家不同），启动时把常见几组都发一遍。发到悬空脚上
 * 只是一串几十微秒的脉冲，没有副作用。 */
static void led_strip_off(int pin)
{
    spi_bus_config_t bus = {
        .mosi_io_num     = pin,
        .miso_io_num     = -1,
        .sclk_io_num     = -1,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 16,
    };
    if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED) != ESP_OK) {
        return;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2400000,
        .mode           = 0,
        .spics_io_num   = -1,
        .queue_size     = 1,
    };
    spi_device_handle_t h;
    if (spi_bus_add_device(SPI2_HOST, &devcfg, &h) == ESP_OK) {
        static const uint8_t black_frame[9] = {
            0x92, 0x49, 0x24, 0x92, 0x49, 0x24, 0x92, 0x49, 0x24
        };
        spi_transaction_t t = {
            .length    = 72,
            .tx_buffer = black_frame,
        };
        spi_device_transmit(h, &t);
        spi_bus_remove_device(h);
    }
    spi_bus_free(SPI2_HOST);

    /* 帧后保持低电平 >50us，让灯珠锁存并复位 */
    gpio_set_level(pin, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
}

static void led_all_off(void)
{
    static const int pins[] = {48, 38, 21, 47, 4, 5, 6, 7, 8};
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << pins[i],
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&io) != ESP_OK) {
            continue;
        }
        gpio_set_level(pins[i], 0);
        led_strip_off(pins[i]);
    }
    ESP_LOGI(TAG, "已向 GPIO48/38/21/47/4/5/6/7/8 发送 WS2812 全黑帧");
}

/* 音源选择：1 = USB Audio Class，0 = 内部 440Hz 正弦波（没有盒子时用来验链路） */
#ifndef ALINK_TX_USE_UAC
#define ALINK_TX_USE_UAC   1
#endif

/* PCM 生产者到发送任务之间的缓冲。8 包 = 40ms，够吸收 UAC 的 1ms 级抖动。 */
#define STREAM_PKTS      8
#define STREAM_BYTES     (STREAM_PKTS * ALINK_PAYLOAD_BYTES)

/* 主机音量 → 线性增益（Q16）。65536 = 0dB = 不改变。
 *
 * 为什么要在这里做：老哥这版 UAC 的 set_volume_cb 只打了一条日志，
 * 音频样本根本不乘音量，所以 Windows/盒子上的音量条对输出完全无效，
 * 等于只能吃 RX 那层固定的 -12dB 校准 —— 这就是"声音大且调不小"的原因。
 * 现在把音量条接回真正的增益，语义与之前的固件一致：
 *   实际电平 = 主机音量（0 ~ -50dB） × 固定校准 -12dB。
 * 音量条 100% 时仍是 -12dB，和之前调好的电平完全对齐。 */
static volatile int32_t s_vol_q16 = 65536;
static volatile bool    s_muted = false;

static uint16_t s_seq = 0;
static StreamBufferHandle_t s_pcm_sb;

/* ---------- ESP-NOW 传输层 ----------
 *
 * 和 UDP 版的唯一区别就在这一层：不开热点、不起 DHCP、不建 socket。
 * 音频算法、UAC 通路、StreamBuffer、攒包逻辑全部逐字保留，方便 AB 对比。
 *
 * 用单播而不是广播：单播每包有硬件 ACK，发送回调直接告诉我们对端收没收到，
 * 等于白送一个逐包丢包探针；广播没有 ACK，只能靠对端序号事后推断。
 *
 * 976 字节一包靠 ESP-NOW v2（上限 1470），两端 IDF 都是 5.5，不需要分片。
 */
/* 接收端 MAC：menuconfig 里配（Component config → Audio link → 接收端 MAC 地址），
 * 不需要改代码。没配时退回下面的默认值，保证老固件行为不变。 */
#ifndef CONFIG_ALINK_RX_MAC
#define CONFIG_ALINK_RX_MAC "84:c7:bb:5a:73:94"
#endif

static uint8_t s_peer[ESP_NOW_ETH_ALEN];

/* 解析 "aa:bb:cc:dd:ee:ff" 或 "aa-bb-cc-dd-ee-ff"。
 * 解析失败返回 false，调用方退到默认 MAC —— 比直接编译期报错友好：
 * 用户填个 "84C7BB5A7394" 也能用，而不是烧进去以后一声不响。 */
static bool mac_from_str(const char *s, uint8_t out[ESP_NOW_ETH_ALEN])
{
    unsigned v[ESP_NOW_ETH_ALEN];
    const int n = sscanf(s, "%x:%x:%x:%x:%x:%x",
                         &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]);
    if (n != ESP_NOW_ETH_ALEN &&
        sscanf(s, "%x-%x-%x-%x-%x-%x",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != ESP_NOW_ETH_ALEN) {
        /* 也接受不带分隔符的 12 位十六进制 */
        unsigned long long all = 0;
        if (sscanf(s, "%12llx", &all) != 1) {
            return false;
        }
        for (int i = ESP_NOW_ETH_ALEN - 1; i >= 0; i--) {
            out[i] = (uint8_t)(all & 0xFF);
            all >>= 8;
        }
        return true;
    }
    for (int i = 0; i < ESP_NOW_ETH_ALEN; i++) {
        out[i] = (uint8_t)v[i];
    }
    return true;
}
static volatile uint32_t s_ack_ok, s_ack_fail;
static volatile int64_t  s_ack_ok_us;      /* 最近一次收到 ACK 的时刻 */

static void on_send(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        s_ack_ok++;
        s_ack_ok_us = esp_timer_get_time();
    } else {
        s_ack_fail++;
    }
}

static void wifi_espnow_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));       /* 关省电，这项最能压延迟 */
    ESP_ERROR_CHECK(esp_wifi_set_channel(ALINK_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(56));        /* 14 dBm，与现役 TX 一致 */

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_send));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    esp_err_t r = esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_11M_L);
    if (r != ESP_OK) ESP_LOGW(TAG, "config_espnow_rate: %s", esp_err_to_name(r));
#pragma GCC diagnostic pop

    /* 解析 menuconfig 里的接收端 MAC。填错就退回默认值并报出来，
     * 免得「烧进去没声音、还不知道是地址写错了」。 */
    if (!mac_from_str(CONFIG_ALINK_RX_MAC, s_peer)) {
        const char *fb = "84:c7:bb:5a:73:94";
        mac_from_str(fb, s_peer);
        ESP_LOGE(TAG, "CONFIG_ALINK_RX_MAC=\"%s\" 格式非法，改用默认 %s",
                 CONFIG_ALINK_RX_MAC, fb);
    }

    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, s_peer, ESP_NOW_ETH_ALEN);
    peer.channel = ALINK_WIFI_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    if (!esp_now_is_peer_exist(s_peer)) {
        ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    }

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGW(TAG, "ESP-NOW 就绪：本机 " MACSTR " → 接收端 " MACSTR " 信道 %d",
             MAC2STR(mac), MAC2STR(s_peer), ALINK_WIFI_CHANNEL);
}

/* ---------- PCM 入口：所有音源都从这里进 ---------- */

bool alink_pcm_push(const void *data, size_t bytes)
{
    if (!s_pcm_sb) {
        return false;
    }
    size_t sent = xStreamBufferSend(s_pcm_sb, data, bytes, 0);
    return sent == bytes;
}

/* ---------- 音源 A：UAC ---------- */

#if ALINK_TX_USE_UAC

static uint32_t s_uac_bytes, s_uac_cb, s_uac_drop;

/* 每 1ms 被 usb_spk_task 调一次，len = 1ms 的数据量（48k/2ch/16bit = 192 字节）。
 * 注意这是 TinyUSB 上下文，不能阻塞，所以 alink_pcm_push 用 0 超时。 */
static esp_err_t uac_output_cb(uint8_t *buf, size_t len, void *cb_ctx)
{
    s_uac_cb++;
    if (!alink_pcm_push(buf, len)) {
        s_uac_drop++;   /* StreamBuffer 满：WiFi 发不出去，丢新数据，不累积延迟 */
    } else {
        s_uac_bytes += len;
    }
    return ESP_OK;
}

static void uac_volume_cb(uint32_t volume, void *cb_ctx)
{
    /* 组件把音量映射成 volume = (dB + 50) * 2，范围 0..100：
     * 100 → 0dB，50 → -25dB，0 → -50dB。步进 2dB 对听感足够。 */
    if (volume > 100) {
        volume = 100;
    }
    const int db = (int)volume / 2 - 50;
    int32_t q = (int32_t)(65536.0f * powf(10.0f, (float)db / 20.0f));
    if (q > 65536) q = 65536;
    if (q < 0)     q = 0;
    s_vol_q16 = q;

    /* 必须限频：拖动音量条时每秒几十条 SET_CUR，
     * 每条日志在 115200 下要 8ms，会把 USB 任务堵住，音量变得不跟手。 */
    static int64_t last = 0;
    int64_t now = esp_timer_get_time();
    if (now - last > 300000) {
        last = now;
        ESP_LOGI(TAG, "主机音量 %lu%%（%d dB）", (unsigned long)volume, db);
    }
}

static void uac_mute_cb(uint32_t mute, void *cb_ctx)
{
    s_muted = (mute != 0);
    /* 这个回调跑在 USB 任务里，主机异常时会以几百 Hz 重发 SET_CUR，
     * 无条件打日志能把 115200 的 UART 彻底塞满，反过来拖死 USB —— 必须限频。 */
    static int64_t last = 0;
    int64_t now = esp_timer_get_time();
    if (now - last > 500000) {
        last = now;
        ESP_LOGI(TAG, "主机%s", mute ? "静音" : "取消静音");
    }
}

static void uac_start(void)
{
    uac_device_config_t cfg = {
        .skip_tinyusb_init = false,
        .output_cb = uac_output_cb,     /* 盒子 → S3，这是我们唯一需要的方向 */
        .input_cb = NULL,               /* 不做麦克风 */
        .set_mute_cb = uac_mute_cb,
        .set_volume_cb = uac_volume_cb,
        .cb_ctx = NULL,
    };
    ESP_ERROR_CHECK(uac_device_init(&cfg));
    ESP_LOGI(TAG, "UAC 已启动，等主机（电视盒子）接入");
}

#else

/* ---------- 音源 B：内部正弦波（验链路用） ---------- */

static float s_phase = 0.0f;
#define TONE_HZ 440.0f
static int16_t s_tone_buf[ALINK_FRAMES_PER_PKT * ALINK_CHANNELS];

static void tone_timer_cb(void *arg)
{
    static uint32_t drop = 0;
    const float step = 2.0f * (float)M_PI * TONE_HZ / (float)ALINK_SAMPLE_RATE;

    for (int i = 0; i < ALINK_FRAMES_PER_PKT; i++) {
        int16_t v = (int16_t)(sinf(s_phase) * 8000.0f);   /* 约 -18dBFS */
        s_tone_buf[i * 2] = v;
        s_tone_buf[i * 2 + 1] = v;
        s_phase += step;
        if (s_phase > 2.0f * (float)M_PI) {
            s_phase -= 2.0f * (float)M_PI;
        }
    }

    if (!alink_pcm_push(s_tone_buf, ALINK_PAYLOAD_BYTES)) {
        drop++;
        if ((drop % 200) == 1) {
            ESP_LOGW(TAG, "音源缓冲满，已丢 %lu 批", (unsigned long)drop);
        }
    }
}

static void tone_start(void)
{
    const esp_timer_create_args_t targs = {
        .callback = tone_timer_cb,
        .name = "tone",
    };
    esp_timer_handle_t t;
    ESP_ERROR_CHECK(esp_timer_create(&targs, &t));
    ESP_ERROR_CHECK(esp_timer_start_periodic(t, ALINK_PKT_MS * 1000));
    ESP_LOGI(TAG, "正弦波音源已启动 %d Hz", TONE_HZ);
}

#endif

/* ---------- 发送：数据驱动 ---------- */

/* 断供补位的包络状态（见 tx_task 里的「补位」段）。
 * s_gap_gain 是 Q8 增益：256 = 满幅，0 = 完全静音。
 * s_last_* 是上一个有效样本，衰减就从它开始，而不是从 0 开始 —— 从 0 开始
 * 等于把波形硬拽到零，那正是原来那声「咔」。 */
static int32_t s_gap_gain = 256;
static int16_t s_last_l = 0, s_last_r = 0;
/* 上一次拼包没凑满（真断供）。用来把等待窗口从 30ms 退回 5ms，
 * 断供期间仍然按 200 包/秒发静音，接住接收端的 I2S 时钟。 */
static bool s_starving = false;

static void tx_task(void *arg)
{
    static uint8_t pkt[ALINK_HDR_SIZE + ALINK_PAYLOAD_BYTES];
    uint8_t *pcm = pkt + ALINK_HDR_SIZE;

    uint32_t sent = 0, dropped = 0, starve = 0, shortread = 0;
    int64_t t_prev = esp_timer_get_time();
    int64_t gap_min = 1 << 30, gap_max = 0, gap_sum = 0;
    uint32_t gap_n = 0;
    /* 速率自检：修完 UAC feedback 之后，这里应该是 ~200 包/秒、~192000 B/s。
     * 如果 UAC 字节数明显高于 192000，说明反馈仍在把主机往 1.8 倍催。 */
    int64_t t_log = esp_timer_get_time();
    uint32_t sent_at_log = 0, uac_at_log = 0;

    while (1) {
        /* 自己拼满一整包再发。
         *
         * 原来靠 xStreamBufferReceive 的 trigger level 攒包，实测不可靠：
         * trigger 设成 960 时它会提前返回半包（约 440 字节），代码补零凑满 960
         * 照样发出去 —— 结果是发 400~450 包/秒（应 200），每包一半真音频一半静音，
         * 440Hz 被反复打断，听感就是「音调正常但一直碎」。
         *
         * 所以 trigger 现在设成 1（见 app_main），只负责「有数据就唤醒」，
         * 攒够一包由这个循环负责，行为完全可预期。 */
        size_t total = 0;
        /* 等多久算「真断供」。
         *
         * 原来把超时设成 ALINK_PKT_MS（5ms），正好等于攒满一包所需的时间：
         * UAC 回调是 1ms 一次、每次 192 字节，960 字节刚好 5ms。于是每一包
         * 都在踩线，TinyUSB 任务稍微晚几百微秒就被判成半包、尾部补零。
         * 实测 125 秒里补位 11 次，全部是这个「刚好踩线」造成的，不是真断流。
         *
         * 30ms 给 6 倍余量：正常情况数据一凑满就立刻发，包率仍是 200/s；
         * 只有音乐真停了才会走到补位。补位状态下退回 5ms 一拍，
         * 保持静音包的节奏，别让接收端 I2S 断流。 */
        const int64_t wait_us = s_starving ? 5000 : 30000;
        const int64_t deadline = esp_timer_get_time() + wait_us;

        while (total < ALINK_PAYLOAD_BYTES) {
            if (esp_timer_get_time() >= deadline) {
                break;
            }
            size_t n = xStreamBufferReceive(s_pcm_sb, pcm + total,
                                            ALINK_PAYLOAD_BYTES - total,
                                            pdMS_TO_TICKS(2));
            total += n;
        }
        s_starving = (total < ALINK_PAYLOAD_BYTES);

        const int nf_missing = (int)((ALINK_PAYLOAD_BYTES - total) / ALINK_FRAME_BYTES);
        bool pad_flag = false;
        if (nf_missing > 0) {
            pad_flag = true;
            if (total == 0) {
                starve++;
                if ((starve % 100) == 1) {
                    ESP_LOGW(TAG, "音源断供 %dms（累计 %lu 次）",
                             ALINK_PKT_MS, (unsigned long)starve);
                }
            } else {
                shortread++;
                if ((shortread % 200) == 1) {
                    ESP_LOGW(TAG, "半包 got=%u/%u（累计 %lu 次）",
                             (unsigned)total, (unsigned)ALINK_PAYLOAD_BYTES,
                             (unsigned long)shortread);
                }
            }
        }

        /* ---- 音量作用到样本 ----
         * 旧写法每包只走剩余量的 1/8：时间常数 40ms，实际从 -50dB 拉到 0dB
         * 要 170ms 才稳。而主机拖音量条时目标每秒变几十次，增益永远落后于
         * 手指 —— 表现就是「晃几下就不跟手，松手半天才到位置」。
         *
         * 改成包内线性插值：一包（5ms）内从当前值走到目标。
         * 样本级连续，没有「一包一个台阶」的块间跳变；5ms 的斜率等同于
         * 一次正常淡入淡出，听不出音量的突变感。 */
        {
            static int32_t g_cur = 65536;
            const int32_t g_tgt = s_muted ? 0 : s_vol_q16;
            const int nfr = (int)(total / ALINK_FRAME_BYTES);
            int16_t *s16 = (int16_t *)pcm;

            if (nfr > 0 && g_cur != g_tgt) {
                /* 均摊到每帧。差值小于帧数时至少走 1，否则会永远差几 LSB 追不上 */
                int32_t per = (g_tgt - g_cur) / nfr;
                if (per == 0) {
                    per = (g_tgt > g_cur) ? 1 : -1;
                }
                for (int i = 0; i < nfr; i++) {
                    g_cur += per;
                    if ((per > 0 && g_cur > g_tgt) || (per < 0 && g_cur < g_tgt)) {
                        g_cur = g_tgt;
                    }
                    /* +32768 是四舍五入。原来直接 >>16 属截断，误差恒为负，
                     * 会变成与信号相关的失真（有声段才听得到的轻微噼啪）。 */
                    s16[2 * i]     = (int16_t)((((int32_t)s16[2 * i]     * g_cur) + 32768) >> 16);
                    s16[2 * i + 1] = (int16_t)((((int32_t)s16[2 * i + 1] * g_cur) + 32768) >> 16);
                }
            } else if (nfr > 0 && g_cur != 65536) {
                for (int i = 0; i < nfr * 2; i++) {
                    s16[i] = (int16_t)((((int32_t)s16[i] * g_cur) + 32768) >> 16);
                }
            }
        }

        /* ---- 补位：绝不硬切静音 ----
         * 原来 total < 960 时直接 memset 0：波形从当前值阶跃到 0，下一包再跳回
         * 真实数据，两次一阶不连续，听感就是「咔」。实测 29 秒零丢包零欠载
         * 仍有零星爆音，源头就在这条路径（UAC 回调偶尔迟到 5ms 就会触发）。
         *
         * 改成沿「最后一个有效样本」做包络衰减：一包走 1/256（Q8 步长 1），
         * 约 10ms 滑到 0；恢复供数时再同样淡入。10ms 的淡入淡出听不出来，
         * 却足以把阶跃变成连续曲线。 */
        {
            int16_t *s16 = (int16_t *)pcm;
            const int nf_pkt = ALINK_FRAMES_PER_PKT;
            if (nf_missing > 0) {
                int16_t *tail = s16 + (int)(total / ALINK_FRAME_BYTES) * 2;
                for (int i = 0; i < nf_missing; i++) {
                    s_gap_gain -= 1;
                    if (s_gap_gain < 0) {
                        s_gap_gain = 0;
                    }
                    tail[2 * i]     = (int16_t)(((int32_t)s_last_l * s_gap_gain) >> 8);
                    tail[2 * i + 1] = (int16_t)(((int32_t)s_last_r * s_gap_gain) >> 8);
                }
            } else if (s_gap_gain < 256) {
                /* 断供后恢复：把真实数据淡入，别从 0 直接跳到满幅 */
                for (int i = 0; i < nf_pkt; i++) {
                    s_gap_gain += 2;
                    if (s_gap_gain > 256) {
                        s_gap_gain = 256;
                    }
                    s16[2 * i]     = (int16_t)(((int32_t)s16[2 * i] * s_gap_gain) >> 8);
                    s16[2 * i + 1] = (int16_t)(((int32_t)s16[2 * i + 1] * s_gap_gain) >> 8);
                }
                pad_flag = true;
            }
            s_last_l = s16[(nf_pkt - 1) * 2];
            s_last_r = s16[(nf_pkt - 1) * 2 + 1];
        }

        /* ESP-NOW 没有「注册」这一步：接收端 MAC 写死，包发出去就有 ACK。
         * 注意**不能**用「没 ACK 就先不发」来做存活判断——第一包永远发不出去，
         * 就永远等不到 ACK，直接死锁（这一版第一稿就是这个错）。
         * 正确的做法是一直发：静音包本身就是探针，收不到 ACK 只报警不断流。 */
        const int64_t now4 = esp_timer_get_time();
        if (s_ack_ok_us == 0 || now4 - s_ack_ok_us > 2000000) {
            if (now4 - t_log >= 2000000) {
                t_log = now4;
                ESP_LOGW(TAG, "接收端无 ACK（未上电或已断 2s）UAC回调=%lu UAC字节=%lu ACK失败=%lu",
                         (unsigned long)s_uac_cb, (unsigned long)s_uac_bytes,
                         (unsigned long)s_ack_fail);
            }
        }

        alink_hdr_t *h = (alink_hdr_t *)pkt;
        memset(h, 0, ALINK_HDR_SIZE);
        h->magic = ALINK_MAGIC;
        h->type = ALINK_MSG_AUDIO;
        h->seq = s_seq++;
        h->ts_us = (uint32_t)esp_timer_get_time();
        h->frames = ALINK_FRAMES_PER_PKT;
        h->flags = pad_flag ? ALINK_FLAG_PAD : 0;

        const int pkt_len = ALINK_HDR_SIZE + ALINK_PAYLOAD_BYTES;
        /* 单播 + 硬件 ACK。发送失败（ACK 超时 / 队列满）只计数，不重发：
         * 重发会把这一包的发送时刻推后，反而制造抖动，交给接收端 PLC 补更划算。 */
        if (esp_now_send(s_peer, pkt, pkt_len) != ESP_OK) {
            dropped++;
        } else {
            sent++;
        }

        int64_t now = esp_timer_get_time();
        int64_t gap = now - t_prev;
        t_prev = now;
        if (gap_n) {
            if (gap < gap_min) gap_min = gap;
            if (gap > gap_max) gap_max = gap;
            gap_sum += gap;
        }
        gap_n++;

        /* 按时间打，不按包数：发不出去的时候原来的 `sent % 1000` 永远不触发，
         * 日志全哑，分不清是没音频还是没接收端。 */
        int64_t now2 = esp_timer_get_time();
        if (now2 - t_log >= 2000000) {
            int64_t dt = now2 - t_log;
            if (dt <= 0) {
                dt = 1;
            }
            const uint32_t aok = s_ack_ok, afail = s_ack_fail;
            const uint32_t ack_pct_x100 = (aok + afail)
                ? (uint32_t)((uint64_t)aok * 10000 / (aok + afail)) : 0;
            uint32_t pps = (uint32_t)((int64_t)(sent - sent_at_log) * 1000000 / dt);
            uint32_t bps = (uint32_t)((int64_t)(s_uac_bytes - uac_at_log) * 1000000 / dt);
            t_log = now2;
            sent_at_log = sent;
            uac_at_log = s_uac_bytes;
            /* %lld 在 ESP_LOG 的 vsnprintf 路径上不可靠（实测会打出失控数据），
             * 全部改 %ld + (long) 强转；32 位平台上 int64 截断不影响这个量级。 */
#if ALINK_TX_USE_UAC
            ESP_LOGI(TAG, "发=%lu(%lu/s) UAC=%lu B/s ACK=%lu/%lu(%.1f%%) 失败=%lu 欠载=%lu 半包=%lu UAC回调=%lu UAC丢=%lu 间隔 min/avg/max=%ld/%ld/%ld us",
                     (unsigned long)sent, (unsigned long)pps, (unsigned long)bps,
                     (unsigned long)aok, (unsigned long)(aok + afail),
                     (double)ack_pct_x100 / 100.0,
                     (unsigned long)dropped,
                     (unsigned long)starve, (unsigned long)shortread,
                     (unsigned long)s_uac_cb, (unsigned long)s_uac_drop,
                     (long)gap_min, (long)(gap_n > 1 ? gap_sum / (int64_t)(gap_n - 1) : 0), (long)gap_max);
#else
            ESP_LOGI(TAG, "发=%lu(%lu/s) ACK=%lu/%lu(%.1f%%) 失败=%lu 欠载=%lu 半包=%lu 间隔 min/avg/max=%ld/%ld/%ld us",
                     (unsigned long)sent, (unsigned long)pps,
                     (unsigned long)aok, (unsigned long)(aok + afail),
                     (double)ack_pct_x100 / 100.0,
                     (unsigned long)dropped,
                     (unsigned long)starve, (unsigned long)shortread,
                     (long)gap_min, (long)(gap_n > 1 ? gap_sum / (int64_t)(gap_n - 1) : 0), (long)gap_max);
#endif
            gap_min = 1 << 30; gap_max = 0; gap_sum = 0; gap_n = 0;
        }
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 第一件事：把板载那颗 RGB 灯灭掉，别等 Wi-Fi 起来 */
    led_all_off();

    /* trigger level 设 1：StreamBuffer 只负责「有数据就唤醒发送任务」，
     * 「攒满一包」由 tx_task 自己循环控制。设成整包时实测会提前返回半包。 */
    s_pcm_sb = xStreamBufferCreate(STREAM_BYTES, 1);
    if (!s_pcm_sb) {
        ESP_LOGE(TAG, "StreamBuffer 创建失败");
        return;
    }

    wifi_espnow_init();

    /* 绑到 core1：core0 是 WiFi 驱动的家（任务优先级 23），tx 不绑核就会被它反复抢占，
     * 一被压住就攒下几包，恢复后连着发出去 —— 接收端看到的队列水位因此从 17 掉到 6
     * 再蹦回去（实测 168 秒里有 7 次这种锯齿，其中 27 次直接把队列吃到 0，欠载）。
     *
     * core1 上只有 tx 和（无亲和性的）USB 任务，节拍稳得多。
     * 优先级保持 6：低于 USB 的 10，保证 UAC 取数永远先于拼包。 */
    xTaskCreatePinnedToCore(tx_task, "tx", 4096, NULL, 6, NULL, 1);

#if ALINK_TX_USE_UAC
    uac_start();
#else
    tone_start();
#endif

    ESP_LOGI(TAG, "发送端就绪（%d Hz，%d 声道，%dms/包，音源=%s）ESP-NOW → " MACSTR,
             ALINK_SAMPLE_RATE, ALINK_CHANNELS, ALINK_PKT_MS,
             ALINK_TX_USE_UAC ? "UAC" : "TONE", MAC2STR(s_peer));
}
