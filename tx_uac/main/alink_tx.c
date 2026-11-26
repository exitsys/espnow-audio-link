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
 * USB 侧缓冲必然缓慢堆积或抽干，而且这个漂移发生在 TX 侧，从接收端的日志里看不见。
 * 所以 tx_task 只做一件事：阻塞到 StreamBuffer 里攒够一整包，就发出去。
 *
 * usb_device_uac 用的是 ASYNCHRONOUS + feedback 端点（见组件
 * tusb_uac/uac_descriptors.h 的 TUSB_ISO_EP_ATT_ASYNCHRONOUS 和 FB_EP），
 * 但本工程的本地补丁把反馈值钉在了标称速率上（见组件 usb_device_uac.c）：
 * TX 只负责把主机送来的数据原样转发，不丢样本也不补样本，
 * 收发两端的时钟差全部交给接收端的水位闭环和 I2S 时钟跟随去消化。
 *
 * 每包只发一份，不重发、不做 FEC。ESP-NOW 单播每包有硬件 ACK，发送回调直接给出
 * 逐包成败；丢掉的包由接收端按 seq 补占位、走 PLC 外推。重发会把这一包的发送时刻
 * 推后，反而制造抖动。
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

    /* 帧后保持低电平 >280us（WS2812B 的 Treset 下限），灯珠才会锁存这一帧。
     * 这段等待必须放在 spi_bus_free() 之前：此刻 MOSI 还归 SPI 管，空闲电平就是 0。
     * spi_bus_free() 会把引脚复位成「禁用 + 上拉」，数据线立刻被拉高，
     * 就不再满足锁存条件了。 */
    vTaskDelay(pdMS_TO_TICKS(2));

    spi_bus_free(SPI2_HOST);

    /* free 之后引脚是「禁用 + 上拉」，gpio_set_level() 改不了它。
     * 重新配成推挽输出并拉低，给数据线一个确定的空闲电平。 */
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io) == ESP_OK) {
        gpio_set_level(pin, 0);
    }
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

/* 断供态追发的两个量（见 tx_task 里的节拍段）。
 *
 * 2026-11-25 上板实测：原来只要拖过一个格点就重新起拍，等于少发一个静音包。
 * 每 2 秒的统计行（和同一拍里的「音源断供」）在串口上要阻塞 10ms 以上，
 * 于是静音期间实际只有 199.8 包/秒。接收端按 200 包/秒消费，队列每秒被抽走
 * 0.2 包，几分钟的暂停就把缓冲抽到 5 包左右，重新播放时余量很浅。
 * 现在落后了照常补发，平均发包率才是严格的 200 包/秒。 */
#define ALINK_TX_CATCHUP_PKTS   10      /* 最多补这么多拍（50ms）；拖得更久才重新起拍 */
#define ALINK_TX_MIN_GAP_US     2500    /* 追发时两包之间的最小间隔 */

/* 主机已经关流（USB 接口切回 alt 0）时，手头的半包最多再等这么久就发出去。
 *
 * 正常的 30ms 等待窗口是给「流还开着、数据晚到一点」用的，那种情况等一等就能无缝接上。
 * 但切歌、暂停时主机会把流关掉再开，一等就是 30ms 以上：这 30ms 里空口上什么都没有，
 * 接收端按 200 包/秒照常消费，队列每次被抽走 5~6 包。2026-11-26 上板实测，连着切歌
 * 80 秒里 10 次关流，队列 16 → 11 → 6 → 1，三次见底。
 * 流既然已经关了，再等也等不来数据，所以只留 5ms（给「关了马上又开」的抖动），然后转去发补位包。
 * 流没关的时候的行为完全不变。 */
#if ALINK_TX_USE_UAC
#define ALINK_TX_CLOSE_WAIT_US  5000

/* 一毫秒的标称字节数：48k × 2 声道 × 16bit。USB 级诊断用它把「少了多少」折成时间。 */
#define ALINK_TX_USB_FRAME_BYTES 192

/* 关流之后要不要追发（见 tx_task 发完之后的那段）。
 * 2026-11-26 上板：同样两次关流，不追时 RX 队列 17 → 15、16 → 13；追了之后 17 → 17、16 → 16。 */
#ifndef ALINK_TX_CLOSE_CATCHUP
#define ALINK_TX_CLOSE_CATCHUP  1
#endif
#endif

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
 * 不需要改代码。没配时退回下面的默认值，保证老固件行为不变。
 * Kconfig.projbuild 里的 default 必须和这里保持一致。 */
#define ALINK_RX_MAC_DEFAULT "84:c7:bb:5a:73:94"
#ifndef CONFIG_ALINK_RX_MAC
#define CONFIG_ALINK_RX_MAC ALINK_RX_MAC_DEFAULT
#endif

static uint8_t s_peer[ESP_NOW_ETH_ALEN];
static volatile uint32_t s_ack_ok, s_ack_fail;
static volatile int64_t  s_ack_ok_us;      /* 最近一次收到 ACK 的时刻 */

/* 空口延迟诊断（2026-11-26）：从 esp_now_send 入队，到收到发送回调（拿到 ACK，或放弃）的时间。
 *
 * 统计行里的「间隔」量的是入队时刻。帧在 WiFi 驱动里排队、在空口上退避重试的时间它看不见：
 * 2026-11-26 两次录到接收端断流 80~100ms 然后成批涌到，而 TX 的入队间隔和 ACK 全部正常 ——
 * 分不清是帧在空口上被挡了，还是接收端自己内部卡了。这个数在空口被挡时会跟着涨，
 * 接收端内部卡的时候不会。
 *
 * 发送任务写、WiFi 任务读，各用各的下标；回调顺序和入队顺序一致（FIFO）。 */
#define ALINK_TX_LAT_SLOTS 32
static int64_t s_lat_call_us[ALINK_TX_LAT_SLOTS];
static volatile uint32_t s_lat_head, s_lat_tail;
static volatile int64_t  s_lat_max_us;        /* 本统计窗口内的最大值 */
static volatile uint32_t s_lat_over20, s_lat_over50;   /* 本统计窗口内超过 20ms / 50ms 的帧数 */

static void on_send(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    const int64_t now = esp_timer_get_time();
    if (status == ESP_NOW_SEND_SUCCESS) {
        s_ack_ok++;
        s_ack_ok_us = now;
    } else {
        s_ack_fail++;
    }
    const uint32_t h = s_lat_head;
    if ((int32_t)(s_lat_tail - h) > 0) {
        const int64_t lat = now - s_lat_call_us[h % ALINK_TX_LAT_SLOTS];
        s_lat_head = h + 1;
        if (lat > s_lat_max_us) {
            s_lat_max_us = lat;
        }
        if (lat > 20000) {
            s_lat_over20++;
            if (lat > 50000) {
                s_lat_over50++;
            }
        }
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
     * 免得「烧进去没声音、还不知道是地址写错了」。
     * 解析是严格的（见 alink_common.h）：少一位、多一段、某段超过 0xFF 都算错，
     * 不会悄悄解析成另一个地址。接受的三种写法见 README。 */
    if (!alink_mac_from_str(CONFIG_ALINK_RX_MAC, s_peer)) {
        alink_mac_from_str(ALINK_RX_MAC_DEFAULT, s_peer);
        ESP_LOGE(TAG, "CONFIG_ALINK_RX_MAC=\"%s\" 格式非法，改用默认 %s",
                 CONFIG_ALINK_RX_MAC, ALINK_RX_MAC_DEFAULT);
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

/* USB 级诊断（usb_device_uac.h 的 uac_diag_t）的变化，有变化才打一行。
 *
 * 2026-11-25 做信道对照时有一段录音里，电脑送来的音频在 32 秒里少了约 104 ms，
 * 接收端队列从 20 包被抽到见底。只看「UAC=… B/s」分不清少掉的是主机没发
 * （零长包、短包、整帧没有数据），还是丢在本机（USB 任务迟到没接住、单缓冲被覆盖）。
 * 组件在每一包进来时记了几个数，这里在每次统计行之后看一遍增量；
 * 平时这些数不动，不打日志。
 *
 * 「清 FIFO」和开关流不算异常：流关闭再打开时组件本来就会清一次 FIFO（攒到水位线再起播），
 * 算进去的话每次切歌都要误报一行。开关流的次数印在基线行和异常行里，方便对着看。 */
#if ALINK_TX_USE_UAC
static void usb_diag_report(void)
{
    static uint32_t prev_rx, prev_zlp, prev_skip, prev_late, prev_wait;
    static uint32_t prev_short, prev_sbytes, prev_open, prev_close;
    const uac_diag_t *d = &g_uac_diag;
    /* 各字段分别读一次就用这一份：USB 任务随时在改，跨字段不追求严格一致，
     * 但增量和「上一次的值」必须出自同一份，否则会漏计或重复计。 */
    const uint32_t rx     = d->rx_cnt;
    const uint32_t zlp    = d->zlp_cnt;
    const uint32_t skip   = d->skip_cnt;
    const uint32_t late   = d->late_cnt;
    const uint32_t wait   = d->wait_cnt;
    const uint32_t shortp = d->short_cnt;
    const uint32_t sbytes = d->short_bytes;
    const uint32_t open   = d->open_cnt;
    const uint32_t close  = d->close_cnt;
    const int32_t  late_max   = d->late_max_us;
    const int32_t  rx_gap_max = d->rx_gap_max_us;
    if (rx == prev_rx) {
        return;                                 /* 主机还没开始送，或者流已经关了：这个窗口没有包进来 */
    }
    /* 这个窗口里不到半秒的包：刚开始流，或者刚关流再开（切歌）。这时增量没有意义，打基线。
     * 流关着的时候 rx 不变，上面已经返回了 —— 否则关流期间每 2 秒都要重复打一遍基线。 */
    const bool begin = (rx - prev_rx) < 500;
    const uint32_t dzlp   = zlp - prev_zlp,     dskip = skip - prev_skip, dlate = late - prev_late;
    const uint32_t dwait  = wait - prev_wait,   dshort = shortp - prev_short;
    const uint32_t dsbyte = sbytes - prev_sbytes;
    const uint32_t dopen  = open - prev_open,   dclose = close - prev_close;
    prev_rx = rx; prev_zlp = zlp; prev_skip = skip; prev_late = late; prev_wait = wait;
    prev_short = shortp; prev_sbytes = sbytes; prev_open = open; prev_close = close;

    /* 这个窗口一共少了多少字节：零长、缺帧各按一整毫秒（192 字节）算，短包按实际少掉的算。
     *
     * 少包只在「比平时多少了 5ms 以上」时才报。电视盒子送的是 47.93k，每秒比标称少 70 个样本，
     * 短包（或者零长、缺帧，看它怎么发）每秒都有几十次 —— 那是盒子的常态，不是异常。
     * 按每次都报的写法，接盒子时每 2 秒要多打两行日志，TX 任务在串口上被堵几十毫秒，
     * 音频反而会被挤丢。「平时」取最近 8 个窗口的中位数；一次流的头 3 个窗口先不报少包。 */
    const uint32_t miss = (dzlp + dskip) * ALINK_TX_USB_FRAME_BYTES + dsbyte;
    static uint32_t miss_hist[8];
    static int miss_n = 0, miss_i = 0;
    uint32_t miss_med = 0;
    if (begin) {
        miss_n = miss_i = 0;         /* 新的一次流：重新学「平时」 */
    } else {
        if (miss_n >= 4) {
            uint32_t sorted[8];
            memcpy(sorted, miss_hist, sizeof(uint32_t) * (size_t)miss_n);
            for (int i = 1; i < miss_n; i++) {       /* 最多 8 个数，插入排序足够 */
                const uint32_t v = sorted[i];
                int j = i;
                while (j > 0 && sorted[j - 1] > v) {
                    sorted[j] = sorted[j - 1];
                    j--;
                }
                sorted[j] = v;
            }
            miss_med = (sorted[(miss_n - 1) / 2] + sorted[miss_n / 2]) / 2;
        }
        miss_hist[miss_i] = miss;
        miss_i = (miss_i + 1) % 8;
        if (miss_n < 8) {
            miss_n++;
        }
    }
    const bool miss_jump = (miss_n >= 4) && (miss > miss_med + 5 * ALINK_TX_USB_FRAME_BYTES);

    g_uac_diag.mark_log_us = esp_timer_get_time();
    if (begin) {
        ESP_LOGI(TAG, "USB 诊断：零长 %lu，短包 %lu（少 %lu 字节），缺帧 %lu，迟到 %lu（最长 %ld us），"
                      "收包最大间隔 %ld us；开流 %lu 次、关流 %lu 次",
                 (unsigned long)zlp, (unsigned long)shortp, (unsigned long)sbytes,
                 (unsigned long)skip, (unsigned long)late, (long)late_max, (long)rx_gap_max,
                 (unsigned long)open, (unsigned long)close);
    } else if (dlate || dwait || miss_jump) {
        ESP_LOGW(TAG, "USB 异常：零长 +%lu，短包 +%lu（少 %lu 字节），缺帧 +%lu，本窗口共少 %lu 字节、平时 %lu；"
                      "迟到 +%lu，没取走 +%lu；开流 +%lu，关流 +%lu；"
                      "零长 / 短包 / 缺帧发生时本机正在发包 %lu 次、正在打印 %lu 次（这两项是累计）；"
                      "收包最大间隔 %ld us",
                 (unsigned long)dzlp, (unsigned long)dshort, (unsigned long)dsbyte,
                 (unsigned long)dskip, (unsigned long)miss, (unsigned long)miss_med,
                 (unsigned long)dlate, (unsigned long)dwait,
                 (unsigned long)dopen, (unsigned long)dclose,
                 (unsigned long)d->hit_rf_cnt, (unsigned long)d->hit_log_cnt, (long)rx_gap_max);
        char buf[160];
        int n = 0;
        for (int k = 0; k < 8 && n < (int)sizeof(buf) - 20; k++) {
            const uint8_t i = (uint8_t)((d->evt_i + k) % 8);
            if (d->evt_kind[i] == 0) {
                continue;
            }
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, "%s%lu:%u", n ? " " : "",
                          (unsigned long)d->evt_ms[i], (unsigned)d->evt_kind[i]);
        }
        if (n > 0) {
            ESP_LOGW(TAG, "  最近异常（开机毫秒数:种类，1 零长 2 缺帧 3 迟到 4 开流 5 关流 6 短包）%s", buf);
        }
    }
    g_uac_diag.mark_log_end_us = esp_timer_get_time();
}
#endif  /* ALINK_TX_USE_UAC */

/* 断供补位的包络状态（见 tx_task 里的「补位」段）。
 * s_gap_gain 是 Q8 增益：256 = 满幅，0 = 完全静音。
 * s_last_* 是上一个有效样本，衰减就从它开始，而不是从 0 开始 —— 从 0 开始
 * 等于把波形硬拽到零，那正是原来那声「咔」。 */
static int32_t s_gap_gain = 256;
static int16_t s_last_l = 0, s_last_r = 0;
/* 上一次拼包没凑满（真断供）。用来把等待窗口从 30ms 退回 5ms，
 * 断供期间仍然按 200 包/秒发静音，接住接收端的 I2S 时钟。 */
static bool s_starving = false;

/* 断供态的绝对节拍（us）：下一个静音包的截止时刻，0 = 不在节拍上。
 *
 * 静音包必须平均严格 200 包/秒。每包就是 5ms 音频，接收端按 200 包/秒消费，
 * 发慢了接收端队列会被持续抽干，一两秒就欠载，然后反复「欠载 → 退回预缓冲」。
 * 「等 5ms 再发」这种相对计时做不到：每次等待都有向上取整的零头，累积起来就慢了。
 * 所以用绝对时刻排队，一拍拖长了下一拍自动补短。 */
static int64_t s_starve_grid = 0;

static void tx_task(void *arg)
{
    static uint8_t pkt[ALINK_HDR_SIZE + ALINK_PAYLOAD_BYTES];
    uint8_t *pcm = pkt + ALINK_HDR_SIZE;

    uint32_t sent = 0, dropped = 0, starve = 0, shortread = 0, misalign = 0;
    /* 发送失败的错误码。2026-11-25 上板时出现过一次：刷完固件后的头两分钟里
     * esp_now_send 连续报错 1209 次，ACK 却是 100%，接收端看到的是成批迟到 —— 空口没丢包，
     * 只是包在驱动里排队。固件只数了次数没记错误码，事后分不清是队列满还是别的原因。 */
    esp_err_t send_err   = ESP_OK;    /* 最近一次失败的错误码 */
    uint32_t drop_logged = 0;         /* 上一次报失败告警时的 dropped，用来限频 */
    int64_t last_send_us = 0;         /* 上一次 esp_now_send 的时刻，断供态追发时限速用 */
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
#if ALINK_TX_USE_UAC && ALINK_TX_CLOSE_CATCHUP
        const bool was_starving = s_starving;   /* 这一拍开始时是不是断供态（见发完之后的「追发」） */
#endif
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
        int64_t deadline;
        {
            const int64_t now = esp_timer_get_time();
            if (!s_starving) {
                deadline      = now + 30000;
#if ALINK_TX_USE_UAC
                if (!g_uac_spk_stream_on) {
                    deadline = now + ALINK_TX_CLOSE_WAIT_US;     /* 主机已经关流，见 ALINK_TX_CLOSE_WAIT_US */
                }
#endif
                s_starve_grid = 0;        /* 有数据的状态不在节拍上 */
            } else {
                /* 断供态按绝对节拍排队（见 s_starve_grid）。刚从 30ms 窗口切过来，
                 * 或者这一拍被拖得太久（超过 ALINK_TX_CATCHUP_PKTS 拍），才重新起拍；
                 * 落后的格点照常补发 —— 重新起拍等于少发静音包，接收端的缓冲会被抽浅。 */
                if (s_starve_grid == 0 ||
                    now - s_starve_grid >= ALINK_TX_CATCHUP_PKTS * ALINK_PKT_MS * 1000) {
                    s_starve_grid = now + ALINK_PKT_MS * 1000;
                }
                deadline      = s_starve_grid;
                s_starve_grid += ALINK_PKT_MS * 1000;
                /* 追发时两包之间至少隔 ALINK_TX_MIN_GAP_US：上一包 976 字节在 11Mbps 上
                 * 要占约 1ms 空口，贴着发会堆在驱动里（入队被拒就是 ESP_ERR_ESPNOW_NO_MEM）。 */
                if (deadline < last_send_us + ALINK_TX_MIN_GAP_US) {
                    deadline = last_send_us + ALINK_TX_MIN_GAP_US;
                }
            }
        }

        while (total < ALINK_PAYLOAD_BYTES) {
            const int64_t now = esp_timer_get_time();
#if ALINK_TX_USE_UAC
            /* 等的过程中主机把流关了（最多 2ms 后在这里发现）：同样不再等满 30ms */
            if (!s_starving && !g_uac_spk_stream_on && deadline > now + ALINK_TX_CLOSE_WAIT_US) {
                deadline = now + ALINK_TX_CLOSE_WAIT_US;
            }
#endif
            if (now >= deadline) {
                break;
            }
            /* 每次最多等 2 个 tick，但不超过距 deadline 的剩余时间（向上取整到 tick）。
             * 原来固定等 2 个 tick、回到循环顶部才看 deadline：5ms 的窗口落在第 3、4 次
             * 等待中间，一拍实际 3~8ms，静音包只有 165 包/秒左右。 */
            const int64_t rem_us    = deadline - now;
            const int64_t ticks_max = (int64_t)pdMS_TO_TICKS(2);
            int64_t ticks = (rem_us * (int64_t)configTICK_RATE_HZ + 999999) / 1000000;
            if (ticks > ticks_max) {
                ticks = ticks_max;
            }
            if (ticks < 1) {
                ticks = 1;
            }
            size_t n = xStreamBufferReceive(s_pcm_sb, pcm + total,
                                            ALINK_PAYLOAD_BYTES - total,
                                            (TickType_t)ticks);
            total += n;
        }
        s_starving = (total < ALINK_PAYLOAD_BYTES);

        /* 帧对齐自检。UAC 组件回调给出的长度应当恒为整帧；不是整帧说明组件侧的
         * 水位线又没对齐（见 usb_device_uac.c 的 level_set）。这里记一笔让它在日志里
         * 现形，同时把不足一帧的尾巴并入第一帧补位，保证 240 帧全部写满 ——
         * 原来按 total/4 截断，这种包的最后一帧会残留上一包的旧数据。 */
        const int nfr = (int)((total + ALINK_FRAME_BYTES - 1) / ALINK_FRAME_BYTES);
        if (total % ALINK_FRAME_BYTES != 0) {
            misalign++;
            if ((misalign % 50) == 1) {
                ESP_LOGW(TAG, "半包长度 %u 不是整帧（累计 %lu 次），检查 UAC 组件的 level_set",
                         (unsigned)total, (unsigned long)misalign);
            }
        }
        const int nf_missing = ALINK_FRAMES_PER_PKT - nfr;
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
         * 改成沿「最后一个有效样本」做包络衰减：每帧减 1（Q8 步长 1），从满幅
         * 滑到 0 要 256 帧 ≈ 5.3ms；恢复供数时每帧加 2，128 帧 ≈ 2.7ms 淡入。
         * 这么短的淡入淡出听不出来，却足以把阶跃变成连续曲线。 */
        {
            int16_t *s16 = (int16_t *)pcm;
            const int nf_pkt = ALINK_FRAMES_PER_PKT;
            if (nf_missing > 0) {
                int16_t *tail = s16 + nfr * 2;
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
#if ALINK_TX_USE_UAC
        g_uac_diag.mark_rf_us = esp_timer_get_time();     /* USB 诊断用：记住发包时刻 */
#endif
        /* 单播 + 硬件 ACK。发送失败（ACK 超时 / 队列满）只计数，不重发：
         * 重发会把这一包的发送时刻推后，反而制造抖动，交给接收端 PLC 补更划算。
         *
         * esp_now_send 只负责把包交给驱动入队，入队被拒几乎都是驱动队列满
         * （ESP_ERR_ESPNOW_NO_MEM，IDF 文档的原话是「过一会儿再发」）。它成串出现，
         * 说明空口或对端有一段时间收不过来，包成批压在驱动里、再成批到达。 */
        s_lat_call_us[s_lat_tail % ALINK_TX_LAT_SLOTS] = esp_timer_get_time();
        s_lat_tail++;
        const esp_err_t se = esp_now_send(s_peer, pkt, pkt_len);
        if (se != ESP_OK) {
            s_lat_tail--;             /* 没入队就不会有回调，撤回这一条 */
            dropped++;
            send_err = se;
            if (dropped - drop_logged >= 100) {
                drop_logged = dropped;
                ESP_LOGW(TAG, "esp_now_send 失败 %s（累计 %lu 次）",
                         esp_err_to_name(se), (unsigned long)dropped);
            }
        } else {
            sent++;
        }

        int64_t now = esp_timer_get_time();
        last_send_us = now;
        int64_t gap = now - t_prev;
#if ALINK_TX_USE_UAC && ALINK_TX_CLOSE_CATCHUP
        /* 主机关流造成的半包：接着原来的 5ms 节拍发补位包，落后的格点照常补上。
         *
         * 不补的话，「关流 → 等完 ALINK_TX_CLOSE_WAIT_US → 半包发出」这段空档会一直留在
         * 接收端的队列里：上一版上板量到每次关流掉 2~3 包，连着切歌几次还是见底。
         * 补上之后这段空档由紧跟着的补位包填回来。节拍锚在上一包之后第 2 拍 ——
         * 半包自己占第 1 拍，它晚发了多少，下一轮循环就按 ALINK_TX_CATCHUP_PKTS 的规矩追发。
         *
         * 只有「关流」造成的半包才补。流还开着、只是 30ms 等不到数据的那种半包不补：
         * 那种情况数据多半只是晚了，随后会连着涌到，再补静音就是凭空插进去一段停顿。 */
        if (s_starving && !was_starving && !g_uac_spk_stream_on) {
            s_starve_grid = t_prev + 2 * ALINK_PKT_MS * 1000;
        }
#endif
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
            const int64_t lat_max = s_lat_max_us;
            const uint32_t lat20 = s_lat_over20, lat50 = s_lat_over50;
            s_lat_max_us = 0;
            s_lat_over20 = 0;
            s_lat_over50 = 0;
            t_log = now2;
            sent_at_log = sent;
            uac_at_log = s_uac_bytes;
            /* %lld 在 ESP_LOG 的 vsnprintf 路径上不可靠（实测会打出失控数据），
             * 全部改 %ld + (long) 强转；32 位平台上 int64 截断不影响这个量级。 */
#if ALINK_TX_USE_UAC
            g_uac_diag.mark_log_us = esp_timer_get_time(); /* USB 诊断用：统计行本身可能阻塞串口 */
            ESP_LOGI(TAG, "发=%lu(%lu/s) UAC=%lu B/s ACK=%lu/%lu(%.1f%%) 失败=%lu 欠载=%lu 半包=%lu UAC回调=%lu UAC丢=%lu 间隔 min/avg/max=%ld/%ld/%ld us 空口延迟max=%ld us(>20ms %lu,>50ms %lu) 最近发送错误=%s",
                     (unsigned long)sent, (unsigned long)pps, (unsigned long)bps,
                     (unsigned long)aok, (unsigned long)(aok + afail),
                     (double)ack_pct_x100 / 100.0,
                     (unsigned long)dropped,
                     (unsigned long)starve, (unsigned long)shortread,
                     (unsigned long)s_uac_cb, (unsigned long)s_uac_drop,
                     (long)gap_min, (long)(gap_n > 1 ? gap_sum / (int64_t)(gap_n - 1) : 0), (long)gap_max,
                     (long)lat_max, (unsigned long)lat20, (unsigned long)lat50, esp_err_to_name(send_err));
            g_uac_diag.mark_log_end_us = esp_timer_get_time();
#else
            ESP_LOGI(TAG, "发=%lu(%lu/s) ACK=%lu/%lu(%.1f%%) 失败=%lu 欠载=%lu 半包=%lu 间隔 min/avg/max=%ld/%ld/%ld us 空口延迟max=%ld us(>20ms %lu,>50ms %lu) 最近发送错误=%s",
                     (unsigned long)sent, (unsigned long)pps,
                     (unsigned long)aok, (unsigned long)(aok + afail),
                     (double)ack_pct_x100 / 100.0,
                     (unsigned long)dropped,
                     (unsigned long)starve, (unsigned long)shortread,
                     (long)gap_min, (long)(gap_n > 1 ? gap_sum / (int64_t)(gap_n - 1) : 0), (long)gap_max,
                     (long)lat_max, (unsigned long)lat20, (unsigned long)lat50, esp_err_to_name(send_err));
#endif
            gap_min = 1 << 30; gap_max = 0; gap_sum = 0; gap_n = 0;
#if ALINK_TX_USE_UAC
            usb_diag_report();
#endif
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
     * USB 任务（TinyUSB 和 usb_spk_task）在 sdkconfig.defaults 里显式绑到了同一个 core1
     * （CONFIG_UAC_TINYUSB_TASK_CORE / CONFIG_UAC_SPK_TASK_CORE），core0 只留 WiFi。
     * 别改回 -1（无亲和性）：IDF 的 SMP 调度对无亲和性任务不限定核，哪个核先调度到
     * 就跑在哪，通常跟着唤醒它的中断走，结果就是回到和 WiFi 抢 CPU 的状态。
     * usb_spk_task 在回调里读的 spk_buf 和 USB 中断共用，被 WiFi 抢占超过 1ms，
     * 下一次中断就会覆盖缓冲或清掉长度，丢 1ms 音频。
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
