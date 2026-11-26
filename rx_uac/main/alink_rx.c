/*
 * audio-link 接收端（ESP32-S3 版）
 *
 * 从 C3 版移植。音频算法（抖动缓冲 / PLC / 连续分数重采样 / dither / 电平校准）
 * 一行没改，只换平台 —— 目的是做单变量对照：如果换到 S3 噼啪就没了，
 * 那 C3 那片板子（或 C3 的 I2S 实现）就是嫌疑源。
 *
 * 平台侧只动三处，都是 S3 相对 C3 的硬差异：
 *   1. 引脚 GPIO10/11/12（C3 是 5/7/10），且不再需要把 DAC 的 SCK 引脚钉低
 *   2. 播放任务独占 core1 —— C3 是单核，播放和 WiFi 必然抢同一个 CPU；
 *      S3 双核，播放任务的 DMA 喂养不会被 WiFi 收发尖峰打断
 *   3. 主频 240MHz（C3 是 160MHz），三次插值 + 抖动运算的余量更宽
 *
 * **I2S 时钟不是差异点**：S3 的 I2S 硬件上没有 APLL 通道（那是 ESP32 / S2 / P4
 * 才有的），时钟源只有 XTAL / PLL_160M / PLL_240M / EXTERNAL。这里保持和 C3
 * 一样的 PLL_160M：160/3.072 = 52+1/12，分母 12 在 63 以内，小数分频能精确
 * 表达，不产生 pattern jitter。所以「C3 时钟不干净」这个假设已经被推翻，
 * 本次对照只剩双核/主频/板子本身三个变量。
 *
 * 想严格复刻 C3 的单核行为做反向对照：把 app_main 里 play 任务的
 * xTaskCreatePinnedToCore 换回 xTaskCreate 即可。
 *
 * ---- 以下是 C3 版的原始说明。传输层后来从 UDP 换成了 ESP-NOW（不再连热点、
 *      不发 HELLO），下面四条播放侧的对策一直沿用 ----
 * 收音频包进抖动缓冲 → I2S 输出到 PCM5102A。
 *
 * 2026-11-18 重写播放侧，解决实测的「连接不稳 + 爆音」。四个根因：
 *
 *   1. 时钟漂移：收发两端晶振差 20~50ppm，队列必然缓慢溢出或见底。
 *      → 水位闭环：EMA 跟队列深度。最初是「偏了就丢一包 / 补一包静音」，
 *        之后两次升级：先是每包插删 1 帧，最终改成连续分数重采样（见 src_process）。
 *
 *   2. 抽空后 DMA 循环播旧数据：auto_clear_after_cb 是 false，DMA 会一直重复
 *      最后一包内容，听感是「哒哒哒」。
 *      → I2S 时钟永不断流，没数据就写静音包；DMA 深度保持在 15~20ms。
 *
 *   3. 丢包直接跳过 5ms：波形阶跃，每丢一包一声「咔」。
 *      → 按 seq 空洞补占位包，时序不错位。
 *
 *   4. 占位包如果填静音，依然是阶跃，还是「咔」。
 *      → PLC：重复上一包输出并逐次衰减（×1.0/0.6/0.35/0.2，第 5 次归零），
 *        相位不连续由 emit() 的 2ms 交叉淡化抹平。
 *
 * TX 每包只发一份，没有冗余副本，PLC 是丢包的唯一兜底。
 * seq 不比上一包新的包（迟到重到的旧包）只做去重。
 *
 * 所有断点（起播 / 丢包 / 补包 / 断流）统一走 2ms 交叉淡化，不再有阶跃。
 */
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "freertos/ringbuf.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "alink_common.h"

static const char *TAG = "alink_rx";

/* 引脚：S3 板（原来那台发射端，排针引出 GPIO9~14）
 *
 *   GPIO10 = BCK / GPIO11 = LCK / GPIO12 = DIN —— 取连续三个，插杜邦线方便。
 *   S3 上 GPIO9~14 全是普通 IO，避开 strapping(0/3/45/46) 与 USB/JTAG(19/20/39~42)，
 *   上电没有特殊电平要求，这六个里挑哪三个都一样。
 *
 * 本版仍不输出 MCLK，PCM5102A 走内部 PLL 从 BCK 恢复时钟 → DAC 的 SCK 必须接 GND。
 * C3 那块板子上 GPIO6 恰好焊到了 DAC 的 SCK，所以在软件里把它钉成输出低来等效接地；
 * S3 这台没有那根线，SCK 直接飞 GND 就行，代码里不再需要 PIN_SCK_GND。 */
#define PIN_BCLK   GPIO_NUM_10
#define PIN_LRCLK  GPIO_NUM_11
#define PIN_DOUT   GPIO_NUM_12
/* 仅在 ALINK_I2S_USE_MCLK=1 时使用，接 DAC 的 SCK（替代接地） */
#define PIN_MCLK   GPIO_NUM_13

/* 播放侧采样率 48kHz。时钟源由下面的 ALINK_I2S_CLK_240M 决定，当前取 PLL_240M。
 *
 * 这里要澄清一个曾经的误判：从 C3 移植时一度怀疑 C3 因为没有 APLL 而产生
 * pattern jitter。实际算一遍：160e6 / (48000×64) = 52 + 1/12，小数分母 12 在
 * 分频器上限 63 以内，能精确表达，不产生周期性补偿。S3 的 I2S 硬件上同样没有
 * APLL 通道（只有 XTAL / PLL_160M / PLL_240M / EXTERNAL），所以这一项两者等价，
 * **不是噼啪的差异来源**。
 *
 * 与盒子的残余失配（盒子约 47.93k vs 本地 48k，约 0.15%）分两级消化：
 * I2S 时钟跟随音源做粗调（见 rx_task 的测速窗口），水位闭环的连续分数重采样做细调。
 * 复核方法：看 RX 日志里「欠载」「漂移丢」「重采样」三个数是否还在缓慢增长。 */
#ifndef ALINK_I2S_RATE
#define ALINK_I2S_RATE 48000
#endif

/* MCLK 输出开关（默认关 = 与 C3 条件完全一致）。
 *
 * 关：DAC 的 SCK 接 GND，PCM5102A 用内部 PLL 从 BCK(3.072MHz) 恢复时钟。
 *
 * 开：从 PIN_MCLK 输出 256×Fs = 12.288MHz 给 DAC 的 SCK。PLL 参考频率提高
 * 4 倍，同样的绝对时间抖动折算成相位误差就小 4 倍。用法是把 DAC 的 SCK
 * 从 GND 改接到 GPIO13。
 *
 * 这是第二步实验，**别和第一步（纯换芯片）混在一起做** —— 第一步必须保持
 * 条件与 C3 完全一致，结论才干净。 */
#ifndef ALINK_I2S_USE_MCLK
#define ALINK_I2S_USE_MCLK 0
#endif

/* I2S 主时钟源
 *
 * 1 = PLL_240M：12.288MHz 得到 19 + 34/64 的整数分频，BCK 零周期抖动
 * 0 = PLL_160M：移植时的初始值（为了和 C3 条件一致），13 + 1/64，每 64 个周期
 *     插一个补偿周期 = 192kHz 抖动
 *
 * 48000×256 = 12.288MHz，而 I2S 小节分频器分母上限是 64。
 * 160/12.288 = 13.0208333（小数 1.3333/64，除不尽）
 * 240/12.288 = 19.53125  （小数 34/64，精确）
 * C3 最高只有 160MHz PLL，所以这一项是 S3 独有的。
 *
 * 对照方法：设 0 重烧，条件与 C3 完全一致。 */
#ifndef ALINK_I2S_CLK_240M
#define ALINK_I2S_CLK_240M 1
#endif

/* 固定输出衰减（dB）。0 = 不衰减（对照用），12 = 对齐现网那版的校准值。 */
#ifndef ALINK_OUT_ATTEN_DB
#define ALINK_OUT_ATTEN_DB 12
#endif

/* 引脚扫描模式（临时诊断用，正常固件保持 0）
 *
 * 换到 S3 之后出现过「日志全好、电平 -24dBFS、I2S 无错，但完全没声音」。
 * 软件侧已经排干净（channel 已 enable、write 一直成功、欠载计数冻结），
 * 剩下的唯一变量是「DAC 那三根线到底落在哪三个 GPIO 上」——
 * 板子丝印的编号不一定等于芯片的 GPIO 号，自己焊的针也可能没连到以为的脚。
 *
 * 打开后不联网、不收包，直接把同一段 1kHz 正弦轮流送到 8 组候选引脚，
 * 每组 4 秒、组间 0.4 秒静音。哪一组出声音，就说明 DAC 接在那组上。
 * 定位完把这里改回 0，重新烧正常固件。 */
#ifndef ALINK_PIN_SCAN
#define ALINK_PIN_SCAN 0
#endif

/* 启动时扫一遍 2.4G（临时诊断用，正常固件保持 0）
 *
 * 2026-11-25 上板实测：链路偶尔出现几秒的「驱动发不出去 → 包成批迟到」——
 * TX 侧 esp_now_send 报 ESP_ERR_ESPNOW_NO_MEM，RX 侧队列先被抽干、再被突发推到满。
 * 空口上没丢包（ACK 100%），只是帧在等空口，怀疑信道 11 上有别的 WiFi 在占。
 *
 * 打开后接收端启动时先把周围的 AP 列一遍（信道、信号强度、SSID），
 * 照着挑一个没人用的信道，改 ALINK_WIFI_CHANNEL。
 * 扫描期间链路不通（约 3 秒），扫完照常工作。看完改回 0 重烧。 */
#ifndef ALINK_WIFI_SCAN
#define ALINK_WIFI_SCAN 0
#endif

/* 启动时量一遍每个信道的实际占用（临时诊断用，正常固件保持 0）
 *
 * ALINK_WIFI_SCAN 只能看出「哪个信道上有 AP、信号多强」，看不出「谁在传数据」。
 * 而让链路卡顿的是流量：一个信号很强的 AP 如果没有客户端在用，只发信标，不碍事；
 * 一个信号一般的 AP 只要在给手机传视频，就足够把相邻信道占满。
 *
 * 打开后接收端启动时依次切到 1~13 信道，各听 ALINK_WIFI_SURVEY_MS 毫秒，
 * 统计帧数、估计占用的空口时间（帧长 ÷ 调制速率，含信标）、信号最强的帧，
 * 以及占用时间最多的 3 个发射端（MAC 可以和 ALINK_WIFI_SCAN 的 AP 列表对上）。
 * 配了发送方白名单（menuconfig 的「发射端 MAC 地址」）时，本链路自己的帧不算在里面；
 * 没配的话，TX 所在信道的头一名就是它自己（2026-11-25 实测：3 秒里 631 帧、约 190‰），
 * 对照着扣掉即可。
 *
 * 13 个信道共约 40 秒，期间链路不通。它是启动时的一张快照，邻居不忙时看到的都是安静；
 * 想抓现行，挑听到卡顿的时候让 RX 单独复位重跑一遍（TX 不用动）。看完改回 0 重烧。 */
#ifndef ALINK_WIFI_SURVEY
#define ALINK_WIFI_SURVEY 0
#endif
#ifndef ALINK_WIFI_SURVEY_MS
#define ALINK_WIFI_SURVEY_MS 3000
#endif

/* DMA 深度直接决定「用户看不见的额外延迟」：i2s_channel_write 只要 DMA 有空位就
 * 立刻返回，所以队列里的包会被迅速灌进 DMA，队列深度本身不代表真实缓冲。
 * 延时 = dma_desc_num × dma_frame_num / fs（dma_frame_num = 240 帧 @48k = 5ms）：
 *   稳定档 4 × 5ms = 20ms，够吸收 WiFi 抖动，又不至于把延迟撑到蓝牙的水平；
 *   低延时档 3 × 5ms = 15ms。
 *   减到 2 会只剩 10ms，play_task 被抢占 8ms 就硬件空转，不划算。 */
#if ALINK_LOW_LATENCY
#define I2S_DMA_DESC   3
#else
#define I2S_DMA_DESC   4
#endif
#define I2S_DMA_FRAMES ALINK_FRAMES_PER_PKT

static i2s_chan_handle_t s_i2s_tx;

/* ESP-NOW 回调 → rx_task 的转运环。回调里只做 memcpy，序号处理全在任务里。 */
static RingbufHandle_t s_rb;
static volatile int s_peer_rssi = 0;     /* 最近一包的信噪，仅用于日志 */

/* 发送方白名单：menuconfig 里配了发射端 MAC（CONFIG_ALINK_TX_MAC）才启用，
 * 留空时 s_filter_tx 恒为 false，行为和不设白名单时完全一致。
 * ESP-NOW 不加密，同信道上任何设备都能往这块板的地址发帧；有了它，别的设备
 * 发来的帧进不了环形缓冲。解析规则见 alink_common.h 的 alink_mac_from_str。 */
#ifndef CONFIG_ALINK_TX_MAC
#define CONFIG_ALINK_TX_MAC ""      /* 还没跑过 menuconfig 的 sdkconfig：不校验发送方 */
#endif
static uint8_t s_peer_tx[ALINK_ETH_ALEN];
static bool    s_filter_tx = false;

/* ---------- 抖动缓冲：单生产者（收包）单消费者（播放） ---------- */

typedef struct {
    uint8_t  data[ALINK_Q_DEPTH][ALINK_PAYLOAD_BYTES];
    uint8_t  fill[ALINK_Q_DEPTH];      /* 1 = 丢包补的静音占位 */
    volatile int head;
    volatile int tail;
    volatile int count;
    portMUX_TYPE mux;
} alink_q_t;

static alink_q_t s_q;

/* 漂移控制/欠载的累计计数。play_task 里只累加，由 rx_task 的秒级日志统一打印 ——
 * 原来在 play_task 里每条都 ESP_LOGW，水位高时一秒能刷 200 条，
 * 115200 波特下光打日志就要 2 秒以上，直接把播放任务堵死，越丢越卡。 */
static volatile uint32_t s_drift_drop, s_underrun;

/* 注意：这两个函数里 960 字节的 memcpy 故意放在临界区之外。
 * C3 是单核，taskENTER_CRITICAL 会关中断几十微秒，而 WiFi 侧此刻正在收包，
 * 关久了就是驱动层丢包 —— 这是「修好了上层还在碎」的隐形来源。
 * 队列是单生产者（rx_task）/单消费者（play_task），head 只有生产者碰、tail 只有消费者碰，
 * 释放槽位时把索引和计数一起原子更新即可，数据搬运不需要保护。 */
static bool q_push(const uint8_t *pcm, bool is_fill)
{
    int i = s_q.head;
    if (s_q.count >= ALINK_Q_DEPTH) {
        return false;       /* 满：直接丢，不覆盖 tail 指向的最旧包 */
    }

    if (pcm) {
        memcpy(s_q.data[i], pcm, ALINK_PAYLOAD_BYTES);
    } else {
        memset(s_q.data[i], 0, ALINK_PAYLOAD_BYTES);
    }
    s_q.fill[i] = is_fill ? 1 : 0;

    bool ok = false;
    taskENTER_CRITICAL(&s_q.mux);
    s_q.head = (i + 1) % ALINK_Q_DEPTH;
    s_q.count++;
    ok = true;
    taskEXIT_CRITICAL(&s_q.mux);
    return ok;
}

static int q_depth(void)
{
    taskENTER_CRITICAL(&s_q.mux);
    int c = s_q.count;
    taskEXIT_CRITICAL(&s_q.mux);
    return c;
}

/* 取队首那包的前 nframes 帧，但不消费 —— 给重采样器做跨包前瞻。
 *
 * 32 抽头的 sinc 要取 i+16 的样本，包尾之后没有真实数据。原来用「复制最后
 * 一帧」顶上，于是每包最后 16 个输出样本（占 6.7%）是拿假数据算出来的，
 * 每 5ms 重复一次 = 200Hz 的周期性失真。片上自测实测这一步就是 -30dB 残差，
 * 而直通（不插值）时是 -96.9dB。
 *
 * 队列里本来就躺着下一包，读它的前 16 帧就够了。只有欠载时才回退到镜像。 */
static int q_peek(int16_t *dst, int nframes)
{
    int got = 0;
    taskENTER_CRITICAL(&s_q.mux);
    if (s_q.count > 0) {
        memcpy(dst, s_q.data[s_q.tail], (size_t)nframes * ALINK_CHANNELS * sizeof(int16_t));
        got = nframes;
    }
    taskEXIT_CRITICAL(&s_q.mux);
    return got;
}

static bool q_pop(uint8_t *out, bool *is_fill)
{
    int i;
    bool ok = false;
    taskENTER_CRITICAL(&s_q.mux);
    if (s_q.count > 0) {
        i = s_q.tail;
        if (is_fill) {
            *is_fill = s_q.fill[i] != 0;
        }
        s_q.tail = (i + 1) % ALINK_Q_DEPTH;
        s_q.count--;
        ok = true;
    }
    taskEXIT_CRITICAL(&s_q.mux);

    if (ok) {
        /* 生产者要绕满 ALINK_Q_DEPTH 个槽才会回到 i，这里 memcpy 早已结束，不会踩 */
        memcpy(out, s_q.data[i], ALINK_PAYLOAD_BYTES);
    }
    return ok;
}

/* 漂移控制用：丢掉最旧的一包 */
static bool q_drop(void)
{
    bool ok = false;
    taskENTER_CRITICAL(&s_q.mux);
    if (s_q.count > 0) {
        s_q.tail = (s_q.tail + 1) % ALINK_Q_DEPTH;
        s_q.count--;
        ok = true;
    }
    taskEXIT_CRITICAL(&s_q.mux);
    return ok;
}

/* ---------- I2S ---------- */

/* DMA 空转核对：on_sent 每发完一个 DMA 缓冲触发一次，正常连续播放时
 * 「on_sent 次数 × 240 帧」应该严格等于应用层写进去的帧数。
 * 这个乘积偏大，就说明硬件在没有数据时又播了一遍旧块 ——
 * `i2s_channel_write` 对此返回 ESP_OK，应用层所有计数器都不动，
 * 但耳朵听得见，正是「所有指标全绿却仍有随机爆音」的那一类。
 *
 * 回调在 ISR 上下文，只允许一次自增。IRAM_ATTR 让它在 flash 操作期间
 * 也能安全执行（CONFIG_I2S_ISR_IRAM_SAFE 打开时必须）。 */
static volatile uint32_t s_dma_done = 0;

static bool IRAM_ATTR i2s_on_sent_cb(i2s_chan_handle_t h,
                                     i2s_event_data_t *ev, void *ctx)
{
    (void)h;
    (void)ev;
    (void)ctx;
    s_dma_done++;
    return false;
}

static void i2s_init_pins(int pin_bclk, int pin_ws, int pin_dout)
{
    /* 不再需要把 SCK 钉低：S3 这台没有 GPIO→DAC SCK 的连线，
     * DAC 的 SCK 直接飞 GND 即可（硬件上做，软件不管）。 */

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = I2S_DMA_DESC;
    chan_cfg.dma_frame_num = I2S_DMA_FRAMES;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_i2s_tx, NULL));

    i2s_std_config_t std_cfg = {
        /* 时钟源 —— 这一项直接决定 BCK 有没有周期性抖动。
         *
         * 目标 sclk = 48000 × 256 = 12.288MHz。I2S 外设的小数分频器分母最大 64，
         * 所以只有能写成 n + k/64 的组合才是精确的：
         *
         *   PLL_160M：160/12.288 = 13 + 1.3333/64
         *             只能取 13 + 1/64，每 64 个周期要插一个补偿周期，
         *             BCK 上就是 192kHz 的周期抖动。
         *   PLL_240M：240/12.288 = 19 + 34/64   ← 整数，零误差
         *
         * PCM5102A 的 SCK 接地，内部 PLL 以 BCK 为参考恢复系统时钟，
         * 上游 BCK 一抖，它的插值时刻就跟着抖 —— 在 8k~15kHz 这种
         * 由内部滤波器主导的频段最明显。开源同类工程（hearlink）记录过
         * 同一现象，原话是「正是这种不均匀的时钟让 PCM5102 变得沙哑」。
         *
         * C3 只有 160M，没有这一项可选；这是 S3 相对 C3 唯一真实的时钟优势。
         * 想退回旧行为做对照：把 ALINK_I2S_CLK_240M 设 0。 */
#if ALINK_I2S_CLK_240M
        .clk_cfg  = {
            .sample_rate_hz  = ALINK_I2S_RATE,
            .clk_src         = I2S_CLK_SRC_PLL_240M,
            .ext_clk_freq_hz = 0,
            .mclk_multiple   = I2S_MCLK_MULTIPLE_256,
            .bclk_div        = 8,
        },
#else
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(ALINK_I2S_RATE),
#endif
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
#if ALINK_I2S_USE_MCLK
            .mclk = PIN_MCLK,            /* 12.288MHz 给 DAC 当 PLL 参考 */
#else
            .mclk = I2S_GPIO_UNUSED,     /* PCM5102A 用内部 PLL 从 BCK 恢复 */
#endif
            .bclk = (gpio_num_t)pin_bclk,
            .ws   = (gpio_num_t)pin_ws,
            .dout = (gpio_num_t)pin_dout,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { false, false, false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_tx, &std_cfg));

    /* 注册「数据已发完」回调，只做计数。必须在 enable 之前注册
     * （通道要处于 REGISTERED / READY 状态）。
     * 用途见 s_dma_done 的注释：核对硬件有没有在缺数据时偷偷重播旧块。 */
    i2s_event_callbacks_t cbs = { 0 };
    cbs.on_sent = i2s_on_sent_cb;
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(s_i2s_tx, &cbs, NULL));

    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_tx));
    ESP_LOGI(TAG, "I2S 就绪 BCK=%d LCK=%d DIN=%d 播放率=%dHz 时钟源=%s MCLK=%s (DMA %d×%d 帧 = %dms)",
             pin_bclk, pin_ws, pin_dout, ALINK_I2S_RATE,
#if ALINK_I2S_CLK_240M
             "PLL_240M(整数分频)",
#else
             "PLL_160M(小数分频)",
#endif
#if ALINK_I2S_USE_MCLK
             "输出(12.288M)",
#else
             "关(SCK 接 GND)",
#endif
             I2S_DMA_DESC, I2S_DMA_FRAMES,
             I2S_DMA_DESC * I2S_DMA_FRAMES * 1000 / ALINK_I2S_RATE);
}

static void i2s_init(void)
{
    i2s_init_pins(PIN_BCLK, PIN_LRCLK, PIN_DOUT);
}

#if ALINK_PIN_SCAN
/* 引脚扫描：不联网、不收包，直接把 1kHz 正弦轮流送到候选引脚组。
 * 每组 4 秒、组间 0.4 秒静音做分隔，日志打印组号 —— 听第几组响就能定位。 */
static void pin_scan_run(void)
{
    static const int groups[][3] = {
        { 4,  5,  6},   /* 1  README 里最早的设计 */
        { 5,  7, 10},   /* 2  C3 那套 */
        { 9, 10, 11},   /* 3 */
        {10, 11, 12},   /* 4  当前固件的设定 */
        {11, 12, 13},   /* 5 */
        {12, 13, 14},   /* 6 */
        {15, 16, 17},   /* 7 */
        {16, 17, 18},   /* 8 */
    };
    const int ngroups = (int)(sizeof(groups) / sizeof(groups[0]));
    const int nframes = ALINK_FRAMES_PER_PKT;      /* 240 帧 = 5ms */
    static int16_t buf[ALINK_FRAMES_PER_PKT * 2];
    float ph = 0.0f;

    /* 开机先等 8 秒再出声：烧录会复位，等串口日志抓起来、人也就位 */
    ESP_LOGW(TAG, "引脚扫描模式，8 秒后开始，共 8 组 × 4 秒，跑两轮");
    vTaskDelay(pdMS_TO_TICKS(8000));

    for (int round = 0; round < 2; round++) {
        if (round) {
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        for (int g = 0; g < ngroups; g++) {
            i2s_init_pins(groups[g][0], groups[g][1], groups[g][2]);
            ESP_LOGW(TAG, "[轮%d] 组 %d/%d  BCK=GPIO%d WS=GPIO%d DIN=GPIO%d  响 %d 声",
                     round + 1, g + 1, ngroups, groups[g][0], groups[g][1], groups[g][2], g + 1);

            /* 每组播「组号」声短音：0.25 秒响 + 0.25 秒停。
             * 用耳朵数就行，第 4 组就是「嘀嘀嘀嘀」四声，不用猜边界。 */
            for (int b = 0; b <= g; b++) {
                for (int done = 0; done < ALINK_I2S_RATE / 4; done += nframes) {
                    for (int i = 0; i < nframes; i++) {
                        float v = 0.7f * sinf(ph);
                        ph += 2.0f * (float)M_PI * 1000.0f / (float)ALINK_I2S_RATE;
                        if (ph > 2.0f * (float)M_PI) ph -= 2.0f * (float)M_PI;
                        int16_t s = (int16_t)(v * 32767.0f);
                        buf[i * 2]     = s;
                        buf[i * 2 + 1] = s;
                    }
                    size_t written = 0;
                    i2s_channel_write(s_i2s_tx, buf, sizeof(buf), &written, portMAX_DELAY);
                }
                memset(buf, 0, sizeof(buf));
                for (int done = 0; done < ALINK_I2S_RATE / 4; done += nframes) {
                    size_t written = 0;
                    i2s_channel_write(s_i2s_tx, buf, sizeof(buf), &written, portMAX_DELAY);
                }
            }

            /* 组间静音 1 秒，把两组分开 */
            memset(buf, 0, sizeof(buf));
            for (int done = 0; done < ALINK_I2S_RATE; done += nframes) {
                size_t written = 0;
                i2s_channel_write(s_i2s_tx, buf, sizeof(buf), &written, portMAX_DELAY);
            }

            i2s_channel_disable(s_i2s_tx);
            i2s_del_channel(s_i2s_tx);
            s_i2s_tx = NULL;
        }
    }

    ESP_LOGW(TAG, "扫描结束。哪一组响就记下组号，再把 ALINK_PIN_SCAN 改回 0 重烧正常固件。");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

/* ---------- 输出：所有不连续点都在这里抹平 ---------- */

/* 「从静音醒来」的判定长度（包）：连续这么多包整包低于 16 LSB 才算数字静音。
 * 3 包 = 15ms。src_process 的相位吸附也以它为准（见 emit），两处必须用同一个数：
 * 吸附之后紧跟的有声包，一定要落在 15ms 淡入里。 */
#define ALINK_WAKE_QUIET_PKTS 3

static int16_t s_tail[ALINK_XFADE_FRAMES][2];   /* 上一次输出的最后 2ms */
static bool    s_xfade;                          /* 下次输出需要淡入（2ms 接缝淡化） */
static int     s_fade_in_left;                   /* 长淡入剩余帧数（断流恢复/静音恢复/时钟重配） */
static int     s_quiet_pkts;                     /* 连续「整包接近全零」的包数，判「从静音醒来」 */
static uint32_t s_wake_fade;                     /* 静音恢复触发次数，供日志 */
static int16_t s_prev[ALINK_FRAMES_PER_PKT * ALINK_CHANNELS];  /* 上一整包，PLC 外推基准 */
static uint8_t s_plc_run;                        /* 连续 PLC 次数 */
static uint32_t s_plc_total;                     /* 累计 PLC 次数，供日志 */

/* TPDF 抖动，幅度约 ±1 LSB。
 *
 * 为什么必须有：数字衰减（乘 Q16 再右移）天然会把低位截掉，截断误差是
 * **与信号相关**的确定性失真，低电平时听起来就是"沙沙/轻微噼啪"，
 * 而静音段全是 0，所以只在有声时出现 —— 正是实听描述的现象。
 * 加抖动后这份误差变成听不见的白噪声（-93dBFS 量级）。
 *
 * 顺手解决的第二个问题：PCM5102A 在 DIN 连续为 0 时会触发内部 auto-mute，
 * 声音一起来再 unmute，那个过渡就是"随声音密度出现的轻咔"。±1 LSB 的
 * 抖动让它永远看不到全零输入，芯片就不会来回 mute。
 * xorshift32 比 rand() 快得多，也不占栈，两次相减得到三角分布。 */
static uint32_t s_dither_s = 0x9E3779B9u;

static inline int32_t dither_tpdf(void)
{
    s_dither_s ^= s_dither_s << 13;
    s_dither_s ^= s_dither_s >> 17;
    s_dither_s ^= s_dither_s << 5;
    int32_t a = (int32_t)(s_dither_s & 1u);

    s_dither_s ^= s_dither_s << 13;
    s_dither_s ^= s_dither_s >> 17;
    s_dither_s ^= s_dither_s << 5;
    int32_t b = (int32_t)(s_dither_s & 1u);

    /* TPDF（两次各取 {0,1} 相减，峰峰 2 LSB）+ 恒定 +1 LSB 偏置。
     *
     * 之前写成 (s & 0xFF) - 128 再相加，峰值 ±255 LSB ≈ -42dBFS，
     * 是能听见的恒定白噪声（2026-11-19 实测"本底白噪声"），已改回。
     *
     * 但只用 a-b 还有另一半问题：结果落在 {-1, 0, +1}，**有 50% 概率恰好是 0**。
     * 经 -12dB 衰减后，弱音段本来就有大量样本被压成 0，再叠加这个抖动，
     * 仍会攒出成串的零，反复触发 PCM5102A 的 zero-data detect。
     * TI 工程师在 E2E 论坛给出的官方规避法就是"永远不要送 0x0000，
     * 弱音段改送 0x0001"，并且明确说明恢复时是 quick unmute（硬切换，
     * 没有 104 样本的软淡入）——所以每次进出都是可闻的轻咔。
     *
     * 2026-11-21 修正：原来写 +1，注释声称"输出恒在 {0,1,2}、DIN 永不出现零帧"
     * ——这是算错了。a、b 各取 {0,1}，a-b+1 的最小值是 0（a=0,b=1），
     * 也就是**仍有 1/3 概率送出 0**。信号本身被衰减舍入成 0 的那些样本，
     * 叠上这个抖动就有三分之一的帧是纯零。
     *
     * 这正是"钢琴/小提琴/吉他单音慢慢衰减时最容易听到轻咔"的直接原因：
     * 只有弱音段才有大量 v==0 的样本，而弱音段恰恰没有别的声部掩蔽。
     * 改成 +2 后输出恒在 {1,2,3}，零帧彻底消失。
     * 代价是恒定 +2 LSB 直流偏移 = 满刻度的 6e-5（约 -84dBFS），不可闻。 */
    return a - b + 2;
}

/* ---------- 连续分数重采样（ASRC） ----------
 * 2026-11-19 自检音对照实测证明：原来"每包离散插 1 帧"的做法会产生
 * **150Hz 电流音 + 噼啪**。原因是它制造了每秒约 150 次的离散事件，
 * 等效一个 150Hz 的脉冲串调制，落在音频带内；插值误差又与信号幅度成正比，
 * 所以声音越大越明显。
 *
 * 改成连续相位累加：输出每一帧，读指针前进 ratio（Q16）个输入帧，
 * 插值发生在相邻两帧之间，相位跨包连续传递。没有任何突变点，
 * 等效于一个 0.4% 的线性插值重采样器 —— 误差是连续的、听感上是轻微柔化而不是噼啪。
 *
 * ratio 由队列水位闭环给出：队列偏多 → ratio 变大（消耗快），偏少 → 变小。 */
#define ALINK_SRC_MAX_OUT   (ALINK_FRAMES_PER_PKT + 16)
/* ±0.3%（Q16≈200）。原来给到 ±0.8%，实测会跑到 +0.62% —— 但 src_process
 * 每包最多输出 n+1 帧，(1/240)=0.417% 就是硬上限，超出部分根本不起作用。
 * 也就是说旧参数让控制环长期工作在「以为在调、其实已经失效」的饱和区。 */
#define ALINK_SRC_ADJ_LIMIT 200
/* 死区：队列在 TARGET±DEADBAND 内一律不重采样，ratio 精确 1.0，
 * 全程走整数帧直取、一个插值样本都不产生。原来 0.75 包太窄，
 * 队列的日常抖动就足以让它一直在调，等于永远不在「完美直通」状态。
 *
 * 3 包：实测队列抖动 ±1 包，3 包足够盖住，ratio 恒 0.00%。
 *
 * 2026-11-21 实测教训：低延时档曾把死区收窄到 1.5 想压缩延时不确定度，
 * 结果日志里重采样一直是 +0.02~0.07% —— 插值器从未停过，
 * 「小电平单音」的音质明显不如死区 3.0 的标准档。
 * 延时不确定度只值 7.5ms，音质代价却是插值残差从 -96dB 掉到 -30dB。
 * 两档统一用 3.0，延时靠 TARGET_Q 和 DMA 降，不靠砍死区。 */
#define ALINK_SRC_DEADBAND  3.0f

/* 起播门槛：队列攒到「稳态水位」才开始播。稳态水位 = 目标 − 死区（见 play_task
 * 和 TUNING.md 的坑 3），所以直接由这两个量算出来，不单独调。
 *
 * 原来是固定 6 包（30ms），离稳态的 15 包太远。起播后靠 P 控制补水很慢：
 * 6 包时约每秒 0.2 包，越接近目标越慢，要 40~60 秒才回到 15 包 —— 这段时间
 * 抗 WiFi 抖动的余量只有一半，而断流恢复、TX 重启之后都会走这条路。
 * 门槛提到稳态水位的代价是每次起播多 45ms 静音。 */
#define ALINK_JITTER_PKTS (ALINK_TARGET_Q - (int)ALINK_SRC_DEADBAND)

/* 低水位重缓冲（2026-11-26）：队列被抽到只剩 ALINK_Q_LOW 包时不再一包包硬撑 ——
 * 按包线性淡出，然后退回预缓冲，等水位回到起播门槛再起播。
 *
 * 为什么不硬撑：P 控制回水很慢（增益 8，每秒约 0.13 包）。队列见底之后，只要包还在来，
 * 下面那条「连续 40 包（200ms）全空才退回预缓冲」就一直不触发，系统会在 0~5 包上瘸一两分钟，
 * 期间任何一点抖动都是一次欠载（PLC）。2026-11-26 注入音源短缺实测（70ms；再 60ms、
 * 每秒 1 包×10、40ms）：TARGET_Q 18 欠载 18 次，20 也还有 14 次，录制结束时都没回到稳态。
 *
 * 改成「见底就干脆停一下」：代价是一次约 90ms 的停顿（15ms 淡出，加约 75ms 静音，
 * 等水位回到起播门槛；起播时 15ms 长淡入），换来水位立刻回到稳态。数字静音期间这次停顿听不出来。
 *
 * 门槛必须低于电视盒子开机头 60 秒的稳态水位：盒子失配 0.146%，增益 8 下水位是
 * TARGET_Q − 3 − 12 = 5 包（TARGET_Q 20），所以这里取 2，留 3 包给队列 ±1 的日常抖动。
 * TARGET_Q 调低之前先重算这个数。 */
#define ALINK_Q_LOW          2
#define ALINK_LOW_FADE_PKTS  3       /* 淡出用几块（每块 5ms），最后一块的增益正好到 0 */
#define ALINK_LOW_ABORT      6       /* 淡出途中队列涨回 ALINK_Q_LOW + 这个数以上就中止（积压的包涌到了） */

/* polyphase windowed-sinc 插值器参数。
 *
 * 为什么不用 4 点三次 Hermite：上电自测（ALINK_SRC_DIAG）实测，
 * ratio=-0.055% 时 15kHz 基波从 12000 掉到 8437（-3.05dB），
 * 2f 谐波 -44.2dB —— 4 点核的高频增益随分数相位剧烈变化，
 * 而分数相位每 5ms 扫一遍，等于每秒 200 次的幅度调制，
 * 听感就是「调频干扰」。8kHz 也有 0.55dB 的增益损失和 -52dB 谐波。
 *
 * 先换成 16 抽头 Kaiser 窗 sinc（β=8）：同条件失真压到 -70dB 以下、高频增益
 * 平坦到 0.1dB。但 16 抽头的过渡带约 Fs/16 = 3kHz 宽，也就意味着从 21kHz 往下
 * 3kHz（即 18kHz）就开始滚降，15kHz 已经在坡上。
 *
 * 最终用 32 抽头：过渡带收到约 Fs/32 = 1.5kHz（16.5kHz 才起滚），15kHz 落在
 * 通带内平坦区。代价是每样本 32 次乘加 + 16 帧历史。 */
#define SRC_TAPS    32
#define SRC_HALF    (SRC_TAPS / 2)           /* 16：尾镜像帧数 */
#define SRC_PHASES  512                      /* 相位分辨率，表项间再做线性插值 */

/* buf 里「本包第 0 帧」的位置。必须留 SRC_TAPS 帧历史在前面：
 * 相位允许为负（见 src_process 结尾），负相位会把采样点拉回历史区，
 * i 最小到 SRC_BASE-SRC_HALF，取 buf[i-(SRC_HALF-1)] 正好落在 buf[1]，
 * 仍有余量不会越界。 */
#define SRC_BASE    SRC_TAPS

/* 自检音：1 = 上电后 50 秒播两段 1kHz 正弦做隔离实验；0 = 正常 */
#ifndef ALINK_SELFTEST
#define ALINK_SELFTEST 0
#endif

static int16_t s_sinc[SRC_PHASES + 1][SRC_TAPS];
static int32_t s_src_phase = 65536;          /* Q16，原点 = 上一包最后一帧 */
static int32_t s_src_ratio = 65536;          /* Q16，1.0 = 直通 */
static int16_t s_src_hist[SRC_TAPS][ALINK_CHANNELS]; /* 上一包的最后 SRC_TAPS 帧 */
static uint32_t s_phase_clamp = 0;           /* 相位钳位次数（应为 0；非 0 = 有硬跳变） */
static bool s_src_snap_ok = false;           /* 本包处于数字静音：结束时允许把相位吸附到整帧（见 emit） */

/* I2S 播放率跟随音源。
 *
 * 为什么必须这么做：自测（src_selftest）实测，ratio=1.0 时残差是数值底
 * （-96dB 以下），只要 ratio 偏离一点点（哪怕 -0.055%）残差立刻掉到 -30dB。
 * 这不是插值核的频响问题（32 抽头后基波已经 +0.01dB 准确），而是重采样
 * 固有的「拍频 / 包边界伪影」—— 见 Belgavi.AI Lab 的 SRC 综述：
 * 任意比率转换对接近 Nyquist 的频率必然产生 beat pattern，
 * 抽头加到 1000 也无法消除。
 *
 * 所以正确解法不是加抽头，而是让重采样比精确等于 1：把 I2S 采样率调到
 * 音源的实际帧率，src_process 全程走整数帧直取分支，零插值零失真。
 * 残余的亚 Hz 误差交给水位闭环的死区慢慢吸收。 */
static volatile uint32_t s_i2s_rate = ALINK_I2S_RATE;
static volatile int32_t  s_clk_req  = 0;     /* 非 0 = 请播放任务切到该采样率 */
static int      s_clk_pending = 0;           /* 已接单、待切换的目标采样率，0 = 无 */
static int      s_clk_fade    = 0;           /* >0 = 重配前还剩几包要淡出 */
static int      s_low_fade    = 0;           /* >0 = 低水位淡出还剩几块要写（见 ALINK_Q_LOW） */
static volatile bool s_low_done = false;     /* 淡出已经写完，播放任务该退回预缓冲了 */
static volatile uint32_t s_low_rebuf = 0;    /* 低水位重缓冲的累计次数（淡出写完才算） */
static volatile uint32_t s_low_abort = 0;    /* 淡出途中队列自己涨回来、中止的次数 */
static uint32_t s_rate_frames = 0;           /* 测速窗口内累计到达帧数 */
static int64_t  s_rate_t0     = 0;           /* 测速窗口起点 */
static int      s_rate_prev   = 0;           /* 上一窗口测出的帧率，用于去抖 */
static uint32_t s_rate_bad0   = 0;           /* 窗口起点时的 丢包+补位 计数，变了就作废 */
static uint32_t s_i2s_err = 0;    /* I2S 写入失败次数，非 0 就说明输出链路有问题 */

/* 实际写进 I2S 的累计帧数 —— 用来直接测出「DAC 那头到底每秒收多少帧」，
 * 而不是靠推断分频比。这是判断 I2S 时钟准不准的唯一硬指标：
 * 如果它等于 48000.0，而重采样比却偏离 0%，那就说明是音源（发送端）
 * 的速率不对；如果它本身偏离，才是本机时钟分频的问题。 */
static volatile uint32_t s_out_frames = 0;
static uint32_t s_out_prev = 0;
static int64_t  s_out_t0   = 0;

/* 输出峰值（衰减后、写 I2S 前的绝对值最大值）。
 * 用来一眼分清「链路好了但内容是静音」和「真的有声」：
 * 安静时 -90dBFS 以下，正常播放 -20~-40dBFS。 */
static volatile int32_t s_peak = 0;

/* ---------- 输出不连续检测（2026-11-21 新增） ----------
 *
 * 为什么需要它：听感上的「随机小爆音」出现时，补位/欠载/PLC/溢出/丢包
 * 五个计数器全部纹丝不动（实测 90 秒零增量），说明爆音不在「缺包」这条
 * 路径上。剩下的可能只有两种：数据内容本身有断层，或者 DAC 之后（电源、
 * 射频、时钟）的问题。软件计数器分不清这两者，只能直接盯波形。
 *
 * 判据：相邻样本差 |x[n]-x[n-1]| > ALINK_JUMP_TH 记一次。
 * 48kHz 下满幅 15kHz 正弦的差分约 0.83 FS，但真实音乐在 10kHz 以上的
 * 能量通常在 -25dB 以下，达不到 0.37 FS；爆音是单点阶跃，一测就中。
 *
 * 关键是分开统计「包边界」和「包中段」—— 写 I2S 每块严格 240 帧，
 * 块内偏移 0 就是包边界：
 *   边界跳变 → 包拼接 / 重采样相位错位（算法问题，可修）
 *   中段跳变 → 数据本身有洞（发送端或 PC 侧）
 *   完全没有 → 硬件（DAC 供电 / 射频耦合 / 时钟毛刺），改算法没用
 */
#ifndef ALINK_JUMP_TH
#define ALINK_JUMP_TH 12000          /* ≈0.37 满量程 */
#endif
static uint32_t s_jump_edge = 0;     /* 发生在块首（包边界）的次数 */
static uint32_t s_jump_mid  = 0;     /* 发生在块中段（包内部）的次数 */
static int32_t  s_jump_max  = 0;     /* 观察到的最大单步跳变（绝对值） */
static int16_t  s_jump_last[ALINK_CHANNELS];
static bool     s_jump_have = false;

/* 连续零帧统计：PCM5102A 连续 8192 个 LRCK 全零就进 mute，退出是硬切换。
 * 弱音段若出现接近 8192 的零串，那声轻咔就来自芯片自身的静音控制。 */
static uint32_t s_zero_run = 0;
static uint32_t s_zero_run_max = 0;

/* 队列水位的每秒最低/最高值。只看某一瞬间的深度会漏掉「冲到 1 又弹回 24」
 * 这种振荡 —— 那正是欠载和 overrun 交替出现的原因，必须看极值才看得见。 */
static int s_q_min = 1 << 20;
static int s_q_max = 0;

/* 最近 4 次跳变的详情，供日志回看。用户听到爆音后报个大概时间，
 * 就能靠这几条对上「是哪种跳变」。 */
typedef struct {
    int32_t off;        /* 块内偏移帧号，0 = 包边界 */
    int16_t prev[ALINK_CHANNELS];
    int16_t cur[ALINK_CHANNELS];
    int32_t step;       /* 有符号跳变量，ch0 的 */
} jump_ev_t;
static jump_ev_t s_jump_ev[4];
static int s_jump_ev_n = 0;

static inline void jump_check_block(const int16_t (*blk)[ALINK_CHANNELS], int nf)
{
    for (int i = 0; i < nf; i++) {
        for (int ch = 0; ch < ALINK_CHANNELS; ch++) {
            const int16_t cv = blk[i][ch];
            if (s_jump_have) {
                int32_t d = (int32_t)cv - (int32_t)s_jump_last[ch];
                const int32_t ad = d < 0 ? -d : d;
                if (ad > s_jump_max) {
                    s_jump_max = ad;
                }
                if (ad > ALINK_JUMP_TH) {
                    if (i == 0) {
                        s_jump_edge++;
                    } else {
                        s_jump_mid++;
                    }
                    jump_ev_t *e = &s_jump_ev[s_jump_ev_n & 3];
                    e->off  = i;
                    e->step = d;
                    for (int c2 = 0; c2 < ALINK_CHANNELS; c2++) {
                        e->prev[c2] = s_jump_last[c2];
                        e->cur[c2]  = blk[i][c2];
                    }
                    s_jump_ev_n++;
                    if (s_jump_ev_n <= 8) {
                        ESP_LOGW(TAG, "输出跳变 #%d 块内偏移=%d ch0 %d→%d (Δ=%ld) ch1 %d→%d",
                                 s_jump_ev_n, i,
                                 (int)e->prev[0], (int)e->cur[0], (long)e->step,
                                 (int)e->prev[1], (int)e->cur[1]);
                    }
                }
            }
            s_jump_last[ch] = cv;
        }
        s_jump_have = true;
    }
}

/* ---------- polyphase windowed-sinc 插值核 ----------
 * 上电时算一次：512 个相位 × 32 抽头（SRC_TAPS），Kaiser 窗 β=8。
 * 每行系数归一化到和为 1（DC 增益恒为 1），再压成 Q15 定点。
 *
 * 32 抽头的过渡带约 1.5kHz，15kHz 仍在通带内、增益平坦，
 * 阻带衰减 -80dB 以上 —— 正是为了压掉 4 点 Hermite 那 -44dB 的谐波。 */
static double bessel_i0(double x)
{
    double sum = 1.0, term = 1.0;
    const double h = x * 0.5;
    for (int k = 1; k <= 14; k++) {
        const double t = h / (double)k;
        term *= t * t;
        sum += term;
        if (term < 1e-14 * sum) {
            break;
        }
    }
    return sum;
}

static void sinc_init(void)
{
    const double beta = 8.0;
    const double i0b  = bessel_i0(beta);

    for (int p = 0; p <= SRC_PHASES; p++) {
        const double d = (double)p / (double)SRC_PHASES;
        double h[SRC_TAPS];
        double sum = 0.0;

        for (int k = 0; k < SRC_TAPS; k++) {
            /* 抽头中心在 k = SRC_HALF-1（32 抽头时是 15），对应 buf[i] */
            const double x = (double)(k - (SRC_HALF - 1)) - d;
            const double s = (fabs(x) < 1e-9) ? 1.0 : sin(M_PI * x) / (M_PI * x);
            const double r = x / (double)SRC_HALF;
            const double w = (fabs(r) >= 1.0) ? 0.0
                           : bessel_i0(beta * sqrt(1.0 - r * r)) / i0b;
            h[k] = s * w;
            sum += h[k];
        }
        if (fabs(sum) < 1e-9) {
            sum = 1.0;
        }
        for (int k = 0; k < SRC_TAPS; k++) {
            s_sinc[p][k] = (int16_t)lround(h[k] / sum * 32767.0);
        }
    }
}

/* 把本包 n 帧（已在 out 里）做连续分数重采样，结果写回 out。
 * 返回输出帧数（通常 239~242）。
 *
 * 插值核是 32 抽头 windowed sinc（SRC_TAPS），不是多项式：
 * 4 点三次 Hermite 在高频的增益随分数相位剧烈摆动（实测 15kHz 掉 3dB、
 * 2f 谐波 -44dB），而分数相位每 5ms 扫一遍，等于每秒 200 次幅度调制，
 * 听感正是「调频干扰」。sinc 核把同条件失真压到 -70dB 以下。
 *
 * reset = true 时相位归零，用于起播/断流恢复，避免历史帧错位。 */
static int src_process(int16_t *out, int n, bool reset, const int16_t *next)
{
    /* 布局：[0..31] 上一包末 SRC_TAPS 帧 | [32..n+31] 本包 n 帧 | 尾部 16 帧前瞻
     * 32 抽头要取 i-15 .. i+16 共 32 帧；i 最小 SRC_BASE-16、最大 SRC_BASE-1+n，
     * 所以头部要 32 帧历史、尾部要 16 帧「未来」。 */
    static int16_t buf[SRC_BASE + ALINK_FRAMES_PER_PKT + SRC_HALF][ALINK_CHANNELS];

    memcpy(&buf[0][0], s_src_hist, sizeof(s_src_hist));
    memcpy(&buf[SRC_BASE][0], out, (size_t)n * ALINK_CHANNELS * sizeof(int16_t));
    if (next) {
        /* 队列里的下一包就是真实的「未来」，比复制末帧准确得多 */
        memcpy(&buf[SRC_BASE + n][0], next,
               (size_t)SRC_HALF * ALINK_CHANNELS * sizeof(int16_t));
    } else {
        /* 只有欠载/起播时队列为空，才退回镜像 */
        for (int k = 0; k < SRC_HALF; k++) {
            buf[SRC_BASE + n + k][0] = buf[SRC_BASE + n - 1][0];
            buf[SRC_BASE + n + k][1] = buf[SRC_BASE + n - 1][1];
        }
    }

    if (reset) {
        /* 断点恢复时 s_src_hist 是旧内容，不能再拿来插值。
         * phase = 1<<16 让第一个采样点正好落在 buf[SRC_BASE]（本包第 0 帧），
         * 等价于「不做跨包插值」，从本包干净起算。 */
        s_src_phase = 65536;
    }

    /* 相位原点 = 上一包最后一帧 buf[SRC_BASE-1]。
     *   phase ∈ [k<<16, (k+1)<<16) → 采样点在 buf[SRC_BASE-1+k] 与 buf[SRC_BASE+k] 之间
     * 所以 i = (SRC_BASE-1) + (phase>>16)，本包最远走到 buf[SRC_BASE+n]（本包最后一帧），
     * 上界是 (n+1)<<16。每包结束时把小数相位原样留给下一包，
     * 下一包从 buf[SRC_BASE-1]（= 旧包最后一帧）之后接着走，跨包连续。
     *
     * 旧写法 i = 2 + ... 把原点放在 buf[2]，包尾的小数相位被直接丢掉，
     * 相当于每包做一次 0~1 帧（0~20.8µs）的相位吸附。200 包/秒 = 每秒
     * 200 次硬跳变：1kHz 只差 -44dB 听不出来，10kHz 以上就是几十度相位
     * 突变、幅度跳满 —— 扫频后段那阵噼啪就是这么来的。
     *
     * 相位允许为负：ratio < 1 时每包消耗的相位少于 advance，减完会变负。
     * 原来直接钳 0 等于「每包吸附一次」，和上面那个 bug 同源。负相位表示
     * 采样点落回历史区 buf[1..31]，32 帧历史足够（下限 -(SRC_HALF-1) 帧）。 */
    const int32_t span_end = (int32_t)(n + 1) << 16;
    const int32_t advance  = (int32_t)n << 16;
    int nout = 0;
    while (s_src_phase < span_end && nout < ALINK_SRC_MAX_OUT) {
        const int     i = (SRC_BASE - 1) + (int)(s_src_phase >> 16);
        const int32_t f = s_src_phase & 0xFFFF;

        if (f == 0) {
            /* 恰好落在整数帧上，直接取，省掉 32 次乘加 */
            out[nout * ALINK_CHANNELS]     = buf[i][0];
            out[nout * ALINK_CHANNELS + 1] = buf[i][1];
        } else {
            /* 相位 p = f*SRC_PHASES/2^16，取相邻两行系数做线性插值。
             * 表项间隔 1/512 相位，行间线性插值把等效相位精度拉到 -120dB 量级，
             * 否则单纯 512 相位的量化误差在 15kHz 上就有 -54dB，压不住。 */
            const int32_t pf  = f * SRC_PHASES;
            const int     pp  = (int)(pf >> 16);
            const int32_t rem = pf & 0xFFFF;
            const int16_t *c0 = s_sinc[pp];
            const int16_t *c1 = s_sinc[pp + 1];

            for (int ch = 0; ch < ALINK_CHANNELS; ch++) {
                /* 每行系数和 = 1，绝对值之和约 1.3，所以
                 * Σ|x_k·c_k| ≤ 32768 × 1.3 × 32768 ≈ 1.4e9，仍在 int32 内，
                 * 不必走 64 位乘法（Xtensa 上 int64 是软件模拟，慢好几倍）。 */
                int32_t acc = 0;
                for (int k = 0; k < SRC_TAPS; k++) {
                    const int32_t c = (int32_t)c0[k] +
                                      ((((int32_t)c1[k] - (int32_t)c0[k]) * rem) >> 16);
                    acc += (int32_t)buf[i - (SRC_HALF - 1) + k][ch] * c;
                }
                int32_t v = (acc + (1 << 14)) >> 15;
                if (v > 32767) {
                    v = 32767;
                }
                if (v < -32768) {
                    v = -32768;
                }
                out[nout * ALINK_CHANNELS + ch] = (int16_t)v;
            }
        }
        nout++;
        s_src_phase += s_src_ratio;
    }
    /* 推进 n 帧：原点从「旧包最后一帧」挪到「本包最后一帧」。
     * 减法用 advance(n) 而不是 span_end(n+1)，是为了把 (1<<16) 的
     * 原点偏移保留在相位里 —— 那一位正是跨包连续性所在。 */
    s_src_phase -= advance;
    /* 负相位是合法的（采样点落回历史区），只在真的越界时才钳。
     * 原来无条件钳 0 会在 ratio<1 时每包丢 0.7~1.7 帧相位，
     * 200 次/秒的硬跳变 —— 小电平单音上就是「咔」。 */
    {
        const int32_t lo = -(int32_t)((SRC_HALF - 1) << 16);
        if (s_src_phase < lo) {
            s_src_phase = lo;
            s_phase_clamp++;
        }
    }

    /* ratio 精确为 1 时，把相位残差吸附到整帧。吸附之后所有采样点都落在整数帧上
     * （走 f == 0 分支），彻底不再插值。
     *
     * 吸附等于把时间轴一次性挪动，所以允许挪多少取决于当时有没有声：
     *
     *   有声：只允许挪不足 0.1 帧（≈2µs）。这个量级是一次性事件，听不出来。
     *         两个方向都吸附 —— ratio 小于 1 时残差通常是「差一点点到下一个整数帧」，
     *         只向下吸附的话这种情况会一直留着。
     *   静音：挪多少都无所谓，吸附到最近的整数帧。要求已经连续
     *         ALINK_WAKE_QUIET_PKTS 包数字静音，紧跟而来的有声包一定落在 15ms 淡入里。
     *         挪 0.5 帧对有声内容是几百 LSB 的单点误差，所以不在有声时做。
     *
     * 原来只有「残差不足 0.1 帧才向下吸附」一条。开机收敛那几十秒积累的小数相位
     * 远超 0.1 帧，于是收敛完基本都停在某个固定的小数相位上 —— 日志里 ratio 是
     * 0.00%，实际每个样本仍在走 32 抽头插值，「零插值直通」并没有发生。 */
    if (s_src_ratio == 65536 && s_src_phase >= 0) {
        const int32_t rem = s_src_phase & 0xFFFF;
        if (rem != 0) {
            if (s_src_snap_ok) {
                s_src_phase += (rem >= 32768) ? (65536 - rem) : -rem;
            } else if (rem < 6554) {
                s_src_phase -= rem;
            } else if (rem >= 65536 - 6554) {
                s_src_phase += 65536 - rem;
            }
        }
    }
    s_src_snap_ok = false;

    /* 历史 = 本包最后 SRC_TAPS 帧（要求 n >= SRC_TAPS，payload 240 帧满足） */
    memcpy(s_src_hist, &buf[SRC_BASE + n - SRC_TAPS][0], sizeof(s_src_hist));

    return nout;
}

/* ---------- 重采样器失真自测（离线，不发声，上电跑一次） ----------
 *
 * 「8~15kHz 纯音听起来像调频干扰，白噪声却正常」是典型的谐波失真特征：
 * 单频才会让谐波（含折叠回带内的）暴露出来，宽带噪声只是稍微亮一点。
 *
 * 本函数把 double 精确合成的正弦喂给 src_process，累积一段输出后用
 * 最小二乘正弦拟合测「残差/基波」，把听感变成 dB 数字。
 *
 * 判读：残差 ≤ -70dB → 插值器够干净，往后查 DAC / I2S 时钟 / 电源
 *       残差 ≈ -45dB → 插值核还是太短，需要加抽头
 *
 * 注意频率要用 f0 × ratio：重采样把时间轴拉伸了，按 f0 测会失配。
 */
#ifndef ALINK_SRC_DIAG
#define ALINK_SRC_DIAG 0
#endif

#if ALINK_SRC_DIAG
/* 最小二乘拟合 A·sin + B·cos，返回幅度与残差 RMS。
 *
 * 为什么不用单频 DFT（Goertzel）：重采样后时间轴被拉伸，信号实际频率
 * 变成 f0 × ratio。按 f0 去测就有 8Hz 失配（15kHz 时），既把幅度压低
 * 1.5dB，又在旁瓣位置制造假的「谐波」。残差法不依赖频率对齐，
 * 残差里包含谐波、互调、调制边带和噪声，是真正的 THD+N 指标。 */
static void fit_sine(const int16_t *x, int n, double f, double fs,
                     double *amp_out, double *resid_out)
{
    const double dw = 2.0 * M_PI * f / fs;
    double ss = 0.0, cc = 0.0, sc = 0.0, xs = 0.0, xc = 0.0;

    for (int i = 0; i < n; i++) {
        const double w = dw * (double)i;
        const double s = sin(w);
        const double c = cos(w);
        const double v = (double)x[i];
        ss += s * s;
        cc += c * c;
        sc += s * c;
        xs += v * s;
        xc += v * c;
    }
    const double det = ss * cc - sc * sc;
    double A = 0.0, B = 0.0;
    if (fabs(det) > 1e-9) {
        A = (xs * cc - xc * sc) / det;
        B = (xc * ss - xs * sc) / det;
    }
    double rss = 0.0;
    for (int i = 0; i < n; i++) {
        const double w  = dw * (double)i;
        const double e  = (double)x[i] - (A * sin(w) + B * cos(w));
        rss += e * e;
    }
    if (amp_out) {
        *amp_out = sqrt(A * A + B * B);
    }
    if (resid_out) {
        *resid_out = sqrt(rss / (double)n);
    }
}

static void src_selftest(void)
{
    enum { NPKT = 10, NS_HEAD = ALINK_FRAMES_PER_PKT, NS_AN = 1872,
           NS_MAX = NPKT * ALINK_FRAMES_PER_PKT + 64 };

    static int16_t sig[(ALINK_FRAMES_PER_PKT + 32) * ALINK_CHANNELS];
    static int16_t mono[NS_MAX];

    static const int32_t ratios[3] = { 65536, 65500, 65000 };
    static const char   *rname[3]  = { "1.00000 直通", "-0.055% 实际", "-0.82% 放大" };
    static const double  freqs[2]  = { 8000.0, 15000.0 };

    ESP_LOGW(TAG, "==== 重采样失真自测 ====");

    for (int fi = 0; fi < 2; fi++) {
        const double f0 = freqs[fi];

        for (int ri = 0; ri < 3; ri++) {
            memset(s_src_hist, 0, sizeof(s_src_hist));
            s_src_phase = 65536;
            const int32_t keep_ratio = s_src_ratio;
            s_src_ratio = ratios[ri];

            int total = 0;
            int k     = 0;
            for (int p = 0; p < NPKT && total < NS_MAX; p++) {
                for (int i = 0; i < ALINK_FRAMES_PER_PKT; i++, k++) {
                    const int16_t v = (int16_t)(12000.0 *
                        sin(2.0 * M_PI * f0 * (double)k / (double)ALINK_I2S_RATE));
                    sig[i * 2]     = v;
                    sig[i * 2 + 1] = v;
                }
                /* 自测也要喂「真实的下一包」，否则测到的是镜像伪影，
                 * 而不是实际运行时的行为。k 此时正好指向下一包第 0 帧。 */
                int16_t nextb[SRC_HALF * ALINK_CHANNELS];
                for (int i = 0; i < SRC_HALF; i++) {
                    const int16_t v = (int16_t)(12000.0 *
                        sin(2.0 * M_PI * f0 * (double)(k + i) / (double)ALINK_I2S_RATE));
                    nextb[i * 2]     = v;
                    nextb[i * 2 + 1] = v;
                }
                const int no = src_process(sig, ALINK_FRAMES_PER_PKT, p == 0, nextb);
                for (int i = 0; i < no && total < NS_MAX; i++) {
                    mono[total++] = sig[i * 2];
                }
            }
            s_src_ratio = keep_ratio;

            if (total < NS_HEAD + NS_AN) {
                ESP_LOGW(TAG, "  %5.0fHz %s : 样本不足(%d)", f0, rname[ri], total);
                continue;
            }

            /* 用重采样后的真实频率去拟合，否则频率失配会把结果全带偏。 */
            const double f_act = f0 * ((double)ratios[ri] / 65536.0);
            double amp = 0.0, resid = 0.0;
            fit_sine(&mono[NS_HEAD], NS_AN, f_act, (double)ALINK_I2S_RATE, &amp, &resid);

            ESP_LOGW(TAG, "  %5.0fHz %-14s 基波=%6.1f (%+5.2fdB)  残差/基波=%6.1fdB  (共%d帧)",
                     f0, rname[ri], amp,
                     20.0 * log10(amp / 12000.0 + 1e-12),
                     20.0 * log10(resid / (amp + 1e-12) + 1e-12),
                     total);
        }
    }

    ESP_LOGW(TAG, "==== 自测结束 ====");

    memset(s_src_hist, 0, sizeof(s_src_hist));
    s_src_phase = 65536;
}
#endif

/* src == NULL 表示静音包 */
static void emit(const int16_t *src)
{
    /* +32 给重采样留余量（输出帧数在 238~242 之间浮动） */
    static int16_t out[ALINK_FRAMES_PER_PKT * ALINK_CHANNELS + 32];
    const int n  = ALINK_FRAMES_PER_PKT;
    const int xf = ALINK_XFADE_FRAMES;
    int nout = n;

#if ALINK_SELFTEST
    /* ---------- 自检音（隔离实验，上电后 50 秒） ----------
     * 绕过网络和衰减，直接生成 1kHz 正弦写 I2S，分两段做单变量对照：
     *   0 ~ 25s ：固定 240 帧/包，完全不动数据        → 最干净的基准
     *   25 ~ 50s：同一正弦，加 ±0.8% 的分数重采样（模拟漂移补偿）
     * 2026-11-19 实测结论：第 1 段干净，第 2 段出现"电流音 + 噼啪"，
     * 由此定位到离散插帧是根因，遂改为 src_process 连续重采样。 */
    {
        const int64_t ms = esp_timer_get_time() / 1000;
        if (ms < 50000) {
            static bool    tab_ok = false;
            static int16_t tab[256];
            static uint32_t ph = 0;
            static int      last_seg = -1;

            if (!tab_ok) {
                for (int i = 0; i < 256; i++) {
                    tab[i] = (int16_t)(sinf(6.2831853f * (float)i / 256.0f) * 16000.0f);
                }
                tab_ok = true;
            }
            const int seg = (ms < 25000) ? 1 : 2;
            if (seg != last_seg) {
                last_seg = seg;
                ESP_LOGW(TAG, "自检音 第%d段（%s）", seg,
                         seg == 1 ? "纯正弦 固定240帧" : "纯正弦 + 分数重采样");
            }
            const uint32_t step = (uint32_t)(256.0 * 1000.0 / ALINK_I2S_RATE * 65536.0);
            for (int i = 0; i < n; i++) {
                ph += step;
                const int16_t v = tab[(ph >> 16) & 0xFFu];
                out[i * 2]     = v;
                out[i * 2 + 1] = v;
            }
            int n2 = n;
            if (seg == 2) {
                const int32_t keep = s_src_ratio;
                s_src_ratio = 65000;                 /* ≈ -0.8%，比实际漂移更明显 */
                n2 = src_process(out, n, false, NULL);
                s_src_ratio = keep;
            }
            size_t wr = 0;
            i2s_channel_write(s_i2s_tx, out,
                              (size_t)n2 * ALINK_CHANNELS * sizeof(int16_t),
                              &wr, portMAX_DELAY);
            return;
        } else if (ms < 50100) {
            ESP_LOGW(TAG, "自检音结束，切回正常播放");
        }
    }
#endif

    if (src) {
        /* ---- 从静音里「醒来」的检测 ----
         *
         * 暂停、切歌之后，主机并不会断开流：Windows 保持端点打开，
         * 一直送全零 PCM。所以接收端根本等不到「断流」，队列永远是满的，
         * 上面那套 starve 判定完全不会触发 —— 而恢复的第一个非零样本
         * 就是从 0 直接跳到某个幅度，低频上就是一声闷响。
         *
         * 判据用「整包几乎全零」而不是「音量小」：连钢琴最弱的尾音
         * 也不会 240 帧全部低于 16 LSB（-66dBFS），所以正常弱奏不会被误判；
         * 而数字静音一定是全零。连续 3 包（15ms）静音之后突然出现
         * 峰 ≥ 64（约 -54dBFS）的内容，就认定是「重新开始放了」。
         * 阈值不能按「好听」定高：实测 25% 系统音量下输出峰值才 168 LSB，
         * 最早写成 1000 时一次都没触发过。 */
        int32_t pk = 0;
        for (int i = 0; i < ALINK_FRAMES_PER_PKT * ALINK_CHANNELS; i++) {
            const int32_t a = (src[i] < 0) ? -(int32_t)src[i] : (int32_t)src[i];
            if (a > pk) {
                pk = a;
            }
        }
        if (pk < 16) {
            if (s_quiet_pkts < 1000) {
                s_quiet_pkts++;
            }
        } else {
            if (s_quiet_pkts >= ALINK_WAKE_QUIET_PKTS && pk >= 64 && s_fade_in_left == 0) {
                s_fade_in_left = ALINK_FADE_IN_FRAMES;
                s_wake_fade++;
                ESP_LOGI(TAG, "从静音恢复（峰=%ld）→ 淡入 %dms",
                         (long)pk, ALINK_FADE_IN_FRAMES * 1000 / ALINK_SAMPLE_RATE);
            }
            s_quiet_pkts = 0;
        }
        memcpy(out, src, ALINK_PAYLOAD_BYTES);
    } else {
        /* 静音包（起播前的空转、PLC 之后的补位）同样计入「安静时长」，
         * 否则断流恢复时 s_quiet_pkts 是 0，醒来检测永远不会触发。 */
        if (s_quiet_pkts < 1000) {
            s_quiet_pkts++;
        }
        memset(out, 0, ALINK_PAYLOAD_BYTES);
    }

    /* 样本级漂移补偿改成连续分数重采样：没有任何离散突变点。
     * 原来"每包插 1 帧"会制造 150Hz 脉冲串（实测听感 = 电流音 + 噼啪）。 */
    {
        const bool reset = s_xfade;      /* 断点恢复时相位归零，避免历史帧错位 */
        int16_t nxt[SRC_HALF * ALINK_CHANNELS];
        const int16_t *pnext = q_peek(nxt, SRC_HALF) ? nxt : NULL;
        /* 已经连续几包数字静音：这一包结束后允许把相位吸附到整帧（见 src_process） */
        s_src_snap_ok = (s_quiet_pkts >= ALINK_WAKE_QUIET_PKTS);
        nout = src_process(out, n, reset, pnext);
    }

    /* 断点处用 2ms 线性交叉淡化：从上一包尾部渐变到本包开头。
     * 起播、漂移丢包、补静音、断流恢复，全部走这一条路径。 */
    if (s_xfade) {
        for (int i = 0; i < xf; i++) {
            int32_t w = ((i + 1) << 8) / xf;      /* 1..256 */
            out[i * 2]     = (int16_t)(((int32_t)s_tail[i][0] * (256 - w) +
                                        (int32_t)out[i * 2]     * w) >> 8);
            out[i * 2 + 1] = (int16_t)(((int32_t)s_tail[i][1] * (256 - w) +
                                        (int32_t)out[i * 2 + 1] * w) >> 8);
        }
        s_xfade = false;
    }

    /* 断流恢复 / 时钟重配后的长淡入：从静音线性升到满幅。
     *
     * 不能和上面那道 2ms 接缝淡化合并 —— 那个是「旧波形 → 新波形」，
     * 两侧都有信号，2ms 够；这里是「静音 → 满幅」，2ms 等于阶跃，
     * 低频上就是一次冲击。15ms 才能做到听不出起点。 */
    if (s_fade_in_left > 0) {
        const int32_t total = ALINK_FADE_IN_FRAMES;
        for (int i = 0; i < nout && s_fade_in_left > 0; i++) {
            const int32_t w = ((total - s_fade_in_left + 1) << 16) / total;
            out[i * 2]     = (int16_t)(((int32_t)out[i * 2]     * w) >> 16);
            out[i * 2 + 1] = (int16_t)(((int32_t)out[i * 2 + 1] * w) >> 16);
            s_fade_in_left--;
        }
    }

    /* 固定电平校准 + 抖动。
     * 主机的音量已经在 TX 侧作用到样本上（见 alink_tx.c 的 uac_volume_cb），
     * 这里是叠在它上面的一层固定衰减：PCM5102A 满刻度是 2.1Vrms，对有源音箱是
     * 12~20dB 的结构性过载，不压一层就是「开一点就很大」。改 ALINK_OUT_ATTEN_DB 可调。
     *
     * 乘法必须带舍入：原来直接 >>16 是截断，误差恒为负，变成与信号相关的
     * 失真（有声时才听得出来，像轻微噼啪）。再加 TPDF 抖动把残余量化误差
     * 摊成白噪声，同时保证 DIN 永不全零，压住 PCM5102A 的 auto-mute。 */
    {
        static int32_t g_q16 = -1;
        if (g_q16 < 0) {
            g_q16 = (int32_t)(65536.0f * powf(10.0f, -ALINK_OUT_ATTEN_DB / 20.0f));
            ESP_LOGI(TAG, "输出电平校准 -%d dB (Q16=%ld)", ALINK_OUT_ATTEN_DB, (long)g_q16);
        }
        for (int i = 0; i < nout * 2; i++) {
            int32_t v = (((int32_t)out[i] * g_q16) + 32768) >> 16;
            v += dither_tpdf();
            if (v > 32767) {
                v = 32767;
            }
            if (v < -32768) {
                v = -32768;
            }
            out[i] = (int16_t)v;
            const int32_t av = v < 0 ? -v : v;
            if (av > s_peak) {
                s_peak = av;
            }
            /* 数「连续零帧」的最长长度 —— PCM5102A 的 zero-detect 要连续
             * 8192 个 LRCK（48kHz 下 170ms）全零才进 mute，退出是硬切换。
             * 如果这里实测的最长零串逼近 8192，就说明芯片在弱音段反复静音，
             * 那声轻咔就是它。按声道 0 统计即可（左右同源）。 */
            if ((i & 1) == 0) {
                if (v == 0) {
                    if (++s_zero_run > s_zero_run_max) {
                        s_zero_run_max = s_zero_run;
                    }
                } else {
                    s_zero_run = 0;
                }
            }
        }
    }

    memcpy(s_tail, &out[(nout - xf) * 2], sizeof(s_tail));

    /* 写入 I2S —— 必须按「整描述符」写，否则 DMA 会瞬时空转。
     *
     * DMA 描述符是 ALINK_FRAMES_PER_PKT 帧（240 帧 = 960 字节）一块，
     * 而重采样输出 nout 是 239~241 帧，**不是描述符的整数倍**。
     * IDF 驱动内部按描述符填装：写 964 字节时，960 字节立刻交给 DMA，
     * 剩下 4 字节留在驱动里等下一包凑满。于是「应用层写入节奏」和
     * 「DMA 消耗节奏」错开，队列里明明有数据、硬件却短暂无块可播。
     *
     * 硬件 underrun 不报错：它会把上一块数据再播一遍（或补零），
     * `i2s_channel_write` 返回 ESP_OK，我们的欠载计数器也不会动。
     * 听感就是一声随机的轻爆 —— 正是「听细腻音乐时的随机小爆音」，
     * 而且所有软件指标全绿，根本查不到。
     *
     * 修法：应用层先攒够 240 帧再写，每次写严格 960 字节。
     * 代价是最多 5ms 额外延迟（队列稳态 15 包 ≈ 75ms 缓冲，无所谓）。 */
    static int16_t acc[ALINK_FRAMES_PER_PKT * 2 + 32][ALINK_CHANNELS];
    static int acc_n = 0;
    memcpy(&acc[acc_n][0], out, (size_t)nout * ALINK_CHANNELS * sizeof(int16_t));
    acc_n += nout;

    while (acc_n >= ALINK_FRAMES_PER_PKT) {
        /* 写之前先判断块波形连不连续。放在这里而不是更早，是因为
         * 这里才是「真正送进 DAC 的那 240 帧」——重采样、淡化、衰减
         * 全部作用完之后的结果，前面任何一步引入的断层都会在这里现形。 */
        jump_check_block((const int16_t (*)[ALINK_CHANNELS])acc,
                         ALINK_FRAMES_PER_PKT);

        /* 时钟重配前的淡出：把这一包真实音频按线性斜坡压向 0。
         * 放在这里而不是 emit() 开头，是因为 acc 才是真正送进 DMA 的
         * 那 240 帧 —— 重采样、淡化、电平校准全部作用完之后的结果。
         * 最后一包 gain 正好为 0，DMA 排空时 DAC 收到的是静音。 */
        if (s_clk_fade > 0) {
            const int32_t g = ((s_clk_fade - 1) * 256) / ALINK_CLK_FADE_PKTS;
            int16_t *p = &acc[0][0];
            for (int i = 0; i < ALINK_FRAMES_PER_PKT * ALINK_CHANNELS; i++) {
                p[i] = (int16_t)(((int32_t)p[i] * g) >> 8);
            }
            s_clk_fade--;
        }

        /* 低水位淡出（见 play_task 的「低水位重缓冲」）。斜坡做在块内：
         * 只有三块，每块一个固定增益的话是三级 −3.5dB 的台阶。 */
        if (s_low_fade > 0) {
            const int32_t top = s_low_fade * 256 / ALINK_LOW_FADE_PKTS;        /* 这一块开头的增益 */
            const int32_t bot = (s_low_fade - 1) * 256 / ALINK_LOW_FADE_PKTS;  /* 这一块结尾的增益 */
            int16_t *p = &acc[0][0];
            for (int i = 0; i < ALINK_FRAMES_PER_PKT; i++) {
                const int32_t g = top + ((bot - top) * (i + 1)) / ALINK_FRAMES_PER_PKT;
                p[2 * i]     = (int16_t)(((int32_t)p[2 * i] * g) >> 8);
                p[2 * i + 1] = (int16_t)(((int32_t)p[2 * i + 1] * g) >> 8);
            }
            if (--s_low_fade == 0) {
                s_low_done = true;
            }
        }

        size_t written = 0;
        const size_t want = (size_t)ALINK_FRAMES_PER_PKT * ALINK_CHANNELS * sizeof(int16_t);
        esp_err_t we = i2s_channel_write(s_i2s_tx, acc, want, &written, portMAX_DELAY);
        s_out_frames += (uint32_t)(written / (ALINK_CHANNELS * sizeof(int16_t)));
        if (we != ESP_OK || written != want) {
            s_i2s_err++;
            if (s_i2s_err == 1 || (s_i2s_err % 200) == 0) {
                ESP_LOGW(TAG, "I2S 写入异常 ret=%d written=%u/%u 累计=%lu",
                         (int)we, (unsigned)written, (unsigned)want,
                         (unsigned long)s_i2s_err);
            }
            s_xfade = true;      /* 断点，下一包淡化接上 */
        }

        acc_n -= ALINK_FRAMES_PER_PKT;
        if (acc_n > 0) {
            memmove(&acc[0][0], &acc[ALINK_FRAMES_PER_PKT][0],
                    (size_t)acc_n * ALINK_CHANNELS * sizeof(int16_t));
        }
    }
}

/* ---------- PLC：丢包隐藏 ----------
 * 丢包时重复上一包，逐次衰减。为什么不是补静音：静音是波形直接阶跃到 0，
 * 播放端喇叭会给出一个宽频冲击，听起来就是「咔」。重复上一包虽然会「糊」，
 * 但频谱连续，大脑会把它补过去。
 *
 * 相位不连续由 emit() 的 2ms 交叉淡化抹平，所以这里不管接缝。
 */
static const uint16_t s_plc_att[ALINK_PLC_DECAY_MAX] = {
    256,    /* ×1.00 */
    154,    /* ×0.60 */
    90,     /* ×0.35 */
    52,     /* ×0.20 */
};

static void emit_plc(void)
{
    static int16_t plc[ALINK_FRAMES_PER_PKT * ALINK_CHANNELS];
    const int n = ALINK_FRAMES_PER_PKT * ALINK_CHANNELS;
    const uint16_t a = (s_plc_run < ALINK_PLC_DECAY_MAX) ? s_plc_att[s_plc_run] : 0;

    if (a == 0) {
        /* 连续丢到 20ms 以上，外推已无信息可用，再重复就是嗡嗡声 */
        memset(plc, 0, ALINK_PAYLOAD_BYTES);
    } else {
        for (int i = 0; i < n; i++) {
            plc[i] = (int16_t)(((int32_t)s_prev[i] * a) >> 8);
        }
    }
    if (s_plc_run < 255) {
        s_plc_run++;
    }
    s_plc_total++;

    s_xfade = true;     /* PLC 从上一包尾部接过来，接缝必须淡化 */
    emit(plc);
}

/* ---------- ESP-NOW 传输层 ----------
 *
 * 与 UDP 版的区别只有这一层：不做 STA 去关联热点、不建 socket、不需要 HELLO。
 * 收包回调跑在 WiFi 任务上下文，不能阻塞、不能做重活，所以只把原始包
 * 塞进环形缓冲，真正的序号处理 / 入队 / 统计仍然在 rx_task 里做 ——
 * 和 UDP 版逐行一致，保证 AB 对比时唯一的变量是传输层。
 */

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len < (int)(ALINK_HDR_SIZE + ALINK_PAYLOAD_BYTES)) {
        return;
    }
    /* 配了发送方白名单就只认它。只比较 6 个字节，在 WiFi 任务里做得起。 */
    if (s_filter_tx && (!info || memcmp(info->src_addr, s_peer_tx, ALINK_ETH_ALEN) != 0)) {
        return;
    }
    if (info && info->rx_ctrl) {
        s_peer_rssi = info->rx_ctrl->rssi;
    }
    /* 0 超时：环形缓冲满就丢这一包，交给上层 PLC 补 ——
     * 语义等同于 UDP 版的 recvbuf 溢出，绝不在这里阻塞 WiFi 任务。 */
    xRingbufferSend(s_rb, data, (size_t)len, 0);
}

#if ALINK_WIFI_SCAN
/* 扫一遍 2.4G，把每个 AP 和每个信道的汇总打进日志。只在诊断固件里编译。 */
static void wifi_scan_print(void)
{
    const wifi_scan_config_t cfg = { .show_hidden = true };
    const esp_err_t r = esp_wifi_scan_start(&cfg, true);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "WiFi 扫描失败 %s", esp_err_to_name(r));
        return;
    }
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    wifi_ap_record_t *rec = calloc(n ? n : 1, sizeof(*rec));
    if (!rec) {
        return;
    }
    uint16_t got = n;
    esp_wifi_scan_get_ap_records(&got, rec);

    int cnt[14] = { 0 }, strongest[14] = { 0 };
    ESP_LOGW(TAG, "==== 2.4G 扫描：%d 个 AP ====", got);
    for (uint16_t i = 0; i < got; i++) {
        const int c = rec[i].primary;
        ESP_LOGW(TAG, "  信道 %2d  %4d dBm  " MACSTR "  %s",
                 c, rec[i].rssi, MAC2STR(rec[i].bssid),
                 rec[i].ssid[0] ? (const char *)rec[i].ssid : "(隐藏)");
        if (c >= 1 && c <= 13) {
            if (cnt[c]++ == 0 || rec[i].rssi > strongest[c]) {
                strongest[c] = rec[i].rssi;
            }
        }
    }
    for (int c = 1; c <= 13; c++) {
        if (cnt[c]) {
            ESP_LOGW(TAG, "  汇总 信道 %2d：%d 个 AP，最强 %d dBm", c, cnt[c], strongest[c]);
        }
    }
    ESP_LOGW(TAG, "==== 扫描结束（本链路用的是信道 %d）====", ALINK_WIFI_CHANNEL);
    free(rec);
    esp_wifi_clear_ap_list();
}
#endif

#if ALINK_WIFI_SURVEY
#define SURV_SLOTS 16
typedef struct {
    uint8_t  mac[6];
    uint32_t frames;
    uint32_t air_us;
    int8_t   rssi;
} survey_tx_t;
static survey_tx_t s_surv_tx[SURV_SLOTS];
static uint32_t s_surv_frames, s_surv_air_us;
static int8_t   s_surv_rssi;

/* 帧的调制速率（bit/s）。只求量级对，用来把帧长折成空口时间。
 * 11n 看 mcs 序号（20MHz、长保护间隔是 6.5M 的整数倍）；非 11n 的 rate 字段是序号：
 * 11b 的 0~3 是 1/2/5.5/11M，11g 的 0~7 是 6/9/12/18/24/36/48/54M。
 * 序号超出这个范围时按「速率 ÷ 500k」理解（别的芯片是这么编码的）。 */
static uint32_t survey_phy_rate(const wifi_pkt_rx_ctrl_t *c)
{
    if (c->sig_mode == 1 || c->sig_mode == 3) {
        return (c->mcs <= 7) ? 6500000u * (c->mcs + 1) : 65000000u;
    }
    if (c->rate <= 7) {
        static const uint32_t b[4] = { 1000000u, 2000000u, 5500000u, 11000000u };
        static const uint32_t g[8] = { 6000000u, 9000000u, 12000000u, 18000000u,
                                       24000000u, 36000000u, 48000000u, 54000000u };
        return (c->sig_mode == 0 && c->rate < 4) ? b[c->rate] : g[c->rate];
    }
    return (uint32_t)c->rate * 500000u;             /* 序号超出以上范围：按「速率 ÷ 500k」理解 */
}

/* 混听回调，跑在 WiFi 任务里：只做累加，每个发射端占一个槽。 */
static void survey_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) {
        return;                     /* 控制帧（ACK/RTS/CTS）很短，不单独计 */
    }
    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
    const wifi_pkt_rx_ctrl_t *c = &pkt->rx_ctrl;
    if (c->sig_len < 24) {
        return;                     /* 管理帧和数据帧的头部至少 24 字节，再短的不是完整帧 */
    }
    /* pkt->payload 就是 802.11 帧头：帧控制 2 + 持续时间 2 + addr1（接收方）6 + addr2（发射端）6，
     * 所以发射端在第 10 字节。2026-11-25 第一版写成第 6 字节，读出来是 addr1 的尾巴接 addr2 的
     * 开头（广播帧就是 ff:ff:ff:ff:xx:xx），白名单也因此没扣掉本链路自己的帧。 */
    const uint8_t *src = pkt->payload + 10;
    if (s_filter_tx && memcmp(src, s_peer_tx, 6) == 0) {
        return;                                    /* 本链路自己的帧不算 */
    }
    const uint32_t rate = survey_phy_rate(c);
    /* 帧长折成时间，再加约 150us 的前导码、帧间隔和 ACK */
    const uint32_t t_us = (uint32_t)((c->sig_len * 8ULL * 1000000u) / rate) + 150u;

    s_surv_frames++;
    s_surv_air_us += t_us;
    if (c->rssi > s_surv_rssi) {
        s_surv_rssi = c->rssi;
    }
    int free_slot = -1;
    for (int i = 0; i < SURV_SLOTS; i++) {
        if (s_surv_tx[i].frames == 0) {
            if (free_slot < 0) {
                free_slot = i;
            }
        } else if (memcmp(s_surv_tx[i].mac, src, 6) == 0) {
            s_surv_tx[i].frames++;
            s_surv_tx[i].air_us += t_us;
            if (c->rssi > s_surv_tx[i].rssi) {
                s_surv_tx[i].rssi = c->rssi;
            }
            return;
        }
    }
    if (free_slot >= 0) {
        memcpy(s_surv_tx[free_slot].mac, src, 6);
        s_surv_tx[free_slot].frames = 1;
        s_surv_tx[free_slot].air_us = t_us;
        s_surv_tx[free_slot].rssi = c->rssi;
    }
}

static void wifi_survey_run(void)
{
    const uint32_t ms = ALINK_WIFI_SURVEY_MS;
    const wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA,
    };
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(survey_cb);
    ESP_LOGW(TAG, "==== 2.4G 占用统计：每个信道听 %lu ms（占用按帧长和速率估，含信标）====",
             (unsigned long)ms);
    for (int ch = 1; ch <= 13; ch++) {
        memset(s_surv_tx, 0, sizeof(s_surv_tx));
        s_surv_frames = 0;
        s_surv_air_us = 0;
        s_surv_rssi = -128;
        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        esp_wifi_set_promiscuous(true);
        vTaskDelay(pdMS_TO_TICKS(ms));
        esp_wifi_set_promiscuous(false);

        if (s_surv_frames == 0) {
            ESP_LOGW(TAG, "  信道 %2d  没有帧", ch);
            continue;
        }
        ESP_LOGW(TAG, "  信道 %2d  %5lu 帧  占用约 %3lu‰  最强 %4d dBm",
                 ch, (unsigned long)s_surv_frames,
                 (unsigned long)(s_surv_air_us / ms), (int)s_surv_rssi);
        /* 按占用时间列出前 3 个发射端 */
        for (int k = 0; k < 3; k++) {
            int best = -1;
            for (int i = 0; i < SURV_SLOTS; i++) {
                if (s_surv_tx[i].frames && (best < 0 || s_surv_tx[i].air_us > s_surv_tx[best].air_us)) {
                    best = i;
                }
            }
            if (best < 0) {
                break;
            }
            ESP_LOGW(TAG, "      发射端 " MACSTR "  %5lu 帧  %3lu‰  %4d dBm",
                     MAC2STR(s_surv_tx[best].mac),
                     (unsigned long)s_surv_tx[best].frames,
                     (unsigned long)(s_surv_tx[best].air_us / ms), (int)s_surv_tx[best].rssi);
            s_surv_tx[best].frames = 0;
        }
    }
    esp_wifi_set_promiscuous_rx_cb(NULL);
    ESP_LOGW(TAG, "==== 占用统计结束（本链路用的是信道 %d）====", ALINK_WIFI_CHANNEL);
}
#endif

/* 解析 menuconfig 里的发送方白名单（Component config → Audio link → 发射端 MAC 地址）。
 * 留空 = 不校验，和没有这个选项时的行为一致。
 * 填了但格式非法时不启用，并且大声报出来：白名单是可选的加固，
 * 不该因为一个拼写错误让整机没声音。 */
static void tx_mac_parse(void)
{
    s_filter_tx = (CONFIG_ALINK_TX_MAC[0] != '\0') &&
                  alink_mac_from_str(CONFIG_ALINK_TX_MAC, s_peer_tx);
    if (CONFIG_ALINK_TX_MAC[0] != '\0' && !s_filter_tx) {
        ESP_LOGE(TAG, "CONFIG_ALINK_TX_MAC=\"%s\" 格式非法，本次不校验发送方",
                 CONFIG_ALINK_TX_MAC);
    }
}

static void wifi_espnow_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* 纯 STA，不连路由器，只用来把信道钉在 ALINK_WIFI_CHANNEL。
     *
     * 这里曾经跑过 APSTA + HTTP 调参面板，结果是丢包明显变多：
     * SoftAP 的 beacon 是广播、HTTP 响应是突发，两者都在和 5ms 一包的
     * ESP-NOW 抢同一个射频；11Mbps 固定速率下空口余量本来就薄。
     * 调参已改回编译期宏，射频上只留 ESP-NOW 一种业务。 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* 两个启动诊断都要先知道发射端的 MAC，才好把本链路自己的帧从统计里去掉。 */
    tx_mac_parse();
#if ALINK_WIFI_SCAN
    wifi_scan_print();
#endif
#if ALINK_WIFI_SURVEY
    wifi_survey_run();
#endif
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));       /* 关省电，这项最能压延迟 */
    ESP_ERROR_CHECK(esp_wifi_set_channel(ALINK_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(56));        /* 14 dBm */

    /* 16 项 × 976 字节，不分割项。够吸收 WiFi 任务被抢占几十毫秒的情况。 */
    s_rb = xRingbufferCreate(16 * (ALINK_HDR_SIZE + ALINK_PAYLOAD_BYTES),
                             RINGBUF_TYPE_NOSPLIT);
    if (!s_rb) {
        ESP_LOGE(TAG, "环形缓冲创建失败");
        return;
    }

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    esp_err_t r = esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_11M_L);
    if (r != ESP_OK) ESP_LOGW(TAG, "config_espnow_rate: %s", esp_err_to_name(r));
#pragma GCC diagnostic pop

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (s_filter_tx) {
        ESP_LOGW(TAG, "ESP-NOW 就绪：本机 " MACSTR " 信道 %d，只收 " MACSTR " 的单播包",
                 MAC2STR(mac), ALINK_WIFI_CHANNEL, MAC2STR(s_peer_tx));
    } else {
        ESP_LOGW(TAG, "ESP-NOW 就绪：本机 " MACSTR " 信道 %d，等发送端的单播包（不校验发送方）",
                 MAC2STR(mac), ALINK_WIFI_CHANNEL);
    }
}

/* ---------- 收包 ---------- */

static void rx_task(void *arg)
{
    static uint8_t buf[ALINK_HDR_SIZE + ALINK_PAYLOAD_BYTES];
    uint32_t got = 0, lost = 0, overrun = 0, filled = 0, dup = 0, pad = 0;
    uint16_t last_seq = 0;
    bool have_last = false;
    int64_t last_report = esp_timer_get_time();

    /* 缓冲深度已由 on_recv 里的环形缓冲给定（16 项），这里不再有 socket 参数。 */

    while (1) {
        size_t n = 0;
        uint8_t *item = xRingbufferReceive(s_rb, &n, portMAX_DELAY);
        if (!item) {
            continue;
        }
        if (n < (size_t)(ALINK_HDR_SIZE + ALINK_PAYLOAD_BYTES)) {
            vRingbufferReturnItem(s_rb, item);
            continue;
        }
        memcpy(buf, item, ALINK_HDR_SIZE + ALINK_PAYLOAD_BYTES);
        vRingbufferReturnItem(s_rb, item);

        alink_hdr_t *h = (alink_hdr_t *)buf;
        if (h->magic != ALINK_MAGIC || h->type != ALINK_MSG_AUDIO) {
            continue;
        }

        if (h->flags & ALINK_FLAG_PAD) {
            /* 发送端自己承认这包尾巴是补的（UAC 断供/半包）。
             * 只要这个数在涨，爆音就和无线无关，得去查 USB 音源侧。 */
            pad++;
        }

        if (have_last) {
            /* 用 int16 差值：seq 回绕时比较仍然正确，也能区分「旧包」和「空洞」。
             * 迟到重到的包 seq 不比上一包新，diff <= 0，直接丢掉。 */
            int16_t diff = (int16_t)(h->seq - last_seq);

            if (diff < -100) {
                /* 对端重启，seq 从头来。不能当重复包一路丢掉，重置基准 */
                ESP_LOGW(TAG, "seq 跳回 %u→%u（对端重启？）", last_seq, h->seq);
                have_last = false;
            } else if (diff <= 0) {
                dup++;      /* 冗余副本，或乱序迟到的旧包 */
                continue;
            } else {
                uint16_t gap = (uint16_t)(diff - 1);
                if (gap > 0 && gap < 50) {
                    lost += gap;
                    /* 补空占位，让后续包落在正确的时间位置上。
                     * 不补的话每次丢包都是 5ms 的时序塌陷。
                     * 占位包播放时走 PLC 外推，不是静音。 */
                    uint16_t fill = gap > ALINK_MAX_FILL ? ALINK_MAX_FILL : gap;
                    for (uint16_t i = 0; i < fill; i++) {
                        if (q_push(NULL, true)) {
                            filled++;
                        }
                    }
                }
            }
        }
        last_seq = h->seq;
        have_last = true;

        if (!q_push(buf + ALINK_HDR_SIZE, false)) {
            overrun++;   /* 队列满：说明 RX 消费不过来，水位闭环会慢慢把它压下去 */
        }
        got++;

        int64_t now = esp_timer_get_time();

        /* ---- 测实际帧率，让 I2S 时钟跟上去 ----
         *
         * 前两个约束是实测踩出来的：
         *
         * 1. **窗口必须够长**。4 秒窗口只有 800 个包，少一个包测出来就偏低
         *    60Hz —— 实测安静时会读出 47715、47966 这种值，被当成「音源变了」
         *    去重配时钟，而重配现在要走 30ms 淡出 + 15ms 淡入，等于凭空凹一下。
         *    30 秒窗口把单包噪声压到 8Hz。
         * 2. **窗口内有丢包就作废**。丢包/补位会直接拉低帧计数，测出来必然偏低。
         * 3. **TX 的补位包不算数**。补位包是 TX 按自己的定时器发的，节奏是 TX 晶振的
         *    200 包/秒，不代表主机帧率。不排除的话，静音期间测出来的「音源帧率」其实是
         *    TX 的发包率：对 47.93k 的盒子，每次暂停后 I2S 会被拉回 48000，恢复播放
         *    30~60 秒后再拉回去 —— 那次 30ms 淡出加 15ms 淡入正好落在音乐中间。
         *    所以一见到补位包，窗口就清零重来。
         *
         * 阈值 60Hz（0.125%）：PC 的 48000.2、测量噪声都在门槛内，不动；
         * 盒子那种 47.93k（差 70Hz）才动作。
         * 真正的重配仍由 play_task 在包边界执行（只有它能碰 I2S 通道）。 */
        if (h->flags & ALINK_FLAG_PAD) {
            s_rate_frames = 0;
            s_rate_t0     = now;
            s_rate_bad0   = lost + filled;
            s_rate_prev   = 0;      /* 作废的窗口不拿去和下一个窗口比抖动 */
        }
        s_rate_frames += (h->flags & ALINK_FLAG_PAD) ? 0 : ALINK_FRAMES_PER_PKT;
        if (s_rate_t0 == 0) {
            s_rate_t0 = now;
            s_rate_bad0 = lost + filled;
        } else if (now - s_rate_t0 >= 30000000) {
            const double span = (double)(now - s_rate_t0) / 1e6;
            const uint32_t bad = lost + filled;
            const int target = (int)((double)s_rate_frames / span + 0.5);
            int diff = target - (int)s_i2s_rate;
            if (diff < 0) {
                diff = -diff;
            }
            int jit = target - s_rate_prev;
            if (jit < 0) {
                jit = -jit;
            }
            if (bad == s_rate_bad0 && target > 40000 && target < 52000 &&
                diff >= 60 && s_rate_prev != 0 && jit < 20) {
                s_clk_req = target;
            }
            s_rate_prev   = target;
            s_rate_frames = 0;
            s_rate_t0     = now;
            s_rate_bad0   = bad;
        }

        if (now - last_report >= 1000000) {
            /* ESP-NOW 下没有「已关联的 AP」可查，RSSI 来自收包回调的 rx_ctrl */
            const int rssi = s_peer_rssi;
            int32_t pk = s_peak;
            s_peak = 0;
            const double dbfs = (pk > 0) ? 20.0 * log10((double)pk / 32768.0) : -120.0;
            /* 实测输出帧率：反映 I2S 时钟的真实频率 */
            const uint32_t of = s_out_frames;
            const double out_rate = (s_out_t0 && now > s_out_t0)
                                    ? (double)(of - s_out_prev) * 1000000.0 / (double)(now - s_out_t0)
                                    : 0.0;
            s_out_prev = of;
            s_out_t0   = now;
            /* 跳变字段放最后：前面几项是「链路/缓冲」层面的，跳变是
             * 「波形」层面的。链路全绿而跳变在涨，就是数据或硬件的问题。
             *
             * `DMA余` = on_sent 次数×240 − 应用层写入帧数。
             * 正常应恒定（只是启动时预填充几个空描述符）；它随时间增大
             * 就说明硬件缺数据、重播了旧块 —— 这层错误应用层完全看不见。 */
            {
                const int32_t dma_gap =
                    (int32_t)(s_dma_done * (uint32_t)ALINK_FRAMES_PER_PKT) -
                    (int32_t)s_out_frames;

            /* 低水位重缓冲的事件行由这里报：播放任务发现时队列只剩 10ms，打不得日志。 */
            static uint32_t low_seen = 0;
            if (s_low_rebuf != low_seen) {
                low_seen = s_low_rebuf;
                ESP_LOGW(TAG, "队列只剩 %d 包（低水位），已淡出并重缓冲（累计 %lu 次）",
                         ALINK_Q_LOW, (unsigned long)s_low_rebuf);
            }

            ESP_LOGI(TAG, "包=%lu 丢=%lu(%.2f%%) 重复=%lu 补位=%lu 占位=%lu PLC=%lu 溢出=%lu 漂移丢=%lu 重采样=%+.2f%% I2S错=%lu 欠载=%lu 重缓冲=%lu 中止=%lu 队列=%d(%d~%d) 输出=%.1f帧/秒 峰=%.1fdBFS RSSI=%d 跳变=%lu(边%lu/中%lu) 最大=%ld 零串=%lu帧(%lums) DMA余=%ld 相位钳=%lu 醒来=%lu",
                         (unsigned long)got, (unsigned long)lost,
                         got ? lost * 100.0 / (got + lost) : 0.0,
                         (unsigned long)dup, (unsigned long)pad, (unsigned long)filled,
                         (unsigned long)s_plc_total,
                         (unsigned long)overrun,
                         (unsigned long)s_drift_drop,
                         (double)(s_src_ratio - 65536) * 100.0 / 65536.0,
                         (unsigned long)s_i2s_err,
                         (unsigned long)s_underrun, (unsigned long)s_low_rebuf, (unsigned long)s_low_abort,
                         q_depth(), s_q_min, s_q_max,
                         out_rate, dbfs, rssi,
                         (unsigned long)(s_jump_edge + s_jump_mid),
                         (unsigned long)s_jump_edge, (unsigned long)s_jump_mid,
                         (long)s_jump_max,
                         (unsigned long)s_zero_run_max,
                         (unsigned long)(s_zero_run_max * 1000 / ALINK_I2S_RATE),
                         (long)dma_gap,
                         (unsigned long)s_phase_clamp,
                         (unsigned long)s_wake_fade);
                s_zero_run_max = 0;      /* 每秒窗口清零，只看这一秒的最长零串 */                s_q_min = 1 << 20;
                s_q_max = 0;
            }
            last_report = now;
        }
    }
}

/* ---------- 播放 ---------- */

static void play_task(void *arg)
{
    static uint8_t pcm[ALINK_PAYLOAD_BYTES];
    bool started = false;
    bool is_fill = false;
    bool rebuf = false;         /* 上一次停播是低水位重缓冲：起播时要走长淡入 */
    float ema = 0.0f;
    uint32_t starve = 0;

    s_xfade = true;   /* 首次输出从静音淡入 */
    s_fade_in_left = ALINK_FADE_IN_FRAMES;

    while (1) {
        /* ---- 把 I2S 时钟对齐到音源 ----
         *
         * 只有本任务能碰 I2S 通道，所以重配放在这里、包边界处执行。
         * 但 disable/enable 会把 DMA 停一下，而 DMA 里通常还压着
         * 10~15ms 已经写进去的音频 —— 直接停，DAC 那头听到的就是
         * 「波形从中间被砍断」，切歌那声爆音有一半来自这里。
         *
         * 所以分两步：先接单，再按 6 包（30ms）把真实音频线性压到 0，
         * 等最后一包已经写成静音，才真正 disable/enable。 */
        if (s_clk_req != 0) {
            if (s_clk_pending == 0 && s_clk_fade == 0) {
                s_clk_pending = (int)s_clk_req;
                s_clk_fade    = ALINK_CLK_FADE_PKTS;
            }
            s_clk_req = 0;      /* 淡出期间来的新请求丢掉，切完重新测 */
        }
        if (s_clk_pending != 0 && s_clk_fade == 0) {
            const int target = s_clk_pending;
            s_clk_pending = 0;
            i2s_channel_disable(s_i2s_tx);
            i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)target);
#if ALINK_I2S_CLK_240M
            /* 重配时也必须指定同一时钟源，否则会悄悄退回 160M，
             * 白白把这个实验的变量又搅回去。 */
            clk.clk_src       = I2S_CLK_SRC_PLL_240M;
            clk.mclk_multiple = I2S_MCLK_MULTIPLE_256;
#endif
            const esp_err_t r = i2s_channel_reconfig_std_clock(s_i2s_tx, &clk);
            i2s_channel_enable(s_i2s_tx);
            if (r == ESP_OK) {
                s_i2s_rate = (uint32_t)target;
                ESP_LOGW(TAG, "I2S 时钟跟随音源 -> %d Hz（重采样比可归 1）", target);
            } else {
                ESP_LOGE(TAG, "I2S 时钟重配失败 %d，保持 %lu Hz", (int)r, (unsigned long)s_i2s_rate);
            }
            /* 恢复时用长淡入，不是 2ms 硬接 —— 停顿过后 DAC 那头是静音 */
            s_fade_in_left = ALINK_FADE_IN_FRAMES;
            s_xfade = false;
        }

        int d = q_depth();
        if (d < s_q_min) s_q_min = d;
        if (d > s_q_max) s_q_max = d;
        /* 时间常数从约 1 秒缩到约 100ms（200 次/秒采样 × 0.05）。
         * 原来 1 秒的滞后配 240ms 的缓冲，控制环根本来不及看住队列，
         * 等它反应过来队列已经撞墙了 —— 这也是振荡的一半原因。 */
        ema = ema * 0.95f + (float)d * 0.05f;

        if (!started) {
            /* 门槛 = 目标水位 − 死区，也就是闭环的稳态水位（见 ALINK_JITTER_PKTS）。
             * 从这个水位起播，闭环一开始就落在死区里，不用再花几十秒把水补上来。 */
            if (d >= ALINK_JITTER_PKTS) {
                started = true;
                starve = 0;
                /* EMA 必须从当前水位起步。从 0 冷启动的话，头几百毫秒 ema 都在目标
                 * 之下，闭环会先把重采样比压到 1 以下，平白多出一段插值。 */
                ema = (float)d;
                s_xfade = true;
                /* 低水位淡出可能还剩一两块没写（断流抢在前面把 started 清掉了，或者
                 * 积压的包一下子涌上来），不清掉的话起播头两块会被压低甚至抹成静音。 */
                s_low_fade = 0;
                s_low_done = false;
                if (rebuf) {
                    /* 重缓冲之后接的是一段静音，从静音到满幅要走 15ms 长淡入（同断流恢复） */
                    rebuf = false;
                    s_fade_in_left = ALINK_FADE_IN_FRAMES;
                }
                ESP_LOGI(TAG, "起播 队列=%d", d);
            } else {
                /* 预缓冲期间照常写静音，保持 I2S 时钟连续 —— 不写就会循环播旧数据 */
                emit(NULL);
                continue;
            }
        }

        /* ---- 低水位重缓冲（见 ALINK_Q_LOW）----
         * 队列只剩 ALINK_Q_LOW 包：不再一包包硬撑，按块淡出，写完最后一块就退回预缓冲。
         * 时钟重配自己有一段淡出，两段不叠在一起。
         * 这里不打日志：队列只剩 10ms，一条日志在串口上就要 10ms；由统计任务报（见 stats）。 */

        /* 中止：淡出还没写完，队列自己又涨回来了 —— 收包路径卡了一下、积压的包一起涌到，
         * 或者空口那头缓过来之后成批到达。这种情况本来就不用管，硬停 90ms 反而是凭空加一次停顿。
         * 2026-11-26 上板遇到过一次：收包路径卡了约 80ms（TX 的发包和 ACK 都正常），
         * 队列 16 → 2 → 20。中止之后用长淡入把刚压下去的电平拉回来。 */
        if ((s_low_fade > 0 || s_low_done) && d >= ALINK_Q_LOW + ALINK_LOW_ABORT) {
            s_low_fade = 0;
            s_low_done = false;
            s_low_abort++;
            s_fade_in_left = ALINK_FADE_IN_FRAMES;
        }
        if (s_low_fade == 0 && s_low_done == false &&
            s_clk_pending == 0 && s_clk_fade == 0 && d <= ALINK_Q_LOW) {
            s_low_fade = ALINK_LOW_FADE_PKTS;
        }
        if (s_low_done) {
            s_low_done = false;
            started = false;
            starve   = 0;
            rebuf    = true;
            s_low_rebuf++;           /* 淡出真的写完了才算一次 */
            continue;               /* 下一轮走上面的预缓冲分支：写静音，等水位 */
        }

        /* ---- 水位闭环：连续调节重采样比 ----
         * 原来靠「每包插/删 1 帧」是离散事件，约 150 次/秒 → 150Hz 电流音 + 噼啪
         * （2026-11-19 自检音实测确认）。改成连续比率：
         * 队列偏多 → ratio 变大（消耗更快）；偏少 → 变小。没有任何突变点。 */
        if (ema > (float)ALINK_Q_HIGH && d > ALINK_Q_HIGH) {
            q_drop();                  /* 真溢出：丢一整包兜底 */
            s_drift_drop++;
            s_xfade = true;
            /* 不在这里刷日志：水位高时一秒几百条，串口会把播放任务堵死 */
        }
        {
            /* 死区：队列在目标水位 ±ALINK_SRC_DEADBAND 之内时一律不做重采样。
             * ratio 精确等于 1.0 时 src_process 全程走整数帧直取分支，
             * 一个插值样本都不产生 —— 这是消除高频「调频干扰」的唯一办法
             * （只要 ratio 偏离 1，包边界就得拿镜像样本顶替，残差 -30dB 起步，
             * 加抽头也压不掉）。死区 3.0 是实测出来的音质底线。 */
            const float dev = ema - (float)ALINK_TARGET_Q;
            int32_t adj;
            if (dev > -ALINK_SRC_DEADBAND && dev < ALINK_SRC_DEADBAND) {
                adj = 0;
            } else {
                const float eff = dev - (dev > 0 ? ALINK_SRC_DEADBAND : -ALINK_SRC_DEADBAND);
                /* 增益 64→8。实测收发时钟差只有 0.011%（约 5.5 帧/秒），
                 * 用 64 的增益意味着队列偏 10 包就把调整量推到硬件上限，
                 * 于是「冲过头→反向冲」，队列在 1 和 24 之间来回撞。
                 * 8 的增益下，偏 10 包只给 0.07%，慢慢收敛，不产生音高可闻的摆动。
                 *
                 * 不要再为了省延时把增益提到 16：那样 ratio 每秒都在变，
                 * 插值器全程退不出来，小电平乐器就开始发糊。 */
                adj = (int32_t)(eff * 8.0f);
            }
            if (adj > ALINK_SRC_ADJ_LIMIT) {
                adj = ALINK_SRC_ADJ_LIMIT;
            }
            if (adj < -ALINK_SRC_ADJ_LIMIT) {
                adj = -ALINK_SRC_ADJ_LIMIT;
            }
            s_src_ratio = 65536 + adj;
        }

        if (q_pop(pcm, &is_fill)) {
            /* 断流超过 20ms 才恢复 → 中间是真静音，必须走长淡入。
             * 短缺口（PLC 接续、漂移丢包）仍交给 2ms 接缝淡化。 */
            if (starve >= ALINK_RESUME_GAP_PKTS && s_fade_in_left == 0) {
                s_fade_in_left = ALINK_FADE_IN_FRAMES;
            }
            starve = 0;
            if (is_fill) {
                emit_plc();          /* 占位包走外推，不补静音 */
            } else {
                s_plc_run = 0;       /* 收到真数据，PLC 链断开 */
                /* PLC 的基准只在这里更新：始终是「最后一个真实包」。
                 * 如果让 PLC 结果覆盖基准，衰减系数会累积相乘（0.6×0.35×0.2），
                 * 第 4 次只剩 4%，曲线完全不是设计的样子。 */
                memcpy(s_prev, pcm, ALINK_PAYLOAD_BYTES);
                emit((const int16_t *)pcm);
            }
        } else {
            /* 断流：DMA 里还有 I2S_DMA_DESC × 5ms 的余量，这里接 PLC，不产生爆音 */
            s_underrun++;
            emit_plc();
            if (++starve == ALINK_STARVE_RESET) {
                started = false;
                ESP_LOGW(TAG, "断流 %dms，退回预缓冲（欠载 %lu 次）",
                         ALINK_STARVE_RESET * ALINK_PKT_MS, (unsigned long)s_underrun);
            }
        }
    }
}

/* ---------- 关于「哑链路自愈」 ----------
 *
 * UDP 版需要它：AP 侧静默踢过 STA 后，C3 收不到 DISCONNECTED 事件就不会重连，
 * 而 TX 那边永远等不到 HELLO 注册 —— 表现是「突然没声、按 reset 也没用」。
 *
 * ESP-NOW 没有关联、没有 DHCP、没有 HELLO，收不到包只能是发送端没在发，
 * 因此这一段逻辑整块删掉。链路「恢复」是自动的：只要 TX 开始发包，
 * 回调就会重新填环，不需要任何握手。
 */

/* ---------- 启动 ---------- */

void app_main(void)
{
    s_q.mux = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if ALINK_PIN_SCAN
    /* 诊断模式：把测试音轮流送到候选引脚组，定位 DAC 到底接在哪三个脚上。
     * 该函数不会返回。定位完把 ALINK_PIN_SCAN 改回 0 重烧。 */
    pin_scan_run();
#endif

    sinc_init();    /* 生成 polyphase sinc 系数表，约 10ms，正常播放也要 */

#if ALINK_SRC_DIAG
    /* 上电先花几十毫秒量一下插值器本身的失真，把「调频干扰」变成 dB 数字。
     * 不发声，不影响正常播放，结果直接看串口日志。 */
    src_selftest();
#endif

    i2s_init();
    wifi_espnow_init();   /* 起 WiFi + ESP-NOW + 环形缓冲，不开 socket */

    /* 双核分配 —— 这是 S3 相对 C3 的结构性优势（C3 是单核 RISC-V，
     * 播放任务必然跟 WiFi 抢同一个 CPU）。
     *
     *   core0：WiFi 驱动 / ESP-NOW 回调（回调里只做一次 ringbuffer 投递）
     *   core1：rx + play —— 播放阻塞在 i2s_channel_write，rx 把环里的包搬进队列
     *
     * 为什么 rx 也要搬到 core1（2026-11-21 改）：
     * 原来 rx 在 core0、优先级 8，排在 WiFi（23）和 lwIP 后面。实测 168 秒里
     * 有几次 rx 被压住 60ms 以上：这期间 WiFi 回调把所有包塞进环形缓冲（丢包仍为 0），
     * 但队列没人补，播放任务把队列吃到 1 → 欠载 27 次 → 听感就是零星爆音。
     * 队列水位在 1~15 之间大幅摆动、下一秒又蹦回 14，正是「先饿死再一次性灌满」
     * 的典型形状。搬到 core1 后 rx 不再和 WiFi 抢 CPU，它只负责搬数据，几乎不耗时间。
     *
     * 优先级：play 12 > rx 9。play 阻塞在 I2S 上会主动让出，rx 有的是时间片；
     * 反过来让 rx 更高会推迟 I2S 喂数，得不偿失。 */
    /* rx 优先级 9→11：只比 play（12）低一级。
     * 原来差 3 级，play 一旦就绪 rx 就完全抢不到 CPU，
     * ringbuffer 里的包搬不进队列，队列看着见底就触发欠载 —— 而实际上
     * 数据一直在 ringbuffer 里躺着。rx 每次只做一次 memcpy + 入队，
     * 几十微秒就让出，不会饿死 play。 */
    xTaskCreatePinnedToCore(rx_task,    "rx",    4096, NULL, 11, NULL, 1);
    xTaskCreatePinnedToCore(play_task,  "play",  4096, NULL, 12, NULL, 1);

    ESP_LOGI(TAG, "接收端就绪（S3 / ESP-NOW / 播放任务独占 core1），等音频包");
}
