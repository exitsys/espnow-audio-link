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
 * ---- 以下是 C3 版的原始说明，链路逻辑完全一致 ----
 * 连发送端的热点 → 发 HELLO 报备 → 收音频包进抖动缓冲 → I2S 输出到 PCM5102A。
 *
 * 2026-11-18 重写播放侧，解决实测的「连接不稳 + 爆音」。三个根因：
 *
 *   1. 时钟漂移：TX 的 32kHz 来自 S3 晶振，RX 的 32kHz 来自 C3 的 I2S 分频，
 *      两者差 20~50ppm，队列必然缓慢溢出或见底。
 *      → 水位闭环：EMA 跟队列深度，偏了就丢一包 / 补一包静音。
 *
 *   2. 抽空后 DMA 循环播旧数据：auto_clear_after_cb 是 false，DMA 会一直重复
 *      最后一包内容，听感是「哒哒哒」。
 *      → I2S 时钟永不断流，没数据就写静音包；DMA 深度收到 20ms。
 *
 *   3. 丢包直接跳过 5ms：波形阶跃，每丢一包一声「咔」。
 *      → 按 seq 空洞补占位包，时序不错位。
 *
 *   4. 占位包如果填静音，依然是阶跃，还是「咔」。
 *      → PLC：重复上一包输出并逐次衰减（×1.0/0.6/0.35/0.2，第 5 次归零），
 *        相位不连续由 emit() 的 2ms 交叉淡化抹平。
 *
 * 另外 TX 现在每包背靠背发两份，这里按 seq 去重（diff == 0 的丢掉）。
 * 双发和 PLC 是两层独立保险：双发挡住单次事件，PLC 兜住双发也救不回来的情况。
 *
 * 所有断点（起播 / 丢包 / 补包 / 断流）统一走 2ms 交叉淡化，不再有阶跃。
 */
#include <string.h>
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
#include "web_ui.h"

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

/* 播放侧采样率 48kHz，时钟源 PLL_160M —— 与 C3 完全一致。
 *
 * 这里要澄清一个曾经的误判：一度怀疑 C3 因为没有 APLL 而产生 pattern jitter。
 * 实际算一遍：160e6 / (48000×64) = 52 + 1/12，小数分母 12 在分频器上限 63
 * 以内，能精确表达，不产生周期性补偿。S3 的 I2S 硬件上同样没有 APLL 通道
 * （只有 XTAL / PLL_160M / PLL_240M / EXTERNAL），所以这一项两者等价，
 * **不是噼啪的差异来源**。
 *
 * 与盒子的残余失配（盒子约 47.93k vs 本地 48k，约 0.15%）仍由水位闭环的
 * 连续分数重采样吸收。复核方法：看 RX 日志里「欠载」「漂移丢」「重采样」
 * 三个数是否还在缓慢增长。 */
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
 * 0 = PLL_160M：默认值，13 + 1/64，每 64 个周期插一个补偿周期 = 192kHz 抖动
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

/* DMA 深度直接决定「用户看不见的额外延迟」：
 * i2s_channel_write 只要 DMA 有空位就立刻返回，所以队列里的包会被迅速灌进
 * DMA，队列深度本身不代表真实缓冲。4 × 160 帧 @32k = 20ms，够吸收 WiFi 抖动，
 * 又不至于把延迟撑到蓝牙的水平。 */
/* DMA 深度直接进端到端延时：dma_desc_num × dma_frame_num / fs。
 * 4×5ms = 20ms 是稳定档；低延时档 3×5ms = 15ms。
 * 减到 2 会只剩 10ms，play_task 被抢占 8ms 就硬件空转，不划算。 */
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

/* 最近一次收到音频包的时刻（us），只用于统计与日志 */
static volatile int64_t s_last_rx_us = 0;

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
        /* 生产者要绕满 24 个槽才会回到 i，这里 memcpy 早已结束，不会踩 */
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

static int16_t s_tail[ALINK_XFADE_FRAMES][2];   /* 上一次输出的最后 2ms */
static bool    s_xfade;                          /* 下次输出需要淡入 */
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

/* polyphase windowed-sinc 插值器参数。
 *
 * 为什么不用 4 点三次 Hermite：上电自测（ALINK_SRC_DIAG）实测，
 * ratio=-0.055% 时 15kHz 基波从 12000 掉到 8437（-3.05dB），
 * 2f 谐波 -44.2dB —— 4 点核的高频增益随分数相位剧烈变化，
 * 而分数相位每 5ms 扫一遍，等于每秒 200 次的幅度调制，
 * 听感就是「调频干扰」。8kHz 也有 0.55dB 的增益损失和 -52dB 谐波。
 *
 * 16 抽头 Kaiser 窗 sinc（β=8）把同条件失真压到 -70dB 以下，
 * 高频增益平坦到 0.1dB，代价是每样本 16 次乘加 + 8 帧历史。 */
/* 16 抽头 Kaiser 窗 sinc（β=8）：过渡带约 Fs/16 = 3kHz 宽，也就意味着
 * 从 21kHz 往下 3kHz（即 18kHz）就开始滚降，15kHz 已经在坡上。
 * 32 抽头把过渡带收到 Fs/32 = 1.5kHz（16.5kHz 才起滚），15kHz 落在通带内
 * 平坦区。代价是每样本 32 次乘加 + 16 帧历史。 */
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
static uint32_t s_rate_frames = 0;           /* 测速窗口内累计到达帧数 */
static int64_t  s_rate_t0     = 0;           /* 测速窗口起点 */
static int      s_rate_prev   = 0;           /* 上一窗口测出的帧率，用于去抖 */
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

/* ---------- 运行时可调参数（网页/串口改，不重编译） ----------
 *
 * 单独拆成 volatile 标量而不是一个结构体：core0 上的 HTTP 任务写、
 * core1 上的播放任务读，32 位对齐的标量在 Xtensa 上是单指令读写，
 * 不需要加锁也不会读到撕裂的值；结构体整体赋值反而可能被编译器拆成多次访问。
 *
 * 初始值仍取编译期宏，保证「不碰网页」时行为和原来完全一致。
 */
static volatile int32_t s_tune_target_q = ALINK_TARGET_Q;
static volatile float   s_tune_deadband = ALINK_SRC_DEADBAND;
static volatile float   s_tune_gain     = 8.0f;
static volatile int32_t s_tune_adj_lim  = ALINK_SRC_ADJ_LIMIT;

/* web 每秒读一次的状态快照（由 rx_task 在刷日志时填） */
static alink_stats_t s_stats;

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
 * 上电时算一次：512 个相位 × 16 抽头，Kaiser 窗 β=8。
 * 每行系数归一化到和为 1（DC 增益恒为 1），再压成 Q15 定点。
 *
 * 16 抽头的过渡带约 Fs/16 = 3kHz，15kHz 仍在通带内、增益平坦，
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
            /* 抽头中心在 k = SRC_HALF-1 = 7，对应 buf[i] */
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
 * 插值核是 16 抽头 windowed sinc，不是多项式：
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
            /* 恰好落在整数帧上，直接取，省掉 3 次 64 位乘 */
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

    /* ratio 精确为 1 时，把不足 0.1 帧的相位残差吸附到整帧。
     * 这一包之后所有采样点都落在整数帧上（走 f == 0 分支），彻底不再插值。
     * 吸附量 ≤ 0.1 帧 ≈ 2µs，一次性事件，不可闻。负相位不吸附（下标含义已变）。 */
    if (s_src_ratio == 65536 && s_src_phase >= 0) {
        const int32_t rem = s_src_phase & 0xFFFF;
        if (rem != 0 && rem < 6554) {
            s_src_phase -= rem;
        }
    }

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
        memcpy(out, src, ALINK_PAYLOAD_BYTES);
    } else {
        memset(out, 0, ALINK_PAYLOAD_BYTES);
    }

    /* 样本级漂移补偿改成连续分数重采样：没有任何离散突变点。
     * 原来"每包插 1 帧"会制造 150Hz 脉冲串（实测听感 = 电流音 + 噼啪）。 */
    {
        const bool reset = s_xfade;      /* 断点恢复时相位归零，避免历史帧错位 */
        int16_t nxt[SRC_HALF * ALINK_CHANNELS];
        const int16_t *pnext = q_peek(nxt, SRC_HALF) ? nxt : NULL;
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

    /* 固定电平校准 + 抖动。
     * 本版沿用老哥的设计，没有实现 UAC 音量（set_volume_cb 只打日志），
     * 而 PCM5102A 满刻度是 2.1Vrms，对有源音箱是 12~20dB 的结构性过载。
     * 在写 I2S 前统一压一层，避免「开一点就很大」。改 ALINK_OUT_ATTEN_DB 可调。
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
     * 代价是最多 5ms 额外延迟（队列有 9 包 ≈ 45ms 缓冲，无所谓）。 */
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
    if (info && info->rx_ctrl) {
        s_peer_rssi = info->rx_ctrl->rssi;
    }
    /* 0 超时：环形缓冲满就丢这一包，交给上层 PLC 补 ——
     * 语义等同于 UDP 版的 recvbuf 溢出，绝不在这里阻塞 WiFi 任务。 */
    xRingbufferSend(s_rb, data, (size_t)len, 0);
}

static void wifi_espnow_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* AP 的 netif 必须在 wifi_init 之前建好，否则 SoftAP 拿不到 IP，
     * 192.168.4.1 会一直 ping 不通。ESP-NOW 本身不需要 IP，
     * 但调参面板要走 HTTP，就得有这一层。 */
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    /* APSTA：STA 侧给 ESP-NOW（不连路由器，只用来定信道），
     * AP 侧给手机/电脑连进来调参。
     *
     * 信道必须是 ALINK_WIFI_CHANNEL，两边一致 —— ESP-NOW 要求收发同信道，
     * 一旦 AP 起在别的信道，TX 那边就彻底联系不上了。
     * beacon 间隔放到 500ms（默认 100ms）：beacon 是广播，会占空口时间，
     * 拉长一点能少干扰 5ms 一包的音频流。 */
    wifi_config_t ap_cfg = { 0 };
    strcpy((char *)ap_cfg.ap.ssid, "alink-rx");
    strcpy((char *)ap_cfg.ap.password, "********");
    ap_cfg.ap.ssid_len        = (uint8_t)strlen("alink-rx");
    ap_cfg.ap.channel         = ALINK_WIFI_CHANNEL;
    ap_cfg.ap.authmode        = WIFI_AUTH_WPA2_PSK;
    ap_cfg.ap.max_connection  = 2;
    ap_cfg.ap.beacon_interval = 500;
    ap_cfg.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
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
    ESP_LOGW(TAG, "ESP-NOW 就绪：本机 " MACSTR " 信道 %d，等发送端的单播包",
             MAC2STR(mac), ALINK_WIFI_CHANNEL);
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
            /* 用 int16 差值：seq 回绕时比较仍然正确，也能区分「重复」和「空洞」。
             * TX 双发时第二份的 seq 与上一包相同，diff == 0。 */
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
        s_last_rx_us = esp_timer_get_time();

        int64_t now = esp_timer_get_time();

        /* ---- 测实际帧率，让 I2S 时钟跟上去 ----
         * 4 秒窗口。要求连续两个窗口测出相近的值（差 < 2Hz）才动作，
         * 避免把测量噪声当成速率偏差来回改时钟。
         * 真正的重配由 play_task 在包边界执行（只有它能碰 I2S 通道）。 */
        s_rate_frames += ALINK_FRAMES_PER_PKT;
        if (s_rate_t0 == 0) {
            s_rate_t0 = now;
        } else if (now - s_rate_t0 >= 4000000) {
            const double span = (double)(now - s_rate_t0) / 1e6;
            const int target = (int)((double)s_rate_frames / span + 0.5);
            int diff = target - (int)s_i2s_rate;
            if (diff < 0) {
                diff = -diff;
            }
            int jit = target - s_rate_prev;
            if (jit < 0) {
                jit = -jit;
            }
            if (target > 40000 && target < 52000 && diff >= 2 &&
                s_rate_prev != 0 && jit < 2) {
                s_clk_req = target;
            }
            s_rate_prev   = target;
            s_rate_frames = 0;
            s_rate_t0     = now;
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

                /* 顺手给网页快照一份。放在日志旁边是刻意的：两边数字永远一致，
                 * 不用为了 web 再算一遍，也就不会出现「网页和串口对不上」的扯皮。 */
                s_stats.pkt         = got;
                s_stats.lost        = lost;
                s_stats.dup         = dup;
                s_stats.pad         = pad;
                s_stats.filled      = filled;
                s_stats.plc         = s_plc_total;
                s_stats.overrun     = overrun;
                s_stats.drift_drop  = s_drift_drop;
                s_stats.i2s_err     = s_i2s_err;
                s_stats.underrun    = s_underrun;
                s_stats.phase_clamp = s_phase_clamp;
                s_stats.queue       = q_depth();
                s_stats.q_min       = s_q_min;
                s_stats.q_max       = s_q_max;
                s_stats.ratio_pct   = (float)(s_src_ratio - 65536) * 100.0f / 65536.0f;
                s_stats.out_rate    = (float)out_rate;
                s_stats.peak_dbfs   = (float)dbfs;
                s_stats.rssi        = s_peer_rssi;
                s_stats.zero_run_max= s_zero_run_max;
                s_stats.dma_gap     = dma_gap;
                s_stats.jump_edge   = s_jump_edge;
                s_stats.jump_mid    = s_jump_mid;
                s_stats.jump_max    = s_jump_max;
                s_stats.i2s_rate    = (int32_t)s_i2s_rate;

                ESP_LOGI(TAG, "包=%lu 丢=%lu(%.2f%%) 重复=%lu 补位=%lu 占位=%lu PLC=%lu 溢出=%lu 漂移丢=%lu 重采样=%+.2f%% I2S错=%lu 欠载=%lu 队列=%d(%d~%d) 输出=%.1f帧/秒 峰=%.1fdBFS RSSI=%d 跳变=%lu(边%lu/中%lu) 最大=%ld 零串=%lu帧(%lums) DMA余=%ld 相位钳=%lu",
                         (unsigned long)got, (unsigned long)lost,
                         got ? lost * 100.0 / (got + lost) : 0.0,
                         (unsigned long)dup, (unsigned long)pad, (unsigned long)filled,
                         (unsigned long)s_plc_total,
                         (unsigned long)overrun,
                         (unsigned long)s_drift_drop,
                         (double)(s_src_ratio - 65536) * 100.0 / 65536.0,
                         (unsigned long)s_i2s_err,
                         (unsigned long)s_underrun, q_depth(), s_q_min, s_q_max,
                         out_rate, dbfs, rssi,
                         (unsigned long)(s_jump_edge + s_jump_mid),
                         (unsigned long)s_jump_edge, (unsigned long)s_jump_mid,
                         (long)s_jump_max,
                         (unsigned long)s_zero_run_max,
                         (unsigned long)(s_zero_run_max * 1000 / ALINK_I2S_RATE),
                         (long)dma_gap,
                         (unsigned long)s_phase_clamp);
                s_zero_run_max = 0;      /* 每秒窗口清零，只看这一秒的最长零串 */
                s_q_min = 1 << 20;
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
    float ema = 0.0f;
    uint32_t starve = 0;

    s_xfade = true;   /* 首次输出从静音淡入 */

    while (1) {
        /* ---- 把 I2S 时钟对齐到音源 ----
         * 只有本任务能碰 I2S 通道，所以重配放在这里、包边界处执行。
         * disable/enable 会让 DMA 停一下，用淡入盖掉那一下爆音。 */
        if (s_clk_req != 0) {
            const int target = (int)s_clk_req;
            s_clk_req = 0;
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
            s_xfade = true;
        }

        int d = q_depth();
        if (d < s_q_min) s_q_min = d;
        if (d > s_q_max) s_q_max = d;
        /* 时间常数从约 1 秒缩到约 100ms（200 次/秒采样 × 0.05）。
         * 原来 1 秒的滞后配 240ms 的缓冲，控制环根本来不及看住队列，
         * 等它反应过来队列已经撞墙了 —— 这也是振荡的一半原因。 */
        ema = ema * 0.95f + (float)d * 0.05f;

        if (!started) {
            if (d >= ALINK_JITTER_PKTS) {
                started = true;
                starve = 0;
                /* EMA 必须从当前水位起步。从 0 冷启动的话，前几百毫秒 ema 都
                 * 低于 ALINK_Q_LOW，会被误判成「RX 偏快」，一路补静音把队列抽干。 */
                ema = (float)d;
                s_xfade = true;
                ESP_LOGI(TAG, "起播 队列=%d", d);
            } else {
                emit(NULL);          /* 保持 I2S 时钟连续，不写静音就会循环播旧数据 */
                continue;
            }
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
            /* 死区：队列在目标水位 ±0.75 包之内时一律不做重采样。
             * ratio 精确等于 1.0 时 src_process 全程走整数帧直取分支，
             * 一个插值样本都不产生 —— 这是消除高频「调频干扰」的唯一办法
             * （只要 ratio 偏离 1，包边界就得拿镜像样本顶替，残差 -30dB 起步，
             * 加抽头也压不掉，实测见 src_selftest）。 */
            const float dev = ema - (float)s_tune_target_q;
            int32_t adj;
            if (dev > -s_tune_deadband && dev < s_tune_deadband) {
                adj = 0;
            } else {
                const float eff = dev - (dev > 0 ? s_tune_deadband : -s_tune_deadband);
                /* 增益 64→8。实测收发时钟差只有 0.011%（约 5.5 帧/秒），
                 * 用 64 的增益意味着队列偏 10 包就把调整量推到硬件上限，
                 * 于是「冲过头→反向冲」，队列在 1 和 24 之间来回撞。
                 * 8 的增益下，偏 10 包只给 0.07%，慢慢收敛，不产生音高可闻的摆动。
                 *
                 * 2026-11-21 回到 8：低延时档曾提到 16 去追盒子 0.146% 的失配，
                 * 但增益 16 + 死区 1.5 的组合让 ratio 每秒都在变，
                 * 插值器全程不退出。宁可收敛慢一点，也要让 ratio 落在 1.0 上直通。 */
                adj = (int32_t)(eff * s_tune_gain);
            }
            if (adj > s_tune_adj_lim) {
                adj = s_tune_adj_lim;
            }
            if (adj < -s_tune_adj_lim) {
                adj = -s_tune_adj_lim;
            }
            s_src_ratio = 65536 + adj;
        }

        if (q_pop(pcm, &is_fill)) {
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
            /* 断流：DMA 里还有 20ms 余量，这里接 PLC，不产生爆音 */
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

/* ---------- 对外接口（供网页调参） ---------- */

void alink_get_stats(alink_stats_t *out)
{
    if (!out) return;
    *out = s_stats;
    out->queue = q_depth();   /* 用瞬时值，快照是一秒前的，看波动不够新 */
}

void alink_get_tune(alink_tune_t *out)
{
    if (!out) return;
    out->target_q  = s_tune_target_q;
    out->deadband  = s_tune_deadband;
    out->gain      = s_tune_gain;
    out->adj_limit = s_tune_adj_lim;
}

bool alink_set_tune(const alink_tune_t *in)
{
    if (!in) return false;

    int32_t tq = in->target_q;
    if (tq < 8)  tq = 8;
    if (tq > 40) tq = 40;

    /* `!(x >= lo)` 而不是 `x < lo`：顺手把 NaN 也挡掉，
     * 前端传个空字符串 atof 成 0 还好，传 "nan" 就会把控制环搞死。 */
    float db = in->deadband;
    if (!(db >= 0.5f)) db = 0.5f;
    if (db > 8.0f)     db = 8.0f;

    float g = in->gain;
    if (!(g >= 1.0f)) g = 1.0f;
    if (g > 40.0f)    g = 40.0f;

    int32_t lim = in->adj_limit;
    if (lim < 20)  lim = 20;
    if (lim > 600) lim = 600;

    const bool changed = (tq != s_tune_target_q) || (db != s_tune_deadband) ||
                         (g != s_tune_gain)      || (lim != s_tune_adj_lim);

    s_tune_target_q = tq;
    s_tune_deadband = db;
    s_tune_gain     = g;
    s_tune_adj_lim  = lim;
    return changed;
}

bool alink_rx_restart_stream(void)
{
    taskENTER_CRITICAL(&s_q.mux);
    s_q.count = 0;
    s_q.head  = 0;
    s_q.tail  = 0;
    taskEXIT_CRITICAL(&s_q.mux);
    return true;
}

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

    /* 调参面板放在音频任务之后起：万一 httpd 起来有内存/任务失败，
     * 也不会连累已经跑起来的播放链路。 */
    web_ui_start();

    ESP_LOGI(TAG, "接收端就绪（S3 / ESP-NOW / 播放任务独占 core1），等音频包");
}
