/*
 * audio-link 共享定义
 *
 * 链路参数：
 *   采样率 48kHz / 立体声 / 16bit —— 音源是电视盒子的 USB Audio Class，
 *   48k 是 Android 的原生速率，不做重采样。早期选 32k 是为了迁就 S3 内置
 *   SAR ADC 的 83.3kHz 立体声上限，走 UAC 之后这个限制不存在了。
 *
 *   5ms 一包 = 240 帧 = 960 字节负载，加 16 字节头 976 字节，
 *   低于 1472 的 MTU，不触发 IP 分片。延迟和抗抖动之间折中：
 *   包再短 WiFi 开销占比上升，包再长延迟上去。
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define ALINK_MAGIC            0x4B4E4C41u   /* "ALNK" */

/* 采样率。走 UAC 之后由电视机顶盒的 USB 音频通路上限决定，48k 是 Android 的
 * 原生速率，不做重采样。原 32k 是为了迁就 S3 内置 SAR ADC 的
 * 83.3kHz 立体声上限，那个限制在 UAC 方案下不存在。
 *
 * 48k × 5ms = 240 帧 = 960 字节负载，加 16 字节头 976 字节，
 * 仍低于 1472 的以太网 MTU，不触发 IP 分片。 */
#ifndef ALINK_SAMPLE_RATE
#define ALINK_SAMPLE_RATE      48000
#endif
#define ALINK_CHANNELS         2
#define ALINK_BYTES_PER_SAMPLE 2
#define ALINK_FRAME_BYTES      (ALINK_CHANNELS * ALINK_BYTES_PER_SAMPLE)   /* 4 */

#define ALINK_PKT_MS           5
#define ALINK_FRAMES_PER_PKT   (ALINK_SAMPLE_RATE * ALINK_PKT_MS / 1000)   /* 240 */
#define ALINK_PAYLOAD_BYTES    (ALINK_FRAMES_PER_PKT * ALINK_FRAME_BYTES)  /* 960 */

/* 网络：ESP-NOW 单播，不开热点、不用 IP，两端只要钉在同一个信道上 */
#define ALINK_WIFI_CHANNEL     11

/* 起播门槛（起播前先攒几包）定义在 rx_uac/main/alink_rx.c —— 它必须跟死区绑定，
 * 所以不和这里的两端共用参数放在一起。 */

/* ---------- 水位闭环（时钟漂移补偿） ----------
 *
 * TX 的 48k 节拍来自主机的 USB 时钟，RX 的 48k 来自本地 I2S 分频，两边晶振
 * 必然差几十 ppm；主机本身也未必是精确的 48k（实测盒子 47.93k，PC 48000.2）。
 * 差额全部在接收端消化，分两级：
 *
 *   粗：I2S 时钟跟随音源 —— 30 秒窗口测实际帧率，差 60Hz 以上才重配；
 *   细：队列水位做 EMA，偏离目标超过 ±死区（ALINK_SRC_DEADBAND）就给重采样比
 *       加一个小偏置（P 控制，增益 8，上限 ±0.3%）。死区之内 ratio 精确等于 1.0，
 *       一个样本都不插值 —— 这是音质底线，别为了省延时砍死区。
 *
 * 调 TARGET_Q / 死区 / 增益之前先算一遍这条稳态方程（P 控制没有积分项）：
 *
 *     稳态水位 = TARGET_Q − 死区 − (收发失配 × 65536) / 增益
 *
 * 盒子的失配 0.146% 在增益 8 下要占掉 12 包，PC 的 0.037% 只占 3 包。
 * 所以同一个 TARGET_Q，接 PC 的水位会比接盒子高约 9 包，这是正常现象，不是 bug。
 * 盒子在开机头 60 秒里粗调还没生效（要两个干净的 30 秒窗口），水位会直接落到
 * 方程给出的那个数上 —— TARGET_Q 调低之前先确认那个数离欠载还够远。
 *
 * 演进过程和实测数据见 TUNING.md。 */
#ifndef ALINK_LOW_LATENCY
#define ALINK_LOW_LATENCY      1
#endif

#if ALINK_LOW_LATENCY
/* 2026-11-26 从 18 提到 20：给「电脑偶发少送音频」多留 10ms 余量（TUNING.md 坑 14）。
 * 代价是延时也多 10ms。余量不够时只能加这里 —— 死区不能砍、增益不能提，
 * 那两条都是实测出来的音质底线。 */
#define ALINK_TARGET_Q         20    /* 低延时档。时钟跟随生效后稳态在 TARGET − 死区 = 17 包 = 85ms */
#define ALINK_Q_HIGH           32    /* 真溢出兜底：EMA 高于此值且瞬时也高 → 丢整包 */
#else
#define ALINK_TARGET_Q         26    /* 稳定档。接盒子时实测稳在 11 包 = 55ms（见上面的稳态方程） */
#define ALINK_Q_HIGH           40
#endif
#define ALINK_Q_DEPTH          48    /* 环形队列容量（包）= 240ms */
#define ALINK_MAX_FILL         4     /* 单次丢包最多补几包占位（播放时走 PLC 外推，不是静音） */

/* 交叉淡化长度：2ms。1ms 对 440Hz 只有半个周期，抹不干净阶跃。 */
#define ALINK_XFADE_FRAMES     (ALINK_SAMPLE_RATE / 500)

/* 断流恢复 / 时钟重配后的淡入长度。
 *
 * 2ms 交叉淡化只适合「接缝」——PLC 接续、漂移丢包这类两侧都有信号的场合。
 * 断流恢复时前面是静音，从静音硬切到满幅只给 2ms，低频就是一次冲击，
 * 钢琴、大提琴这类持续音上会听成「咚」的一下闷响，也就是切歌时那声爆音。
 * 15ms 是听感上「完全听不出起点」又不拖慢响应的折中。 */
#define ALINK_FADE_IN_FRAMES   (ALINK_SAMPLE_RATE * 15 / 1000)

/* 从「多久没数据」算断流恢复：连续这么多包（20ms）没有真数据，就认为
 * 中间断过，恢复时必须走长淡入而不是 2ms 接缝淡化。 */
#define ALINK_RESUME_GAP_PKTS  4

/* 时钟重配前的淡出包数。DMA 里通常还压着 10~15ms 已写入的音频，
 * 直接 disable 等于把波形从中间砍断；先按线性斜坡把 6 包（30ms）
 * 真实音频压到 0 再停，切点就落在静音上。 */
#define ALINK_CLK_FADE_PKTS    6

/* 连续多少包没数据就退回预缓冲状态（40 包 = 200ms） */
#define ALINK_STARVE_RESET     40

/* ---------- PLC（丢包隐藏） ----------
 * 丢包时不再补静音 —— 静音是「咔」的来源：波形直接阶跃到 0 再跳回来。
 * 改成重复上一包输出，逐次衰减，再接 emit() 的 2ms 交叉淡化抹平接缝。
 * 听感从「咔一声」变成「糊一下」，后者大脑会自动补过去。
 *
 * 衰减系数按「连续第 n 次 PLC」取，4 次（20ms）后归零，与 ALINK_MAX_FILL 对齐。
 * 归零是因为连续丢 20ms 以上，外推已经没有信息可用，继续重复会变成嗡嗡声。
 */
#define ALINK_PLC_DECAY_MAX    4


/* 线上消息类型。1 是 UDP 时代的 HELLO（接收端报备 IP），ESP-NOW 用不到，已废弃；
 * 现有取值不要改，老固件还认这些数。 */
typedef enum {
    ALINK_MSG_AUDIO = 2,   /* 发送端 → 接收端，音频数据 */
} alink_msg_type_t;

/* 16 字节定长头 */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t type;
    uint16_t seq;
    uint32_t ts_us;      /* 发送时刻（esp_timer 低 32 位）。接收端目前不读，占位 */
    uint16_t frames;     /* 恒为 ALINK_FRAMES_PER_PKT。接收端目前不读，占位 */
    uint16_t flags;      /* 原 rsvd，现用标志位，见 ALINK_FLAG_* */
} alink_hdr_t;

/* 包头 flags 位定义。
 *
 * ALINK_FLAG_PAD：本包有「补位」—— 发送端 UAC 侧没喂满一包（音源断供或半包），
 * 尾巴上的样本是用包络淡出补出来的。接收端只做计数，不改变播放行为。
 *
 * 存在的意义是**可观测**：零丢包、零欠载时仍然有空隙爆音，光看接收端计数
 * 分不清是「网络没送到」还是「发送端本来就没数据」。有了这一位，
 * 接收端 `补位=N` 一涨就说明问题在 USB 音源侧，跟无线无关。 */
#define ALINK_FLAG_PAD        0x0001u

#define ALINK_HDR_SIZE sizeof(alink_hdr_t)

#define ALINK_ETH_ALEN 6     /* 与 ESP_NOW_ETH_ALEN 相同 */

/* ---------- MAC 字符串解析（两端共用） ----------
 *
 * 接受三种写法，大小写都行，首尾空白会被去掉，其余必须严格匹配：
 *   aa:bb:cc:dd:ee:ff   aa-bb-cc-dd-ee-ff   每段 1~2 位十六进制，分隔符不能混用
 *   aabbccddeeff        正好 12 位十六进制
 * 少一位、多一位、某段超过 0xFF、带多余字符，一律返回 false。
 *
 * 故意不放宽。配置串写错时，调用方会退回默认值并把原始串打进日志；
 * 静默解析成「另一个合法地址」才是烧进去没声音又查不出来的那种错。 */
static inline int alink_hex_val(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static inline bool alink_mac_from_str(const char *s, uint8_t out[ALINK_ETH_ALEN])
{
    while (*s == ' ' || *s == '\t') {
        s++;
    }
    size_t len = strlen(s);
    while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t')) {
        len--;
    }

    /* 以第一次出现的分隔符为准 */
    char sep = 0;
    for (size_t i = 0; i < len; i++) {
        if (s[i] == ':' || s[i] == '-') {
            sep = s[i];
            break;
        }
    }

    memset(out, 0, ALINK_ETH_ALEN);

    if (sep == 0) {
        /* 不带分隔符：正好 12 位 */
        if (len != ALINK_ETH_ALEN * 2) {
            return false;
        }
        for (int i = 0; i < ALINK_ETH_ALEN; i++) {
            const int hi = alink_hex_val(s[i * 2]);
            const int lo = alink_hex_val(s[i * 2 + 1]);
            if (hi < 0 || lo < 0) {
                return false;
            }
            out[i] = (uint8_t)((hi << 4) | lo);
        }
        return true;
    }

    int n = 0, acc = 0, digits = 0;
    for (size_t i = 0; i <= len; i++) {
        /* 走到末尾时补一个分隔符，把最后一段收掉 */
        const char c = (i < len) ? s[i] : sep;
        const int  v = (c == sep) ? -1 : alink_hex_val(c);
        if (v >= 0) {
            acc = acc * 16 + v;
            if (++digits > 2) {
                return false;       /* 这一段超过 0xFF */
            }
        } else if (c == sep) {
            if (digits == 0 || n >= ALINK_ETH_ALEN) {
                return false;       /* 空段，或多于 6 段 */
            }
            out[n++] = (uint8_t)acc;
            acc    = 0;
            digits = 0;
        } else {
            return false;           /* 非十六进制字符，或两种分隔符混用 */
        }
    }
    return n == ALINK_ETH_ALEN;
}
