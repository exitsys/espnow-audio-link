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

#include <stdint.h>

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

/* 网络 */
#define ALINK_WIFI_SSID        "alink-audio"
#define ALINK_WIFI_PASS        "********"
#define ALINK_WIFI_CHANNEL     11
#define ALINK_UDP_PORT         33333
#define ALINK_AP_IP            "192.168.4.1"

/* 接收端起播前先攒几包。6 包 = 30ms。
 * 这是「起播门槛」，不是「持续水位」。 */
#define ALINK_JITTER_PKTS      6

/* ---------- 水位闭环（时钟漂移补偿） ----------
 * TX 的 48k 采样节拍来自电视盒子的 USB 时钟，RX 的 48k 来自 C3 的 I2S 分频。
 * 实测盒子送 191703 B/s ≈ 47926Hz，RX 播 48000Hz，差 74 帧/秒（0.15%）。
 *
 * 2026-11-19 改法：细调不再用「整包丢/补」。
 * 整包丢就是 5ms 突变，即使配 2ms 交叉淡化，每秒来几次也会听成"噼啪"
 * （实测：丢包 0.02%、PLC 仍每秒约 1 次，噼啪还在）。
 * 改成样本级滑移：每包最多插入/删除 1 帧（1/240 = 0.42%，调节能力 200 帧/秒，
 * 远超 74 帧/秒的失配），用线性插值，听感上无痕。
 * 整包丢弃只在真溢出时保留，做粗调兜底。
 *
 * 同时把水位从 6 包抬到 10 包：30ms 缓冲太浅，WiFi 一抖动就见底。
 *
 * 2026-11-21 再调：实测队列在 1~24 之间来回摆（容量就 24），两头都撞墙 ——
 * 触底是欠载走 PLC，撞顶是 overrun 丢掉整包，两个都是「咔」。
 * 病根是闭环增益：原来 dev × 64，队列偏 10 包就直接打满 ±0.8%，
 * 而实际收发时钟差只有 0.01%（5.5 帧/秒），等于用 80 倍的力去推一个微调，
 * 必然过冲 → 反向过冲。现在：
 *   缓冲 24→48 包（240ms），目标 10→20 包，让控制环有腾挪空间；
 *   增益 64→8，上限 0.8%→0.3%，从「猛推到饱和」改成「缓慢收敛」。
 *
 * 目标水位定 26 而不是 20：这是 P 控制的稳态误差，不是随便填的。
 * 实测收发失配约 0.04%，闭环要输出对应的调整量才有平衡，
 * 而 adj = (|dev|-3)×8，解出来 |dev|≈6 —— 目标定 20 就会真的稳在 14，
 * 白扔 30ms 缓冲。定 26 恰好让平衡点落在 20 包（100ms），
 * 留足抗 WiFi 突发空档的余量。
 *
 * ---------- 2026-11-21 加低延时档（看视频，要音画同步） ----------
 *
 * 端到端延时基本等于「队列水深 + DMA 深度」，其余各段合计不到 10ms。
 * 接安卓盒子看电视时，声音晚 100ms 就是明显的口型不同步，所以开了这个档：
 * 目标 26→14、增益 8→16、DMA 4→3，设备链路 103ms → 63ms。
 *
 * ⚠ 三项必须一起改，缺一不可，原因是 P 控制的那条稳态方程：
 *
 *     稳态时 |dev| = 需要的 adj / 增益
 *
 *   盒子的失配是 **0.146%**（实测送 47.93k，本地 48k），不是 PC 的 0.037%。
 *   要输出 adj = -96 才补得上：
 *     · 增益还是 8 → 需要 |dev| = 12，目标定 14 就只剩 2 包 → 直接欠载
 *     · 增益提到 16 → 需要 |dev| = 6，稳态落在 8 包（40ms），刚好
 *
 *   所以「只把 TARGET_Q 调小」在 PC 上看着没事，一接盒子必崩。
 *   反过来说，PC 上测出来的水位会比盒子上高 4~5 包，这是正常现象，别当成 bug。
 *
 * DMA 从 4 减到 3：15ms 的硬件缓冲。play_task 在 core1 优先级 12，
 * 写满 3 个描述符要 10ms 内被调度到，压力比 4 描述符大，但余量仍在。
 */
#ifndef ALINK_LOW_LATENCY
#define ALINK_LOW_LATENCY      1
#endif

#if ALINK_LOW_LATENCY
#define ALINK_TARGET_Q         18    /* 低延时档：死区 3.0 → 稳态 15~21 包，均值 90ms */
#define ALINK_Q_HIGH           32    /* 真溢出兜底：EMA 高于此值且瞬时也高 → 丢整包 */
#else
#define ALINK_TARGET_Q         26    /* 稳定档：接盒子时实际稳在 11 包 = 55ms */
#define ALINK_Q_HIGH           40
#endif
#define ALINK_Q_LOW            8     /* 保留定义，细调由 ema ± ALINK_SRC_DEADBAND 判定 */
#define ALINK_Q_DEPTH          48    /* 环形队列容量（包）= 240ms */
#define ALINK_DRIFT_DEADBAND   2     /* 队列偏离目标超过这么多包，才启动样本级滑移 */
#define ALINK_MAX_FILL         4     /* 单次丢包最多补几包静音，防连环补包 */

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

/* ---------- 冗余副本（双发） ----------
 * 发完一包立刻背靠背再发一份完全相同的包，接收端按 seq 去重。
 *
 * 为什么不是 FEC：XOR 奇偶校验包要等整组数据包发完才算得出来，晚 1~4 个包才到，
 * 接收端必须把整组缓存住才能用得上，等于把抖动缓冲再加 5~20ms。而这个场景带宽
 * 根本不紧张 —— 48k 立体声 1.54Mbps，翻倍 3.1Mbps，11n 单流实际吞吐 20Mbps 量级，
 * 只占 15%。用带宽换延迟，比用延迟换带宽划算。
 *
 * 收益边界：能吃掉驱动层丢包、接收 mbox 溢出这类「单次事件」；对持续的信道衰落
 * 无效（两份会一起丢）。所以它是保险，不是万灵药。
 */
#ifndef ALINK_TX_DUP
#define ALINK_TX_DUP           0
#endif

/* ---------- PLC（丢包隐藏） ----------
 * 丢包时不再补静音 —— 静音是「咔」的来源：波形直接阶跃到 0 再跳回来。
 * 改成重复上一包输出，逐次衰减，再接 emit() 的 2ms 交叉淡化抹平接缝。
 * 听感从「咔一声」变成「糊一下」，后者大脑会自动补过去。
 *
 * 衰减系数按「连续第 n 次 PLC」取，4 次（20ms）后归零，与 ALINK_MAX_FILL 对齐。
 * 归零是因为连续丢 20ms 以上，外推已经没有信息可用，继续重复会变成嗡嗡声。
 */
#define ALINK_PLC_DECAY_MAX    4


typedef enum {
    ALINK_MSG_HELLO = 1,   /* 接收端 → 发送端，告知自己的 IP */
    ALINK_MSG_AUDIO = 2,   /* 发送端 → 接收端，音频数据 */
} alink_msg_type_t;

/* 16 字节定长头 */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t type;
    uint16_t seq;
    uint32_t ts_us;      /* 发送时刻，供接收端估算网络耗时 */
    uint16_t frames;
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

/* 采样源：先用内部正弦波验链路，接上 ADC 后切到 ADC */
typedef enum {
    ALINK_SRC_TONE = 0,
    ALINK_SRC_ADC  = 1,
} alink_src_t;

#ifndef ALINK_TX_SOURCE
#define ALINK_TX_SOURCE ALINK_SRC_TONE
#endif
