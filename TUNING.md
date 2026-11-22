# 参数档位记录

改动前先看这张表，别再凭记忆试。每改完一轮实测，把结果补进来。

参数只有 6 个，全部集中在这两个文件：

| 参数 | 位置 |
|---|---|
| `ALINK_TARGET_Q` / `ALINK_Q_HIGH` / `ALINK_Q_DEPTH` | `components/alink_common/include/alink_common.h` |
| `ALINK_SRC_DEADBAND` / P 控制增益 / `I2S_DMA_DESC` | `rx_uac/main/alink_rx.c` |
| `ALINK_FADE_IN_FRAMES` / `ALINK_RESUME_GAP_PKTS` / `ALINK_CLK_FADE_PKTS` | `components/alink_common/include/alink_common.h` |

淡化的三个参数会影响「切歌顺不顺」，不影响延时和水位，调音质时不用动它们。

TX 侧（`tx_uac/main/alink_tx.c`）只影响拼包和补位淡出，不影响水位；
接收端 MAC 走 menuconfig（`Component config → Audio link → 接收端 MAC 地址`）。

## 档位对照

| 档位 | TARGET_Q | 死区 | 增益 | DMA | 实测队列 | 重采样 | 设备侧延时 | 听感 |
|---|---|---|---|---|---|---|---|---|
| 标准 | 26 | 3.0 | 8 | 4 | 29 包 | 0.00% | ~165ms | 音质最好，延时大 |
| 88ms 激进 | 16 | 1.5 | 16 | 3 | 15 包 | +0.03~0.07% | ~91ms | 小电平单音发糊 |
| 余量加大 | 20 | 1.5 | 16 | 3 | 22~23 包 | +0.03~0.07% | ~130ms | 切歌仍有概率爆音 |
| **当前** | **18** | **3.0** | **8** | **3** | **15 包** | **0.00%** | **~95ms** | 待验证 |

设备侧延时 = 队列包数 × 5ms + DMA × 5ms + 空口/拼包 5ms。
接 PC 再加 WASAPI 25ms + USB 2ms；接魔百盒另算。

## 调参方式：编译期宏（网页面板已移除）

参数只有 6 个，改完重新编译烧录。**不要再加回 WiFi 调参面板。**

2026-11-22 移除了 SoftAP + HTTP 调参页面，原因：ESP-NOW 是 200 包/秒、
11Mbps 固定速率的连续音频流，SoftAP 的 beacon 是广播、HTTP 响应是突发，
两者都在和音频抢同一个射频，实测丢包明显上升。射频上只留 ESP-NOW 一种业务。

移除的范围：`web_ui.c` / `web_ui.h` / `index.html`、`EMBED_TXTFILES`、
`esp_http_server` 组件、`s_tune_*` 四个 volatile 变量、`alink_get_stats/
get_tune/set_tune`、`alink_rx_restart_stream`、`WIFI_MODE_APSTA` 与 AP 配置。
P 控制重新直接读 `ALINK_TARGET_Q` / `ALINK_SRC_DEADBAND` / `ALINK_SRC_ADJ_LIMIT`，
增益写死 8.0。

**收敛慢是物理特性，不是 bug**：改 `ALINK_TARGET_Q` 后队列按每秒约 0.2 包
（增益 8）逼近目标，29→18 要一分钟。烧录后看串口日志的「队列」慢慢爬是正常的。

## 踩过的坑（别重复推导）

1. **死区窄 = 音质差**。死区收到 1.5 后，队列日常 ±1 包的抖动就足以反复
   触发调节，ratio 永远回不到 1.0，插值器全程在跑。实测残差从 −96dB 掉到
   −30dB，这就是「小电平简单乐器能听到爆音」的主要来源。
   **死区 3.0 是音质底线，不要为了省 7.5ms 延时去砍它。**

2. **增益高 = 收敛快但振荡**。增益 64 会让队列在 1~24 之间撞；16 能追盒子
   0.146% 的时钟失配但 ratio 一直在动；8 稳定但收敛慢（29→18 要 50 秒）。
   增益和死区必须成对看，单独调一个必翻车。

3. **dev 恰好等于死区边界时 adj=0**。`|dev| == DEADBAND` 不落进死区分支，
   但 `eff = dev - DEADBAND = 0`，得到 ratio 1.0，队列就卡在 `TARGET-DEADBAND`
   不再往回走。实测 TARGET 18 / 死区 3.0 稳定在 15 包就是这个原因。
   想让它精确停在 TARGET，死区要略窄于目标偏差，或者接受这个 −3 包的偏移
   （反而是免费的延时红利）。

4. **相位钳位会丢连续性**。`if (s_src_phase < 0) s_src_phase = 0;` 在 ratio<1
   时每包丢 0.7~1.7 帧相位，等于 200 次/秒硬跳变。已改成允许负相位
   （历史区 32 帧），日志里看 `相位钳` 计数，正常应为 0。

5. **切歌爆音**：三个来源叠加，2026-11-22 一并处理。
   - `i2s_channel_reconfig_std_clock()` **必须先 disable**（IDF 的硬要求，见
     `i2s_std.h` 注释），而 DMA 里还压着 10~15ms 已写入的音频。直接停
     = 波形从中间砍断。**现在先接单，按 6 包（`ALINK_CLK_FADE_PKTS`）把真实
     音频线性压到 0，最后一包 gain 正好为 0，才真正 disable/enable。**
   - 恢复时的 2ms 交叉淡化是给「接缝」用的（两侧都有信号）。断流恢复时
     前面是静音，从静音到满幅只给 2ms = 阶跃，低频上就是一次冲击。
     **现在断流超过 `ALINK_RESUME_GAP_PKTS`（4 包 = 20ms）就改走
     `ALINK_FADE_IN_FRAMES`（15ms）长淡入。**
   - TX 补位段的淡出只有 2.67ms，同源问题，已按 Q8 包络拉长到约 10ms。
   - **主因（2026-11-22 深夜实测确认）**：暂停/切歌时 **Windows 不会断开流**，
     端点保持打开、一直送全零 PCM。所以接收端永远等不到「断流」，队列始终是满的，
     `starve` 判定一次都不触发 —— 恢复时第一个非零样本就是从 0 直接跳到
     某个幅度。判据改成「连续 3 包（15ms）整包低于 16 LSB → 突然出现 ≥64 的包」，
     命中就 15ms 淡入。日志里对应 `从静音恢复（峰=...）→ 淡入 15ms` 和 `醒来=N`。
     阈值 64 是实测定的：25% 系统音量下输出峰值只有 168 LSB，
     最早写 1000 时一次都没触发。

   2ms 接缝淡化仍然保留 —— 它解决的是 PLC 接续、漂移丢包这类
   「两侧都有信号」的场合，和长淡入是两件事，不要合并。

## 回退方法

```bash
cd espnow_audio
git log --oneline          # 看历史
git tag -l                 # 看档位标签
git checkout v1.0-direct-95ms -- .
```

改完参数记得：先 `git commit`，再编译烧录。否则又是一轮「改不回去」。
