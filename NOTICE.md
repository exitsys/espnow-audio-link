# 第三方代码

## tx_uac/components/usb_device_uac

USB Audio Class 2 设备组件，来自 Espressif 的 [esp-iot-solution](https://github.com/espressif/esp-iot-solution)
仓库（`components/usb/usb_device_uac`，v1.3.2，commit `58c2f85`）。

- 许可：Apache License 2.0，原文见 `tx_uac/components/usb_device_uac/license.txt`
- 版权：Copyright 2023-2026 Espressif Systems (Shanghai) CO LTD
- 组件目录里的 `CHECKSUMS.json` 是上游各文件的 SHA-256，可以用来核对哪些文件被改过

本项目**只修改了其中两个文件**：`usb_device_uac.c` 和 `include/usb_device_uac.h`
（`usb_device_uac.c` 文件头有改动声明，改动处都标了「本工程补丁」）。

1. `tud_audio_set_itf_cb`：删掉两处 `printf`。这个回调运行在 USB 中断上下文里，
   裸 `printf` 和 `ESP_LOG` 用的不是同一把 stdout 锁，混用会破坏日志输出，严重时让 USB 任务卡死。
2. `tud_audio_rx_done_isr`，**反馈值固定为标称采样率**，不再使用 TinyUSB 的 FIFO_COUNT 反馈算法。
   原算法在电视盒子上会把反馈值压到 47.01kHz，主机随之降速，接收端持续欠载。
   收发两端的时钟差改由本项目的接收端消化（水位闭环 + I2S 时钟跟随）。
3. `tud_audio_rx_done_isr`，**只读走 FIFO 半满线以上的数据**，把水位钉在 `level_set`。
   原版每帧无条件读走 1ms，把 FIFO 抽空，反馈端点因此一直催主机加速，最终爬到 1.8 倍速率。
   起播前的等待水位也相应改成 `level_set`。
4. `tud_audio_rx_done_isr`，**`level_set` 按帧长向下取整**，保证交给应用层的始终是整帧。
   不取整时首块是 266 字节（66 帧多 1 个样本），应用层在一次半包之后会左右声道错位。
5. `tud_audio_rx_done_isr`，**上一块数据没被取走时先不读 FIFO**（2026-11-26）。
   原版读进同一块 `spk_buf`，`usb_spk_task` 没来得及交给应用，下一次读就把上一块覆盖了，丢 1ms 音频。
   现在数据留在 USB FIFO 里（放得下 11ms），等下一次再读。
6. `usb_device_uac.h` 的 `uac_diag_t`、`tud_sof_cb`、`tud_mount_cb`：**USB 级诊断计数**（2026-11-26）。
   每一包进来时记零长包、缺帧、USB 任务迟到等几个数，应用层有变化才打日志，
   用来判断「音频少了」是主机没发还是丢在本机。为了拿帧号，挂载后打开了 TinyUSB 的 SOF 回调。
   不影响音频通路。

组件的其余文件与上游逐字节一致。音量的范围、步进，以及 `set_volume_cb` 的 0~100 映射，
都是上游原有实现，本项目没有改。

任务优先级和绑核（`CONFIG_UAC_*_TASK_PRIORITY` / `CONFIG_UAC_*_TASK_CORE`）属于应用配置，
在 `tx_uac/sdkconfig.defaults` 里，不是对组件的改动。

## 其余

`rx_uac/`、`tx_uac/main/`、`components/`、`legacy/` 下的代码为本项目原创，MIT 许可（见根目录 `LICENSE`）。
