# 第三方代码

## tx_uac/components/usb_device_uac

USB Audio Class 2 设备组件，来自 Espressif 的 [esp-iot-solution](https://github.com/espressif/esp-iot-solution)
仓库（`components/usb/usb_device_uac`，v1.3.2，commit `58c2f85`）。

- 许可：Apache License 2.0，原文见 `tx_uac/components/usb_device_uac/license.txt`
- 版权：Copyright 2023-2026 Espressif Systems (Shanghai) CO LTD
- 本项目的改动：主机音量回调映射到 −50~0 dB（`uac_descriptors.h`、
  `usb_device_uac.c` 中的音量范围与步进），以及任务优先级的调整。功能逻辑未改。

## 其余

`rx_uac/`、`tx_uac/main/`、`components/`、`legacy/` 下的代码为本项目原创，MIT 许可（见根目录 `LICENSE`）。
