cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "../components")

# 只编译真正用到的组件。必须在 include project.cmake 之前设置。
#
# 不指定 COMPONENTS 时 IDF 会编译全部组件（esp_lcd / bt / mbedtls 全家桶），
# 首次编译十几分钟，而且 esp_lcd 的 RGB 驱动在 gcc 14.2.0 上会触发
#   internal compiler error: Segmentation fault
#   during RTL pass: ira
# 这是编译器自身的问题（大函数 + -Og），跟本工程代码无关。
# 接收端压根不需要 LCD，直接不编它。
#
# 这里只列叶子组件，REQUIRES 依赖链（freertos / lwip / esp_phy / wpa_supplicant
# / mbedtls / spi_flash ...）由 IDF 自动展开，不用手写。
set(COMPONENTS
    main
    alink_common
    esp_wifi
    esp_netif
    esp_event
    nvs_flash
    esp_timer
    esp_driver_i2s
    esp_driver_gpio
    esp_ringbuf
    esp_http_server
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(alink_rx_enow)
