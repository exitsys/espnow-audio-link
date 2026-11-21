idf_component_register(SRCS "alink_tx.c"
                       INCLUDE_DIRS "."
                       REQUIRES alink_common esp_wifi esp_netif esp_event nvs_flash esp_timer
                                usb_device_uac driver)
