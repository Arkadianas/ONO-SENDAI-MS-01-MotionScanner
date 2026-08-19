idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        display_ui
        wifi_scan
        imu_tracking
        localization
        device_classification
        storage_config
        board_config
        esp_wifi
        nvs_flash
        i2c_bsp
        esp_touch
        esp_lcd_sh8601
)
