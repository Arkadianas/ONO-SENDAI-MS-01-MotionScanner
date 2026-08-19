idf_component_register(
    SRCS
        "display_ui.c"
        "radar_screen.c"
        "radar_renderer.c"
        "radar_canvas.c"
        "radar_engine.c"

    INCLUDE_DIRS
        "include"

    REQUIRES
        lvgl
        localization
        device_classification
        board_config
        wifi_scan
        esp_touch
        esp_timer
)
