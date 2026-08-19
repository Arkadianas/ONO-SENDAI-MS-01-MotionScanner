#pragma once
#include "esp_err.h"
#include "lvgl.h"
#include "localization.h"
#include <stdint.h>

esp_err_t display_ui_init(void);
void display_ui_show_radar_screen(void);
void display_ui_radar_tick(float current_heading_deg);
void display_ui_set_target_count(uint8_t count);
void display_ui_set_signal_level(uint8_t level);
void display_ui_set_device_name(const char *type_label, const char *name);
void display_ui_update_devices(const tracked_device_t *devices, int count);
void display_ui_set_battery(int pct);
void display_ui_task_step(void);
