/*
 * display_ui.c — ONO-SENDAI MotionScanner MS-01
 */

#include "display_ui.h"
#include "radar_screen.h"
#include "radar_engine.h"
#include "esp_log.h"

static const char *TAG = "display_ui";
static lv_obj_t   *s_radar_screen = NULL;

esp_err_t display_ui_init(void)
{
    ESP_LOGI(TAG, "display_ui_init ready");
    return ESP_OK;
}

void display_ui_show_radar_screen(void)
{
    ESP_LOGI(TAG, "Loading ONO-SENDAI radar screen");
    s_radar_screen = radar_screen_create();
    lv_scr_load(s_radar_screen);
    ESP_LOGI(TAG, "Radar screen active");
}

void display_ui_radar_tick(float heading_deg)
{
    if (!s_radar_screen) return;
    radar_screen_set_heading(heading_deg);
    radar_screen_update();
}

void display_ui_set_target_count(uint8_t count)
{
    radar_screen_set_target_count(count);
}

void display_ui_set_signal_level(uint8_t level)
{
    radar_screen_set_signal_level(level);
}

void display_ui_set_device_name(const char *type_label, const char *name)
{
    radar_screen_set_device_name(type_label, name);
}

void display_ui_update_devices(const tracked_device_t *devices, int count)
{
    radar_screen_update_devices(devices, count);
}

void display_ui_set_battery(int pct)
{
    radar_screen_set_battery(pct);
}

void display_ui_task_step(void)
{
    lv_timer_handler();
}
