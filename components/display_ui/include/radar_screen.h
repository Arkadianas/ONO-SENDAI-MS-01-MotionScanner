/*
 * radar_screen.h — ONO-SENDAI MotionScanner MS-01
 */
#pragma once
#include "lvgl.h"
#include "localization.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

lv_obj_t *radar_screen_create(void);
void radar_screen_set_heading(float heading_deg);
void radar_screen_set_target_count(uint8_t count);
void radar_screen_set_signal_level(uint8_t level);
void radar_screen_set_device_name(const char *type_label, const char *name);
void radar_screen_update_devices(const tracked_device_t *devices, int count);
void radar_screen_show_detail(int device_idx);
void radar_screen_show_list(void);
void radar_screen_hide_detail(void);
void radar_screen_update(void);

#ifdef __cplusplus
}
#endif

/* Update battery display. pct = 0-100, -1 = unknown */
void radar_screen_set_battery(int pct);
