/*
 * radar_engine.h
 *
 * ONO-SENDAI MotionScanner — MS-01
 * Radar engine public API.
 */

#pragma once

#include "lvgl.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RADAR_MAX_TARGETS 32

typedef struct {
    bool       active;
    float      bearing_deg;     /* world bearing [0..360) */
    float      distance;        /* normalised radius [0..1] */
    float      confidence;      /* [0..1] from localization */
    lv_color_t color;

    /* Computed by radar_engine_update() */
    float      screen_angle;    /* display-relative angle [0..360) */
    float      screen_radius;   /* normalised [0..1] */
} radar_target_t;

void radar_engine_init(void);
void radar_engine_set_heading(float heading_deg);

/* Called by feeder at start of each update cycle */
void radar_engine_clear(void);

/* Primary tracked API — pass MAC as identity key */
void radar_engine_add_target_tracked(
    float         bearing_deg,
    float         distance,
    float         confidence,
    lv_color_t    color,
    const uint8_t mac[6]);

/* Legacy API — no persistent identity, no smoothing */
void radar_engine_add_target(float bearing_deg, float distance, lv_color_t color);

/* Called once per renderer frame */
void radar_engine_update(void);

const radar_target_t *radar_engine_get_targets(void);
uint8_t               radar_engine_target_count(void);

#ifdef __cplusplus
}
#endif
