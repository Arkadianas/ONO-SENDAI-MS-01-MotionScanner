/*
 * radar_renderer.h
 *
 * ONO-SENDAI MotionScanner — MS-01
 * Radar renderer public API.
 *
 * The renderer is the ONLY module that draws.  It reads target data
 * exclusively from radar_engine_get_targets() and never performs any
 * BLE, IMU, or localisation logic.
 *
 * Coordinate flow:
 *   World bearing + scanner heading  →  radar_engine (screen_angle)
 *   screen_angle + screen_radius     →  radar_renderer (pixel x,y)
 *   pixel x,y                        →  LVGL widget position
 */

#pragma once

#include "lvgl.h"
#include "radar_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create all radar renderer widgets under @p parent.
 *
 * Builds: four range rings, crosshair, center dot, target dot pool,
 * sweep line, and starts the sweep animation timer.
 *
 * Must be called from the LVGL task context.
 *
 * @param parent  Parent LVGL object (typically the radar screen).
 */
void radar_renderer_create(lv_obj_t *parent);

/**
 * @brief Notify renderer of scanner heading change.
 *
 * Reserved for future heading indicator widget.
 * Engine heading must be set separately via radar_engine_set_heading().
 *
 * @param heading  Current heading in degrees [0..360).
 */
void radar_renderer_set_heading(float heading);

/**
 * @brief No-op — provided for API symmetry.
 *
 * Target pool management is handled inside radar_renderer_render().
 * Targets are added via radar_engine_add_target().
 */
void radar_renderer_clear_targets(void);

/**
 * @brief No-op — provided for API symmetry.
 *
 * Inject targets via radar_engine_add_target(), not this function.
 */
void radar_renderer_add_target(float bearing, float distance, lv_color_t color);

/**
 * @brief Render one frame: update engine and redraw all target dots.
 *
 * Call once per frame from radar_screen_update().
 * Sweep animation advances automatically via an internal LVGL timer.
 */
void radar_renderer_render(void);

/**
 * @brief Directly set sweep line angle (degrees).
 *
 * Normally the sweep is animated automatically.  This function allows
 * manual override (e.g. for testing or external synchronisation).
 *
 * @param angle_deg  Desired angle [0..360).
 */
void radar_renderer_set_sweep(float angle_deg);

#ifdef __cplusplus
}
#endif
