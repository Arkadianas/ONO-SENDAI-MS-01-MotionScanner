/*
 * radar_renderer.c
 *
 * ONO-SENDAI MotionScanner — MS-01
 * Radar display renderer for LVGL 8.3.x on ESP32-S3 (220x220 AMOLED)
 */

#include "radar_renderer.h"
#include "radar_engine.h"
#include "lvgl.h"

#include <math.h>
#include <string.h>

/* -----------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------- */

#define RADAR_SIZE         220
#define RADAR_CENTER_X     (RADAR_SIZE / 2)   /* 110 */
#define RADAR_CENTER_Y     (RADAR_SIZE / 2)   /* 110 */
#define RADAR_OUTER_RADIUS 108

#define TARGET_DOT_SIZE    7
#define TARGET_POOL_SIZE   RADAR_MAX_TARGETS

#define SWEEP_RADIUS       RADAR_OUTER_RADIUS
#define SWEEP_PERIOD_MS    3000

/* -----------------------------------------------------------------------
 * Module state
 * --------------------------------------------------------------------- */

static lv_obj_t *s_radar_root = NULL;
static lv_obj_t *s_ring[4];
static lv_obj_t *s_cross_v;
static lv_obj_t *s_cross_h;
static lv_obj_t *s_center_dot;
static lv_obj_t *s_target_dot[TARGET_POOL_SIZE];
static lv_obj_t *s_sweep_line = NULL;
static float     s_sweep_angle_deg = 0.0f;
static lv_timer_t *s_sweep_timer  = NULL;

/* -----------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------- */

static lv_obj_t *renderer_create_ring(lv_obj_t *parent, int size)
{
    lv_obj_t *arc = lv_arc_create(parent);
    lv_obj_set_size(arc, size, size);
    lv_obj_center(arc);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_value(arc, 100);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_set_style_arc_color(arc, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 1, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 1, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    return arc;
}

static lv_obj_t *renderer_create_line(lv_obj_t *parent, int w, int h)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_style_all(obj);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    return obj;
}

static void renderer_build_rings(void)
{
    static const int diameters[4] = { 220, 165, 110, 55 };
    for (int i = 0; i < 4; i++) {
        s_ring[i] = renderer_create_ring(s_radar_root, diameters[i]);
    }
}

static void renderer_build_crosshair(void)
{
    s_cross_v = renderer_create_line(s_radar_root, 1, RADAR_SIZE);
    lv_obj_center(s_cross_v);
    s_cross_h = renderer_create_line(s_radar_root, RADAR_SIZE, 1);
    lv_obj_center(s_cross_h);
}

static void renderer_build_center_dot(void)
{
    s_center_dot = lv_obj_create(s_radar_root);
    lv_obj_remove_style_all(s_center_dot);
    lv_obj_set_size(s_center_dot, 6, 6);
    lv_obj_set_style_radius(s_center_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_center_dot, lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_bg_opa(s_center_dot, LV_OPA_COVER, 0);
    lv_obj_center(s_center_dot);
    lv_obj_clear_flag(s_center_dot, LV_OBJ_FLAG_CLICKABLE);
}

static void renderer_build_target_pool(void)
{
    for (int i = 0; i < TARGET_POOL_SIZE; i++) {
        lv_obj_t *dot = lv_obj_create(s_radar_root);
        lv_obj_remove_style_all(dot);
        lv_obj_set_size(dot, TARGET_DOT_SIZE, TARGET_DOT_SIZE);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(dot, lv_palette_main(LV_PALETTE_ORANGE), 0);
        lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE);

        /*
         * FIX: Remove all padding from the dot itself — lv_obj_create()
         * adds default padding in LVGL 8 which shifts child content.
         * Dots have no children so this has no effect except ensuring
         * lv_obj_set_pos() places them exactly where we compute.
         */
        lv_obj_set_style_pad_all(dot, 0, 0);

        lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
        s_target_dot[i] = dot;
    }
}

static void renderer_build_sweep(void)
{
    s_sweep_line = lv_obj_create(s_radar_root);
    lv_obj_remove_style_all(s_sweep_line);
    lv_obj_set_size(s_sweep_line, 1, SWEEP_RADIUS);

    /*
     * Position: top edge starts at radar center, extends upward.
     * x = center - 0 (1px wide), y = center - SWEEP_RADIUS
     */
    lv_obj_set_pos(s_sweep_line,
                   RADAR_CENTER_X,
                   RADAR_CENTER_Y - SWEEP_RADIUS);

    lv_obj_set_style_bg_color(s_sweep_line, lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_bg_opa(s_sweep_line, LV_OPA_70, 0);

    /* Pivot at bottom of the line = radar center */
    lv_obj_set_style_transform_pivot_x(s_sweep_line, 0, 0);
    lv_obj_set_style_transform_pivot_y(s_sweep_line, SWEEP_RADIUS, 0);

    lv_obj_clear_flag(s_sweep_line, LV_OBJ_FLAG_CLICKABLE);
}

/* -----------------------------------------------------------------------
 * Sweep timer
 * --------------------------------------------------------------------- */

static void renderer_sweep_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    /* 20ms tick, SWEEP_PERIOD_MS for full 360° */
    s_sweep_angle_deg += (360.0f / (float)SWEEP_PERIOD_MS) * 20.0f;
    if (s_sweep_angle_deg >= 360.0f) {
        s_sweep_angle_deg -= 360.0f;
    }

    if (s_sweep_line) {
        lv_obj_set_style_transform_angle(
            s_sweep_line,
            (int16_t)(s_sweep_angle_deg * 10.0f),
            0);
    }
}

/* -----------------------------------------------------------------------
 * Polar → pixel
 * --------------------------------------------------------------------- */

/*
 * Convert engine polar coordinates to screen pixel position.
 *
 * angle_deg: screen-relative angle (0 = North/up, 90 = East/right)
 * radius:    normalised [0.0 .. 1.0], 1.0 maps to RADAR_OUTER_RADIUS px
 *
 * NOTE: lv_obj_set_pos() on a child of s_radar_root uses coordinates
 * relative to s_radar_root's top-left corner (0,0), so no screen offset
 * is needed — RADAR_CENTER_X/Y are already in that space.
 */
static void renderer_polar_to_pixel(float angle_deg, float radius,
                                    int *px, int *py)
{
    /* -90° so that 0° == North (top of display) */
    float rad  = (angle_deg - 90.0f) * ((float)M_PI / 180.0f);
    float r_px = radius * (float)RADAR_OUTER_RADIUS;

    *px = RADAR_CENTER_X + (int)(r_px * cosf(rad));
    *py = RADAR_CENTER_Y + (int)(r_px * sinf(rad));
}

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

void radar_renderer_create(lv_obj_t *parent)
{
    s_radar_root = lv_obj_create(parent);
    lv_obj_remove_style_all(s_radar_root);
    lv_obj_set_size(s_radar_root, RADAR_SIZE, RADAR_SIZE);
    lv_obj_center(s_radar_root);
    lv_obj_set_style_bg_opa(s_radar_root, LV_OPA_TRANSP, 0);

    /*
     * FIX: Zero out all padding on the root container.
     * LVGL 8 lv_obj_create() sets default padding (typically 8px) which
     * shifts child widget positions computed by lv_obj_set_pos().
     * Without this, dots appear offset from their intended pixel coords.
     */
    lv_obj_set_style_pad_all(s_radar_root, 0, 0);
    lv_obj_set_style_pad_gap(s_radar_root, 0, 0);

    lv_obj_clear_flag(s_radar_root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(s_radar_root, LV_OBJ_FLAG_CLICKABLE);

    /* Z-order: rings → crosshair → targets → sweep → center dot */
    renderer_build_rings();
    renderer_build_crosshair();
    renderer_build_target_pool();
    renderer_build_sweep();
    renderer_build_center_dot();

    /* Sweep animation timer, 20ms period */
    s_sweep_timer = lv_timer_create(renderer_sweep_timer_cb, 20, NULL);
}

void radar_renderer_set_heading(float heading)
{
    LV_UNUSED(heading);
}

void radar_renderer_clear_targets(void)
{
    /* No-op: managed in radar_renderer_render() */
}

void radar_renderer_add_target(float bearing, float distance, lv_color_t color)
{
    LV_UNUSED(bearing);
    LV_UNUSED(distance);
    LV_UNUSED(color);
}

void radar_renderer_render(void)
{
    /* Recompute engine screen coordinates */
    radar_engine_update();

    const radar_target_t *targets = radar_engine_get_targets();
    uint8_t count                 = radar_engine_target_count();

    if (count > TARGET_POOL_SIZE) count = TARGET_POOL_SIZE;

    for (uint8_t i = 0; i < count; i++) {
        const radar_target_t *t = &targets[i];

        /*
         * FIX: Do not rely on t->active flag alone.
         * radar_engine_init() only resets target_count to 0 — it does NOT
         * memset the array, so t->active may contain garbage for slots that
         * were never written. We iterate only up to target_count (already
         * clamped above) so every slot we touch was explicitly added via
         * radar_engine_add_target(), which sets active = true.
         * Guard is kept for safety but should always pass here.
         */
        /* Guard against corrupted slot data */
        if (!t->active || t->screen_radius > 1.0f ||
            t->screen_angle < 0.0f || t->screen_angle > 360.0f) {
            lv_obj_add_flag(s_target_dot[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }

        int px, py;
        renderer_polar_to_pixel(t->screen_angle, t->screen_radius, &px, &py);

        /* Guard against off-screen positions */
        if (px < 0 || px > 220 || py < 0 || py > 220) {
            lv_obj_add_flag(s_target_dot[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }

        /*
         * px, py are the center of where the dot should appear.
         * lv_obj_set_pos() sets the top-left corner, so subtract half size.
         */
        lv_obj_set_pos(s_target_dot[i],
                       px - TARGET_DOT_SIZE / 2,
                       py - TARGET_DOT_SIZE / 2);

        lv_obj_set_style_bg_color(s_target_dot[i], t->color, 0);

        /* Confidence → opacity: min 40% so all detected devices are clearly visible */
        lv_opa_t opa = LV_OPA_40 + (lv_opa_t)(t->confidence * (float)(LV_OPA_COVER - LV_OPA_40));
        lv_obj_set_style_bg_opa(s_target_dot[i], opa, 0);

        lv_obj_clear_flag(s_target_dot[i], LV_OBJ_FLAG_HIDDEN);
    }

    /* Hide unused pool slots */
    for (uint8_t i = count; i < TARGET_POOL_SIZE; i++) {
        lv_obj_add_flag(s_target_dot[i], LV_OBJ_FLAG_HIDDEN);
    }
}

void radar_renderer_set_sweep(float angle_deg)
{
    s_sweep_angle_deg = angle_deg;
    if (s_sweep_line) {
        lv_obj_set_style_transform_angle(
            s_sweep_line,
            (int16_t)(angle_deg * 10.0f),
            0);
    }
}
