/*
 * radar_engine.c
 *
 * ONO-SENDAI MotionScanner — MS-01
 * Radar engine: coordinate transforms, target tracking, position smoothing.
 *
 * Responsibilities:
 *   - Maintain a persistent tracked target list (keyed by identity hash)
 *   - Smooth target positions frame-to-frame (exponential moving average)
 *   - Gate target visibility by confidence threshold
 *   - Convert world bearing + scanner heading → screen polar coordinates
 *
 * This module NEVER draws. It NEVER reads BLE or IMU directly.
 * All input comes through radar_engine_add_target() / radar_engine_clear().
 */

#include <string.h>
#include <math.h>
#include "radar_engine.h"

/* -----------------------------------------------------------------------
 * Tracking constants
 * --------------------------------------------------------------------- */

/*
 * Position smoothing factor for exponential moving average.
 * Range: 0.0 (never moves) .. 1.0 (no smoothing, raw jumps).
 * 0.15 gives ~7 frame lag at 5 Hz feed = ~1.4 s settling time.
 * Feels stable on a handheld device without being sluggish.
 */
#define TRACK_ALPHA         0.15f

/*
 * Minimum confidence [0..1] before a target is shown on the radar.
 * Set to 0.0 so ALL detected devices appear immediately as dots.
 * Confidence still affects dot opacity — uncertain targets are dimmer.
 * Bearing accuracy improves as you walk around the device.
 */
#define CONFIDENCE_MIN_SHOW 0.0f

/*
 * How many consecutive feed cycles a slot can miss before it is evicted.
 * At 5 Hz feed rate: 25 cycles = 5 seconds of silence → slot freed.
 * localization_expire_stale() handles the primary 8 s timeout.
 */
#define TRACK_MAX_MISS      25

/* Wrap angle to [0, 360) — inline avoids GCC statement-expression warnings */
static inline float deg_wrap(float a)
{
    while (a <   0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

/* -----------------------------------------------------------------------
 * Internal tracked slot — extends radar_target_t with tracking state
 * --------------------------------------------------------------------- */

typedef struct {
    radar_target_t pub;         /* public fields visible to renderer */

    uint32_t       identity;    /* hash of MAC / identity key from feeder */
    bool           occupied;    /* slot in use */
    uint8_t        miss_count;  /* consecutive cycles without a feed update */

    /* Smoothed world-coordinate state (before heading transform) */
    float          smooth_bearing;
    float          smooth_distance;
} track_slot_t;

/* -----------------------------------------------------------------------
 * Module state
 * --------------------------------------------------------------------- */

static track_slot_t  s_slots[RADAR_MAX_TARGETS];

/* Flat export array — copied from slots on each update().
 * Renderer iterates this as a plain radar_target_t array (correct stride). */
static radar_target_t s_export[RADAR_MAX_TARGETS];
static uint8_t       s_slot_count = 0;   /* highest occupied index + 1 */
static float         s_heading    = 0.0f;

/* -----------------------------------------------------------------------
 * Identity management
 *
 * The feeder (radar_feed_task) calls radar_engine_clear() + N x
 * radar_engine_add_target() each cycle. To do persistent tracking we
 * need to match incoming add_target() calls to existing slots.
 *
 * We use a simple sequential identity counter: the feeder must call
 * radar_engine_begin_frame() before the clear/add sequence and
 * radar_engine_add_target_tracked() to pass an identity key.
 *
 * For backward compatibility radar_engine_add_target() (no identity)
 * still works — it auto-assigns a temporary identity from call order.
 * --------------------------------------------------------------------- */

static uint8_t  s_frame_slot_used[RADAR_MAX_TARGETS]; /* bitmap for this frame */
static uint8_t  s_frame_count = 0;   /* targets added this frame */

/* -----------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------- */

/* Simple djb2-style hash of 6-byte MAC */
static uint32_t hash_identity(const uint8_t *key, int len)
{
    uint32_t h = 5381;
    for (int i = 0; i < len; i++) {
        h = ((h << 5) + h) ^ key[i];
    }
    return h ? h : 1;   /* never return 0 (reserved for "no identity") */
}

/* Find existing slot by identity, or return -1 */
static int find_slot(uint32_t identity)
{
    for (int i = 0; i < RADAR_MAX_TARGETS; i++) {
        if (s_slots[i].occupied && s_slots[i].identity == identity) {
            return i;
        }
    }
    return -1;
}

/* Allocate a free slot, or return -1 if full */
static int alloc_slot(void)
{
    for (int i = 0; i < RADAR_MAX_TARGETS; i++) {
        if (!s_slots[i].occupied) {
            memset(&s_slots[i], 0, sizeof(track_slot_t));
            s_slots[i].occupied = true;
            if (i + 1 > s_slot_count) s_slot_count = i + 1;
            return i;
        }
    }
    return -1;
}

/*
 * Shortest-path angular interpolation.
 * Prevents the dot from spinning the long way around when crossing 0°/360°.
 */
static float smooth_angle(float current, float target, float alpha)
{
    float diff = target - current;
    /* Wrap diff to [-180, +180] */
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return deg_wrap(current + alpha * diff);
}

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

void radar_engine_init(void)
{
    memset(s_slots, 0, sizeof(s_slots));
    memset(s_frame_slot_used, 0, sizeof(s_frame_slot_used));
    s_slot_count  = 0;
    s_frame_count = 0;
    s_heading     = 0.0f;
}

void radar_engine_set_heading(float heading_deg)
{
    s_heading = heading_deg;
}

/*
 * radar_engine_clear() — called by feeder at the start of each update cycle.
 * Instead of actually clearing, we mark all slots as "not yet seen this frame"
 * and reset the frame counter. Slots are evicted after TRACK_MAX_MISS misses.
 */
void radar_engine_clear(void)
{
    memset(s_frame_slot_used, 0, sizeof(s_frame_slot_used));
    s_frame_count = 0;
}

/*
 * radar_engine_add_target_tracked() — primary tracked API.
 *
 * @param bearing_deg   World bearing [0..360)
 * @param distance      Normalised radius [0..1]
 * @param confidence    Confidence [0..1] from localization
 * @param color         Display colour from classification
 * @param mac           6-byte MAC address used as identity key
 */
void radar_engine_add_target_tracked(
    float         bearing_deg,
    float         distance,
    float         confidence,
    lv_color_t    color,
    const uint8_t mac[6])
{
    if (s_frame_count >= RADAR_MAX_TARGETS) return;

    uint32_t identity = hash_identity(mac, 6);
    int idx = find_slot(identity);

    if (idx < 0) {
        /* New device — allocate a slot and seed with raw values */
        idx = alloc_slot();
        if (idx < 0) return;   /* table full */

        s_slots[idx].identity       = identity;
        s_slots[idx].smooth_bearing  = bearing_deg;
        s_slots[idx].smooth_distance = distance;
        s_slots[idx].miss_count      = 0;
    } else {
        /* Known device — exponential moving average */
        s_slots[idx].smooth_bearing  = smooth_angle(
            s_slots[idx].smooth_bearing, bearing_deg, TRACK_ALPHA);
        s_slots[idx].smooth_distance = s_slots[idx].smooth_distance * (1.0f - TRACK_ALPHA)
                                     + distance * TRACK_ALPHA;
        s_slots[idx].miss_count      = 0;
    }

    /* Update public target fields */
    radar_target_t *t   = &s_slots[idx].pub;
    t->active           = (confidence >= CONFIDENCE_MIN_SHOW);
    t->bearing_deg      = s_slots[idx].smooth_bearing;
    t->distance         = s_slots[idx].smooth_distance;
    t->confidence       = confidence;
    t->color            = color;

    s_frame_slot_used[idx] = 1;
    s_frame_count++;
}

/*
 * radar_engine_add_target() — legacy / test API (no MAC identity).
 * Uses call-order as a temporary identity key. No smoothing applied
 * because there is no persistent identity to match against.
 */
void radar_engine_add_target(
    float      bearing_deg,
    float      distance,
    lv_color_t color)
{
    if (s_frame_count >= RADAR_MAX_TARGETS) return;

    /* Use call order as temporary identity (fine for test targets) */
    uint8_t key[6] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, s_frame_count };
    radar_engine_add_target_tracked(bearing_deg, distance, 1.0f, color, key);
}

/*
 * radar_engine_update() — called once per renderer frame.
 *
 * 1. Evict slots that have been missed too many consecutive cycles.
 * 2. Increment miss counter for slots not updated this frame.
 * 3. Apply heading transform: world bearing → screen angle.
 */
void radar_engine_update(void)
{
    for (int i = 0; i < s_slot_count; i++) {
        if (!s_slots[i].occupied) continue;

        if (!s_frame_slot_used[i]) {
            /* Not seen this frame */
            s_slots[i].miss_count++;
            if (s_slots[i].miss_count >= TRACK_MAX_MISS) {
                /* Evict */
                s_slots[i].occupied    = false;
                s_slots[i].pub.active  = false;
                continue;
            }
            /* Fade confidence while missing */
            s_slots[i].pub.confidence *= 0.7f;
            if (s_slots[i].pub.confidence < CONFIDENCE_MIN_SHOW) {
                s_slots[i].pub.active = false;
            }
        }

        /* Heading transform: world bearing → display-relative angle */
        float angle = s_slots[i].pub.bearing_deg - s_heading;
        while (angle <   0.0f) angle += 360.0f;
        while (angle >= 360.0f) angle -= 360.0f;

        s_slots[i].pub.screen_angle  = angle;
        s_slots[i].pub.screen_radius = s_slots[i].pub.distance;

        /* Copy to flat export array — correct stride for renderer */
        s_export[i] = s_slots[i].pub;
    }
}

const radar_target_t *radar_engine_get_targets(void)
{
    return s_export;
}

uint8_t radar_engine_target_count(void)
{
    /* Return slot count so renderer iterates all slots (active check inside) */
    return (uint8_t)s_slot_count;
}
