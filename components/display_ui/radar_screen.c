/*
 * radar_screen.c — ONO-SENDAI MotionScanner MS-01
 */

#include "radar_screen.h"
#include "radar_renderer.h"
#include "radar_engine.h"
#include "localization.h"
#include "device_classification.h"
#include "touch_bsp.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* -----------------------------------------------------------------------
 * Screen layout (portrait 280x456 after rotation)
 *
 *  ┌────────────────────────────┐  Y=0
 *  │ ONO-SENDAI   MS-01         │  header  Y=6
 *  ├────────────────────────────┤  Y=28
 *  │                            │
 *  │    [  220x220 radar  ]     │  center Y=118..338
 *  │                            │
 *  ├────────────────────────────┤  Y=340
 *  │ BLE1  AA:BB:CC  -75  3m   │  info box Y=342
 *  │ HP1   DD:EE:FF  -82  6m   │
 *  │ BLE2  11:22:33  -79  4m   │
 *  ├────────────────────────────┤  Y=430
 *  │ HDG 045°    TGT 03 ▼  ●●  │  status  Y=438
 *  └────────────────────────────┘  Y=456
 * --------------------------------------------------------------------- */

#define SCREEN_W        280
#define SCREEN_H        456

/* Radar root: 220x220, lv_obj_center on 280x456 → top-left = (30, 118) */
#define RADAR_ROOT_X    30
#define RADAR_ROOT_Y    118
#define RADAR_SIZE      220
#define RADAR_CENTER_X  (RADAR_ROOT_X + RADAR_SIZE/2)   /* 140 */
#define RADAR_CENTER_Y  (RADAR_ROOT_Y + RADAR_SIZE/2)   /* 228 */
#define RADAR_OUTER_PX  108
#define TAP_RADIUS      20

/* Info box below radar */
#define INFO_X          0
#define INFO_Y          340
#define INFO_W          SCREEN_W
#define INFO_H          88
#define INFO_LINE_H     18

/* Status bar */
#define STATUS_Y        (-8)
#define SIG_SEG_W       6
#define SIG_SEG_H_MAX   10
#define SIG_SEG_GAP     3
#define SIG_SEGS        4

#define HDR_FONT        (&lv_font_montserrat_16)
#define SMALL_FONT      (&lv_font_montserrat_12)

/* -----------------------------------------------------------------------
 * State
 * --------------------------------------------------------------------- */
static lv_obj_t *s_screen       = NULL;
static lv_obj_t *s_hdg_label    = NULL;
static lv_obj_t *s_tgt_label    = NULL;
static lv_obj_t *s_sig_seg[SIG_SEGS];

/* Dot labels — CHILDREN OF SCREEN, positioned in screen coordinates */
#define MAX_LABELS RADAR_MAX_TARGETS
static lv_obj_t *s_dot_label[MAX_LABELS];
static char      s_dot_names[MAX_LABELS][10];

/* Info box — fixed panel below radar */
static lv_obj_t *s_info_box     = NULL;
static lv_obj_t *s_info_list    = NULL;   /* scrollable inner */

/* Detail popup on tap */
static lv_obj_t *s_detail       = NULL;
static lv_obj_t *s_bat_label    = NULL;
static bool      s_detail_vis   = false;

/* Touch */
static lv_timer_t *s_touch_tmr  = NULL;

/* Device cache */
static tracked_device_t s_dev[RADAR_MAX_TARGETS];
static int              s_dev_count = 0;
static float            s_heading   = 0.0f;
static uint8_t          s_tgt_count = 0;
static uint8_t          s_sig_level = 0;

/* -----------------------------------------------------------------------
 * Short name generator
 * --------------------------------------------------------------------- */
static void make_names(void)
{
    /* Counters per device type */
    int n_md=0, n_rt=0, n_pc=0, n_hp=0, n_wr=0, n_cam=0, n_iot=0, n_uk=0;

    for (int i = 0; i < s_dev_count; i++) {
        device_type_t t = s_dev[i].classification.type;
        switch (t) {
            case DEV_TYPE_BLE_PHONE:
                /* MD = confirmed Mobile Device (OUI matched phone maker) */
                snprintf(s_dot_names[i], 10, "MD%d", ++n_md);
                break;
            case DEV_TYPE_WIFI_CLIENT:
                /* MD = likely mobile device sending probe request
                 * (randomised MAC prevents OUI identification) */
                snprintf(s_dot_names[i], 10, "MD%d", ++n_md);
                break;
            case DEV_TYPE_WIFI_AP:
                /* RT = Router */
                snprintf(s_dot_names[i], 10, "RT%d", ++n_rt);
                break;
            case DEV_TYPE_BLE_LAPTOP:
                /* PC = Laptop/Computer */
                snprintf(s_dot_names[i], 10, "PC%d", ++n_pc);
                break;
            case DEV_TYPE_BLE_AUDIO:
                /* HP = Headphones/Audio */
                snprintf(s_dot_names[i], 10, "HP%d", ++n_hp);
                break;
            case DEV_TYPE_BLE_WEARABLE:
                /* WR = Wearable (watch, band) */
                snprintf(s_dot_names[i], 10, "WR%d", ++n_wr);
                break;
            case DEV_TYPE_CAMERA_SUSPECTED:
                /* CM = Camera */
                snprintf(s_dot_names[i], 10, "CM%d", ++n_cam);
                break;
            case DEV_TYPE_BLE_GENERIC:
                /* IOT = Generic BLE device */
                snprintf(s_dot_names[i], 10, "IOT%d", ++n_iot);
                break;
            default:
                /* UK = Unknown */
                snprintf(s_dot_names[i], 10, "UK%d", ++n_uk);
                break;
        }
    }
}

/* -----------------------------------------------------------------------
 * Polar → screen pixel
 * --------------------------------------------------------------------- */
static void polar_to_screen(float angle, float radius, int *sx, int *sy)
{
    float rad = (angle - 90.0f) * ((float)M_PI / 180.0f);
    float rpx = radius * (float)RADAR_OUTER_PX;
    *sx = RADAR_CENTER_X + (int)(rpx * cosf(rad));
    *sy = RADAR_CENTER_Y + (int)(rpx * sinf(rad));
}

/* -----------------------------------------------------------------------
 * Separator
 * --------------------------------------------------------------------- */
static void add_sep(lv_obj_t *p, lv_align_t a, int y)
{
    lv_obj_t *l = lv_obj_create(p);
    lv_obj_remove_style_all(l);
    lv_obj_set_size(l, SCREEN_W, 1);
    lv_obj_set_style_bg_color(l, lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_bg_opa(l, LV_OPA_COVER, 0);
    lv_obj_align(l, a, 0, y);
    lv_obj_clear_flag(l, LV_OBJ_FLAG_CLICKABLE);
}

/* -----------------------------------------------------------------------
 * Detail popup (appears on dot tap, covers info box area)
 * --------------------------------------------------------------------- */
static void detail_close_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    lv_obj_add_flag(s_detail, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_info_box, LV_OBJ_FLAG_HIDDEN);
    s_detail_vis = false;
}

static void build_detail(lv_obj_t *parent)
{
    s_detail = lv_obj_create(parent);
    lv_obj_remove_style_all(s_detail);
    lv_obj_set_size(s_detail, SCREEN_W - 8, INFO_H + 40);
    lv_obj_set_pos(s_detail, 4, INFO_Y - 40);
    lv_obj_set_style_bg_color(s_detail, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_detail, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(s_detail,
        lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_border_width(s_detail, 1, 0);
    lv_obj_set_style_pad_all(s_detail, 6, 0);
    lv_obj_clear_flag(s_detail, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_detail, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *close = lv_label_create(s_detail);
    lv_label_set_text(close, "[X]");
    lv_obj_set_style_text_color(close,
        lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_text_font(close, SMALL_FONT, 0);
    lv_obj_align(close, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_add_flag(close, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(close, detail_close_cb, LV_EVENT_CLICKED, NULL);
}

static void show_detail(int idx)
{
    if (!s_detail || idx < 0 || idx >= s_dev_count) return;
    const tracked_device_t *d = &s_dev[idx];
    char buf[200];
    uint32_t age = ((uint32_t)(esp_timer_get_time()/1000) -
                    d->last_seen_ms) / 1000;

    snprintf(buf, sizeof(buf),
        "%s  %s\n"
        "MAC: %02X:%02X:%02X:%02X:%02X:%02X\n"
        "RSSI: %ddBm   DIST: %.0fm\n"
        "BEAR: %.0f\xc2\xb0   CONF: %d%%   %lus ago",
        s_dot_names[idx],
        device_type_label(d->classification.type),
        d->mac[0], d->mac[1], d->mac[2],
        d->mac[3], d->mac[4], d->mac[5],
        d->rssi_smoothed,
        d->estimated_distance_m,
        d->estimated_bearing_deg,
        d->bearing_confidence_pct,
        (unsigned long)age);

    /* Remove old content except close button */
    uint32_t child_cnt = lv_obj_get_child_cnt(s_detail);
    for (int i = (int)child_cnt - 1; i >= 1; i--) {
        lv_obj_del(lv_obj_get_child(s_detail, i));
    }

    lv_obj_t *lbl = lv_label_create(s_detail);
    lv_obj_remove_style_all(lbl);
    lv_label_set_text(lbl, buf);
    lv_obj_set_style_text_color(lbl,
        lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_text_font(lbl, SMALL_FONT, 0);
    lv_obj_set_pos(lbl, 0, 16);
    lv_obj_set_width(lbl, SCREEN_W - 20);

    lv_obj_add_flag(s_info_box, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_detail, LV_OBJ_FLAG_HIDDEN);
    s_detail_vis = true;
}

/* -----------------------------------------------------------------------
 * Info box — fixed list below radar
 * --------------------------------------------------------------------- */
static void build_info_box(lv_obj_t *parent)
{
    s_info_box = lv_obj_create(parent);
    lv_obj_remove_style_all(s_info_box);
    lv_obj_set_size(s_info_box, INFO_W, INFO_H);
    lv_obj_set_pos(s_info_box, INFO_X, INFO_Y);
    lv_obj_set_style_bg_color(s_info_box, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_info_box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(s_info_box,
        lv_palette_darken(LV_PALETTE_ORANGE, 2), 0);
    lv_obj_set_style_border_width(s_info_box, 1, 0);
    lv_obj_set_style_pad_all(s_info_box, 4, 0);
    lv_obj_set_style_pad_gap(s_info_box, 0, 0);

    /* Scrollable inner container */
    s_info_list = lv_obj_create(s_info_box);
    lv_obj_remove_style_all(s_info_list);
    lv_obj_set_size(s_info_list, INFO_W - 8, LV_SIZE_CONTENT);
    lv_obj_set_pos(s_info_list, 0, 0);
    lv_obj_set_style_bg_opa(s_info_list, LV_OPA_TRANSP, 0);
    lv_obj_set_flex_flow(s_info_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_info_list, 1, 0);
    lv_obj_set_style_pad_all(s_info_list, 0, 0);
}

static void refresh_info_box(void)
{
    if (!s_info_list) return;
    lv_obj_clean(s_info_list);

    if (s_dev_count == 0) {
        lv_obj_t *r = lv_label_create(s_info_list);
        lv_obj_remove_style_all(r);
        lv_obj_set_style_text_color(r,
            lv_palette_main(LV_PALETTE_ORANGE), 0);
        lv_obj_set_style_text_font(r, SMALL_FONT, 0);
        lv_label_set_text(r, "  Scanning...");
        return;
    }

    for (int i = 0; i < s_dev_count; i++) {
        const tracked_device_t *d = &s_dev[i];
        char buf[60];
        snprintf(buf, sizeof(buf),
            "%-5s %02X:%02X:%02X:%02X:%02X:%02X %ddBm %.0fm",
            s_dot_names[i],
            d->mac[0], d->mac[1], d->mac[2],
            d->mac[3], d->mac[4], d->mac[5],
            d->rssi_smoothed,
            d->estimated_distance_m);

        lv_obj_t *row = lv_label_create(s_info_list);
        lv_obj_remove_style_all(row);
        lv_label_set_long_mode(row, LV_LABEL_LONG_DOT);
        lv_obj_set_width(row, INFO_W - 8);
        lv_obj_set_style_text_font(row, SMALL_FONT, 0);

        lv_obj_set_style_text_color(row, lv_color_white(), 0);

        lv_label_set_text(row, buf);
    }
}

/* -----------------------------------------------------------------------
 * Touch
 * --------------------------------------------------------------------- */
static void touch_cb(lv_timer_t *t)
{
    LV_UNUSED(t);
    uint16_t tx, ty;
    if (!getTouch(&tx, &ty)) return;

    /* Dismiss detail on tap outside */
    if (s_detail_vis) {
        detail_close_cb(NULL);
        return;
    }

    /* Tap on radar — find nearest dot */
    const radar_target_t *tgts = radar_engine_get_targets();
    uint8_t cnt = radar_engine_target_count();
    int best = -1;
    float best_d2 = TAP_RADIUS * TAP_RADIUS;

    for (int i = 0; i < (int)cnt && i < s_dev_count; i++) {
        if (!tgts[i].active) continue;
        if (tgts[i].screen_radius > 1.0f) continue;
        int px, py;
        polar_to_screen(tgts[i].screen_angle, tgts[i].screen_radius,
                        &px, &py);
        float dx = tx - px, dy = ty - py;
        float d2 = dx*dx + dy*dy;
        if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    if (best >= 0) show_detail(best);
}

/* -----------------------------------------------------------------------
 * Signal bar
 * --------------------------------------------------------------------- */
static void build_sig_bar(lv_obj_t *p)
{
    int bx = SCREEN_W - 8 - SIG_SEGS*(SIG_SEG_W+SIG_SEG_GAP);
    int by = SCREEN_H - 14;
    for (int i = 0; i < SIG_SEGS; i++) {
        int h = SIG_SEG_H_MAX*(i+1)/SIG_SEGS;
        lv_obj_t *s = lv_obj_create(p);
        lv_obj_remove_style_all(s);
        lv_obj_set_size(s, SIG_SEG_W, h);
        lv_obj_set_pos(s, bx+i*(SIG_SEG_W+SIG_SEG_GAP), by-h);
        lv_obj_set_style_bg_color(s,
            lv_palette_main(LV_PALETTE_ORANGE), 0);
        lv_obj_set_style_bg_opa(s, LV_OPA_30, 0);
        lv_obj_set_style_radius(s, 1, 0);
        lv_obj_clear_flag(s, LV_OBJ_FLAG_CLICKABLE);
        s_sig_seg[i] = s;
    }
}

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */
lv_obj_t *radar_screen_create(void)
{
    s_screen = lv_obj_create(NULL);
    lv_obj_clear_flag(s_screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(s_screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_screen, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_screen, 0, 0);
    lv_obj_set_style_pad_all(s_screen, 0, 0);

    /* Header */
    lv_obj_t *hdr = lv_label_create(s_screen);
    lv_label_set_text(hdr, "ONO-SENDAI   MS-01");
    lv_obj_set_style_text_color(hdr,
        lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_text_font(hdr, HDR_FONT, 0);
    lv_obj_align(hdr, LV_ALIGN_TOP_LEFT, 8, 6);

    /* Battery label - top right */
    s_bat_label = lv_label_create(s_screen);
    lv_label_set_text(s_bat_label, "BAT ?");
    lv_obj_set_style_text_color(s_bat_label,
        lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_text_font(s_bat_label, SMALL_FONT, 0);
    lv_obj_align(s_bat_label, LV_ALIGN_TOP_RIGHT, -6, 8);

    add_sep(s_screen, LV_ALIGN_TOP_MID, 28);

    /* Radar */
    radar_renderer_create(s_screen);

    add_sep(s_screen, LV_ALIGN_TOP_MID, INFO_Y - 2);

    /* Info box */
    build_info_box(s_screen);

    add_sep(s_screen, LV_ALIGN_BOTTOM_MID, -26);

    /* HDG label */
    s_hdg_label = lv_label_create(s_screen);
    lv_label_set_text(s_hdg_label, "HDG ---\xc2\xb0");
    lv_obj_set_style_text_color(s_hdg_label,
        lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_text_font(s_hdg_label, SMALL_FONT, 0);
    lv_obj_align(s_hdg_label, LV_ALIGN_BOTTOM_LEFT, 8, STATUS_Y);

    /* TGT label */
    s_tgt_label = lv_label_create(s_screen);
    lv_label_set_text(s_tgt_label, "TGT --");
    lv_obj_set_style_text_color(s_tgt_label,
        lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_text_font(s_tgt_label, SMALL_FONT, 0);
    lv_obj_align(s_tgt_label, LV_ALIGN_BOTTOM_MID, 0, STATUS_Y);

    build_sig_bar(s_screen);

    /* Dot labels — children of screen, screen-absolute coordinates */
    for (int i = 0; i < MAX_LABELS; i++) {
        lv_obj_t *lbl = lv_label_create(s_screen);
        lv_obj_remove_style_all(lbl);
        lv_label_set_text(lbl, "");
        lv_obj_set_style_text_font(lbl, SMALL_FONT, 0);
        lv_obj_set_style_text_color(lbl,
            lv_color_white(), 0);
        lv_obj_add_flag(lbl, LV_OBJ_FLAG_HIDDEN);
        s_dot_label[i] = lbl;
    }

    /* Detail popup */
    build_detail(s_screen);

    touch_Init();
    s_touch_tmr = lv_timer_create(touch_cb, 50, NULL);

    return s_screen;
}

void radar_screen_set_battery(int status)
{
    if (!s_bat_label) return;
    /* status: -2 = USB charging, -1 = on battery */
    if (status == -2) {
        lv_label_set_text(s_bat_label, "USB");
        lv_obj_set_style_text_color(s_bat_label,
            lv_color_make(0, 220, 80), 0);   /* green = charging */
    } else {
        lv_label_set_text(s_bat_label, "BAT");
        lv_obj_set_style_text_color(s_bat_label,
            lv_palette_main(LV_PALETTE_ORANGE), 0);
    }
}

void radar_screen_set_heading(float h)
{
    s_heading = h;
    radar_engine_set_heading(h);
    radar_renderer_set_heading(h);
}

void radar_screen_set_target_count(uint8_t c) { s_tgt_count = c; }
void radar_screen_set_signal_level(uint8_t l)
{
    s_sig_level = l < SIG_SEGS ? l : SIG_SEGS;
}

void radar_screen_set_device_name(const char *type, const char *name)
{
    (void)type; (void)name; /* handled via info box now */
}

void radar_screen_update_devices(const tracked_device_t *d, int n)
{
    if (!d || n <= 0) { s_dev_count = 0; return; }
    int cnt = n < RADAR_MAX_TARGETS ? n : RADAR_MAX_TARGETS;
    memcpy(s_dev, d, cnt * sizeof(tracked_device_t));
    s_dev_count = cnt;
    make_names();
    if (!s_detail_vis) refresh_info_box();
}

void radar_screen_show_detail(int idx) { show_detail(idx); }
void radar_screen_show_list(void) { /* info box always visible */ }
void radar_screen_hide_detail(void)
{
    if (s_detail) lv_obj_add_flag(s_detail, LV_OBJ_FLAG_HIDDEN);
    if (s_info_box) lv_obj_clear_flag(s_info_box, LV_OBJ_FLAG_HIDDEN);
    s_detail_vis = false;
}

void radar_screen_update(void)
{
    radar_renderer_render();

    /* HDG */
    if (s_hdg_label) {
        char buf[16];
        int h = (int)s_heading % 360;
        if (h < 0) h += 360;
        snprintf(buf, sizeof(buf), "HDG %03d\xc2\xb0", h);
        lv_label_set_text(s_hdg_label, buf);
    }

    /* TGT */
    if (s_tgt_label) {
        char buf[12];
        snprintf(buf, sizeof(buf), "TGT %02u", s_tgt_count);
        lv_label_set_text(s_tgt_label, buf);
    }

    /* Signal bar */
    for (int i = 0; i < SIG_SEGS; i++) {
        if (!s_sig_seg[i]) continue;
        lv_obj_set_style_bg_opa(s_sig_seg[i],
            i < s_sig_level ? LV_OPA_COVER : LV_OPA_30, 0);
    }

    /* Dot labels — placed at screen coordinates next to each dot */
    const radar_target_t *tgts = radar_engine_get_targets();
    uint8_t cnt = radar_engine_target_count();

    for (int i = 0; i < MAX_LABELS; i++) {
        if (!s_dot_label[i]) continue;

        if (i >= (int)cnt || i >= s_dev_count ||
            !tgts[i].active ||
            tgts[i].screen_radius < 0.0f ||
            tgts[i].screen_radius > 1.0f) {
            lv_obj_add_flag(s_dot_label[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }

        int sx, sy;
        polar_to_screen(tgts[i].screen_angle,
                        tgts[i].screen_radius, &sx, &sy);

        /* Keep label inside screen */
        if (sx < RADAR_ROOT_X || sx > RADAR_ROOT_X + RADAR_SIZE ||
            sy < RADAR_ROOT_Y || sy > RADAR_ROOT_Y + RADAR_SIZE) {
            lv_obj_add_flag(s_dot_label[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }

        /* Offset label 8px right of dot */
        int lx = sx + 8;
        int ly = sy - 6;
        /* Clamp to screen */
        if (lx > SCREEN_W - 36) lx = sx - 36;
        if (ly < RADAR_ROOT_Y) ly = sy + 4;

        lv_obj_set_pos(s_dot_label[i], lx, ly);
        lv_label_set_text(s_dot_label[i], s_dot_names[i]);

        lv_obj_set_style_text_color(s_dot_label[i],
            lv_color_white(), 0);

        lv_obj_clear_flag(s_dot_label[i], LV_OBJ_FLAG_HIDDEN);
    }
}
