#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "device_classification.h"
#include "app_config.h"

typedef enum {
    RADIO_WIFI,
    RADIO_BLE,
} radio_kind_t;

typedef struct {
    uint8_t mac[6];
    radio_kind_t radio;

    /* RSSI história (kruhový buffer) */
    int8_t rssi_history[APP_RSSI_HISTORY_LEN];
    uint8_t rssi_history_len;
    uint8_t rssi_history_idx;
    int8_t rssi_smoothed;         /* EMA vyhladené RSSI */

    /* Bearing histogram: suma RSSI váh a počet vzoriek na uhlový bin */
    float bearing_weight[APP_BEARING_BINS];
    uint16_t bearing_count[APP_BEARING_BINS];
    float estimated_bearing_deg;  /* circular-weighted-mean výsledok */
    uint8_t bearing_confidence_pct;

    /* Odhad vzdialenosti z path-loss modelu */
    float estimated_distance_m;

    /* Klasifikácia */
    classification_result_t classification;
    char ssid_or_name[33];

    uint32_t first_seen_ms;
    uint32_t last_seen_ms;
    bool active;                  /* false = vypadlo z dosahu / timeout, slot voľný */
} tracked_device_t;

/* Path-loss kalibračné konštanty (menia sa cez calibration modul) */
typedef struct {
    int16_t rssi0_dbm;     /* RSSI pri referenčnej vzdialenosti */
    float ref_distance_m;
    float pathloss_n;      /* environment koeficient */
} pathloss_profile_t;

void localization_init(void);

/* Nastaví/aktualizuje kalibračný profil path-loss modelu. */
void localization_set_pathloss_profile(const pathloss_profile_t *profile);
pathloss_profile_t localization_get_pathloss_profile(void);

/* Zavolaj pri prijatí WiFi/BLE vzorky. current_heading_deg príde z imu_tracking.
 * Interne aktualizuje/vytvorí záznam v device registry, prepočíta vzdialenosť
 * a bearing histogram. */
void localization_ingest_sample(radio_kind_t radio, const uint8_t mac[6],
                                 int8_t rssi, const char *name_or_ssid,
                                 float current_heading_deg,
                                 const classification_input_t *classify_in);

/* Odstráni záznamy, ktoré neboli videné dlhšie ako timeout_ms. */
void localization_expire_stale(uint32_t now_ms, uint32_t timeout_ms);

/* Prístup k registru pre UI vrstvu (READ-ONLY z pohľadu volajúceho). */
int localization_get_devices(tracked_device_t *out_array, int max_out);
int localization_get_device_count(void);

/* Vráti smerový vektor (dx, dy) v "scanner-relatívnom" súradnicovom systéme
 * na základe estimated_bearing_deg a estimated_distance_m - vhodné priamo
 * pre vykreslenie blipu na radare. */
void localization_get_relative_xy(const tracked_device_t *dev, float *out_dx, float *out_dy);

/* Clear all tracked devices (call on radio mode switch) */
void localization_clear_all(void);
