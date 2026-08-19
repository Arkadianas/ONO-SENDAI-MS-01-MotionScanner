#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "localization.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char *TAG = "localization";

static tracked_device_t s_devices[APP_MAX_TRACKED_DEVICES];
static int s_device_count = 0;              /* počet použitých slotov (vrátane neaktívnych) */
static SemaphoreHandle_t s_mutex = NULL;

/* Calibrated for ESP32-S3 + external 2.4GHz antenna.
 * rssi0_dbm = expected RSSI at 1 metre with this hardware (-60 dBm typical).
 * pathloss_n = 2.5 (indoor, light walls). Range: 2.0 open, 3.5 heavy walls.
 * To recalibrate: hold a known device at exactly 1m, read its RSSI from the
 * FEED log, and set rssi0_dbm to that value. */
static pathloss_profile_t s_profile = {
    .rssi0_dbm = -70,   /* calibrated: ~-70 dBm at 1m for this hardware */
    .ref_distance_m = APP_DEFAULT_REF_DIST_M,
    .pathloss_n = 2.0f, /* indoor pathloss exponent */
};


void localization_init(void) {
    memset(s_devices, 0, sizeof(s_devices));
    s_device_count = 0;
    s_mutex = xSemaphoreCreateMutex();
}

void localization_set_pathloss_profile(const pathloss_profile_t *profile) {
    if (!profile) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_profile = *profile;
    xSemaphoreGive(s_mutex);
}

pathloss_profile_t localization_get_pathloss_profile(void) {
    pathloss_profile_t p;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    p = s_profile;
    xSemaphoreGive(s_mutex);
    return p;
}

static float rssi_to_distance_m(int8_t rssi, const pathloss_profile_t *p) {
    /* d = d0 * 10 ^ ((RSSI0 - RSSI) / (10*n))  [log-distance path-loss model] */
    float exponent = (p->rssi0_dbm - (float)rssi) / (10.0f * p->pathloss_n);
    float d = p->ref_distance_m * powf(10.0f, exponent);
    if (d < 0.1f) d = 0.1f;
    if (d > 100.0f) d = 100.0f; /* orezanie nezmyselných extrémov */
    return d;
}

static int find_or_create_slot(radio_kind_t radio, const uint8_t mac[6]) {
    int free_slot = -1;
    for (int i = 0; i < APP_MAX_TRACKED_DEVICES; i++) {
        if (s_devices[i].active && s_devices[i].radio == radio &&
            memcmp(s_devices[i].mac, mac, 6) == 0) {
            return i;
        }
        if (!s_devices[i].active && free_slot < 0) {
            free_slot = i;
        }
    }
    if (free_slot >= 0) {
        memset(&s_devices[free_slot], 0, sizeof(tracked_device_t));
        memcpy(s_devices[free_slot].mac, mac, 6);
        s_devices[free_slot].radio = radio;
        s_devices[free_slot].active = true;
        s_devices[free_slot].first_seen_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (free_slot + 1 > s_device_count) s_device_count = free_slot + 1;
        return free_slot;
    }
    return -1; /* registry plný */
}

static void update_rssi_history(tracked_device_t *d, int8_t rssi) {
    d->rssi_history[d->rssi_history_idx] = rssi;
    d->rssi_history_idx = (d->rssi_history_idx + 1) % APP_RSSI_HISTORY_LEN;
    if (d->rssi_history_len < APP_RSSI_HISTORY_LEN) d->rssi_history_len++;

    /* Exponenciálne vyhladenie (EMA) - potláča RSSI šum bez veľkého oneskorenia */
    const float alpha = 0.3f;
    if (d->rssi_smoothed == 0 && d->rssi_history_len == 1) {
        d->rssi_smoothed = rssi;
    } else {
        d->rssi_smoothed = (int8_t)lroundf(alpha * rssi + (1.0f - alpha) * d->rssi_smoothed);
    }
}

static int heading_to_bin(float heading_deg) {
    int bin = (int)(heading_deg / (360.0f / APP_BEARING_BINS));
    if (bin < 0) bin = 0;
    if (bin >= APP_BEARING_BINS) bin = APP_BEARING_BINS - 1;
    return bin;
}

static void update_bearing_histogram(tracked_device_t *d, int8_t rssi, float heading_deg) {
    int bin = heading_to_bin(heading_deg);
    /* Váha = normalizovaná "sila signálu" (silnejší RSSI = vyššia váha).
     * RSSI je záporné číslo (napr. -80..-30), preto posun o +100 aby bola
     * váha vždy kladná a väčšia pre silnejší signál. */
    float weight = (float)(rssi + 100);
    if (weight < 1.0f) weight = 1.0f;

    d->bearing_weight[bin] += weight;
    d->bearing_count[bin]++;

    /* Circular weighted mean cez všetky biny - robustnejšie než len "najsilnejší bin",
     * pretože zohľadňuje aj susedné uhly kam signál "presakuje". */
    float sum_sin = 0.0f, sum_cos = 0.0f, total_weight = 0.0f;
    int bins_with_data = 0;
    for (int i = 0; i < APP_BEARING_BINS; i++) {
        if (d->bearing_count[i] == 0) continue;
        bins_with_data++;
        float bin_center_deg = (i + 0.5f) * (360.0f / APP_BEARING_BINS);
        float rad = bin_center_deg * (float)M_PI / 180.0f;
        float w = d->bearing_weight[i];
        sum_sin += sinf(rad) * w;
        sum_cos += cosf(rad) * w;
        total_weight += w;
    }
    if (total_weight > 0.0f) {
        float mean_rad = atan2f(sum_sin, sum_cos);
        float mean_deg = mean_rad * 180.0f / (float)M_PI;
        if (mean_deg < 0) mean_deg += 360.0f;
        d->estimated_bearing_deg = mean_deg;
    }

    /* Confidence rastie s pokrytím rôznych uhlov (viac binov = užívateľ sa
     * viac otočil = spoľahlivejší odhad) a s počtom vzoriek. */
    float coverage_ratio = (float)bins_with_data / (float)APP_BEARING_BINS;
    float sample_ratio = fminf(1.0f, total_weight / 500.0f);
    d->bearing_confidence_pct = (uint8_t)(100.0f * (0.6f * coverage_ratio + 0.4f * sample_ratio));
}

void localization_ingest_sample(radio_kind_t radio, const uint8_t mac[6],
                                 int8_t rssi, const char *name_or_ssid,
                                 float current_heading_deg,
                                 const classification_input_t *classify_in) {
    if (!s_mutex) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    int idx = find_or_create_slot(radio, mac);
    if (idx < 0) {
        ESP_LOGW(TAG, "Device registry plný (%d), vzorka zahodená", APP_MAX_TRACKED_DEVICES);
        xSemaphoreGive(s_mutex);
        return;
    }
    tracked_device_t *d = &s_devices[idx];

    update_rssi_history(d, rssi);
    update_bearing_histogram(d, rssi, current_heading_deg);

    pathloss_profile_t profile = s_profile;
    d->estimated_distance_m = rssi_to_distance_m(d->rssi_smoothed, &profile);

    if (name_or_ssid) {
        strncpy(d->ssid_or_name, name_or_ssid, sizeof(d->ssid_or_name) - 1);
    }
    if (classify_in) {
        d->classification = device_classify(classify_in);
    }

    d->last_seen_ms = (uint32_t)(esp_timer_get_time() / 1000);
    xSemaphoreGive(s_mutex);
}

void localization_expire_stale(uint32_t now_ms, uint32_t timeout_ms) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < s_device_count; i++) {
        if (s_devices[i].active && (now_ms - s_devices[i].last_seen_ms) > timeout_ms) {
            s_devices[i].active = false;
        }
    }
    xSemaphoreGive(s_mutex);
}

int localization_get_devices(tracked_device_t *out_array, int max_out) {
    if (!out_array) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int n = 0;
    for (int i = 0; i < s_device_count && n < max_out; i++) {
        if (s_devices[i].active) {
            out_array[n++] = s_devices[i];
        }
    }
    xSemaphoreGive(s_mutex);
    return n;
}

int localization_get_device_count(void) {
    int n = 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < s_device_count; i++) {
        if (s_devices[i].active) n++;
    }
    xSemaphoreGive(s_mutex);
    return n;
}

void localization_get_relative_xy(const tracked_device_t *dev, float *out_dx, float *out_dy) {
    if (!dev || !out_dx || !out_dy) return;
    float rad = dev->estimated_bearing_deg * (float)M_PI / 180.0f;
    *out_dx = dev->estimated_distance_m * sinf(rad);  /* X = "right/east" v scanner rámci */
    *out_dy = dev->estimated_distance_m * cosf(rad);  /* Y = "forward/north" */
}

void localization_clear_all(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memset(s_devices, 0, sizeof(s_devices));
    s_device_count = 0;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "Device registry cleared");
}
