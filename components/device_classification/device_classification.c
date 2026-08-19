#include "device_classification.h"
#include <string.h>
#include <strings.h>
#include <stdio.h>

/* -----------------------------------------------------------------------
 * OUI table — camera and security device manufacturers
 * --------------------------------------------------------------------- */
typedef struct { uint8_t oui[3]; const char *vendor; } oui_entry_t;

static const oui_entry_t s_camera_oui_table[] = {
    {{0x28,0x57,0xBE}, "Hikvision"},
    {{0x4C,0x11,0xBF}, "Hikvision"},
    {{0xC0,0x51,0x7E}, "Hikvision"},
    {{0x44,0x19,0xB6}, "Hikvision"},
    {{0x00,0x0F,0x7C}, "Dahua"},
    {{0x3C,0xEF,0x8C}, "Dahua"},
    {{0x9C,0x8E,0xCD}, "Dahua"},
    {{0x00,0x1C,0xA2}, "Foscam"},
    {{0x00,0x40,0x8C}, "Axis"},
    {{0xAC,0xCC,0x8E}, "Axis"},
    {{0x00,0x18,0x85}, "Vivotek"},
    {{0x00,0x0E,0xC6}, "Ubiquiti"},
    {{0xB0,0xC5,0x54}, "TP-Link Tapo"},
    {{0x50,0x91,0xE3}, "TP-Link Tapo"},
};

/* -----------------------------------------------------------------------
 * OUI table — phone manufacturers
 * These MACs appear on real (non-randomised) BLE advertisements
 * --------------------------------------------------------------------- */
static const oui_entry_t s_phone_oui_table[] = {
    {{0xF4,0xF1,0x9A}, "Samsung"},
    {{0x8C,0xF5,0xA3}, "Samsung"},
    {{0xCC,0x2D,0xE0}, "Samsung"},
    {{0x00,0x26,0x37}, "Samsung"},
    {{0x54,0xEE,0x75}, "Apple"},
    {{0xAC,0xBC,0x32}, "Apple"},
    {{0xF0,0xDB,0xE2}, "Apple"},
    {{0x00,0x23,0x12}, "Apple"},
    {{0xA8,0x66,0x7F}, "Apple"},
    {{0xDC,0x2B,0x2A}, "Huawei"},
    {{0xAC,0x4E,0x91}, "Huawei"},
    {{0x00,0xE0,0xFC}, "Huawei"},
    {{0x38,0xD5,0x47}, "Xiaomi"},
    {{0x28,0x6C,0x07}, "Xiaomi"},
    {{0xCC,0x2D,0x83}, "OnePlus"},
    {{0x8C,0xBE,0xBE}, "Google"},
    {{0xF8,0x8A,0x76}, "Sony"},
    {{0x00,0x1D,0xBA}, "Sony"},
};

/* -----------------------------------------------------------------------
 * OUI table — audio devices (headphones, earbuds, speakers)
 * --------------------------------------------------------------------- */
static const oui_entry_t s_audio_oui_table[] = {
    {{0x94,0x16,0x25}, "JBL"},
    {{0x00,0x1B,0xDC}, "JBL/Harman"},
    {{0xA0,0xE9,0xDB}, "Bose"},
    {{0x04,0x52,0xC7}, "Bose"},
    {{0xAC,0x7A,0x4D}, "Sony Audio"},
    {{0x14,0x3F,0xA6}, "Sony Audio"},
    {{0x00,0x24,0x33}, "Sony"},
    {{0x28,0x11,0xA5}, "Samsung Audio"},   /* Galaxy Buds */
    {{0xEC,0x2C,0x09}, "Samsung Buds"},
    {{0x5C,0xAA,0xFD}, "Apple AirPods"},
    {{0x38,0xCA,0xDA}, "Apple AirPods"},
    {{0x00,0x30,0xCD}, "Jabra"},
    {{0x50,0xC2,0xED}, "Jabra"},
    {{0x9C,0xB7,0x0D}, "Plantronics"},
    {{0x00,0x1F,0x20}, "Plantronics"},
    {{0x44,0x5C,0xE9}, "Sennheiser"},
    {{0x00,0x1B,0x66}, "Sennheiser"},
    {{0xCC,0x98,0x8B}, "Beats"},
    {{0xB8,0xC1,0x11}, "Anker/Soundcore"},
};

/* -----------------------------------------------------------------------
 * OUI table — laptop/PC manufacturers
 * --------------------------------------------------------------------- */
static const oui_entry_t s_laptop_oui_table[] = {
    {{0x8C,0x85,0x90}, "Dell"},
    {{0xBC,0xEE,0x7B}, "Dell"},
    {{0x18,0x03,0x73}, "Dell"},
    {{0x98,0xFA,0x9B}, "HP"},
    {{0x9C,0xB6,0xD0}, "HP"},
    {{0x00,0x17,0xF2}, "Apple Mac"},
    {{0x60,0xF8,0x1D}, "Lenovo"},
    {{0x00,0x0E,0x61}, "Lenovo"},
    {{0xD0,0xC6,0x37}, "Microsoft"},
    {{0x28,0x18,0x78}, "Asus"},
    {{0x00,0x26,0x18}, "Asus"},
};

static const char *s_camera_keywords[] = {
    "camera","cam-","ipcam","ip-cam","nvr","dvr","hik","dahua",
    "foscam","wyze","eufy","ring","nest","yi-cam","reolink","tapo","annke",
};

static const char *s_audio_keywords[] = {
    "buds","airpods","headphone","headset","earphone","earbuds","speaker",
    "jbl","bose","sony wh","sony wf","sennheiser","jabra","plantronics",
    "beats","soundcore","anker","galaxy buds","wh-","wf-","ath-",
};

static const char *s_phone_keywords[] = {
    "iphone","samsung","galaxy","huawei","xiaomi","pixel","oneplus",
    "redmi","oppo","vivo","realme","motorola","nokia",
};

/* -----------------------------------------------------------------------
 * Helpers
 * --------------------------------------------------------------------- */
static bool mac_matches_oui(const uint8_t mac[6], const uint8_t oui[3]) {
    return mac[0]==oui[0] && mac[1]==oui[1] && mac[2]==oui[2];
}

static const char *find_oui(const uint8_t mac[6],
                             const oui_entry_t *table, size_t count) {
    for (size_t i = 0; i < count; i++) {
        if (mac_matches_oui(mac, table[i].oui)) return table[i].vendor;
    }
    return NULL;
}

#define FIND_CAMERA(mac) find_oui(mac, s_camera_oui_table, \
    sizeof(s_camera_oui_table)/sizeof(s_camera_oui_table[0]))
#define FIND_PHONE(mac)  find_oui(mac, s_phone_oui_table,  \
    sizeof(s_phone_oui_table)/sizeof(s_phone_oui_table[0]))
#define FIND_AUDIO(mac)  find_oui(mac, s_audio_oui_table,  \
    sizeof(s_audio_oui_table)/sizeof(s_audio_oui_table[0]))
#define FIND_LAPTOP(mac) find_oui(mac, s_laptop_oui_table, \
    sizeof(s_laptop_oui_table)/sizeof(s_laptop_oui_table[0]))

static bool text_has_keyword(const char *text,
                              const char **kw, size_t count) {
    if (!text || !text[0]) return false;
    for (size_t i = 0; i < count; i++) {
        if (strcasestr(text, kw[i])) return true;
    }
    return false;
}

#define HAS_CAMERA_KW(t) text_has_keyword(t, s_camera_keywords, \
    sizeof(s_camera_keywords)/sizeof(s_camera_keywords[0]))
#define HAS_AUDIO_KW(t)  text_has_keyword(t, s_audio_keywords,  \
    sizeof(s_audio_keywords)/sizeof(s_audio_keywords[0]))
#define HAS_PHONE_KW(t)  text_has_keyword(t, s_phone_keywords,  \
    sizeof(s_phone_keywords)/sizeof(s_phone_keywords[0]))

/* BLE Audio service UUIDs (Bluetooth SIG assigned) */
static bool is_audio_uuid(uint16_t uuid) {
    switch (uuid) {
        case 0x1108: /* Headset */
        case 0x110B: /* Audio Sink */
        case 0x110C: /* A/V Remote Control Target */
        case 0x110E: /* A/V Remote Control */
        case 0x111E: /* Handsfree */
        case 0x1131: /* Phonebook Access */
        case 0x184E: /* Audio Input Control */
        case 0x184F: /* Audio Output Control */
        case 0x1850: /* Hearing Access */
        case 0x1853: /* Telephone Bearer */
            return true;
        default: return false;
    }
}

/* BLE Appearance codes for audio (GAP Appearance, Bluetooth SIG) */
static bool is_audio_appearance(uint16_t appearance) {
    uint16_t cat = appearance >> 6;
    /* Category 0x08 = Audio/Media, includes headphones, earbuds, speakers */
    if (cat == 0x08) return true;
    /* Specific values */
    switch (appearance) {
        case 0x0941: /* Earbud */
        case 0x0942: /* Headset */
        case 0x0943: /* Headphones */
        case 0x0944: /* Neck Band */
            return true;
        default: return false;
    }
}

static bool is_wearable_uuid(uint16_t uuid) {
    switch (uuid) {
        case 0x180D: /* Heart Rate */
        case 0x1812: /* HID */
        case 0x1814: /* Running Speed */
        case 0x1816: /* Cycling Speed */
        case 0x181C: /* User Data */
        case 0x1826: /* Fitness Machine */
            return true;
        default: return false;
    }
}

/* -----------------------------------------------------------------------
 * Main classifier
 * --------------------------------------------------------------------- */
classification_result_t device_classify(const classification_input_t *in) {
    classification_result_t r = {
        .type = DEV_TYPE_UNKNOWN,
        .confidence_pct = 0,
        .reason = "no signal"
    };
    if (!in) return r;

    const char *name = in->is_ble ? in->ble_name : in->ssid;

    /* ── 1. Camera — highest priority ──────────────────────────────── */
    const char *cam_oui = FIND_CAMERA(in->mac);
    bool cam_kw = HAS_CAMERA_KW(name);
    if (cam_oui && cam_kw) {
        r.type = DEV_TYPE_CAMERA_SUSPECTED;
        r.confidence_pct = 90;
        r.reason = "OUI + keyword";
        return r;
    }
    if (cam_oui) {
        r.type = DEV_TYPE_CAMERA_SUSPECTED;
        r.confidence_pct = 65;
        r.reason = cam_oui;
        return r;
    }
    if (cam_kw) {
        r.type = DEV_TYPE_CAMERA_SUSPECTED;
        r.confidence_pct = 55;
        r.reason = "name keyword";
        return r;
    }

    /* ── 2. BLE devices ─────────────────────────────────────────────── */
    if (in->is_ble) {

        /* Audio — headphones, earbuds, speakers */
        const char *audio_oui = FIND_AUDIO(in->mac);
        bool audio_app = is_audio_appearance(in->ble_appearance);
        bool audio_uuid = in->has_16bit_service_uuid &&
                          is_audio_uuid(in->service_uuid_16);
        bool audio_kw = HAS_AUDIO_KW(name);

        if (audio_oui || audio_app || audio_uuid || audio_kw) {
            r.type = DEV_TYPE_BLE_AUDIO;
            r.confidence_pct = audio_oui ? 80 :
                               audio_app ? 75 :
                               audio_uuid ? 70 : 55;
            r.reason = audio_oui ? audio_oui :
                       audio_app ? "BLE appearance:audio" :
                       audio_uuid ? "audio UUID" : "name keyword";
            return r;
        }

        /* Phone */
        const char *phone_oui = FIND_PHONE(in->mac);
        bool phone_kw = HAS_PHONE_KW(name);
        if (phone_oui || phone_kw) {
            r.type = DEV_TYPE_BLE_PHONE;
            r.confidence_pct = phone_oui ? 75 : 60;
            r.reason = phone_oui ? phone_oui : "name keyword";
            return r;
        }

        /* Laptop */
        const char *laptop_oui = FIND_LAPTOP(in->mac);
        if (laptop_oui) {
            r.type = DEV_TYPE_BLE_LAPTOP;
            r.confidence_pct = 70;
            r.reason = laptop_oui;
            return r;
        }

        /* Wearable — smartwatch, fitness band */
        uint16_t cat = in->ble_appearance >> 6;
        if (cat == 0x0C ||
            (in->has_16bit_service_uuid &&
             is_wearable_uuid(in->service_uuid_16))) {
            r.type = DEV_TYPE_BLE_WEARABLE;
            r.confidence_pct = 70;
            r.reason = "appearance/UUID";
            return r;
        }

        /* Generic BLE */
        r.type = DEV_TYPE_BLE_GENERIC;
        r.confidence_pct = 40;
        r.reason = "BLE, no match";
        return r;
    }

    /* ── 3. WiFi ────────────────────────────────────────────────────── */
    if (in->ssid && in->ssid[0]) {
        /* Has SSID = Access Point */
        r.type = DEV_TYPE_WIFI_AP;
        r.confidence_pct = 80;
        r.reason = "SSID broadcast";
    } else {
        /* No SSID = probe request from a client device (phone, laptop etc) */
        /* Check OUI for phone manufacturers */
        const char *phone = FIND_PHONE(in->mac);
        const char *laptop = FIND_LAPTOP(in->mac);
        if (phone) {
            r.type = DEV_TYPE_BLE_PHONE;
            r.confidence_pct = 75;
            r.reason = phone;
        } else if (laptop) {
            r.type = DEV_TYPE_BLE_LAPTOP;
            r.confidence_pct = 70;
            r.reason = laptop;
        } else {
            r.type = DEV_TYPE_WIFI_CLIENT;
            r.confidence_pct = 50;
            r.reason = "probe request";
        }
    }
    return r;
}

uint32_t device_type_color(device_type_t type) {
    switch (type) {
        case DEV_TYPE_CAMERA_SUSPECTED: return 0xFF3B30; /* red    — warning */
        case DEV_TYPE_BLE_AUDIO:        return 0xFFD60A; /* yellow — headphones */
        case DEV_TYPE_BLE_PHONE:        return 0x30D5FF; /* cyan   — phone */
        case DEV_TYPE_BLE_LAPTOP:       return 0x30FF99; /* green  — laptop */
        case DEV_TYPE_BLE_WEARABLE:     return 0xC77DFF; /* purple — watch */
        case DEV_TYPE_WIFI_AP:          return 0xFF9F0A; /* orange — AP */
        case DEV_TYPE_WIFI_CLIENT:      return 0x3AFFA0; /* mint   — wifi client */
        case DEV_TYPE_BLE_GENERIC:      return 0x8A8FFF; /* blue   — generic */
        default:                        return 0x808080; /* grey   — unknown */
    }
}

const char *device_type_label(device_type_t type) {
    switch (type) {
        case DEV_TYPE_CAMERA_SUSPECTED: return "CAM?";
        case DEV_TYPE_BLE_AUDIO:        return "AUDIO";
        case DEV_TYPE_BLE_PHONE:        return "PHONE";
        case DEV_TYPE_BLE_LAPTOP:       return "LAPTOP";
        case DEV_TYPE_BLE_WEARABLE:     return "WEAR";
        case DEV_TYPE_WIFI_AP:          return "AP";
        case DEV_TYPE_WIFI_CLIENT:      return "WIFI";
        case DEV_TYPE_BLE_GENERIC:      return "BLE";
        default:                        return "?";
    }
}
