#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    DEV_TYPE_UNKNOWN = 0,
    DEV_TYPE_WIFI_CLIENT,
    DEV_TYPE_WIFI_AP,
    DEV_TYPE_BLE_GENERIC,
    DEV_TYPE_BLE_WEARABLE,
    DEV_TYPE_BLE_AUDIO,        /* headphones, earbuds, speakers */
    DEV_TYPE_BLE_PHONE,        /* smartphones */
    DEV_TYPE_BLE_LAPTOP,       /* laptops, PCs */
    DEV_TYPE_CAMERA_SUSPECTED,
    DEV_TYPE_COUNT
} device_type_t;

typedef struct {
    const char    *ssid;
    const uint8_t  mac[6];
    bool           is_ble;
    uint16_t       ble_appearance;
    const char    *ble_name;
    bool           has_16bit_service_uuid;
    uint16_t       service_uuid_16;
} classification_input_t;

typedef struct {
    device_type_t  type;
    uint8_t        confidence_pct;
    const char    *reason;
} classification_result_t;

classification_result_t device_classify(const classification_input_t *in);
uint32_t    device_type_color(device_type_t type);
const char *device_type_label(device_type_t type);
