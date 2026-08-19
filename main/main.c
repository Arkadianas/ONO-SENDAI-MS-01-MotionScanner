#include <stdio.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "soc/usb_serial_jtag_reg.h"
#include "driver/gpio.h"

#include "lvgl.h"
#include "lv_demos.h"
#include "display_ui.h"
#include "radar_engine.h"
#include "esp_lcd_sh8601.h"
#include "i2c_bsp.h"
#include "touch_bsp.h"
#include "app_config.h"
#include "wifi_scan.h"
#include "imu_tracking.h"
#include "localization.h"
#include "device_classification.h"
#include "storage_config.h"
static const char *TAG = "example";
esp_lcd_panel_handle_t g_panel_handle = NULL;
static SemaphoreHandle_t lvgl_mux = NULL;

#define LCD_HOST  SPI2_HOST

#if CONFIG_LV_COLOR_DEPTH == 32
#define LCD_BIT_PER_PIXEL       (24)
#elif CONFIG_LV_COLOR_DEPTH == 16
#define LCD_BIT_PER_PIXEL       (16)
#endif

#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_LCD_CS            (GPIO_NUM_9)
#define EXAMPLE_PIN_NUM_LCD_PCLK          (GPIO_NUM_10) 
#define EXAMPLE_PIN_NUM_LCD_DATA0         (GPIO_NUM_11)
#define EXAMPLE_PIN_NUM_LCD_DATA1         (GPIO_NUM_12)
#define EXAMPLE_PIN_NUM_LCD_DATA2         (GPIO_NUM_13)
#define EXAMPLE_PIN_NUM_LCD_DATA3         (GPIO_NUM_14)
#define EXAMPLE_PIN_NUM_LCD_RST           (GPIO_NUM_21)
#define EXAMPLE_PIN_NUM_BK_LIGHT          (-1)

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              280 
#define EXAMPLE_LCD_V_RES              456 

#define EXAMPLE_USE_TOUCH               1
#define EXAMPLE_Rotate_90

#define EXAMPLE_LVGL_BUF_HEIGHT        (EXAMPLE_LCD_V_RES / 4)
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

void setBrightnes(esp_lcd_panel_io_handle_t ioHand,uint8_t brig);

static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0x11, (uint8_t []){0x00}, 0, 80},   
    {0xC4, (uint8_t []){0x80}, 1, 0},
   
    {0x35, (uint8_t []){0x00}, 1, 0},

    {0x53, (uint8_t []){0x20}, 1, 1},
    {0x63, (uint8_t []){0xFF}, 1, 1},
    {0x51, (uint8_t []){0x00}, 1, 1},

    {0x29, (uint8_t []){0x00}, 0, 10},

    {0x51, (uint8_t []){0xFF}, 1, 0},    //亮度
};

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    const int offsetx1 = area->x1 + 0x14;
    const int offsetx2 = area->x2 + 0x14;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

#if LCD_BIT_PER_PIXEL == 24
    uint8_t *to = (uint8_t *)color_map;
    uint8_t temp = 0;
    uint16_t pixel_num = (offsetx2 - offsetx1 + 1) * (offsety2 - offsety1 + 1);

    // Special dealing for first pixel
    temp = color_map[0].ch.blue;
    *to++ = color_map[0].ch.red;
    *to++ = color_map[0].ch.green;
    *to++ = temp;
    // Normal dealing for other pixels
    for (int i = 1; i < pixel_num; i++) {
        *to++ = color_map[i].ch.red;
        *to++ = color_map[i].ch.green;
        *to++ = color_map[i].ch.blue;
    }
#endif

    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

void example_lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;

    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2;

    // round the start of coordinate down to the nearest 2M number
    area->x1 = (x1 >> 1) << 1;
    area->y1 = (y1 >> 1) << 1;
    // round the end of coordinate up to the nearest 2N+1 number
    area->x2 = ((x2 >> 1) << 1) + 1;
    area->y2 = ((y2 >> 1) << 1) + 1;
}

#if EXAMPLE_USE_TOUCH
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t tp_x;
    uint16_t tp_y;
    uint8_t win = getTouch(&tp_x,&tp_y);
    if (win)
    {
        data->point.x = tp_y;
        data->point.y = tp_x;
        if(data->point.x > EXAMPLE_LCD_H_RES)
        data->point.x = EXAMPLE_LCD_H_RES;
        if(data->point.y > EXAMPLE_LCD_V_RES)
        data->point.y = EXAMPLE_LCD_V_RES;
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
#endif

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static bool example_lvgl_lock(int timeout_ms)
{
    assert(lvgl_mux && "bsp_display_start must be called first");

    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

static void example_lvgl_unlock(void)
{
    assert(lvgl_mux && "bsp_display_start must be called first");
    xSemaphoreGive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting ONO-SENDAI UI");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}


/* -----------------------------------------------------------------------
 * BLE scan callback — fires on every received advertisement.
 * Runs in the NimBLE host task context (NOT the LVGL or radar task).
 * We only call localization_ingest_sample() here — it is mutex-protected
 * internally and safe to call from any task.
 * --------------------------------------------------------------------- */
/* -----------------------------------------------------------------------
 * Radar feed task — reads localization registry at 5 Hz and pushes
 * active devices into the radar engine for the renderer to draw.
 *
 * Runs at 5 Hz (200 ms period) — fast enough for smooth target updates,
 * slow enough not to thrash the engine with unnecessary clears.
 *
 * Distance clamping: localization returns metres (0.1 .. 100 m).
 * We map this to normalised radar radius [0.05 .. 1.0] using a log scale
 * so nearby targets don't all pile up at the center and distant ones
 * don't disappear off the outer ring.
 *   radius = log10(dist_m + 1) / log10(MAX_DIST_M + 1)
 * MAX_DIST_M = 30 m covers a typical room/corridor scenario.
 * --------------------------------------------------------------------- */
#define RADAR_MAX_DIST_M   15.0f    /* max radar range: 15m indoor */
#define STALE_TIMEOUT_MS   15000    /* remove targets not seen for 15 s */

static float dist_to_radius(float dist_m)
{
    if (dist_m < 0.1f) dist_m = 0.1f;
    if (dist_m > RADAR_MAX_DIST_M) dist_m = RADAR_MAX_DIST_M;
    float r = log10f(dist_m + 1.0f) / log10f(RADAR_MAX_DIST_M + 1.0f);
    if (r < 0.05f) r = 0.05f;
    if (r > 1.0f)  r = 1.0f;
    return r;
}

static void radar_feed_task(void *arg)
{
    static tracked_device_t devices[RADAR_MAX_TARGETS];

    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        /* Age out devices not seen recently */
        localization_expire_stale(now_ms, STALE_TIMEOUT_MS);

        /* Fetch current active device list */
        int count = localization_get_devices(devices, RADAR_MAX_TARGETS);

        /* Push into radar engine — must hold LVGL lock because
         * radar_engine is read by the renderer inside the LVGL task */
        if (example_lvgl_lock(10)) {
            radar_engine_clear();

            for (int i = 0; i < count; i++) {
                tracked_device_t *d = &devices[i];

                float radius     = dist_to_radius(d->estimated_distance_m);
                float confidence = d->bearing_confidence_pct / 100.0f;

                uint32_t rgb = device_type_color(d->classification.type);
                lv_color_t col = lv_color_make(
                    (rgb >> 16) & 0xFF,
                    (rgb >>  8) & 0xFF,
                     rgb        & 0xFF
                );

                /* Use tracked API — MAC is the persistent identity key */
                radar_engine_add_target_tracked(
                    d->estimated_bearing_deg,
                    radius,
                    confidence,
                    col,
                    d->mac
                );
            }

            /* Push live stats to the status bar */
            display_ui_set_target_count((uint8_t)count);

            /* Signal level: map active target count to 0..4 bar segments */
            uint8_t sig = 0;
            if      (count >= 8) sig = 4;
            else if (count >= 4) sig = 3;
            else if (count >= 2) sig = 2;
            else if (count >= 1) sig = 1;
            display_ui_set_signal_level(sig);

            /* Device name: find the closest target (smallest radius = strongest
             * signal) that has a name and display it in the status bar.
             * WiFi APs have SSIDs, BLE devices have names if they advertise one. */
            const char *best_name  = NULL;
            const char *best_type  = NULL;
            float       best_dist  = 2.0f;   /* larger than max radius 1.0 */

            for (int i = 0; i < count; i++) {
                tracked_device_t *d = &devices[i];
                if (d->ssid_or_name[0] == '\0') continue;
                if (d->estimated_distance_m < best_dist ||
                    best_name == NULL) {
                    best_dist  = d->estimated_distance_m;
                    best_name  = d->ssid_or_name;
                    best_type  = device_type_label(d->classification.type);
                }
            }
            display_ui_set_device_name(best_type, best_name);

            /* Pass full device list to screen for tap-to-inspect */
            display_ui_update_devices(devices, count);

            example_lvgl_unlock();
        }

        vTaskDelay(pdMS_TO_TICKS(200));   /* 5 Hz */
    }
}

/* -----------------------------------------------------------------------
 * WiFi AP scan callback — fires after each active scan cycle.
 * --------------------------------------------------------------------- */
static void on_wifi_ap_result(const wifi_scan_result_t *r, void *ctx)
{
    classification_input_t ci = {
        .mac  = { r->bssid[0], r->bssid[1], r->bssid[2],
                  r->bssid[3], r->bssid[4], r->bssid[5] },
        .is_ble = false,
        .ssid   = r->ssid[0] ? r->ssid : NULL,
    };
    localization_ingest_sample(
        RADIO_WIFI, r->bssid, r->rssi,
        r->ssid[0] ? r->ssid : NULL,
        imu_get_last()->heading_deg, &ci);
}

/* -----------------------------------------------------------------------
 * WiFi promiscuous callback — fires on every 802.11 management frame.
 * Catches probe requests from phones and other clients.
 * --------------------------------------------------------------------- */
static void on_wifi_promisc_result(const wifi_scan_result_t *r, void *ctx)
{
    if (r->bssid[0] & 0x01) return;  /* skip broadcast/multicast */
    static const uint8_t zero[6] = {0};
    if (memcmp(r->bssid, zero, 6) == 0) return;

    classification_input_t ci = {
        .mac    = { r->bssid[0], r->bssid[1], r->bssid[2],
                    r->bssid[3], r->bssid[4], r->bssid[5] },
        .is_ble = false,
        .ssid   = NULL,
    };
    localization_ingest_sample(
        RADIO_WIFI, r->bssid, r->rssi,
        NULL, imu_get_last()->heading_deg, &ci);
}

/* -----------------------------------------------------------------------
 * Charging status task
 * Uses USB Serial JTAG peripheral to detect host connection.
 * When USB host is connected (charging), shows green USB.
 * When on battery, shows orange BAT.
 * --------------------------------------------------------------------- */
static void battery_task(void *arg)
{
    while (1) {
        /* Read USB Serial JTAG EP1 config register.
         * SERIAL_IN_EP_DATA_FREE bit = 1 when USB host is connected
         * and the IN endpoint buffer is available. */
        bool usb_connected = !!(REG_READ(USB_SERIAL_JTAG_EP1_CONF_REG)
                                 & USB_SERIAL_JTAG_SERIAL_IN_EP_DATA_FREE);

        if (example_lvgl_lock(10)) {
            display_ui_set_battery(usb_connected ? -2 : -1);
            example_lvgl_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* -----------------------------------------------------------------------
 * Power button task — hold BOOT button (GPIO 0) for 3 seconds to
 * shut down gracefully. Short press is ignored (used by bootloader).
 * On shutdown: display "POWERING OFF", wait 1s, then deep sleep.
 * Deep sleep with no wakeup source = effectively powered off.
 * Wake by pressing the reset/power button to restart.
 * --------------------------------------------------------------------- */
static void power_button_task(void *arg)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_BTN_BOOT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    uint32_t held_ms = 0;
    const uint32_t SHUTDOWN_HOLD_MS = 3000;
    const uint32_t POLL_MS          = 50;

    while (1) {
        if (gpio_get_level(PIN_BTN_BOOT) == 0) {
            /* Button pressed (active low) */
            held_ms += POLL_MS;
            if (held_ms >= SHUTDOWN_HOLD_MS) {
                ESP_LOGI("PWR", "Shutdown initiated");

                /* Show shutdown message on display */
                if (example_lvgl_lock(100)) {
                    lv_obj_t *scr = lv_scr_act();
                    lv_obj_t *msg = lv_label_create(scr);
                    lv_label_set_text(msg, "POWERING OFF");
                    lv_obj_set_style_text_color(msg,
                        lv_palette_main(LV_PALETTE_ORANGE), 0);
                    lv_obj_set_style_text_font(msg,
                        &lv_font_montserrat_16, 0);
                    lv_obj_center(msg);
                    example_lvgl_unlock();
                }

                vTaskDelay(pdMS_TO_TICKS(1500));

                /* Stop BLE scanner cleanly */
                wifi_scan_stop();
                vTaskDelay(pdMS_TO_TICKS(200));

                /* Turn off AMOLED display before sleep */
                extern esp_lcd_panel_handle_t g_panel_handle;
                if (g_panel_handle) {
                    esp_lcd_panel_disp_on_off(g_panel_handle, false);
                }
                vTaskDelay(pdMS_TO_TICKS(100));

                /* Short delay then sleep regardless of button state */
                vTaskDelay(pdMS_TO_TICKS(500));

                /* No wakeup source = only RST button wakes device. */
                esp_deep_sleep_start();
            }
        } else {
            held_ms = 0;   /* released — reset counter */
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

/* -----------------------------------------------------------------------
 * IMU task — reads QMI8658 at APP_IMU_SAMPLE_RATE_HZ (100 Hz).
 * Heading is integrated from gyro-Z by imu_update() internally.
 * Call imu_reset_reference() to zero the heading at any time.
 * --------------------------------------------------------------------- */
static void imu_task_fn(void *arg)
{
    /* Small startup delay — let the sensor settle after power-on */
    vTaskDelay(pdMS_TO_TICKS(100));

    const TickType_t period = pdMS_TO_TICKS(1000 / APP_IMU_SAMPLE_RATE_HZ);
    TickType_t last_wake    = xTaskGetTickCount();

    while (1) {
        imu_update(NULL);   /* updates internal cache; read via imu_get_last() */
        vTaskDelayUntil(&last_wake, period);
    }
}

/* -----------------------------------------------------------------------
 * Radar update task
 * Calls display_ui_radar_tick() at ~20 Hz so targets are rendered every
 * frame. The LVGL mutex is taken for each call exactly as the LVGL task
 * does — this is safe because both tasks use the same lvgl_mux.
 * heading_deg = 0 until IMU is wired in; replace with imu_get_heading().
 * --------------------------------------------------------------------- */
static void radar_update_task(void *arg)
{
    while (1) {
        if (example_lvgl_lock(10)) {
            display_ui_radar_tick(imu_get_last()->heading_deg);
            example_lvgl_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(50));    /* 20 Hz */
    }
}

void app_main(void)
{ESP_LOGI(TAG, "Starting Motion Scanner...");

ESP_ERROR_CHECK(storage_init());

localization_init();
ESP_ERROR_CHECK(imu_init());
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions
#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif

    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_PCLK,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA0,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA1,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA2,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA3,
                                                                 EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * LCD_BIT_PER_PIXEL / 8);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS,
                                                                                example_notify_lvgl_flush_ready,
                                                                                &disp_drv);
    sh8601_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_LOGI(TAG, "Install SH8601 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    g_panel_handle = panel_handle;


#if EXAMPLE_USE_TOUCH
touch_Init();
#endif

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    //alloc draw buffers used by LVGL
    //it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    //initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.rounder_cb = example_lvgl_rounder_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
#ifdef EXAMPLE_Rotate_90
    disp_drv.sw_rotate = 1;
    disp_drv.rotated = LV_DISP_ROT_180;
#endif
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    //Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

#if EXAMPLE_USE_TOUCH
    static lv_indev_drv_t indev_drv;           // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    lv_indev_drv_register(&indev_drv);
#endif

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL demos");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1)) {
        radar_engine_init();
        display_ui_show_radar_screen();
        example_lvgl_unlock();
    }

    /* WiFi-only scanning — more effective for detecting phones.
     * WiFi promiscuous mode captures probe requests from all nearby
     * devices including phones, tablets, laptops.
     * Active scan also finds all APs (routers, hotspots). */
    ESP_ERROR_CHECK(wifi_scan_init());
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(wifi_scan_start(on_wifi_ap_result, NULL));
    ESP_ERROR_CHECK(wifi_scan_enable_promiscuous(on_wifi_promisc_result, NULL));
    ESP_LOGI(TAG, "WiFi scanning started — AP + promiscuous mode");

    imu_reset_reference();

    /* Battery monitor task */
    xTaskCreate(battery_task, "bat_mon", 3072, NULL, 1, NULL);

    /* Power button task — monitors BOOT button for 3-second shutdown */
    xTaskCreate(power_button_task, "pwr_btn", 2048, NULL, 3, NULL);

    /* IMU task — high priority, time-critical 100 Hz sensor read */
    xTaskCreate(imu_task_fn, "imu_task", 3072, NULL, 5, NULL);

    /* Radar update task — 20 Hz display refresh */
    xTaskCreate(radar_update_task, "radar_tick", 4096, NULL, 1, NULL);

    /* Radar feed task — pulls localization data into engine, 5 Hz */
    xTaskCreate(radar_feed_task, "radar_feed", 4096, NULL, 2, NULL);
}

void setBrightnes(esp_lcd_panel_io_handle_t io,uint8_t brig) 
{
    uint32_t lcd_cmd = 0x51;
    lcd_cmd &= 0xff;
    lcd_cmd <<= 8;
    lcd_cmd |= 0x02 << 24;
    uint8_t param = brig;
    esp_lcd_panel_io_tx_param(io, lcd_cmd, &param,1);
}
