#include "ble_stream.h"

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_stream";

#define DEVICE_NAME "POC4TRIAGE_APP"

// ---- Connection state ----
static uint16_t s_conn_handle      = BLE_HS_CONN_HANDLE_NONE;
static bool     s_vitals_subscribed = false;
static bool     s_ecg_subscribed    = false;

// ---- GATT attribute handles (populated by NimBLE stack) ----
static uint16_t s_vitals_handle;
static uint16_t s_ecg_handle;

// Forward declarations
static void start_advertising(void);
static int  gap_event_handler(struct ble_gap_event *event, void *arg);

// ============================================================
// UUIDs — 128-bit, little-endian byte order for BLE
// ============================================================

// Service: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f,
                     0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f);

// Vitals: beb54841-36e1-4688-b7f5-ea07361b26a8
static const ble_uuid128_t vitals_chr_uuid =
    BLE_UUID128_INIT(0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
                     0x88, 0x46, 0xe1, 0x36, 0x41, 0x48, 0xb5, 0xbe);

// ECG: beb54842-36e1-4688-b7f5-ea07361b26a8
static const ble_uuid128_t ecg_chr_uuid =
    BLE_UUID128_INIT(0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
                     0x88, 0x46, 0xe1, 0x36, 0x42, 0x48, 0xb5, 0xbe);

// ============================================================
// GATT — notify-only characteristics, no read/write
// ============================================================

static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return 0;   // nothing to read/write
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = &vitals_chr_uuid.u,
                .access_cb  = chr_access_cb,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_vitals_handle,
            },
            {
                .uuid       = &ecg_chr_uuid.u,
                .access_cb  = chr_access_cb,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_ecg_handle,
            },
            { 0 },     // terminator
        },
    },
    { 0 },             // terminator
};

// ============================================================
// Advertising
// ============================================================

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};

    fields.flags                 = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl            = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    const char *name     = ble_svc_gap_device_name();
    fields.name          = (uint8_t *)name;
    fields.name_len      = strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising started");
    }
}

// ============================================================
// GAP event handler
// ============================================================

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            gpio_set_level(BLE_LED_PIN, 1);
            ESP_LOGI(TAG, ">>> Connected (handle=%d)", s_conn_handle);
        } else {
            ESP_LOGW(TAG, "Connection failed: %d", event->connect.status);
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "<<< Disconnected (reason=%d)",
                 event->disconnect.reason);
        s_conn_handle       = BLE_HS_CONN_HANDLE_NONE;
        s_vitals_subscribed = false;
        s_ecg_subscribed    = false;
        gpio_set_level(BLE_LED_PIN, 0);
        start_advertising();        // restart advertising
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_vitals_handle) {
            s_vitals_subscribed = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "Vitals notify %s",
                     s_vitals_subscribed ? "ON" : "OFF");
        } else if (event->subscribe.attr_handle == s_ecg_handle) {
            s_ecg_subscribed = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "ECG notify %s",
                     s_ecg_subscribed ? "ON" : "OFF");
        }
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated: %d", event->mtu.value);
        break;

    default:
        break;
    }

    return 0;
}

// ============================================================
// NimBLE host callbacks
// ============================================================

static void on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset: reason=%d", reason);
}

static void host_task_fn(void *param)
{
    nimble_port_run();              // blocks until nimble_port_stop()
    nimble_port_freertos_deinit();
}

// ============================================================
// Public API
// ============================================================

void ble_stream_init(void)
{
    ESP_LOGI(TAG, "Initializing BLE");

    // NVS required by NimBLE (idempotent if already initialized)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize NimBLE port (controller + host)
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Host callbacks
    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    // Standard GAP & GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Register custom GATT service
    int rc = ble_gatts_count_cfg(gatt_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svcs);
    assert(rc == 0);

    // Device name and MTU (matches Arduino: setMTU(517))
    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_att_set_preferred_mtu(517);

    // BLE status LED
    gpio_set_direction(BLE_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BLE_LED_PIN, 0);

    // Start NimBLE host task (runs on Core 0 by default)
    nimble_port_freertos_init(host_task_fn);

    ESP_LOGI(TAG, "BLE ready, advertising as '%s'", DEVICE_NAME);
}

void ble_stream_send_vitals(const ble_vitals_packet_t *pkt)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_vitals_subscribed)
        return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt, sizeof(*pkt));
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_vitals_handle, om);
    }
}

void ble_stream_send_ecg(const ble_ecg_slice_t bundle[BLE_ECG_BUNDLE_SIZE])
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_ecg_subscribed)
        return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(
        bundle, sizeof(ble_ecg_slice_t) * BLE_ECG_BUNDLE_SIZE);
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_ecg_handle, om);
    }
}