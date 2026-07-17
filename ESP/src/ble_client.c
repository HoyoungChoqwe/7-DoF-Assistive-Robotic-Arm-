#include "ble_client.h"

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

static const char *TAG = "BLE_CLIENT";

#define ESP_ADV_NAME "ESP_JOYSTICK"
#define PHONE_UART_APP_ID 1
#define PHONE_UART_PROFILE_INST_ID 0
#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

// Nordic UART Service UUIDs used by Adafruit Bluefruit Connect UART.
static uint8_t NUS_SERVICE_UUID128[16] = {
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6E
};
static uint8_t NUS_RX_UUID128[16] = {
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x02,0x00,0x40,0x6E
};
static uint8_t NUS_TX_UUID128[16] = {
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x03,0x00,0x40,0x6E
};

// ===== Custom UUIDs (match STM peripheral) =====
// Service UUID: 12345678-1234-5678-1234-56789abcdef0
static const uint8_t SVC_UUID128[16] = {
    0xF0,0xDE,0xBC,0x9A,0x78,0x56,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12,0x56,0x78
};
// RX Char UUID: 12345678-1234-5678-1234-56789abcdef1
static const uint8_t RX_UUID128[16] = {
    0xF1,0xDE,0xBC,0x9A,0x78,0x56,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12,0x56,0x78
};

static esp_gatt_if_t g_gattc_if = ESP_GATT_IF_NONE;
static uint16_t g_conn_id = 0xFFFF;
static esp_bd_addr_t g_bda = {0};
static esp_ble_addr_type_t g_addr_type = BLE_ADDR_TYPE_PUBLIC;

static char g_target_name[32] = {0};

static bool g_scanning = false;
static bool g_connecting = false;
static bool g_connected = false;
static bool g_ready = false;

static uint16_t g_svc_start = 0, g_svc_end = 0;
static uint16_t g_rx_handle = 0;

static ble_client_on_ready_cb_t g_on_ready = NULL;
static uint8_t g_adv_config_done = 0;

enum {
    UART_IDX_SVC,
    UART_IDX_RX_CHAR,
    UART_IDX_RX_VAL,
    UART_IDX_TX_CHAR,
    UART_IDX_TX_VAL,
    UART_IDX_TX_CCC,
    UART_IDX_NB,
};

static uint16_t g_uart_handle_table[UART_IDX_NB];
static esp_gatt_if_t g_uart_gatts_if = ESP_GATT_IF_NONE;
static uint16_t g_uart_conn_id = 0xFFFF;
static bool g_uart_connected = false;
static bool g_uart_notify_enabled = false;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static uint8_t uart_rx_value[20] = {0};
static uint8_t uart_tx_value[20] = {0};
static uint8_t uart_ccc_value[2] = {0x00, 0x00};

static const esp_gatts_attr_db_t g_uart_gatt_db[UART_IDX_NB] = {
    [UART_IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
         sizeof(NUS_SERVICE_UUID128), sizeof(NUS_SERVICE_UUID128), NUS_SERVICE_UUID128}
    },
    [UART_IDX_RX_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write}
    },
    [UART_IDX_RX_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, NUS_RX_UUID128, ESP_GATT_PERM_WRITE,
         sizeof(uart_rx_value), 0, uart_rx_value}
    },
    [UART_IDX_TX_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_notify}
    },
    [UART_IDX_TX_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, NUS_TX_UUID128, ESP_GATT_PERM_READ,
         sizeof(uart_tx_value), 0, uart_tx_value}
    },
    [UART_IDX_TX_CCC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uart_ccc_value), sizeof(uart_ccc_value), uart_ccc_value}
    },
};

static esp_ble_adv_params_t g_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t g_adv_data = {
    .set_scan_rsp       = false,
    .include_name       = false,
    .include_txpower    = true,
    .min_interval       = 0x20,
    .max_interval       = 0x40,
    .appearance         = 0,
    .manufacturer_len   = 0,
    .p_manufacturer_data = NULL,
    .service_data_len   = 0,
    .p_service_data     = NULL,
    .service_uuid_len   = sizeof(NUS_SERVICE_UUID128),
    .p_service_uuid     = NUS_SERVICE_UUID128,
    .flag               = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

static esp_ble_adv_data_t g_scan_rsp_data = {
    .set_scan_rsp       = true,
    .include_name       = true,
    .include_txpower    = false,
    .min_interval       = 0x20,
    .max_interval       = 0x40,
    .appearance         = 0,
    .manufacturer_len   = 0,
    .p_manufacturer_data = NULL,
    .service_data_len   = 0,
    .p_service_data     = NULL,
    .service_uuid_len   = 0,
    .p_service_uuid     = NULL,
    .flag               = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

static void maybe_start_phone_advertising(void)
{
    if (g_adv_config_done == 0) {
        ESP_LOGI(TAG, "Advertising as '%s' with Adafruit UART service", ESP_ADV_NAME);
        ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&g_adv_params));
    }
}

static void start_phone_advertising(void)
{
    if (g_uart_connected) return;

    g_adv_config_done = ADV_CONFIG_FLAG | SCAN_RSP_CONFIG_FLAG;
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(ESP_ADV_NAME));
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&g_adv_data));
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&g_scan_rsp_data));
}

static bool adv_name_matches(const uint8_t *adv, uint8_t adv_len, const char *name)
{
    (void)adv_len;
    uint8_t out_len = 0;

    // esp_ble_resolve_adv_data takes non-const pointer in its signature
    uint8_t *p = esp_ble_resolve_adv_data((uint8_t*)adv, ESP_BLE_AD_TYPE_NAME_CMPL, &out_len);
    if (!p || out_len == 0) return false;

    if (strlen(name) != out_len) return false;
    return (memcmp(p, name, out_len) == 0);
}

static void start_scan(void)
{
    esp_ble_scan_params_t scan_params = {
        .scan_type              = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval          = 0x50,
        .scan_window            = 0x30,
        .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
    };

    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));
}

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    // Correct event name in your IDF
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan params set, starting scan...");
        g_scanning = true;
        ESP_ERROR_CHECK(esp_ble_gap_start_scanning(0)); // continuous
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *sr = param;

        if (sr->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            if (!g_connecting &&
                adv_name_matches(sr->scan_rst.ble_adv, sr->scan_rst.adv_data_len, g_target_name)) {

                ESP_LOGI(TAG, "Found '%s' -> connecting...", g_target_name);
                memcpy(g_bda, sr->scan_rst.bda, sizeof(esp_bd_addr_t));
                g_addr_type = sr->scan_rst.ble_addr_type;

                g_connecting = true;
                // stop scan (optional) then open; opening immediately is OK
                esp_ble_gap_stop_scanning();

                if (g_gattc_if != ESP_GATT_IF_NONE) {
                    ESP_ERROR_CHECK(esp_ble_gattc_open(g_gattc_if, g_bda, g_addr_type, true));
                }
            }
        } else if (sr->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
            g_scanning = false;
            ESP_LOGI(TAG, "Scan complete");
        }
        break;
    }

    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        g_adv_config_done &= ~ADV_CONFIG_FLAG;
        maybe_start_phone_advertising();
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        g_adv_config_done &= ~SCAN_RSP_CONFIG_FLAG;
        maybe_start_phone_advertising();
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Phone-visible advertising started");
        } else {
            ESP_LOGW(TAG, "Advertising start failed: status=0x%x", param->adv_start_cmpl.status);
        }
        break;

    default:
        break;
    }
}

static void gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "Phone UART GATTS registered");
        g_uart_gatts_if = gatts_if;
        ESP_ERROR_CHECK(esp_ble_gatts_create_attr_tab(
            g_uart_gatt_db,
            gatts_if,
            UART_IDX_NB,
            PHONE_UART_PROFILE_INST_ID
        ));
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK ||
            param->add_attr_tab.num_handle != UART_IDX_NB) {
            ESP_LOGE(TAG, "Phone UART attr table failed: status=0x%x handles=%u",
                     param->add_attr_tab.status, param->add_attr_tab.num_handle);
            break;
        }

        memcpy(g_uart_handle_table, param->add_attr_tab.handles, sizeof(g_uart_handle_table));
        ESP_ERROR_CHECK(esp_ble_gatts_start_service(g_uart_handle_table[UART_IDX_SVC]));
        start_phone_advertising();
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "Phone UART connected");
        g_uart_connected = true;
        g_uart_notify_enabled = false;
        g_uart_conn_id = param->connect.conn_id;
        g_uart_gatts_if = gatts_if;
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Phone UART disconnected");
        g_uart_connected = false;
        g_uart_notify_enabled = false;
        g_uart_conn_id = 0xFFFF;
        start_phone_advertising();
        break;

    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == g_uart_handle_table[UART_IDX_TX_CCC] && param->write.len == 2) {
            uint16_t descr_value = param->write.value[0] | (param->write.value[1] << 8);
            g_uart_notify_enabled = (descr_value == 0x0001);
            ESP_LOGI(TAG, "Phone UART notifications %s",
                     g_uart_notify_enabled ? "enabled" : "disabled");
            if (g_uart_notify_enabled) {
                ble_uart_send_text("ESP_JOYSTICK UART ready\r\n");
            }
        } else if (param->write.handle == g_uart_handle_table[UART_IDX_RX_VAL]) {
            ESP_LOGI(TAG, "Phone UART RX write len=%u", param->write.len);
        }
        break;

    default:
        break;
    }
}

static void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(TAG, "GATTC registered");
        g_gattc_if = gattc_if;
        start_scan();
        break;

    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(TAG, "Connected (conn_id=%u)", param->connect.conn_id);
        g_connected = true;
        g_connecting = false;
        g_conn_id = param->connect.conn_id;

        // Search only for our custom service UUID
        esp_bt_uuid_t svc_uuid = {.len = ESP_UUID_LEN_128};
        memcpy(svc_uuid.uuid.uuid128, SVC_UUID128, 16);
        ESP_ERROR_CHECK(esp_ble_gattc_search_service(gattc_if, g_conn_id, &svc_uuid));
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
        // Because we searched with a UUID, any result here should be the right service.
        g_svc_start = param->search_res.start_handle;
        g_svc_end   = param->search_res.end_handle;
        ESP_LOGI(TAG, "Service range: start=%u end=%u", g_svc_start, g_svc_end);
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT: {
        ESP_LOGI(TAG, "Service search complete");
        if (g_svc_start == 0 || g_svc_end == 0) {
            ESP_LOGE(TAG, "Service not found; disconnecting");
            esp_ble_gattc_close(gattc_if, g_conn_id);
            break;
        }

        // Synchronous characteristic lookup by UUID (supported in your IDF)
        esp_bt_uuid_t rx_uuid = {.len = ESP_UUID_LEN_128};
        memcpy(rx_uuid.uuid.uuid128, RX_UUID128, 16);

        esp_gattc_char_elem_t char_elem[1];
        uint16_t count = 1;

        esp_gatt_status_t st = esp_ble_gattc_get_char_by_uuid(
            gattc_if,
            g_conn_id,
            g_svc_start,
            g_svc_end,
            rx_uuid,
            char_elem,
            &count
        );

        if (st != ESP_GATT_OK || count == 0) {
            ESP_LOGE(TAG, "RX characteristic not found (status=0x%x)", st);
            esp_ble_gattc_close(gattc_if, g_conn_id);
            break;
        }

        g_rx_handle = char_elem[0].char_handle;
        g_ready = true;
        ESP_LOGI(TAG, "READY. RX handle=%u", g_rx_handle);
        if (g_on_ready) g_on_ready();
        break;
    }

    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGW(TAG, "Disconnected reason=0x%02X", param->disconnect.reason);
        g_connected = false;
        g_connecting = false;
        g_ready = false;

        g_conn_id = 0xFFFF;
        g_svc_start = g_svc_end = 0;
        g_rx_handle = 0;

        // Restart scan
        if (!g_scanning) start_scan();
        break;

    default:
        break;
    }
}

bool ble_client_ready(void)
{
    return g_ready && g_connected && (g_rx_handle != 0) && (g_conn_id != 0xFFFF);
}

bool ble_client_send(const uint8_t *data, uint16_t len)
{
    if (!ble_client_ready()) return false;

    esp_err_t err = esp_ble_gattc_write_char(
        g_gattc_if,
        g_conn_id,
        g_rx_handle,
        len,
        (uint8_t*)data,
        ESP_GATT_WRITE_TYPE_NO_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "write failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool ble_uart_send_text(const char *text)
{
    if (!g_uart_connected || !g_uart_notify_enabled || g_uart_gatts_if == ESP_GATT_IF_NONE) {
        return false;
    }

    const uint8_t *p = (const uint8_t *)text;
    size_t remaining = strlen(text);
    while (remaining > 0) {
        uint16_t chunk = (remaining > 20) ? 20 : (uint16_t)remaining;
        esp_err_t err = esp_ble_gatts_send_indicate(
            g_uart_gatts_if,
            g_uart_conn_id,
            g_uart_handle_table[UART_IDX_TX_VAL],
            chunk,
            (uint8_t *)p,
            false
        );
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Phone UART notify failed: %s", esp_err_to_name(err));
            return false;
        }
        p += chunk;
        remaining -= chunk;
    }

    return true;
}

bool ble_uart_ready(void)
{
    return g_uart_connected && g_uart_notify_enabled && g_uart_gatts_if != ESP_GATT_IF_NONE;
}

void ble_client_init(const char *target_name, ble_client_on_ready_cb_t on_ready)
{
    g_on_ready = on_ready;
    strncpy(g_target_name, target_name, sizeof(g_target_name) - 1);

    // NVS required by BT stack
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    esp_bluedroid_config_t bcfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bcfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_cb));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(PHONE_UART_APP_ID));

    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(0));
}
