#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_gatt_common_api.h"

// 日志标签
#define TAG "BLE_GATT_TIMER_IDF55"

// 自定义 GATT 服务和特征值 UUID
#define GATT_SERVICE_UUID        0x00FF
#define GATT_CHARACTERISTIC_UUID 0xFF01

// 蓝牙 GATT 相关宏定义
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

// 定时发送间隔（单位：ms）
#define SEND_INTERVAL_MS 2000

// 全局变量
static uint16_t gatt_if = 0;
static uint16_t conn_id = 0;
static uint16_t char_handle = INVALID_HANDLE;
static TimerHandle_t send_timer = NULL;
static bool is_connected = false;

// UUID 字节数组（适配 IDF5.5 uuid_p = uint8_t*）
static uint8_t primary_service_uuid[2] = {0x28, 0x00};  // 0x2800 主服务声明
static uint8_t char_declare_uuid[2]    = {0x28, 0x03};  // 0x2803 特征值声明
static uint8_t custom_char_uuid[2]     = {0x01, 0xFF};  // 0xFF01 自定义特征值
static uint8_t cccd_uuid[2]            = {0x29, 0x02};  // 0x2902 CCCD 描述符

// 模拟发送的数据
static uint8_t send_data[] = {0x00, 0x01, 0x02, 0x03};
static uint32_t data_counter = 0;

// ==== IDF5.5 正确的 GATT 数据库定义 ====
static esp_gatts_attr_db_t gatt_db[] = {
    // 0: 主服务声明
    [0] = {
        .attr_control = {0},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = primary_service_uuid,
            .perm = ESP_GATT_PERM_READ,
            .max_length = 2,
            .length = 2,
            .value = (uint8_t *)"\xFF\x00"  // 服务 UUID: 0x00FF (小端序)
        }
    },
    // 1: 特征值声明 (Characteristic Declaration)
    [1] = {
        .attr_control = {0},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = char_declare_uuid,
            .perm = ESP_GATT_PERM_READ,
            .max_length = 5,
            .length = 1,
            .value = (uint8_t *)"\x02"  // 属性: READ|WRITE
        }
    },
    // 2: 自定义特征值 UUID
    [2] = {
        .attr_control = {0},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = custom_char_uuid,
            .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            .max_length = 20,
            .length = 0,
            .value = NULL
        }
    },
    // 3: CCCD 描述符 (Client Characteristic Configuration Descriptor)
    [3] = {
        .attr_control = {0},
        .att_desc = {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p = cccd_uuid,
            .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            .max_length = 2,
            .length = 2,
            .value = (uint8_t *)"\x00\x00"
        }
    },
};

// GATT 事件回调
static void gatts_profile_a_event_cb(esp_gatts_cb_event_t event,
                                     esp_gatt_if_t gatts_if,
                                     esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_REG_EVT, gatts_if=%d", gatts_if);
            gatt_if = gatts_if;

            // 设置设备名称
            esp_ble_gap_set_device_name("ESP32_IDF55_GATT");

            // 配置广播数据 - 使用 esp_ble_adv_data_t (IDF 4.x/5.x)
            esp_ble_adv_data_t adv_data = {
                .flags = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
                .uuid_len = 2,
                .p_uuid = primary_service_uuid,
                .name = (uint8_t *)"ESP32_IDF55_GATT",
                .name_len = strlen("ESP32_IDF55_GATT"),
                .include_txpower = true,
                .set_scan_rsp = false,
                .min_interval = 0x20,
                .max_interval = 0x40,
                .adv_type = ADV_TYPE_IND,
                .primary_adv_phy = ESP_BLE_GAP_PHY_1M,
            };

            // 设置广播数据
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Config adv data failed, err=%d", ret);
            }

            // 创建 GATT 服务表
            esp_gatts_create_attr_tab(gatt_db, gatts_if, PROFILE_NUM,
                                     sizeof(gatt_db) / sizeof(esp_gatts_attr_db_t));
            break;


        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status == ESP_OK) {
                ESP_LOGI(TAG, "Create attr tab success");
                char_handle = param->add_attr_tab.handles[2]; // 特征值句柄
                esp_ble_gap_start_advertising(NULL); // 启动广播
            } else {
                ESP_LOGE(TAG, "Create attr tab failed, status=%d", param->add_attr_tab.status);
            }
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id=%d", param->connect.conn_id);
            conn_id = param->connect.conn_id;
            is_connected = true;
            xTimerStart(send_timer, 0);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason=%d", param->disconnect.reason);
            is_connected = false;
            conn_id = 0;
            xTimerStop(send_timer, 0);
            esp_ble_gap_start_advertising(NULL);
            break;

        case ESP_GATTS_WRITE_EVT:
            // 必须回复写操作响应
            if (param->write.need_rsp) {
                esp_gatts_send_response(gatts_if, param->write.conn_id,
                                       param->write.trans_id, ESP_GATT_OK, NULL);
            }

            // 检测 CCCD 写操作
            if (param->write.handle == gatt_db[3].att_desc.handle) { // CCCD 句柄
                uint16_t cccd_val = (param->write.value[1] << 8) | param->write.value[0];
                ESP_LOGI(TAG, "CCCD write: 0x%04X %s", cccd_val,
                        (cccd_val == 0x0001) ? "(Notify enabled)" : "(Notify disabled)");
            }
            break;

        default:
            break;
    }
}

// 蓝牙初始化
static void ble_gatt_init(void)
{
    // 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化蓝牙控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // 初始化 Bluedroid 协议栈
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // 注册 GATT 回调和应用
    ESP_ERROR_CHECK(esp_gatts_register_callback(gatts_profile_a_event_cb));
    ESP_ERROR_CHECK(esp_gatts_app_register(PROFILE_A_APP_ID));

    // 配置广播参数
    esp_ble_gap_adv_params_t adv_params = {
        .adv_int_min        = 0x20,
        .adv_int_max        = 0x40,
        .adv_type           = ADV_TYPE_IND,
        .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
        .channel_map        = ADV_CHNL_ALL,
        .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };
    ESP_ERROR_CHECK(esp_ble_gap_set_adv_params(&adv_params));

    // 创建定时器
    send_timer = xTimerCreate("ble_send_timer",
                             pdMS_TO_TICKS(SEND_INTERVAL_MS),
                             pdTRUE, (void *)0, send_data_timer_cb);
    if (send_timer == NULL) {
        ESP_LOGE(TAG, "Create timer failed");
        return;
    }
}

// 主函数
void app_main(void)
{
    ESP_LOGI(TAG, "BLE GATT Timer Example (IDF5.5) Start");
    ble_gatt_init();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}