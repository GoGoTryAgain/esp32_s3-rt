#include "I2c_ctrl.h"
#include "esp_log.h"
//#include "i2c.h"
#include "driver/i2c.h"

static const char *TAG = "I2C_CTRL";

static i2c_master_bus_handle_t I2C_bus_handle[2] = {NULL};


void I2C_master_device_probe(i2c_port_t i2c_num)
{
    int found_devices = 0;
    uint8_t addr;
    esp_err_t err;
    ESP_LOGI(TAG, "Scanning I2C bus...");

    // 遍历7位I2C地址（0x00~0x7F，其中0x00~0x07、0x78~0x7F为保留地址）
    for (addr = 1; addr < 127; addr++) {
        GetI2CBusHandle(i2c_num);
        err = i2c_master_probe(I2C_bus_handle[i2c_num], addr, 10);
        // 尝试添加设备（实际是发送探测包）
        if (err == ESP_OK) {
            // 地址响应成功，记录设备地址
            ESP_LOGI(TAG, "Found I2C device at address: 0x%02X", addr);
            found_devices++;
        } else {
            ESP_LOGV(TAG, "no I2C device at address: 0x%02X", addr);
            // 地址无响应，继续扫描
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(5));  // 短暂延时，避免总线过载
    }
    ESP_LOGI(TAG, "find devices num: %d", found_devices);

}
static void I2c_master_init(i2c_port_t i2c_num)
{
    if (I2C_bus_handle[i2c_num] != NULL) {
        return;
    }
    ESP_ERROR_CHECK(i2c_num < I2C_NUM_MAX ? ESP_OK : ESP_ERR_INVALID_ARG);

    i2c_master_bus_config_t bus_config = {
        .i2c_port = i2c_num,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    if (i2c_num == I2C_NUM_1) {
        bus_config.sda_io_num = I2C_MASTER_SDA_IO;
        bus_config.scl_io_num = I2C_MASTER_SCL_IO;
    }

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &I2C_bus_handle[i2c_num])); 

}

i2c_master_bus_handle_t GetI2CBusHandle(i2c_port_t i2c_num)
{
    if (I2C_bus_handle[i2c_num] == NULL) {
        I2c_master_init(i2c_num);
    }
    return I2C_bus_handle[i2c_num];
}

esp_err_t I2C_register_Device(i2c_port_t i2c_num, i2c_device_config_t *dev_cfg, i2c_master_dev_handle_t *dev_handle)
{
    return i2c_master_bus_add_device(GetI2CBusHandle(i2c_num), dev_cfg, dev_handle);
}