#include "I2c_ctrl.h"
#include "esp_log.h"
//#include "i2c.h"
#include "driver/i2c.h"
#include "freertos/semphr.h"
#include "driver/i2c_types.h"

static const char *TAG = "I2C_CTRL";
#define I2C_DEVICE_BUS_MAP_MAX 10

static i2c_master_bus_handle_t I2C_bus_handle[2] = {NULL};
SemaphoreHandle_t bus_lock_mux[I2C_NUM_MAX] = {NULL};

void LockI2CTransfer(i2c_dev_t I2C_cdev, uint8_t lockEn)
{
    if (lockEn) {
        xSemaphoreTake(bus_lock_mux[I2C_cdev.portIndex], portMAX_DELAY);
    } else {
        xSemaphoreGive(bus_lock_mux[I2C_cdev.portIndex]);
    }
}

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
    bus_lock_mux[i2c_num] = xSemaphoreCreateMutex();
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
    if (dev_cfg == NULL || dev_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (i2c_num >= I2C_NUM_MAX){
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_bus_add_device(GetI2CBusHandle(i2c_num), dev_cfg, dev_handle);
    return ret;
}

void I2C_Unregister_Device(i2c_master_dev_handle_t dev_handle)
{
    i2c_master_bus_rm_device(dev_handle);
}

void I2C_ReadBytes(i2c_dev_t I2C_cdev, uint8_t regAddr, uint8_t length, uint8_t *data)
{
    uint8_t read_buffer[length];
    LockI2CTransfer(I2C_cdev, 1);
    ESP_ERROR_CHECK(i2c_master_transmit_receive(I2C_cdev.i2c_dev, &regAddr, sizeof(regAddr), read_buffer, length, I2C_MASTER_TIMEOUT_MS));
    LockI2CTransfer(I2C_cdev, 0);
    memcpy(data, read_buffer, length);
}

void ReadByte(i2c_dev_t I2C_cdev, uint8_t regAddr, uint8_t *data)
{
    I2C_ReadBytes(I2C_cdev, regAddr, 1, data);
}

void I2C_ReadBit(i2c_dev_t I2C_cdev, uint8_t regAddr, uint8_t bitNum, uint8_t *enable)
{
    uint8_t tmpdata = 0;
    ReadByte(I2C_cdev, regAddr, &tmpdata);
    *enable = tmpdata & (1 << bitNum);
}

void I2C_ReadBits(i2c_dev_t I2C_cdev, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    uint8_t tmpdata = 0;
    ReadByte(I2C_cdev, regAddr, &tmpdata);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmpdata &= mask;
    tmpdata >>= (bitStart - length + 1);
    *data = tmpdata;
}

void I2C_WriteBit(i2c_dev_t I2C_cdev, uint8_t reg_addr, uint8_t bitNum, uint8_t enable)
{
    uint8_t tmpdata = 0;
    ReadByte(I2C_cdev, reg_addr, &tmpdata);

    if (enable) {
        tmpdata |= (1 << bitNum);
    } else {
        tmpdata &= ~(1 << bitNum);
    }

    uint8_t write_buffer[2];
    write_buffer[0] = reg_addr;
    write_buffer[1] = tmpdata;
    LockI2CTransfer(I2C_cdev, 1);
    ESP_ERROR_CHECK(i2c_master_transmit(I2C_cdev.i2c_dev, write_buffer, sizeof(write_buffer), I2C_MASTER_TIMEOUT_MS));
    LockI2CTransfer(I2C_cdev, 0);
}


void I2C_WriteBits(i2c_dev_t I2C_cdev, uint8_t reg_addr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t tmpdata = 0;
    ReadByte(I2C_cdev, reg_addr, &tmpdata);

    uint8_t mask = ((1 << length) - 1) << bitStart;
    tmpdata &= ~mask;
    tmpdata |= ((data << bitStart) & mask);

    uint8_t write_buffer[2];
    write_buffer[0] = reg_addr;
    write_buffer[1] = tmpdata;
    LockI2CTransfer(I2C_cdev, 1);
    ESP_ERROR_CHECK(i2c_master_transmit(I2C_cdev.i2c_dev, write_buffer, sizeof(write_buffer), I2C_MASTER_TIMEOUT_MS));
    LockI2CTransfer(I2C_cdev, 0);
}


esp_err_t I2C_WriteBytes(i2c_dev_t I2C_cdev, uint8_t reg_addr, uint8_t* write_buf, uint8_t length)
{
    LockI2CTransfer(I2C_cdev, 1);
    esp_err_t ret = i2c_master_transmit(I2C_cdev.i2c_dev, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
    LockI2CTransfer(I2C_cdev, 0);
    return ret;
}