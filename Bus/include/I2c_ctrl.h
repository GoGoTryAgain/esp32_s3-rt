#ifndef I2C_CTRL_H
#define I2C_CTRL_H

#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO           35                          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           36                          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_INDEX              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          300000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000


extern i2c_master_bus_handle_t GetI2CBusHandle(i2c_port_t i2c_num);
void I2C_master_device_probe(i2c_port_t i2c_num);

esp_err_t I2C_register_Device(i2c_port_t i2c_num, i2c_device_config_t *dev_cfg, i2c_master_dev_handle_t *dev_handle);

#endif // I2C_CTRL_H
