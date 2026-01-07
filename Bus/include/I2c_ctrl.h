#ifndef I2C_CTRL_H
#define I2C_CTRL_H

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

#define I2C_MASTER_SCL_IO           35                          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           36                          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_INDEX              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          300000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000


typedef struct {
    i2c_port_t portIndex;
    i2c_master_dev_handle_t i2c_dev;
} i2c_dev_t;


extern i2c_master_bus_handle_t GetI2CBusHandle(i2c_port_t i2c_num);
void I2C_master_device_probe(i2c_port_t i2c_num);

esp_err_t I2C_register_Device(i2c_port_t i2c_num, i2c_device_config_t *dev_cfg, i2c_master_dev_handle_t *dev_handle);



void I2C_ReadBit(i2c_dev_t I2C_cdev, uint8_t regAddr, uint8_t bitNum, uint8_t *enable);
void I2C_ReadBits(i2c_dev_t I2C_cdev, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);

void ReadByte(i2c_dev_t I2C_cdev, uint8_t regAddr, uint8_t *data);
void I2C_ReadBytes(i2c_dev_t I2C_cdev, uint8_t regAddr, uint8_t length, uint8_t *data);

void I2C_WriteBit(i2c_dev_t I2C_cdev, uint8_t reg_addr, uint8_t bitNum, uint8_t enable);
void I2C_WriteBits(i2c_dev_t I2C_cdev, uint8_t reg_addr, uint8_t bitStart, uint8_t length, uint8_t data);
esp_err_t I2C_WriteBytes(i2c_dev_t I2C_cdev, uint8_t* write_buf, uint8_t length);

void I2C_Unregister_Device(i2c_master_dev_handle_t dev_handle);

#endif // I2C_CTRL_H
