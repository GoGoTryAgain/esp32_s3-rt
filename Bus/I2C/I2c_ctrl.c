#include "I2c_ctrl.h"


i2c_master_bus_handle_t I2C_bus_handle[2] = {NULL};

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