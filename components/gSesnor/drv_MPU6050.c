#include "drv_MPU6050.h"
#include "esp_log.h"
#include "esp_err.h"



static const char *TAG = "MPU6050";

static i2c_master_dev_handle_t I2C_dev_handle = NULL;
static i2c_master_bus_handle_t I2C_bus_handle = NULL;



static esp_err_t i2c_master_init()
{
    I2C_bus_handle = GetI2CBusHandle(I2C_MASTER_NUM);
    if (I2C_bus_handle == NULL) {
        ESP_LOGE(TAG, "Failed to get I2C bus handle");
        return ESP_FAIL;
    }
    
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDRESS_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(I2C_bus_handle, &dev_config, &I2C_dev_handle));
    return ESP_OK;
}

static void ReadBytes(uint8_t reg_addr, uint8_t length, uint8_t *data)
{
    uint8_t read_buffer[length];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(I2C_dev_handle, &reg_addr, sizeof(reg_addr), read_buffer, length, I2C_MASTER_TIMEOUT_MS));
    memcpy(data, read_buffer, length);
}

static void ReadByte(uint8_t reg_addr, uint8_t *data)
{
    ReadBytes(reg_addr, 1, data);
}

static void I2C_WriteBit(uint8_t reg_addr, uint8_t bitNum, uint8_t enable)
{
    uint8_t tmpdata = 0;
    ReadByte(reg_addr, &tmpdata);

    if (enable) {
        tmpdata |= (1 << bitNum);
    } else {
        tmpdata &= ~(1 << bitNum);
    }

    uint8_t write_buffer[2];
    write_buffer[0] = reg_addr;
    write_buffer[1] = tmpdata;
    ESP_ERROR_CHECK(i2c_master_transmit(I2C_dev_handle, write_buffer, sizeof(write_buffer), I2C_MASTER_TIMEOUT_MS));
}


static void I2C_WriteBits(uint8_t reg_addr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t tmpdata = 0;
    ReadByte(reg_addr, &tmpdata);

    uint8_t mask = ((1 << length) - 1) << bitStart;
    tmpdata &= ~mask;
    tmpdata |= ((data << bitStart) & mask);

    uint8_t write_buffer[2];
    write_buffer[0] = reg_addr;
    write_buffer[1] = tmpdata;
    ESP_ERROR_CHECK(i2c_master_transmit(I2C_dev_handle, write_buffer, sizeof(write_buffer), I2C_MASTER_TIMEOUT_MS));

}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void SetClockSource(uint8_t source) 
{
    I2C_WriteBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void SetFullScaleGyroRange(uint8_t range) 

{
    I2C_WriteBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void SetFullScaleAccelRange(uint8_t range)
{
    I2C_WriteBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void SetSleepEnabled(uint8_t enabled) 
{
    I2C_WriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

esp_err_t MPU6050_Init(void) 
{ 
    ESP_ERROR_CHECK(i2c_master_init());
/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */

    SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
    SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
    SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    SetSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}