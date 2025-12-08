#include "drv_MPU6050.h"
#include "I2c_ctrl.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "freertos/semphr.h"



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

static void ReadBytes(uint8_t regAddr, uint8_t length, uint8_t *data)
{
    uint8_t read_buffer[length];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(I2C_dev_handle, &regAddr, sizeof(regAddr), read_buffer, length, I2C_MASTER_TIMEOUT_MS));
    memcpy(data, read_buffer, length);
}

static void ReadByte(uint8_t regAddr, uint8_t *data)
{
    ReadBytes(regAddr, 1, data);
}

static void I2C_ReadBit(uint8_t regAddr, uint8_t bitNum, uint8_t *enable)
{
    uint8_t tmpdata = 0;
    ReadByte(regAddr, &tmpdata);
    *enable = tmpdata & (1 << bitNum);
}

static void I2C_ReadBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    uint8_t tmpdata = 0;
    ReadByte(regAddr, &tmpdata);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmpdata &= mask;
    tmpdata >>= (bitStart - length + 1);
    *data = tmpdata;
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


/** Get temperature FIFO enabled value.
 * When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L (Registers 65 and
 * 66) to be written into the FIFO buffer.
 * @return Current temperature FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool GetTempFIFOEnabled() {
    uint8_t data = 0;
    I2C_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, &data);
    return data;
}
/** Set temperature FIFO enabled value.
 * @param enabled New temperature FIFO enabled value
 * @see getTempFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void SetTempFIFOEnabled(bool enabled)
{
    I2C_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, enabled);
}
/** Get gyroscope X-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and
 * 68) to be written into the FIFO buffer.
 * @return Current gyroscope X-axis FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool GetXGyroFIFOEnabled() {
    uint8_t data = 0;
    I2C_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, &data);
    return data;
}
/** Set gyroscope X-axis FIFO enabled value.
 * @param enabled New gyroscope X-axis FIFO enabled value
 * @see getXGyroFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void SetXGyroFIFOEnabled(bool enabled)
{
    I2C_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, enabled);
}
/** Get gyroscope Y-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and
 * 70) to be written into the FIFO buffer.
 * @return Current gyroscope Y-axis FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool GetYGyroFIFOEnabled() {
    uint8_t data = 0;
    I2C_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, &data);
    return data;
}
/** Set gyroscope Y-axis FIFO enabled value.
 * @param enabled New gyroscope Y-axis FIFO enabled value
 * @see getYGyroFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void SetYGyroFIFOEnabled(bool enabled)
{
    I2C_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, enabled);
}
/** Get gyroscope Z-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and
 * 72) to be written into the FIFO buffer.
 * @return Current gyroscope Z-axis FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool GetZGyroFIFOEnabled() {
    uint8_t data = 0;
    I2C_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, &data);
    return data;
}
/** Set gyroscope Z-axis FIFO enabled value.
 * @param enabled New gyroscope Z-axis FIFO enabled value
 * @see getZGyroFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void SetZGyroFIFOEnabled(bool enabled)
{
    I2C_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, enabled);
}
/** Get accelerometer FIFO enabled value.
 * When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
 * ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be
 * written into the FIFO buffer.
 * @return Current accelerometer FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool GetAccelFIFOEnabled() {
    uint8_t data = 0;
    I2C_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, &data);
    return data;
}
/** Set accelerometer FIFO enabled value.
 * @param enabled New accelerometer FIFO enabled value
 * @see getAccelFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void SetAccelFIFOEnabled(bool enabled)
{
    I2C_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, enabled);
}
/** Get Slave 2 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 2 to be written into the FIFO buffer.
 * @return Current Slave 2 FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool GetSlave2FIFOEnabled() {
    uint8_t data = 0;
    I2C_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, &data);
    return data;
}
/** Set Slave 2 FIFO enabled value.
 * @param enabled New Slave 2 FIFO enabled value
 * @see getSlave2FIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void SetSlave2FIFOEnabled(bool enabled)
{
    I2C_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, enabled);
}
/** Get Slave 1 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 1 to be written into the FIFO buffer.
 * @return Current Slave 1 FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool GetSlave1FIFOEnabled() {
    uint8_t data = 0;
    I2C_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, &data);
    return data;
}
/** Set Slave 1 FIFO enabled value.
 * @param enabled New Slave 1 FIFO enabled value
 * @see getSlave1FIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void SetSlave1FIFOEnabled(bool enabled) 
{
    I2C_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, enabled);
}
/** Get Slave 0 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 0 to be written into the FIFO buffer.
 * @return Current Slave 0 FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool GetSlave0FIFOEnabled() {
    uint8_t data = 0;
    I2C_ReadBit(MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, &data);
    return data;
}
/** Set Slave 0 FIFO enabled value.
 * @param enabled New Slave 0 FIFO enabled value
 * @see getSlave0FIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void SetSlave0FIFOEnabled(bool enabled)
{
    I2C_WriteBit(MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, enabled);
}

// I2C_MST_CTRL register

/** Get multi-master enabled value.
 * Multi-master capability allows multiple I2C masters to operate on the same
 * bus. In circuits where multi-master capability is required, set MULT_MST_EN
 * to 1. This will increase current drawn by approximately 30uA.
 *
 * In circuits where multi-master capability is required, the state of the I2C
 * bus must always be monitored by each separate I2C Master. Before an I2C
 * Master can assume arbitration of the bus, it must first confirm that no other
 * I2C Master has arbitration of the bus. When MULT_MST_EN is set to 1, the
 * MPU-60X0's bus arbitration detection logic is turned on, enabling it to
 * detect when the bus is available.
 *
 * @return Current multi-master enabled value
 * @see MPU6050_RA_I2C_MST_CTRL
 */
bool GetMultiMasterEnabled()
{
    uint8_t data = 0;
    I2C_ReadBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, &data);
    return data;
}
/** Set multi-master enabled value.
 * @param enabled New multi-master enabled value
 * @see getMultiMasterEnabled()
 * @see MPU6050_RA_I2C_MST_CTRL
 */
void SetMultiMasterEnabled(bool enabled) {
    I2C_WriteBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, enabled);
}
/** Get wait-for-external-sensor-data enabled value.
 * When the WAIT_FOR_ES bit is set to 1, the Data Ready interrupt will be
 * delayed until External Sensor data from the Slave Devices are loaded into the
 * EXT_SENS_DATA registers. This is used to ensure that both the internal sensor
 * data (i.e. from gyro and accel) and external sensor data have been loaded to
 * their respective data registers (i.e. the data is synced) when the Data Ready
 * interrupt is triggered.
 *
 * @return Current wait-for-external-sensor-data enabled value
 * @see MPU6050_RA_I2C_MST_CTRL
 */
bool GetWaitForExternalSensorEnabled()
{
    uint8_t data = 0;
    I2C_ReadBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, &data);
    return data;
}
/** Set wait-for-external-sensor-data enabled value.
 * @param enabled New wait-for-external-sensor-data enabled value
 * @see getWaitForExternalSensorEnabled()
 * @see MPU6050_RA_I2C_MST_CTRL
 */
void setWaitForExternalSensorEnabled(bool enabled) {
    I2C_WriteBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, enabled);
}
/** Get Slave 3 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 3 to be written into the FIFO buffer.
 * @return Current Slave 3 FIFO enabled value
 * @see MPU6050_RA_MST_CTRL
 */
bool GetSlave3FIFOEnabled()
{
    uint8_t data = 0;
    I2C_ReadBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT, &data);
    return data;
}
/** Set Slave 3 FIFO enabled value.
 * @param enabled New Slave 3 FIFO enabled value
 * @see getSlave3FIFOEnabled()
 * @see MPU6050_RA_MST_CTRL
 */
void setSlave3FIFOEnabled(bool enabled) {
    I2C_WriteBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT, enabled);
}
/** Get slave read/write transition enabled value.
 * The I2C_MST_P_NSR bit configures the I2C Master's transition from one slave
 * read to the next slave read. If the bit equals 0, there will be a restart
 * between reads. If the bit equals 1, there will be a stop followed by a start
 * of the following read. When a write transaction follows a read transaction,
 * the stop followed by a start of the successive write will be always used.
 *
 * @return Current slave read/write transition enabled value
 * @see MPU6050_RA_I2C_MST_CTRL
 */
bool GetSlaveReadWriteTransitionEnabled() {
    uint8_t data = 0;
    I2C_ReadBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT, &data);
    return data;
}
/** Set slave read/write transition enabled value.
 * @param enabled New slave read/write transition enabled value
 * @see getSlaveReadWriteTransitionEnabled()
 * @see MPU6050_RA_I2C_MST_CTRL
 */
void SetSlaveReadWriteTransitionEnabled(bool enabled)
{
    I2C_WriteBit(MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT, enabled);
}
/** Get I2C master clock speed.
 * I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the
 * MPU-60X0 internal 8MHz clock. It sets the I2C master clock speed according to
 * the following table:
 *
 * <pre>
 * I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
 * ------------+------------------------+-------------------
 * 0           | 348kHz                 | 23
 * 1           | 333kHz                 | 24
 * 2           | 320kHz                 | 25
 * 3           | 308kHz                 | 26
 * 4           | 296kHz                 | 27
 * 5           | 286kHz                 | 28
 * 6           | 276kHz                 | 29
 * 7           | 267kHz                 | 30
 * 8           | 258kHz                 | 31
 * 9           | 500kHz                 | 16
 * 10          | 471kHz                 | 17
 * 11          | 444kHz                 | 18
 * 12          | 421kHz                 | 19
 * 13          | 400kHz                 | 20
 * 14          | 381kHz                 | 21
 * 15          | 364kHz                 | 22
 * </pre>
 *
 * @return Current I2C master clock speed
 * @see MPU6050_RA_I2C_MST_CTRL
 */
uint8_t getMasterClockSpeed() {
    uint8_t data = 0;
    I2C_ReadBits(MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT, MPU6050_I2C_MST_CLK_LENGTH, &data);
    return data;
}
/** Set I2C master clock speed.
 * @reparam speed Current I2C master clock speed
 * @see MPU6050_RA_I2C_MST_CTRL
 */
void SetMasterClockSpeed(uint8_t speed)
{
    I2C_WriteBits(MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT, MPU6050_I2C_MST_CLK_LENGTH, speed);
}


void MPU6050_WhoAmI()
{
    uint8_t tmpdata = 0;
    uint8_t reg_addr = MPU6050_WHO_AM_I_ADDR;
    uint8_t length = 1;
    esp_err_t err = i2c_master_transmit_receive(I2C_dev_handle, &reg_addr, sizeof(reg_addr), &tmpdata, length, I2C_MASTER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050_WHO_AM_I_ADDR");
        return;
    }
    // ReadByte(MPU6050_WHO_AM_I_ADDR, &tmpdata);
    ESP_LOGI(TAG, "MPU6050_WHO_AM_I_ADDR: 0x%x", tmpdata);
    if (tmpdata == MPU6050_ADDRESS_ADDRESS) {
        ESP_LOGI(TAG, "MPU6050 device found with 0x%x", tmpdata);
    } else {
        ESP_LOGE(TAG, "MPU6050 device not found! Read 0x%x", tmpdata);
    }
}

void MPU6050_Init(void *arg) 
{ 
    ESP_ERROR_CHECK(i2c_master_init());
/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */

    MPU6050_WhoAmI();
    SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
    SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
    SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    SetSleepEnabled(false); 

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    vTaskDelete(NULL); // 删除当前任务，不会触发错误

}