#ifndef _DRV_MPU6050_H_
#define _DRV_MPU6050_H_

#include "I2c_ctrl.h"


#define MPU6050_ADDRESS_AD0_LOW     (0x68) // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    (0x69) // address pin high (VCC)

#define MPU6050_ADDRESS_ADDRESS       MPU6050_ADDRESS_AD0_LOW

#define MPU6050_SCL_PIN            I2C_MASTER_SCL_IO
#define MPU6050_SDA_PIN            I2C_MASTER_SDA_IO



#endif /* _DRV_MPU6050_H_ */

