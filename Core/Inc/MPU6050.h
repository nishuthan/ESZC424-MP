#ifndef MPU6050_H  // Corrected the header guard check
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define MPU6050_ADDRESS 0x68

#define GYRO_CONF_REG	0x1B
#define GYRO_FS_250		0x0<<3
#define GYRO_FS_500		0x1<<3
#define GYRO_FS_1000 	0x2<<3
#define GYRO_FS_2000 	0x3<<3

#define ACC_CONG_REG	0x1C
#define ACC_FS_2G		0x0<<3
#define ACC_FS_4G		0x1<<3
#define ACC_FS_8G		0x2<<3
#define ACC_FS_16G		0x3<<3

#define POWER_MGMT_REG			0x6B

#define DATA_REG_ACC_XOUT_H		0x3B
#define DATA_REG_ACC_XOUT_L		0x3C

#define DATA_REG_ACC_YOUT_H		0x3D
#define DATA_REG_ACC_YOUT_L		0x3E

#define DATA_REG_ACC_ZOUT_H		0x3F
#define DATA_REG_ACC_ZOUT_L		0x40

#define ACC_SENSITIVITY_2G 		16384
#define ACC_SENSITIVITY_4G 		8192
#define ACC_SENSITIVITY_8G 		4096
#define ACC_SENSITIVITY_16G 	2048


void MPU6050Init(void);
HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t data);
HAL_StatusTypeDef MPU6050_ReadData(uint8_t reg, uint8_t *data, uint16_t size);
void ReadAccelerometerData(void);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H
