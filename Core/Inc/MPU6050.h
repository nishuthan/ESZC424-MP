#ifndef MPU6050_H // Prevent multiple inclusions
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

// MPU6050 I2C address
#define MPU6050_ADDRESS 0x68

// Gyroscope configuration registers and settings
#define GYRO_CONF_REG 0x1B
#define GYRO_FS_250 (0x0 << 3)
#define GYRO_FS_500 (0x1 << 3)
#define GYRO_FS_1000 (0x2 << 3)
#define GYRO_FS_2000 (0x3 << 3)

// Accelerometer configuration registers and settings
#define ACC_CONF_REG 0x1C
#define ACC_FS_2G (0x0 << 3)
#define ACC_FS_4G (0x1 << 3)
#define ACC_FS_8G (0x2 << 3)
#define ACC_FS_16G (0x3 << 3)

// Power management register
#define POWER_MGMT_REG 0x6B

// Accelerometer data registers
#define DATA_REG_ACC_XOUT_H 0x3B
#define DATA_REG_ACC_XOUT_L 0x3C
#define DATA_REG_ACC_YOUT_H 0x3D
#define DATA_REG_ACC_YOUT_L 0x3E
#define DATA_REG_ACC_ZOUT_H 0x3F
#define DATA_REG_ACC_ZOUT_L 0x40

// Accelerometer LSB sensitivity values
#define ACC_SENSITIVITY_2G 16384.0f
#define ACC_SENSITIVITY_4G 8192.0f
#define ACC_SENSITIVITY_8G 4096.0f
#define ACC_SENSITIVITY_16G 2048.0f

// Gyroscope data registers
#define DATA_REG_GYRO_XOUT_H 0x43
#define DATA_REG_GYRO_XOUT_L 0x44
#define DATA_REG_GYRO_YOUT_H 0x45
#define DATA_REG_GYRO_YOUT_L 0x46
#define DATA_REG_GYRO_ZOUT_H 0x47
#define DATA_REG_GYRO_ZOUT_L 0x48

// Gyroscope LSB sensitivity values (in degrees per second)
#define GYRO_SENSITIVITY_250 131.0f
#define GYRO_SENSITIVITY_500 65.5f
#define GYRO_SENSITIVITY_1000 32.8f
#define GYRO_SENSITIVITY_2000 16.4f

// Number of samples for averaging
#define MAX_WINDOW 1000

// Struct for storing pitch, roll, and yaw angles
typedef struct {
    float aX, aY, aZ;
    float pitch;
    float roll;
    float yaw;
} MPU6050_data_t;

// Function prototypes
void MPU6050Init(void);
HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t data);
HAL_StatusTypeDef MPU6050_ReadData(uint8_t reg, uint8_t *data, uint16_t size);
void readAccelerometerData(float *aX, float *aY, float *aZ);
void readGyroData(float *gX, float *gY, float *gZ);
MPU6050_data_t calculateTiltAngles(void);
void calibrateAccelerometer(void);
void calibrateGyro(void);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H
