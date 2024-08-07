/*
 * MPU6050.c
 *
 *  Created on: Aug 7, 2024
 *      Author: razor
 */

#include "MPU6050.h"
#include "usbd_cdc.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

void MPU6050Init(void) {

	// I2C initialization of MPU6050 @ address 0x68
	if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDRESS << 1, 1, 100) == HAL_OK) {

		//Set gyro fullscale range to 500
		if (MPU6050_WriteReg(GYRO_CONF_REG, (uint8_t) GYRO_FS_500) != HAL_OK) {
			char TX_Buffer2[128] = "Unable to configure gyro full scale\r\n";
			CDC_Transmit_FS((uint8_t*) TX_Buffer2, strlen(TX_Buffer2));
			return;
		}

		if (MPU6050_WriteReg(ACC_CONG_REG, (uint8_t) ACC_FS_4G) != HAL_OK) {
			char TX_Buffer2[128] =
					"Unable to configure acceleration full scale\r\n";
			CDC_Transmit_FS((uint8_t*) TX_Buffer2, strlen(TX_Buffer2));
			return;
		}

		if (MPU6050_WriteReg(POWER_MGMT_REG, (uint8_t) 0) != HAL_OK) {
			char TX_Buffer2[128] =
					"Unable to configure power management\r\n";
			CDC_Transmit_FS((uint8_t*) TX_Buffer2, strlen(TX_Buffer2));
			return;
		}

		char TX_Buffer2[128] = "MPU6050 is ready\r\n";
		CDC_Transmit_FS((uint8_t*) TX_Buffer2, strlen(TX_Buffer2));
		return;
	} else {
		char TX_Buffer2[128] = "MPU6050 is not ready\r\n";
		CDC_Transmit_FS((uint8_t*) TX_Buffer2, strlen(TX_Buffer2));
		HAL_Delay(1000);
		MPU6050Init();

	}

}

void ReadAccelerometerData(void) {
    uint8_t accel_data[6]; // To store X, Y, and Z axis data (2 bytes each)
    if (MPU6050_ReadData(0x3B, accel_data, 6) == HAL_OK) {
        int16_t accel_x = (accel_data[0] << 8) | accel_data[1];
        int16_t accel_y = (accel_data[2] << 8) | accel_data[3];
        int16_t accel_z = (accel_data[4] << 8) | accel_data[5];

        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Accel X: %d, Y: %d, Z: %d\r\n", accel_x/ACC_SENSITIVITY_4G, accel_y/ACC_SENSITIVITY_4G, accel_z/ACC_SENSITIVITY_4G);
        CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
    } else {
        CDC_Transmit_FS((uint8_t*)"Failed to read accelerometer data\r\n", 35);
    }
}


HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t data) {
	return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS << 1, reg,
			I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_ReadData(uint8_t reg, uint8_t *data, uint16_t size) {
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}
