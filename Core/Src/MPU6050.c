/*
 * MPU6050.c
 *
 *  Created on: Aug 7, 2024
 *      Author: razor
 */

#include "MPU6050.h"
#include "stm32f1xx_hal.h"
#include <math.h>

// External I2C handler declaration
extern I2C_HandleTypeDef hi2c1;

// Global variable to store axis offsets for calibration
float xAxisOffset = 0.0f;
float yAxisOffset = 0.0f;
float zAxisOffset = 0.0f;

// Global variables to store gyroscope offsets
float gyroXOffset = 0.0f;
float gyroYOffset = 0.0f;
float gyroZOffset = 0.0f;

// Function to initialize MPU6050
void MPU6050Init(void) {
	// Check if MPU6050 is ready on I2C bus at address 0x68
	if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDRESS << 1, 1, 100) == HAL_OK) {
		// Configure gyroscope to full-scale range of ±250 degrees/second
		if (MPU6050_WriteReg(GYRO_CONF_REG, (uint8_t) GYRO_FS_250) != HAL_OK) {
			// Unable to set range
			return;
		}

		// Configure accelerometer to full-scale range of ±2g
		if (MPU6050_WriteReg(ACC_CONF_REG, (uint8_t) ACC_FS_2G) != HAL_OK) {
			return;
		}

		// Wake up the MPU6050 by clearing the power management register
		if (MPU6050_WriteReg(POWER_MGMT_REG, (uint8_t) 0) != HAL_OK) {
			return;
		}

		// Optionally perform calibration here or call calibrateAccelerometer() externally

		return;
	} else {
		// Retry initialization after a delay
		HAL_Delay(1000);
		MPU6050Init(); // Retry initialization
	}
}

// Function to read accelerometer data
void readAccelerometerData(float *aX, float *aY, float *aZ) {
	uint8_t accel_data[6]; // Buffer to store X, Y, and Z axis data (2 bytes each)
	if (MPU6050_ReadData(DATA_REG_ACC_XOUT_H, accel_data, 6) == HAL_OK) {
		int16_t accel_x = (accel_data[0] << 8) | accel_data[1];
		int16_t accel_y = (accel_data[2] << 8) | accel_data[3];
		int16_t accel_z = (accel_data[4] << 8) | accel_data[5];

		// Apply offset and convert raw values to acceleration in g units
		*aX = (accel_x / ACC_SENSITIVITY_2G) + xAxisOffset;
		*aY = (accel_y / ACC_SENSITIVITY_2G) + yAxisOffset;
		*aZ = (accel_z / ACC_SENSITIVITY_2G) + zAxisOffset;
	} else {
		// If data cannot be read, set to zero
		*aX = 0;
		*aY = 0;
		*aZ = 0;
	}
}

// Function to read gyroscope data
void readGyroData(float *gX, float *gY, float *gZ) {
	uint8_t gyro_data[6];
	if (MPU6050_ReadData(DATA_REG_GYRO_XOUT_H, gyro_data, 6) == HAL_OK) {
		int16_t gyro_x = (gyro_data[0] << 8) | gyro_data[1];
		int16_t gyro_y = (gyro_data[2] << 8) | gyro_data[3];
		int16_t gyro_z = (gyro_data[4] << 8) | gyro_data[5];

		// Convert the raw values to degrees per second
		*gX = gyro_x / GYRO_SENSITIVITY_250;
		*gY = gyro_y / GYRO_SENSITIVITY_250;
		*gZ = gyro_z / GYRO_SENSITIVITY_250;

		*gX -=  gyroXOffset;
		*gY -=  gyroYOffset;
		*gZ -=  gyroZOffset;

	} else {
		// If data cannot be read, return zeros
		*gX = 0.0;
		*gY = 0.0;
		*gZ = 0.0;
	}
}


// Function to calculate tilt angles using accelerometer data
MPU6050_data_t calculateTiltAngles(void) {

	MPU6050_data_t data = { 0, 0, 0 };
	// Accelerometer data
	float aX = 0.0f;
	float aY = 0.0f;
	float aZ = 0.0f;

	// Read accelerometer data (average over MAX_WINDOW samples)
	for (int i = 0; i < MAX_WINDOW; i++) {
		float avg_aX = 0.0f, avg_aY = 0.0f, avg_aZ = 0.0f;
		readAccelerometerData(&avg_aX, &avg_aY, &avg_aZ);
		aX += avg_aX;
		aY += avg_aY;
		aZ += avg_aZ;
	}

	aX /= MAX_WINDOW;
	aY /= MAX_WINDOW;
	aZ /= MAX_WINDOW;

	// Calculate accelerometer-based pitch and roll
	data.pitch = atan2(aY, sqrt(aX * aX + aZ * aZ)) * 180.0 / M_PI;
	data.roll = atan2(-aX, sqrt(aY * aY + aZ * aZ)) * 180.0 / M_PI;


	data.aX = aX;
	data.aY = aY;
	data.aZ = aZ;

	return data;
}

// Function to calibrate the MPU6050 by calculating offsets for all axes
void calibrateAccelerometer(void) {
	float sumX = 0.0f;
	float sumY = 0.0f;
	float sumZ = 0.0f;
	float aX = 0.0f, aY = 0.0f, aZ = 0.0f;
	int samples = 100; // Number of samples to average

	// Collect multiple samples to calculate average offsets
	for (int i = 0; i < samples; i++) {
		readAccelerometerData(&aX, &aY, &aZ); // Read the accelerometer data
		sumX += aX; // Sum the X-axis readings
		sumY += aY; // Sum the Y-axis readings
		sumZ += aZ; // Sum the Z-axis readings
		HAL_Delay(10); // Small delay between readings
	}

	// Calculate the average values for each axis
	float avgX = sumX / samples;
	float avgY = sumY / samples;
	float avgZ = sumZ / samples;

	// Calculate the offsets (expected value is 0 for X and Y, and 1g for Z)
	xAxisOffset = 0.0f - avgX;
	yAxisOffset = 0.0f - avgY;
	zAxisOffset = 1.0f - avgZ;
}

// Function to calibrate the gyroscope by calculating offsets for all axes
void calibrateGyro(void) {
	float sumX = 0.0f;
	float sumY = 0.0f;
	float sumZ = 0.0f;
	float gX, gY, gZ;
	int samples = 100; // Number of samples to average

	// Collect multiple samples to calculate average offsets
	for (int i = 0; i < samples; i++) {
		readGyroData(&gX, &gY, &gZ); // Read the gyroscope data
		sumX += gX; // Sum the X-axis readings
		sumY += gY; // Sum the Y-axis readings
		sumZ += gZ; // Sum the Z-axis readings
		HAL_Delay(10); // Small delay between readings
	}

	// Calculate the average values for each axis
	gyroXOffset = sumX / samples;
	gyroYOffset = sumY / samples;
	gyroZOffset = sumZ / samples;
}

// Function to write data to a specific MPU6050 register
HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t data) {
	return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS << 1, reg,
	I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

// Function to read data from a specific MPU6050 register
HAL_StatusTypeDef MPU6050_ReadData(uint8_t reg, uint8_t *data, uint16_t size) {
	return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS << 1, reg,
	I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}
