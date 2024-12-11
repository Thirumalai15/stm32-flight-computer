/*
 * mpu6050.c
 *
 *  Created on: Nov 23, 2024
 *      Author: root
 */

#include <mpu6050.h>

#include <main.h>

#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

void mpu6050_init() {
  HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady( & hi2c1, (MPU6050_ADDR << 1) + 0, 1, 100);

  if (ret == HAL_OK) {
    // i2c connected slow blink
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    HAL_Delay(800);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
    HAL_Delay(800);

    printf("The MPU6050 device is ready \n");
  } else {
    // i2c not connected fast blink
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
    HAL_Delay(100);
    printf("Failed to initialize MPU6050 make sure i2c is connected properly \n");
  }

  uint8_t temp_data = FS_GYRO_500;
  ret = HAL_I2C_Mem_Write( & hi2c1, (MPU6050_ADDR << 1) + 0, REG_CONFIG_GYRO, 1, & temp_data, 1, 100);

  if (ret == HAL_OK) {
    printf("Configuring gyroscope, writing regsiter to 28\n");
  } else {
    printf("Gyroscope Failed writing to register 28 \n");
  }

  temp_data = FS_ACCL_4G;
  ret = HAL_I2C_Mem_Write( & hi2c1, (MPU6050_ADDR << 1) + 0, REG_CONFIG_ACCL, 1, & temp_data, 1, 100);

  if (ret == HAL_OK) {
    printf("Configuring Accelerometer: Writing to register 29 \n");
  } else {
    printf("Accelerometer: Failed writing to register 28 \n");
  }

  temp_data = 0;
  ret = HAL_I2C_Mem_Write( & hi2c1, (MPU6050_ADDR << 1) + 0, REG_USR_CTRL, 1, & temp_data, 1, 100);

  if (ret == HAL_OK) {
    printf("Exiting from sleep mode \n");
  } else {
    printf("Failed to exit from sleep mode \n");
  }

}



void mpu6050_read()
{
	uint8_t data[2];
	int16_t x_acc;

	HAL_I2C_Mem_Read (&hi2c1,(MPU6050_ADDR << 1) + 0, ACC_REG_DATA, 1, data, 2, 1000);
	//Adding 2 BYTES into 16 bit integer
//	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
//	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
//	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	x_acc = ((int16_t)data[0] << 8) + data[1];

	printf("x axis acceleration %d \n", x_acc);


}


void mpu6050_read_all() {

    uint8_t data[14];
    int16_t ACCEL_RAW_X, ACCEL_RAW_Y, ACCEL_RAW_Z;
    int16_t GYRO_RAW_X, GYRO_RAW_Y, GYRO_RAW_Z;

    float acc_x,acc_y,acc_z; // actual values
    float gyro_x,gyro_y,gyro_z;


    HAL_I2C_Mem_Read(&hi2c1, (MPU6050_ADDR << 1), ACC_REG_DATA, 1, data, 14, 1000);


    ACCEL_RAW_X = (int16_t)(data[0] << 8 | data[1]);
    ACCEL_RAW_Y = (int16_t)(data[2] << 8 | data[3]);
    ACCEL_RAW_Z = (int16_t)(data[4] << 8 | data[5]);

    GYRO_RAW_X = (int16_t)(data[8] << 8 | data[9]);
    GYRO_RAW_Y = (int16_t)(data[10] << 8 | data[11]);
    GYRO_RAW_Z = (int16_t)(data[12] << 8 | data[13]);


    // Raw values to actual values

	acc_x = (float)ACCEL_RAW_X/16384.0;
	acc_y = (float)ACCEL_RAW_Y/16384.0;
	acc_z = (float)ACCEL_RAW_Z/16384.0;

	gyro_x = (float)GYRO_RAW_X/131.0;
	gyro_y = (float)GYRO_RAW_Y/131.0;
	gyro_z = (float)GYRO_RAW_Z/131.0;


    printf("Accelerometer (g): X=%.2f, Y=%.2f, Z=%.2f \n", acc_x, acc_y, acc_z);
    printf("Gyroscope : X=%.2f, Y=%.2f, Z=%.2f \n", gyro_x, gyro_y, gyro_z);
}


