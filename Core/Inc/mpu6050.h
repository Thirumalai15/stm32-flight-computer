/*
 * mpu6050.h
 *
 *  Created on: Nov 23, 2024
 *      Author: root
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define MPU6050_ADDR 0x68


#define FS_GYRO_250   0
#define FS_GYRO_500   8
#define FS_GYRO_1000  9
#define FS_GYRO_2000  10

#define FS_ACCL_2G    0
#define FS_ACCL_4G    8
#define FS_ACCL_8G    9
#define FS_ACCL_16G   10


#define REG_CONFIG_GYRO  27
#define REG_CONFIG_ACCL  28
#define REG_USR_CTRL     107
#define ACC_REG_DATA     59
#define ACCEL_XOUT_H     0x3B




void mpu6050_init();

void mpu6050_read_all();

#endif /* INC_MPU6050_H_ */
