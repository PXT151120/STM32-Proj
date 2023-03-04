/*
 * MPU6050.h
 *
 *  Created on: Jan 25, 2023
 *      Author: ASUS
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#define MPU6050 			0x68
#define MPU6050_ADDRESS 	0x68 << 1	//	STM32 i2c 8 -> 7 + 1 (R/W)
#define WHO_AM_I 			0x75
#define PWR_MGMT			0x6B
#define SMPLRT_DIV			0x19
#define GYRO_CONFIG			0x1B
#define ACCEL_CONFIG		0x1C
#define GYRO_OUT_REG		0x43
#define ACCEL_OUT_REG		0x3B
#define TEMP_OUT_REG		0x41
#define RADIAN_TO_DEGREE	57.295779513082320876798154814105

typedef enum{
	degS250  = 0,
	degS500,
	degS1000,
	degS2000
}gyroScale_t;

typedef enum{
	g2  = 0,
	g4,
	g8,
	g16
}accelScale_t;

typedef struct
{
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
}Kalman_t;

typedef struct
{
	float comRoll;
	float comPicth;
	float alpha;
}Complementary_t;



void MPU6050_Init(void);
void MPU6050_PowerOn(void);
void MPU6050_Sampling(void);
void MPU6050_GyroScale(gyroScale_t);
void MPU6050_AccelScale(accelScale_t);
float GyroScaleSelect(gyroScale_t);
float AccScaleSelect(accelScale_t);
void MPU6050_Config(I2C_HandleTypeDef*, gyroScale_t , accelScale_t );
void MPU6050_GyroReadRaw(int16_t*);
void MPU6050_AccelReadRaw(int16_t*);
void MPU6050_TempReadRaw(int16_t);
void MPU6050_TempReadRaw(int16_t);
void Accel_RollPitchAngle(float*);
void Gyro_RollPitchYawAngle(float*);
float Temp_DegreeData(int16_t);

void ComplementaryFilter_Angle(Complementary_t*);
void Kalman_Init(Kalman_t*);
float KalmanFilter_Angle(Kalman_t*, float, float, float);
void Kalman_AngleX(Kalman_t*);
void Kalman_AngleY(Kalman_t*);
void Complementary_Init(Complementary_t*, float);
#endif /* INC_MPU6050_H_ */
