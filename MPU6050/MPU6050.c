/*
 * MPU6050.c
 *
 *  Created on: Jan 25, 2023
 *      Author: ASUS
 */
#include "MPU6050.h"
#include <stdbool.h>

I2C_HandleTypeDef *i2c;



uint8_t icCheck;
uint32_t timer;

gyroScale_t gyroScale;
accelScale_t accelScale;

unsigned long prevT = 0;

void MPU6050_Init(void)
{
	if (HAL_I2C_GetState(i2c) == HAL_I2C_STATE_READY)
	{
		icCheck = 1;
		uint8_t data = 0x00;
		HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, PWR_MGMT, 1, &data, 1, 1000);

		HAL_Delay(50);

		data = 1 << 7;
		HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, PWR_MGMT, 1, &data, 1, 1000);

		HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, WHO_AM_I, 1, &icCheck, 1, 1000);

	}
	else if (HAL_I2C_IsDeviceReady(i2c, MPU6050_ADDRESS, 2, 1000) != HAL_OK)
	{
		icCheck = 2;

	}
}
void MPU6050_PowerOn(void)
{

	uint8_t data = 0x01;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, PWR_MGMT, 1, &data, 1, 100);
}

void MPU6050_Sampling(void)
{
	uint8_t data = 0x07;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, SMPLRT_DIV, 1, &data, 1, 100);
}

void MPU6050_GyroScale(gyroScale_t gyroScale)
{
	uint8_t data = 0x00 | (gyroScale << 3);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, GYRO_CONFIG, 1, &data, 1, 100);
}

void MPU6050_AccelScale(accelScale_t accelScale)
{
	uint8_t data = 0x00 | (accelScale << 3);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, ACCEL_CONFIG, 1, &data, 1, 100);
}

float AccScaleSelect(accelScale_t as)
{
	if (as == 0)	return 16384.0;
	else if (as == 1) return 8192.0;
	else if (as == 2) return 4096.0;
	else if (as == 3) return 2048.0;
	else return 1;
}

float GyroScaleSelect(gyroScale_t gs)
{
	if (gs == 0) return 131.0;
	else if (gs == 1) return 65.5;
	else if (gs == 2) return 32.8;
	else if (gs == 3) return 16.4;
	else return 1;
}



void MPU6050_Config(I2C_HandleTypeDef *hi2c, gyroScale_t gscale, accelScale_t ascale)
{
	i2c = hi2c;

	MPU6050_Init();

	if (icCheck == MPU6050)
	{
		MPU6050_PowerOn();
		MPU6050_Sampling();
		MPU6050_GyroScale(gscale);
		MPU6050_AccelScale(ascale);
	}

	gyroScale = gscale;
	accelScale = ascale;
	HAL_Delay(50);
}

void MPU6050_GyroReadRaw(int16_t *rawData)
{
	uint8_t gRawData[6];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, GYRO_OUT_REG, 1, gRawData, 6, 100);
	rawData[0] = (int16_t)(gRawData[0] << 8 | gRawData[1]);
	rawData[1] = (int16_t)(gRawData[2] << 8 | gRawData[3]);
	rawData[2] = (int16_t)(gRawData[4] << 8 | gRawData[5]);
}

void MPU6050_AccelReadRaw(int16_t *rawData)
{
	uint8_t aRawData[6];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, ACCEL_OUT_REG, 1, aRawData, 6, 100);
	rawData[0] = (int16_t)(aRawData[0] << 8 | aRawData[1]);
	rawData[1] = (int16_t)(aRawData[2] << 8 | aRawData[3]);
	rawData[2] = (int16_t)(aRawData[4] << 8 | aRawData[5]);
}


void MPU6050_TempReadRaw(int16_t rawData)
{
	uint8_t tempData[2];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, TEMP_OUT_REG, 1, tempData, 2, 100);
	rawData = (int16_t)(tempData[0] << 8 | tempData[1]);
}

void Accel_RollPitchAngle(float *rpAngle)
{
	int16_t rawData[3];
	MPU6050_AccelReadRaw(rawData);

	float accelVal[3];
	float s = AccScaleSelect(accelScale);

	accelVal[0] = (float)rawData[0] / s;
	accelVal[1] = (float)rawData[1] / s;
	accelVal[2] = (float)rawData[2] / s;

	rpAngle[0] = RADIAN_TO_DEGREE * atanf(accelVal[1] / sqrtf(pow(accelVal[0], 2) + pow(accelVal[2], 2)));
	rpAngle[1] = -RADIAN_TO_DEGREE * atanf(accelVal[0] / sqrtf(pow(accelVal[1], 2) + pow(accelVal[2], 2)));
}

void Gyro_RollPitchYawAngle(float *rpyAngle)
{
	int16_t rawData[3] = {0, 0, 0};
	MPU6050_GyroReadRaw(rawData);

	float gyroVal[3] = {0, 0, 0};
	float gs = GyroScaleSelect(gyroScale);

	for (int i = 0; i < 3; i++)
		gyroVal[i] = (float)rawData[i] / gs;

	float elapsedT = (HAL_GetTick() - prevT) * 0.001;

	rpyAngle[0] = rpyAngle[0] +  gyroVal[0] * elapsedT;
	rpyAngle[1] = rpyAngle[1] +  gyroVal[1] * elapsedT;
	rpyAngle[2] = rpyAngle[2] +  gyroVal[2] * elapsedT;

	prevT = HAL_GetTick();

}

float Temp_DegreeData(int16_t regData)
{
	MPU6050_TempReadRaw(regData);
	return regData / 340.0 + 36.53;
}

void ComplementaryFilter_Angle(Complementary_t *comAngle)
{
	int16_t accRawData[3], gyroRawData[3];

	MPU6050_AccelReadRaw(accRawData);

	float accelVal[3];
	float as = AccScaleSelect(accelScale);
	accelVal[0] = (float)accRawData[0] / as;
	accelVal[1] = (float)accRawData[1] / as;
	accelVal[2] = (float)accRawData[2] / as;

	float accRoll = RADIAN_TO_DEGREE * atanf(accelVal[1] / sqrtf(pow(accelVal[0], 2) + pow(accelVal[2], 2)));
	float accPitch = -RADIAN_TO_DEGREE * atanf(accelVal[0] / sqrtf(pow(accelVal[1], 2) + pow(accelVal[2], 2)));


	MPU6050_GyroReadRaw(gyroRawData);

	float gyroVal[3];
	float gs = GyroScaleSelect(gyroScale);
	gyroVal[0] = (float) gyroRawData[0] / gs;
	gyroVal[1] = (float) gyroRawData[1] / gs;
	gyroVal[2] = (float) gyroRawData[2] / gs;

	float elapsedT = (HAL_GetTick() - prevT) * 0.001;

	comAngle->comRoll = comAngle->alpha * (comAngle->comRoll + gyroVal[0] * elapsedT) + (1 - comAngle->alpha) * accRoll;
	comAngle->comPicth = comAngle->alpha * (comAngle->comPicth + gyroVal[1] * elapsedT) + (1 - comAngle->alpha) * accPitch;

	prevT = HAL_GetTick();
}

float KalmanFilter_Angle(Kalman_t* kalman, float newRate, float newAngle, float dt)
{
	// Step 1
	float rate = newRate - kalman->bias;
	kalman->angle += rate * dt;

	// Step 2
	kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
	kalman->P[0][1] -= dt * kalman->P[1][1];
	kalman->P[1][0] -= dt * kalman->P[1][1];
	kalman->P[1][1] += dt * kalman->Q_bias;

	// Step 3
	float y = newAngle - kalman->angle;

	// Step 4
	float S = kalman->P[0][0] + kalman->R_measure;

	// Step 5
	float K0 = kalman->P[0][0] / S;
	float K1 = kalman->P[1][0] / S;

	// Step 6
	kalman->angle += K0 * y;
	kalman->bias += K1 * y;

	// Step 7
	float P00_temp = kalman->P[0][0];
	float P01_temp = kalman->P[0][1];

	kalman->P[0][0] -= K0 * P00_temp;
	kalman->P[0][1] -= K0 * P01_temp;
	kalman->P[1][0] -= K1 * P00_temp;
	kalman->P[1][1] -= K1 * P01_temp;

	return kalman->angle;

}


void Kalman_AngleX(Kalman_t *kalmanAngle)
{
	int16_t accRawData[3], gyroRawData[3];

	MPU6050_AccelReadRaw(accRawData);

	float accelVal[3];
	float as = AccScaleSelect(accelScale);
	accelVal[0] = (float) accRawData[0] / as;
	accelVal[1] = (float) accRawData[1] / as;
	accelVal[2] = (float) accRawData[2] / as;

	float accRoll = RADIAN_TO_DEGREE * atanf(accelVal[1] / sqrtf(pow(accelVal[0], 2) + pow(accelVal[2], 2)));

	MPU6050_GyroReadRaw(gyroRawData);

	float gyroVal[3];
	float gs = GyroScaleSelect(gyroScale);
	gyroVal[0] = (float) gyroRawData[0] / gs;

	float elapsedT = (HAL_GetTick() - prevT) * 0.001;


	KalmanFilter_Angle(kalmanAngle, gyroVal[0], accRoll, elapsedT);

	prevT = HAL_GetTick();

}

void Kalman_AngleY(Kalman_t *kalmanAngle)
{
	int16_t accRawData[3], gyroRawData[3];

	MPU6050_AccelReadRaw(accRawData);

	float accelVal[3];
	float as = AccScaleSelect(accelScale);
	accelVal[0] = (float) accRawData[0] / as;
	accelVal[1] = (float) accRawData[1] / as;
	accelVal[2] = (float) accRawData[2] / as;

	float accPitch = -RADIAN_TO_DEGREE * atanf(accelVal[0] / sqrtf(pow(accelVal[1], 2) + pow(accelVal[2], 2)));

	MPU6050_GyroReadRaw(gyroRawData);

	float gyroVal[3];
	float gs = GyroScaleSelect(gyroScale);
	gyroVal[1] = (float) gyroRawData[1] / gs;


	float elapsedT = (HAL_GetTick() - prevT) * 0.001;


	KalmanFilter_Angle(kalmanAngle, gyroVal[1], accPitch, elapsedT);

	prevT = HAL_GetTick();

}

void Kalman_Init(Kalman_t *kStart)
{
	kStart->Q_angle = 0.001f;
	kStart->Q_bias = 0.003f;
	kStart->R_measure = 0.03f;
}

void Complementary_Init(Complementary_t* cStart, float alpha)
{
	cStart->alpha = alpha;
	cStart->comPicth = 0.0f;
	cStart->comRoll = 0.0f;
}
