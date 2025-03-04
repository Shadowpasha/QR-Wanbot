/*
 * MPU6050.c
 *
 *  Created on: Mar 27, 2023
 *      Author: amera
 */

#include "MPU6050.h"


void MPUInit(MPU6050_t *mpu6050, I2C_HandleTypeDef *hi2c,uint8_t rate, uint8_t AccelerometerSensitivity, uint8_t GyroscopeSensitivity, float sample_time, float alpha){

	uint8_t data;
	uint8_t temp;
	mpu6050->hi2c = hi2c;
	mpu6050->alpha = alpha;
	mpu6050->sample_time = sample_time;


	data = 0x00;
	HAL_I2C_Mem_Write(mpu6050->hi2c, (uint16_t)0xD0, MPU6050_PWR_MGMT_1, 1,&data, 1, 500);

	/* Set sample rate to 1kHz */
	data = rate;
	HAL_I2C_Mem_Write(mpu6050->hi2c, (uint16_t)0xD0, MPU6050_SMPLRT_DIV, 1,&data, 1, 500);

	/* Config accelerometer */
	uint8_t reg = MPU6050_ACCEL_CONFIG;
	HAL_I2C_Master_Transmit(mpu6050->hi2c, (uint16_t)0xD0, &reg, 1, 500);
	HAL_I2C_Master_Receive(mpu6050->hi2c, (uint16_t)0xD0, &temp, 1, 500);
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	HAL_I2C_Master_Transmit(mpu6050->hi2c, (uint16_t)0xD0,&temp, 1, 500);

	switch (AccelerometerSensitivity) {
	case MPU6050_Accelerometer_2G:
		mpu6050->Acc_Mult = (float)1 / MPU6050_ACCE_SENS_2;
		break;
	case MPU6050_Accelerometer_4G:
		mpu6050->Acc_Mult = (float)1 / MPU6050_ACCE_SENS_4;
		break;
	case MPU6050_Accelerometer_8G:
		mpu6050->Acc_Mult = (float)1 / MPU6050_ACCE_SENS_8;
		break;
	case MPU6050_Accelerometer_16G:
		mpu6050->Acc_Mult = (float)1 / MPU6050_ACCE_SENS_16;
		break;
	default:
		break;
	}

	/* Config Gyroscope */
	reg = MPU6050_GYRO_CONFIG;
	HAL_I2C_Master_Transmit(mpu6050->hi2c, (uint16_t)0xD0, &reg, 1, 500);
	HAL_I2C_Master_Receive(mpu6050->hi2c, (uint16_t)0xD0, &temp, 1, 500);
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity  << 3;
	HAL_I2C_Master_Transmit(mpu6050->hi2c, (uint16_t)0xD0,&temp, 1, 500);

	switch (GyroscopeSensitivity) {
	case MPU6050_Gyroscope_250s:
		mpu6050->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250;
		break;
	case MPU6050_Gyroscope_500s:
		mpu6050->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500;
		break;
	case MPU6050_Gyroscope_1000s:
		mpu6050->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000;
		break;
	case MPU6050_Gyroscope_2000s:
		mpu6050->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000;
		break;
	default:
		break;
	}

}

void MPUSetOffsets(MPU6050_t *mpu6050, int16_t ACCx, int16_t ACCy, int16_t ACCz, int16_t GYROx, int16_t GYROy, int16_t GYROz){

	mpu6050->Acc_offsets[0] = ACCx;
	mpu6050->Acc_offsets[1] = ACCy;
	mpu6050->Acc_offsets[2] = ACCz;

	mpu6050->Gyro_offsets[0] = GYROx;
	mpu6050->Gyro_offsets[1] = GYROy;
	mpu6050->Gyro_offsets[2] = GYROz;

}

void MPUReqAccGyro(MPU6050_t *mpu6050){

	uint8_t buff[6];

	HAL_I2C_Mem_Read(mpu6050->hi2c, (uint16_t)0xD0, MPU6050_ACCEL_XOUT_H, 1, buff, 6, 5);

	mpu6050->Accx = (int16_t)(buff[0] << 8 | buff[1]);
	mpu6050->Accy = (int16_t)(buff[2] << 8 | buff[3]);
	mpu6050->Accz = (int16_t)(buff[4] << 8 | buff[5]);

	mpu6050->Acc_f[0] = (mpu6050->Accx + mpu6050->Acc_offsets[0]) * mpu6050->Acc_Mult;
	mpu6050->Acc_f[1] = (mpu6050->Accy + mpu6050->Acc_offsets[1]) * mpu6050->Acc_Mult;
	mpu6050->Acc_f[2] = (mpu6050->Accz + mpu6050->Acc_offsets[2]) * mpu6050->Acc_Mult;

	HAL_I2C_Mem_Read(mpu6050->hi2c, (uint16_t)0xD0, MPU6050_GYRO_XOUT_H, 1, buff, 6, 5);

	mpu6050->Gyrox = (int16_t)(buff[0] << 8 | buff[1]);
	mpu6050->Gyroy = (int16_t)(buff[2] << 8 | buff[3]);
	mpu6050->Gyroz = (int16_t)(buff[4] << 8 | buff[5]);

	mpu6050->Gyro_f[0] = (mpu6050->Gyrox + mpu6050->Gyro_offsets[0]) * mpu6050->Gyro_Mult;
	mpu6050->Gyro_f[1] = (mpu6050->Gyroy + mpu6050->Gyro_offsets[1]) * mpu6050->Gyro_Mult;
	mpu6050->Gyro_f[2] = (mpu6050->Gyroz + mpu6050->Gyro_offsets[2]) * mpu6050->Gyro_Mult;

}


void CompPitchRoll(MPU6050_t *mpu6050){

	if(mpu6050->Acc_f[1] && mpu6050->Acc_f[2] && mpu6050->Acc_f[0]){
		float acc_pitch = atan ((mpu6050->Acc_f[1]) / (mpu6050->Acc_f[2])) *  57.3248;
		float acc_roll = - atan ((mpu6050->Acc_f[0]) / (mpu6050->Acc_f[2])) * 57.3248;

		mpu6050->pitch = (acc_pitch * mpu6050->alpha) + ((mpu6050->pitch + mpu6050->Gyro_f[0]*mpu6050->sample_time)*(1-mpu6050->alpha));
		mpu6050->roll = (acc_roll * mpu6050->alpha) + ((mpu6050->roll + mpu6050->Gyro_f[1]*mpu6050->sample_time)*(1-mpu6050->alpha));
	}

}


//void EulertoQuat(MPU6050_t *mpu6050, float pitch, float roll, float yaw){
//
//	double cr = cos(roll * 0.5);
//	double sr = sin(roll * 0.5);
//	double cp = cos(pitch * 0.5);
//	double sp = sin(pitch * 0.5);
//	double cy = cos(yaw * 0.5);
//	double sy = sin(yaw * 0.5);
//
//
//	mpu6050->quat[0] = cr * cp * cy + sr * sp * sy;
//	mpu6050->quat[1] = sr * cp * cy - cr * sp * sy;
//	mpu6050->quat[2] = cr * sp * cy + sr * cp * sy;
//	mpu6050->quat[3] = cr * cp * sy - sr * sp * cy;
//
//}
