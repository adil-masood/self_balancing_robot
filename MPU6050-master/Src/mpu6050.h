/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>
#include "stm32f4xx_hal.h"

// MPU6050 structure
typedef struct
{

    float Accel_X_RAW;
    float Accel_Y_RAW;
    float Accel_Z_RAW;
    float Gx;
    float Gy;
    float Gz;

    float KalmanAngleX;
    float KalmanAngleY;
} MPU6050_t;

// Kalman structure
typedef struct
{
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
} Kalman_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt);
