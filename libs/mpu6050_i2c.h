#ifndef MPU6050_I2C_H
#define MPU6050_I2C_H
#include "stm32f4xx_hal.h"
#define DPS_250 0x00U
#define DPS_500 0x08U
#define DPS_1000 0x10U
#define DPS_2000 0x18U
#define LPF_BEST 0x06U
#define LPF_MEDIUM 0x04U
#define LPF_FAIR 0x02U
#define LPF_NO 0x00U
#define ACCEL_2G 0x00U
#define ACCEL_4G	0x08U
#define ACCEL_8G	0x10U
#define ACCEL_16G 0x18U
#define ACCEL_LPF_BEST 0x06U
#define ACCEL_LPF_MEDIUM 0x04U
#define ACCEL_LPF_FAIR 0x02U
#define ACCEL_LPF_NO 0x00U

typedef struct magnetoraw
{
    int16_t x;
    int16_t y;
    int16_t z;
}magnetoraw;
typedef struct magnetoadj
{
    int16_t x;
    int16_t y;
    int16_t z;
}magnetoadj;
typedef struct magtesla
{
    float x;
    float y;
    float z;
}magtesla;
typedef struct gyroraw
{
    int16_t x;
    int16_t y;
    int16_t z;
}gyroraw;
typedef struct gyrodps{
	float x;
	float y;
	float z;
}gyrodps;
typedef struct gyroavg{
	int16_t x;
	int16_t y;
	int16_t z;
}gyroavg;
typedef struct gyroangle{
	float x;
	float y;
	float z;
}gyroangle;
typedef struct accelraw
{
    int16_t x;
    int16_t y;
    int16_t z;
}accelraw;
typedef struct accelg{
	float x;
	float y;
	float z;
}accelg;
typedef struct accelavg{
	int16_t x;
	int16_t y;
	int16_t z;
}accelavg;
void gyroinit(UART_HandleTypeDef *huart,I2C_HandleTypeDef *hi2c,uint8_t LPF_value,uint8_t DPS);
void get_gyroraw(I2C_HandleTypeDef *hi2c,gyroraw *self);
void to_dps(I2C_HandleTypeDef *hi2c,gyroraw *self,gyrodps *dpself,float sample_time);
void get_status(I2C_HandleTypeDef *hi2c,uint8_t *status);
void mpu_write(I2C_HandleTypeDef *hi2c,uint8_t address,uint8_t *buffer,uint8_t size);
void mpu_Read(I2C_HandleTypeDef *hi2c,uint8_t address,uint8_t *buffer,uint8_t size);
void gyro_avg(I2C_HandleTypeDef *hi2c,gyroavg *avgself);
void mpu_print(UART_HandleTypeDef *huart,I2C_HandleTypeDef *hi2c,uint8_t address);
void accelinit(UART_HandleTypeDef *huart,I2C_HandleTypeDef *hi2c,uint8_t LPF_value,uint8_t G_value);
void get_accelraw(I2C_HandleTypeDef *hi2c,accelraw *self);
void accel_avg(I2C_HandleTypeDef *hi2c,accelavg *avgself);
void to_g(accelraw *self,accelg *dpself);
void get_magraw(I2C_HandleTypeDef *hi2c,magnetoraw *self);
void magnetoinit(UART_HandleTypeDef *huart,I2C_HandleTypeDef *hi2c);
void get_magadj(I2C_HandleTypeDef *hi2c,magnetoadj *self);
void to_utesla(magnetoraw *self,magtesla *gself);
#endif
