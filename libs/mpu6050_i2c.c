#include "mpu6050_i2c.h"
//#include "stm32f3xx_hal.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "uart_print.h"
#include "math.h"
//#include "stm32f3xx_hal_i2c.h"
const uint8_t mpu9250 = (0x68<<1);
const uint8_t AK8963 = (0x0C<<1);
// 1st pointer to register at address 0x19
  // 2nd addr=0x19 <SMPLRT_DIV registr> || "SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV)"
  // 3rd 00000110 addr=0x1A <CONFIG registr> || "DLPF <Fs=1KHz, mode=6>, fifo mode replace"
  // 4th 1000dps addr=0x1B <GYRO CONFIG registr> || "gyro full scale +-250dps, enable DLPF by fchoice_b = 2b'00"

void gyroinit(UART_HandleTypeDef *huart,I2C_HandleTypeDef *hi2c,uint8_t LPF_value,uint8_t DPS){
	uint8_t SMPL_DIV = 0;
	//smpl_address,smpl_div,conf,gyro_conf
	uint8_t init[4] = {0x19,SMPL_DIV,LPF_value,DPS};
	//int pin configs
	//uint8_t buff2[3] = {0x37,0x10,0x01};
	
	if(HAL_I2C_Master_Transmit(hi2c,mpu9250,init,4,500) != HAL_OK){
		HAL_UART_Transmit(huart,(uint8_t *)"i2c_error: gyro_init() (1)",strlen("i2c_error: gyro_init() (1)"),HAL_MAX_DELAY);
	}
}
void accelinit(UART_HandleTypeDef *huart,I2C_HandleTypeDef *hi2c,uint8_t LPF_value,uint8_t G_value){
	uint8_t init[3] = {0x1C,G_value,LPF_value};
	//int pin configs
	//uint8_t buff2[3] = {0x37,0x10,0x01};
	if(HAL_I2C_Master_Transmit(hi2c,mpu9250,init,3,500) != HAL_OK){
		HAL_UART_Transmit(huart,(uint8_t *)"i2c_error: accel_init() (1)",strlen("i2c_error: accel_init() (1)"),HAL_MAX_DELAY);
	}
}
void magnetoinit(UART_HandleTypeDef *huart,I2C_HandleTypeDef *hi2c){
	uint8_t buff2[2] = {0x37,0x02};
	uint8_t i2c_master_disable[2] = {0x6A,0x00};
	uint8_t mag_cnt[2] = {0x0A,0x11};

	if(HAL_I2C_Master_Transmit(hi2c,mpu9250,i2c_master_disable,2,200) != HAL_OK){
		HAL_UART_Transmit(huart,(uint8_t *)"i2c_error: gyro_init() (3)",strlen("i2c_error: gyro_init() (3)"),HAL_MAX_DELAY);
	}
	if(HAL_I2C_Master_Transmit(hi2c,mpu9250,buff2,2,200) != HAL_OK){
		HAL_UART_Transmit(huart,(uint8_t *)"i2c_error: gyro_init() (2)",strlen("i2c_error: gyro_init() (2)"),HAL_MAX_DELAY);
	}
	mag_cnt[1] = 0x00;
	if(HAL_I2C_Master_Transmit(hi2c,AK8963,mag_cnt,2,200) != HAL_OK){
		HAL_UART_Transmit(huart,(uint8_t *)"i2c_error: gyro_init() (4)",strlen("i2c_error: gyro_init() (4)"),HAL_MAX_DELAY);
	}
	mag_cnt[1] = 0x16;
	if(HAL_I2C_Master_Transmit(hi2c,AK8963,mag_cnt,2,200) != HAL_OK){
		HAL_UART_Transmit(huart,(uint8_t *)"i2c_error: gyro_init() (4)",strlen("i2c_error: gyro_init() (4)"),HAL_MAX_DELAY);
	}
	HAL_Delay(200);
}
void get_gyroraw(I2C_HandleTypeDef *hi2c,gyroraw *self){
	uint8_t gyro_add = 0x43;
	uint8_t gyroval[6];
	HAL_I2C_Master_Transmit(hi2c,mpu9250,&gyro_add,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c,mpu9250,gyroval,6,HAL_MAX_DELAY);
	self->x = ((int16_t)((gyroval[0]<<8) | gyroval[1]));
	self->y = ((int16_t)((gyroval[2]<<8) | gyroval[3]));
	self->z = ((int16_t)((gyroval[4]<<8) | gyroval[5]));
}

void to_dps(I2C_HandleTypeDef *hi2c,gyroraw *self,gyrodps *dpself,float sample_time){
	dpself->x= ((float)self->x)/100;
	dpself->x= ((float)((int)dpself->x))*100;
	dpself->x= (((1000)/32768.)*(dpself->x)*(sample_time)*180/3.14159);
	dpself->y= ((float)self->y)/100;
	dpself->y= ((float)((int)dpself->y))*100;
	dpself->y= (((1000)/32768.)*(dpself->y)*(sample_time)*180/3.14159);
	dpself->z= ((float)self->z)/100;
	dpself->z= ((float)((int)dpself->z))*100;
	dpself->z= (((1000)/32768.)*(dpself->z)*(sample_time)*180/3.14159);
}
void gyro_avg(I2C_HandleTypeDef *hi2c,gyroavg *avgself){
	gyroraw self = {0,0,0};
	int x=0,y=0,z=0;
	int i=0;
	while(i<20){
		get_gyroraw(hi2c,&self);
		x+=(int)self.x;
		y+=(int)self.y;
		z+=(int)self.z;
		HAL_Delay(200);
		i++;
	}
	avgself->x = (int16_t)(x/i);
	avgself->y = (int16_t)(y/i);
	avgself->z = (int16_t)(z/i);
}

void get_status(I2C_HandleTypeDef *hi2c,uint8_t *status){
	uint8_t status_add = 0x3A;
	HAL_I2C_Master_Transmit(hi2c,mpu9250,&status_add,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c,mpu9250,status,1,HAL_MAX_DELAY);
}
// ACCEL

void get_accelraw(I2C_HandleTypeDef *hi2c,accelraw *self){
	uint8_t accel_add = 0x3B;
	uint8_t accelval[6];
	HAL_I2C_Master_Transmit(hi2c,mpu9250,&accel_add,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c,mpu9250,accelval,6,HAL_MAX_DELAY);
	self->x = ((int16_t)((accelval[0]<<8) | accelval[1]));
	self->y = ((int16_t)((accelval[2]<<8) | accelval[3]));
	self->z = ((int16_t)((accelval[4]<<8) | accelval[5]));
}
void to_g(accelraw *self,accelg *gself){
	gself->x= ((float)self->x)/100;
	gself->x= ((float)((int)gself->x))*100;
	gself->x=(gself->x/32768)*(4);
	gself->y= ((float)self->y)/100;
	gself->y= ((float)((int)gself->y))*100;
	gself->y=(gself->y/32768)*(4);
	gself->z= ((float)self->z)/100;
	gself->z= ((float)((int)gself->z))*100;
	gself->z=(gself->z/32768)*(4);
}

void accel_avg(I2C_HandleTypeDef *hi2c,accelavg *avgself){
	accelraw self = {0,0,0};
	int x=0,y=0,z=0;
	int i=0;
	while(i<20){
		get_accelraw(hi2c,&self);
		x+=(int)self.x;
		y+=(int)self.y;
		z+=(int)self.z;
		HAL_Delay(200);
		i++;
	}
	avgself->x = (int16_t)(x/i);
	avgself->y = (int16_t)(y/i);
	avgself->z = (int16_t)(z/i);
}
void to_utesla(magnetoraw *self,magtesla *mself){
	mself->x=(((float)self->x)/32768)*(4800);
	mself->y=(((float)self->y)/32768)*(4800);
	mself->z=(((float)self->z)/32768)*(4800);
}
void get_magraw(I2C_HandleTypeDef *hi2c,magnetoraw *self){
	uint8_t magneto_add = 0x03;
	static uint8_t magnetoval[7] = {0};
	uint8_t ST1=0;
	uint8_t ADD_ST1=0x02;
	uint8_t test = 0x0A;
	static uint8_t test_read = 0x66;
	HAL_I2C_Master_Transmit(hi2c,AK8963,&test,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c,AK8963,&test_read,1,HAL_MAX_DELAY);

	HAL_I2C_Master_Transmit(hi2c,AK8963,&ADD_ST1,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c,AK8963,&ST1,1,HAL_MAX_DELAY);
	if((ST1&0x01)==0x01){
		HAL_I2C_Master_Transmit(hi2c,AK8963,&magneto_add,1,HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(hi2c,AK8963,magnetoval,7,HAL_MAX_DELAY);
		self->x = ((int16_t)((magnetoval[1]<<8) | magnetoval[0]));
		self->y = ((int16_t)((magnetoval[3]<<8) | magnetoval[2]));
		self->z = ((int16_t)((magnetoval[5]<<8) | magnetoval[4]));
	}
	
}
void get_magadj(I2C_HandleTypeDef *hi2c,magnetoadj *self){
	uint8_t mag_cnt[2] = {0x0A,0x01};
	uint8_t value[3];
	uint8_t add = 0x10;
	mag_cnt[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c,AK8963,mag_cnt,2,200);
	mag_cnt[1] = 0x0F;
	HAL_I2C_Master_Transmit(hi2c,AK8963,mag_cnt,2,200);
	HAL_I2C_Master_Transmit(hi2c,AK8963,&add,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c,AK8963,value,3,HAL_MAX_DELAY);
	self->x = (int16_t)((((value[0]-128)*0.5)/128)+1);
	self->y = (int16_t)((((value[1]-128)*0.5)/128)+1);
	self->z = (int16_t)((((value[2]-128)*0.5)/128)+1);
	mag_cnt[1] = 0x00;
	HAL_I2C_Master_Transmit(hi2c,AK8963,mag_cnt,2,200);
	mag_cnt[1] = 0x16;
	HAL_I2C_Master_Transmit(hi2c,AK8963,mag_cnt,2,200);
	HAL_Delay(200);
}
void mpu_write(I2C_HandleTypeDef *hi2c,uint8_t address,uint8_t *buffer,uint8_t size){
	HAL_I2C_Master_Transmit(hi2c,mpu9250,&address,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c,mpu9250,buffer,size,HAL_MAX_DELAY);
}
void mpu_Read(I2C_HandleTypeDef *hi2c,uint8_t address,uint8_t *buffer,uint8_t size){
	HAL_I2C_Master_Transmit(hi2c,mpu9250,&address,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c,mpu9250,buffer,size,HAL_MAX_DELAY);
}
void mpu_print(UART_HandleTypeDef *huart,I2C_HandleTypeDef *hi2c,uint8_t address){
	uint8_t value;
	HAL_I2C_Master_Transmit(hi2c,mpu9250,&address,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c,mpu9250,&value,sizeof(value),HAL_MAX_DELAY);
	print_hex(huart,&value,sizeof(value));
}
