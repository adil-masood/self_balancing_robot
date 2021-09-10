#include<stdio.h>
#include<string.h>
//#include "stm32f3xx_hal.h"
#include "stm32f4xx_hal.h"
#include "uart_print.h"
#define WINDOWS 1
#define LINUX 2
static uint8_t CRLF[2] = {0x0D,0x0A};
static uint8_t LF = 0x0A; 
void print_tick(UART_HandleTypeDef *huart){
	uint32_t tick = HAL_GetTick();
	print_uint(huart,&(tick),sizeof(uint32_t));
	lineend(huart,WINDOWS);
}
void print_int(UART_HandleTypeDef *huart,void *any,int size){
    int16_t x16;
		int8_t x8;
		int32_t x32;
		char x_str[12];
	switch(size){
		case 2:
			x16 = *(int16_t *)any;
			sprintf(x_str,"%d",x16);
			HAL_UART_Transmit(huart,(uint8_t *)x_str,strlen(x_str),HAL_MAX_DELAY);
			break;
		case 1:
			x8 = *(int8_t *)any;
			sprintf(x_str,"%d",x8);
			HAL_UART_Transmit(huart,(uint8_t *)x_str,strlen(x_str),HAL_MAX_DELAY);
			break;
		case 4:
			x32 = *(int32_t *)any;
			sprintf(x_str,"%d",x32);
			HAL_UART_Transmit(huart,(uint8_t *)x_str,strlen(x_str),HAL_MAX_DELAY);
			break;
		default:
			HAL_UART_Transmit(huart,(uint8_t *)"ERROR_print_int: NO match found!",strlen(x_str),HAL_MAX_DELAY);
			HAL_UART_Transmit(huart,(uint8_t *)x_str,strlen(x_str),HAL_MAX_DELAY);
	} 
}
void print_uint(UART_HandleTypeDef *huart,void *any,int size){
    uint16_t x16;
		uint8_t x8;
		uint32_t x32;
		char x_str[12];
	switch(size){
		case 2:
			x16 = *(uint16_t *)any;
			sprintf(x_str,"%d",x16);
			HAL_UART_Transmit(huart,(uint8_t *)x_str,strlen(x_str),HAL_MAX_DELAY);
			break;
		case 1:
			x8 = *(uint8_t *)any;
			sprintf(x_str,"%d",x8);
			HAL_UART_Transmit(huart,(uint8_t *)x_str,strlen(x_str),HAL_MAX_DELAY);
			break;
		case 4:
			x32 = *(uint32_t *)any;
			sprintf(x_str,"%d",x32);
			HAL_UART_Transmit(huart,(uint8_t *)x_str,strlen(x_str),HAL_MAX_DELAY);
			break;
		default:
			HAL_UART_Transmit(huart,(uint8_t *)"ERROR_print_int: NO match found!",strlen(x_str),HAL_MAX_DELAY);
			HAL_UART_Transmit(huart,(uint8_t *)x_str,strlen(x_str),HAL_MAX_DELAY);
	} 
}
void print_float(UART_HandleTypeDef *huart,float *any){
	float value = *any;
	char x_str[12];
	sprintf(x_str,"%f",value);
	HAL_UART_Transmit(huart,(uint8_t *)x_str,strlen(x_str),HAL_MAX_DELAY);
}
void print_double(UART_HandleTypeDef *huart,double *any){
	double value = *any;
	char x_str[20];
	sprintf(x_str,"%lf",value);
	HAL_UART_Transmit(huart,(uint8_t *)x_str,strlen(x_str),HAL_MAX_DELAY);
}
void print_hex(UART_HandleTypeDef *huart,void *any,int size){
	int16_t x16;
		int8_t x8;
		int32_t x32;
		char x_str[12];
	switch(size){
		case 2:
			x16 = *(int16_t *)any;
			sprintf(x_str,"0x%04X",x16);
			HAL_UART_Transmit(huart,(uint8_t *)x_str,strlen(x_str),HAL_MAX_DELAY);
			break;
		case 1:
			x8 = *(int8_t *)any;
			sprintf(x_str,"0x%04X",x8);
			HAL_UART_Transmit(huart,(uint8_t *)x_str,strlen(x_str),HAL_MAX_DELAY);
			break;
		case 4:
			x32 = *(int32_t *)any;
			sprintf(x_str,"0x%04X",x32);
			HAL_UART_Transmit(huart,(uint8_t *)x_str,strlen(x_str),HAL_MAX_DELAY);
			break;
		default:
			HAL_UART_Transmit(huart,(uint8_t *)"ERROR_print_int: NO match found!",strlen(x_str),HAL_MAX_DELAY);
			HAL_UART_Transmit(huart,(uint8_t *)x_str,strlen(x_str),HAL_MAX_DELAY);
	}
}
void print_str(UART_HandleTypeDef *huart,char *str){
	HAL_UART_Transmit(huart,(uint8_t *)str,strlen(str),HAL_MAX_DELAY);
}
void lineend(UART_HandleTypeDef *huart,int OS_TYPE){
	if(OS_TYPE == WINDOWS){
		HAL_UART_Transmit(huart,CRLF,2,HAL_MAX_DELAY);
	}
	else if(OS_TYPE == LINUX){
		HAL_UART_Transmit(huart,&LF,1,HAL_MAX_DELAY);
	}
}
int scan_int(UART_HandleTypeDef *huart){
	int x;
	int i = -1;
	char newstr[11] = "";
	while(strcmp(&newstr[i],"\r")!=0){
		i++;
		HAL_UART_Receive(huart,(uint8_t *)&newstr[i],sizeof(char),HAL_MAX_DELAY);
	}
	strcpy(&newstr[i],"\0");
	sscanf(newstr,"%d",&x);
	return x;
}
void scan_str(UART_HandleTypeDef *huart,char *str){
	int i = -1;
	char newstr[11] = "";
	while(strcmp(&newstr[i],"\r")!=0){
		i++;
		HAL_UART_Receive(huart,(uint8_t *)&newstr[i],sizeof(char),HAL_MAX_DELAY);
	}
	strcpy(&newstr[i],"\0");
	strcpy(str,newstr);
}

