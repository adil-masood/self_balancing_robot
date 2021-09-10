#ifndef UART_PRINT_H
#define UART_PRINT_H
#include<stm32f4xx_hal.h>
#define WINDOWS 1
#define LINUX 2
void print_int(UART_HandleTypeDef *huart,void *any,int size);
void print_uint(UART_HandleTypeDef *huart,void *any,int size);
void lineend(UART_HandleTypeDef *huart,int OS_TYPE);
void print_str(UART_HandleTypeDef *huart,char *str);
void print_float(UART_HandleTypeDef *huart,float *any);
void print_double(UART_HandleTypeDef *huart,double *any);
void print_tick(UART_HandleTypeDef *huart);
void print_hex(UART_HandleTypeDef *huart,void *any,int size);
int scan_int(UART_HandleTypeDef *huart);
void scan_str(UART_HandleTypeDef *huart,char *str);
#endif
