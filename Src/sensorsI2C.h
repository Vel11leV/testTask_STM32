

#ifndef SENSORSI2C_H_
#define SENSORSI2C_H_

#include "stm32f4xx_hal.h" 
#include "BMP280/bmp280.h"

static BMP280_HandleTypedef bmp280;  
int8_t sensorsI2C_BMP280Initialization( I2C_HandleTypeDef *hi2c1,UART_HandleTypeDef *huart, uint8_t *Data, uint16_t size );
int8_t sensorsI2C_BMP280GetData( I2C_HandleTypeDef *hi2c1,UART_HandleTypeDef *huart, uint8_t *Data, uint16_t size, float *temperature, float* pressure,float * humidity);

#endif