#include "sensorsI2C.c"

uint8_t SensorBMP280Initialization(BMP280_HandleTypedef * bmp280, I2C_HandleTypeDef *hi2c1,UART_HandleTypeDef *huart, char *Data, uint16_t size  );
uint8_t SensorBMP280GetData(BMP280_HandleTypedef * bmp280, I2C_HandleTypeDef *hi2c1,UART_HandleTypeDef *huart, char *Data, uint16_t size, float *temperature, float* pressure,float * humidity);