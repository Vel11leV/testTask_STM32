#include "sensorsI2C.h"


int8_t sensorsI2C_BMP280Initialization( I2C_HandleTypeDef *hi2c1,UART_HandleTypeDef *huart, uint8_t *Data, uint16_t size  )
{
    if(hi2c1==NULL | huart==NULL | Data==NULL) return -1;
    
    bmp280_init_default_params(&(bmp280.params));
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = hi2c1;

	while (!bmp280_init(&bmp280, &(bmp280.params))) {
		size = sprintf((char *)Data, "BMP280 initialization failed\n");
		//HAL_UART_Transmit(huart, Data, size, 1000);
		HAL_Delay(2000);
	}
	bool bme280p = bmp280.id == BME280_CHIP_ID;
	size = sprintf((char *)Data, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
	//HAL_UART_Transmit(huart, Data, size, 1000);
    
    return 1; 
    
}


int8_t sensorsI2C_BMP280GetData( I2C_HandleTypeDef *hi2c1,UART_HandleTypeDef *huart, uint8_t *Data, uint16_t size, float *temperature, float* pressure,float * humidity)
{
    if( hi2c1==NULL | huart==NULL | Data==NULL | temperature==NULL | pressure==NULL ) return -1;
    
    
     HAL_Delay(100);
		while (!bmp280_read_float(&bmp280, temperature, pressure, humidity)) {
			size = sprintf((char *)Data,
					"Temperature/pressure reading failed\n");
			//HAL_UART_Transmit(huart, Data, size, 1000);
			HAL_Delay(200);
		}

		size = sprintf((char *)Data,"Pressure: %.2f Pa, Temperature: %.2f C",
				*pressure, *temperature);
		//HAL_UART_Transmit(huart, Data, size, 1000);
		if (bmp280.id == BME280_CHIP_ID) {
			size = sprintf((char *)Data,", Humidity: %.2f\n", *humidity);
			//HAL_UART_Transmit(huart, Data, size, 1000);
		}

		else {
			size = sprintf((char *)Data, "\n");
			//HAL_UART_Transmit(huart, Data, size, 1000);
		}
		HAL_Delay(500); 
     return 1;  
   
}
