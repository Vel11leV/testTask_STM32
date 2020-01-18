/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include "BMP280/bmp280.h"
#include "MQTTPacket/src/MQTTPacket.h"



#define STRLEN(STR)  sizeof(STR)/sizeof(uint8_t)
#define ESPBUF       1000
#define SIZEUART1RES 1
#define SIZEUART2RES 1
#define ATSENDSIZE   48

#define SSID       "summer"
#define PASS       "Milan2019"
#define SERVERURL  "quickstart.messaging.internetofthings.ibmcloud.com"
#define TCPPORT    "1883"
#define MQTTTOPIC  "iot-2/evt/status/fmt/json"
#define MQTTID     "d:quickstart:arduino:6001941ce5bf"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osMutexId dataMutex01Handle;
osMutexId uart2Mutex02Handle;

////////////////////bmp280////////////////////////
BMP280_HandleTypedef bmp280;
/////////////////////end_bmp280///////////////////
float pressure, temperature, humidity;
uint16_t size;
uint8_t Data[256];
uint8_t AT_SSID_CON_[100];
uint8_t AT_CONNECT_TO_SERVER_[100];



/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t espReply[ESPBUF];
uint16_t espCount=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void sensorThread1(void const * argument);
void serverThread2(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


uint8_t * uint8_tToString(uint8_t value, uint8_t *buffer) //buff =3!!!
{
  
    int i =2;
   do
   {
      buffer[i] = value % 10 + '0';
      value /= 10;
       i--;
   }
   while (value != 0);
   return buffer;  
}
uint8_t * uint32_tToString(uint32_t value, uint8_t *buffer) //buff =11!!!
{
  
    int i = 10;
   do
   {
      buffer[i] = value % 10 + '0';
      value /= 10;
       i--;
   }
   while (value != 0);
   return buffer;  
}

void strSendUART1 (uint8_t *str, uint16_t theStrlen )
{
   
   if(*str!=NULL)
   HAL_UART_Transmit(&huart1,str, theStrlen,10);
}
void strSendUART2 (uint8_t *str, uint16_t theStrlen )
{
   
  //osMutexWait(uart2Mutex02Handle, osWaitForever);  
   if(*str!=NULL)
   HAL_UART_Transmit(&huart2,str,theStrlen,10);
  //osMutexRelease(uart2Mutex02Handle);
}

void espReplyNewData(uint8_t chr)
{ 
  //osMutexWait(dataMutex01Handle, osWaitForever);
    
    if (espCount<ESPBUF)
    {
        espReply[espCount] = chr;
        espCount++;
    }
    else
    {
        espCount=0;
        espReply[espCount] = chr;
        espCount++;
    }
    
  //osMutexRelease(dataMutex01Handle);
}

void espReplyToUART2()
{   
   
  //osMutexWait(dataMutex01Handle, osWaitForever);
    
    int dataSizeInBuffer = 0;
    for(int i=0;i<ESPBUF;i++)
    {
        if(espReply[i]!=NULL)
            dataSizeInBuffer++;
        else break;
    }
    strSendUART2(espReply,dataSizeInBuffer); 
    
  //osMutexRelease(dataMutex01Handle);
}

void espReplyClear()
{
   //osMutexWait(dataMutex01Handle, osWaitForever);
    
    for(int i=0;i<ESPBUF;i++)
    {
        espReply[i]=NULL;
    }
    espCount = 0;
    
   //osMutexRelease(dataMutex01Handle);
}

uint8_t espReplyIsSubstring(uint8_t *subStr, uint16_t theSubstrlen)
{
  //osMutexWait(dataMutex01Handle, osWaitForever);
    
    for(int i=0;i<(ESPBUF-theSubstrlen+1);i++)
    {
        for(int j=0;j<theSubstrlen;j++)
        {
            if (espReply[i+j]!=subStr[j])
                break;
            if (j==theSubstrlen-1)
            {
                
                uint8_t status []="\nStatus: ";
                uint8_t nLine []="\n";
                /*
                HAL_UART_Transmit(&huart2,status,9,10);
                HAL_UART_Transmit(&huart2,subStr, theSubstrlen,10);
                HAL_UART_Transmit(&huart2,nLine,1,10);
                */
                strSendUART2(status,STRLEN(status));
                strSendUART2(subStr,theSubstrlen);
                strSendUART2(nLine,STRLEN(nLine));
                
                return 1;
                
            }
        }
        if (i==ESPBUF-theSubstrlen)
                return 0;
    }    
    
  //osMutexRelease(dataMutex01Handle);    
}

uint8_t sendRequest(uint8_t *request, uint8_t theStrlen)
{
    if (theStrlen>ATSENDSIZE){ uint8_t error[] = "ERROR: Message was not sent, it is too long ...\n";  strSendUART2(error,STRLEN(error)); return 0;}
    if (theStrlen==0){ uint8_t error[] = "ERROR: No data to send ...\n";  strSendUART2(error,STRLEN(error)); return 0;}
   
        
    uint8_t AT_CIPSEND[] =  "AT+CIPSEND="; //AT+CIPSEND=48\r\n;  ATSENDSIZE max via one transaction
    uint8_t AT_END [] = "\r\n";
    uint8_t rep1[] ="OK";
    uint8_t _WAIT[] ="Waiting for AT reply ...\n";  
    
    uint8_t numb [3] ={' ',' ',' '};     
    uint8_tToString(theStrlen,numb); 
    int sizeOfNumb=3;
    for(int i=0;i<3;i++)
    {
     if( numb[i]==' ') --sizeOfNumb;
     else break;
    }
    
     uint8_t length = STRLEN(AT_CIPSEND)-1+sizeOfNumb+2;   
     uint8_t *AT_CIPSEND_NUMB_ENDL = (uint8_t *)malloc(length);
    
     uint8_t *DATA = (uint8_t *)malloc(theStrlen);
     for(int i=0;i<theStrlen;i++)
        {DATA[i]=*(request++);}
    
       int i=0;     
                for(int j=0; j<sizeof(AT_CIPSEND)-1; j++)
                {
                    AT_CIPSEND_NUMB_ENDL[i]=AT_CIPSEND[j];
                    i++;
                }                              
                for(int j=STRLEN(numb)-sizeOfNumb; j<STRLEN(numb); j++)
                {
                    AT_CIPSEND_NUMB_ENDL[i]=numb[j];
                    i++;
                }
                for(int j=0;j<STRLEN(AT_END)-1;j++)
                {
                    AT_CIPSEND_NUMB_ENDL[i]=AT_END[j];
                    i++;
                }
                
   espReplyClear(); 
               
   
   strSendUART2(AT_CIPSEND_NUMB_ENDL,length); 
   strSendUART2(DATA,theStrlen);                 
               
                
    strSendUART1(AT_CIPSEND_NUMB_ENDL,length);  
        while(espReplyIsSubstring(rep1,2)==0){ 
                //strSendUART2(_WAIT,STRLEN(_WAIT));
      
                }
   espReplyToUART2();     
   espReplyClear(); 
           
   strSendUART1(DATA,theStrlen); 
        while(espReplyIsSubstring(rep1,2)==0){ 
               // strSendUART2(_WAIT,STRLEN(_WAIT));     
                }
 
   espReplyToUART2();     
   espReplyClear();  
                
       free (AT_CIPSEND_NUMB_ENDL); 
       free (DATA);   
       
return 1;
}




uint8_t sendRequestAnySize(uint8_t *request, uint16_t theStrlen)
{
    uint16_t remainder=0;
    uint16_t whole=0;
    
    remainder = theStrlen%ATSENDSIZE;
    whole = (uint16_t)(theStrlen/ATSENDSIZE);
    
    int i=0;
    for(;i<whole; i++)
    {
        sendRequest(request+i*ATSENDSIZE, ATSENDSIZE);
    }
    
    if (remainder>0&&whole>0)
    {
        i++;
        sendRequest(request+i*ATSENDSIZE, remainder);
    }
    else if(remainder>0&&whole==0)
    { sendRequest(request, remainder);}
      
    return 1;
}


uint8_t SensorBMP280Initialization(BMP280_HandleTypedef * bmp280, I2C_HandleTypeDef *hi2c1,UART_HandleTypeDef *huart, uint8_t *Data, uint16_t size  )
{
    bmp280_init_default_params(&(bmp280->params));
	bmp280->addr = BMP280_I2C_ADDRESS_0;
	bmp280->i2c = hi2c1;

	while (!bmp280_init(bmp280, &(bmp280->params))) {
		size = sprintf((char *)Data, "BMP280 initialization failed\n");
		//HAL_UART_Transmit(huart, Data, size, 1000);
		HAL_Delay(2000);
	}
	bool bme280p = bmp280->id == BME280_CHIP_ID;
	size = sprintf((char *)Data, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
	//HAL_UART_Transmit(huart, Data, size, 1000);
    
    return 1; 
    
}


uint8_t SensorBMP280GetData(BMP280_HandleTypedef * bmp280, I2C_HandleTypeDef *hi2c1,UART_HandleTypeDef *huart, uint8_t *Data, uint16_t size, float *temperature, float* pressure,float * humidity)
{
     HAL_Delay(100);
		while (!bmp280_read_float(bmp280, temperature, pressure, humidity)) {
			size = sprintf((char *)Data,
					"Temperature/pressure reading failed\n");
			//HAL_UART_Transmit(huart, Data, size, 1000);
			HAL_Delay(200);
		}

		size = sprintf((char *)Data,"Pressure: %.2f Pa, Temperature: %.2f C",
				*pressure, *temperature);
		//HAL_UART_Transmit(huart, Data, size, 1000);
		if (bmp280->id == BME280_CHIP_ID) {
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






/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t myDataRX2  [SIZEUART2RES];
uint8_t myDataRX1  [SIZEUART1RES];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart2,myDataRX2, SIZEUART2RES);
  HAL_UART_Receive_DMA(&huart1,myDataRX1, SIZEUART1RES);
  
  
  
 
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of dataMutex01 */
  osMutexDef(dataMutex01);
  dataMutex01Handle = osMutexCreate(osMutex(dataMutex01));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, sensorThread1, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, serverThread2, osPriorityIdle, 0, 528);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  espReplyClear();
  while (1)
  {

  /* USER CODE END WHILE */              
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
    
    if(huart==&huart1)
    {
        
        
        //strSendUART2(myDataRX1,STRLEN(myDataRX1));
        osMutexWait(dataMutex01Handle, osWaitForever);
           espReplyNewData(*myDataRX1); 
        osMutexRelease(dataMutex01Handle);
        
    }
    if(huart==&huart2)
    {
        //strSendUART2(myDataRX2,STRLEN(myDataRX2));
        
        uint8_t sub[] ="192";     
        
        if(*myDataRX2=='o')
            espReplyToUART2();   
        else if(*myDataRX2=='e')
            espReplyClear();
        else if(*myDataRX2=='s')
            {
                espReplyIsSubstring(sub,3);  
            }
        /*    
        else
        {
          osMutexWait(dataMutex01Handle, osWaitForever);
           espReplyNewData(*myDataRX2); 
          osMutexRelease(dataMutex01Handle);
        }
        */
       
    }
  
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
}

/* USER CODE END 4 */

/* sensorThread1 function */
void sensorThread1(void const * argument)
{
////////////////////bmp280/////////////////////
SensorBMP280Initialization( &bmp280, &hi2c1, &huart2, Data, size  ); 
//////////////////end_bmp280///////////////////////    
    
  for(;;)
  {
///////////////////bmp280//////////////////////    
osMutexWait(uart2Mutex02Handle, osWaitForever);       
    SensorBMP280GetData(&bmp280, &hi2c1,&huart2, Data, size, &temperature, &pressure, &humidity);
osMutexRelease(dataMutex01Handle);
/////////////////end_bmp280////////////////////////  
      
    osDelay(100);
  }
  /* USER CODE END 5 */ 
}

/* serverThread2 function */
void serverThread2(void const * argument)
{

///////////////////MQTT/////////////////////
 
MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
uint16_t rc = 0;
uint8_t buf[200];
int buflen = sizeof(buf);    
MQTTString topicString = MQTTString_initializer;
uint8_t payload[100];
data.clientID.cstring = MQTTID;
data.keepAliveInterval = 20;
data.cleansession = 1;
topicString.cstring = MQTTTOPIC;
    
//////////////////endMQTT///////////////////
    
    
  /* USER CODE BEGIN serverThread2 */
  /* Infinite loop */
       

    uint8_t AT_RST[]    = "AT+RST\r\n";
    uint8_t AT_CIPMODE[]= "AT+CIPMODE=1\r\n";
    uint8_t AT_CIPSEND[]= "AT+CIPSEND\r\n";
    uint8_t AT_CIFSR[]  = "AT+CIFSR\r\n";
    uint8_t AT_CIPMODECLOSE[3]={'+','+','+'};
    uint8_t AT_CONNECT_TO_SERVER[]=  "AT+CIPSTART=\"TCP\",\"quickstart.messaging.internetofthings.ibmcloud.com\",1883\r\n"; 
    sprintf((char *)AT_SSID_CON_,"AT+CWJAP=\"%s\",\"%s\"\r\n",SSID,PASS);
   
    
    //strSendUART2(AT_SSID_CON_,STRLEN(AT_SSID_CON_));

    
//////////////////////////////////////////    
    ////////////ESP8266 status////////////
    uint8_t _WAIT_WIFI[]      ="connecting to WiFi...\n";
    uint8_t _WAIT_IP[]        ="getting IP...\n";
    uint8_t _WAIT_SERVERCON[] ="Connecting to the server...\n";  
    uint8_t _WIFI_connected[] ="WiFi connected.\n";
    uint8_t _WAIT_CIPMODE[]   ="CIPMODE...\n"; 
    
    ////////////ESP8266 reply////////////
    uint8_t rep_OK[] =     "OK";
    uint8_t rep_STAIP[] =  "STAIP";
    uint8_t rep_CONNECT[]= "CONNECT";
    uint8_t rep_CIPSEND[]= ">";
    uint8_t _WAIT_CIPSEND[] ="SIPMODE ON\n"; 
    uint8_t NEWLINE[] ="\n";
///////////////////////////////////////////    
    

       
/*---------------------ESP8266 setting---------------------*/   

espReplyClear(); 
strSendUART1(AT_CIPMODECLOSE,STRLEN(AT_CIPMODECLOSE));
HAL_Delay(2000);        

    //connectiong to WiFi AP
      espReplyClear();    
      strSendUART1(AT_SSID_CON_,STRLEN(AT_SSID_CON_)-1);
      while(espReplyIsSubstring(rep_OK,STRLEN(rep_OK)-1)==0){ 
            espReplyClear();
            strSendUART1(AT_CIFSR,STRLEN(AT_CIFSR)-1);
            strSendUART2( _WAIT_WIFI,STRLEN( _WAIT_WIFI));
            HAL_Delay(300);         
            }
           

    //GET IP        
    while(espReplyIsSubstring(rep_STAIP,STRLEN(rep_STAIP)-1)==0){ 
            espReplyToUART2();
            espReplyClear();
            strSendUART1(AT_CIFSR,STRLEN(AT_CIFSR));
            strSendUART2(_WAIT_IP,STRLEN(_WAIT_IP));
            HAL_Delay(300);         
            }
    espReplyToUART2();          
    espReplyClear(); 

    
   //set ESP8266 set in CIPMODE (uart<->WiFi)         
   strSendUART1(AT_CIPMODE,STRLEN(AT_CIPMODE)-1);  
   while(espReplyIsSubstring(rep_OK,STRLEN(rep_OK)-1)==0){ 
            strSendUART2(_WAIT_CIPMODE,STRLEN(_WAIT_CIPMODE));
            HAL_Delay(300);         
            }            
   espReplyToUART2();
   espReplyClear();  

/*---------------------endESP8266 setting ---------------------*/  


            


            
  for(;;)
  {
     /*---------Connect to the server and transmit the data---------*/


            
    //Connecting to the server           
    strSendUART1(AT_CONNECT_TO_SERVER,STRLEN(AT_CONNECT_TO_SERVER));
    while(espReplyIsSubstring(rep_CONNECT,STRLEN(rep_CONNECT)-1)==0){ 
            strSendUART2(_WAIT_SERVERCON,STRLEN(_WAIT_SERVERCON));
            HAL_Delay(100);         
            }
   espReplyToUART2();
   espReplyClear();  
            
   // CIPMODE (uart<->WiFi) gets ready to start
   strSendUART1(AT_CIPSEND,STRLEN(AT_CIPSEND));  
   while(espReplyIsSubstring(rep_CIPSEND,STRLEN(rep_CIPSEND)-1)==0){ 
            strSendUART2(_WAIT_CIPSEND,STRLEN(_WAIT_CIPSEND));
            HAL_Delay(100);         
            }            
   espReplyToUART2();
   espReplyClear(); //NEWLINE   
   strSendUART2(NEWLINE,STRLEN(NEWLINE));
            
////////////////
osMutexWait(uart2Mutex02Handle, osWaitForever);               
uint16_t payloadlen =sprintf((char *)payload,"{\n\"d\":{\n\"myName\":\"STM32\",\n\"temp (C)\": %.2f\n}\n}",temperature); // 
osMutexRelease(dataMutex01Handle);

            
uint16_t len = MQTTSerialize_connect(buf, buflen, &data); 
len += (uint16_t)MQTTSerialize_publish(buf + len, buflen - len, 0, 0, 0, 0, topicString, payload, payloadlen);
len += MQTTSerialize_disconnect(buf + len, buflen - len);

strSendUART2(NEWLINE,STRLEN(NEWLINE));            
strSendUART2(buf,len); 
strSendUART2(NEWLINE,STRLEN(NEWLINE));

            
   strSendUART1(buf,len); 
   HAL_Delay(500);             
/*            
   for(int i=0;i<10;i++)
            {
   HAL_Delay(100);              
   espReplyToUART2();  
   espReplyClear(); 
          }*/
////////////////

            
    //CIPMODE close        
   strSendUART1(AT_CIPMODECLOSE,STRLEN(AT_CIPMODECLOSE)); //
   //HAL_Delay(300);   
            
/*---------end Connect to the server and transmit the data---------*/  
      
      
      
   
    osDelay(200);    
  }
  /* USER CODE END serverThread2 */
}




/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
