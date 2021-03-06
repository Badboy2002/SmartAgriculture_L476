/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rtthread.h>
#include "sgp30.h"
#include <string.h> 

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static struct rt_thread led_thread;
static char led_thread_stack[512];

static struct rt_thread sensor_thread;
static char sensor_thread_stack[1024];

static struct rt_thread L610_thread;
static char L610_thread_stack[1024];

static struct rt_thread watchdog_thread;
static char watchdog_thread_stack[512];

static struct rt_thread SGP30_thread;
static char SGP30_thread_stack[512];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t Lux = 1000;   
uint32_t cloudmessage = 0;
rt_uint8_t humidity = 50,temperature = 25;
rt_uint16_t ADC_Value = 1000;
double voltage;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern UART_HandleTypeDef huart1;  // ??????
extern UART_HandleTypeDef huart4;  // ????????
extern UART_HandleTypeDef huart5;  // ??????

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void delay_us(uint16_t us)   // ????????????us????????
{
    uint16_t differ=0xffff-us-5;
    /*??????????????????????????????????.
     ????0xfffE????????1us,??????????????
    ??????????????????????????????????0xffff??
    ????????????????????????????????????arr????
    ????????????(0xffff+1)us
    */
    HAL_TIM_Base_Start(&htim2);

    __HAL_TIM_SetCounter(&htim2,differ);

    while(differ<0xffff-5)
    {
        differ=__HAL_TIM_GetCounter(&htim2);
    }
    HAL_TIM_Base_Stop(&htim2);
}


void UART5_output(const char *str)   // ????????????????
{
	rt_size_t i = 0, size = 0;
  uint8_t a = 0xff;
  __HAL_UNLOCK(&huart5);
  size = rt_strlen(str);
  for (i = 0; i < size; i++)
  {
		HAL_UART_Transmit(&huart5, (uint8_t*)(str + i), 1, 1);
  }
	HAL_UART_Transmit(&huart5, (uint8_t*)&a, 1, 1);
	HAL_UART_Transmit(&huart5, (uint8_t*)&a, 1, 1);
	HAL_UART_Transmit(&huart5, (uint8_t*)&a, 1, 1);
}


void USART1_output(const char *str)  // ??????????
{
	rt_size_t i = 0, size = 0;
  __HAL_UNLOCK(&huart1);
  size = rt_strlen(str);
  for (i = 0; i < size; i++)
  {
		HAL_UART_Transmit(&huart1, (uint8_t*)(str + i), 1, 1);
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // ????????????
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	for(int i=0;i<10;i++)
	{
		delay_us(50*1000);   
		delay_us(50*1000);
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
}

rt_uint8_t DHT11_Reset()    // ??????DHT11 ??????????????????????
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       // ???????????? ?????? 
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);
	delay_us(30*1000);                                              // ????????30ms ??DHT11??????
	HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);
	delay_us(30);
	
	GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;       // ???????????? ????DHT11??????
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
	
	rt_uint8_t cnt=0;
	while(HAL_GPIO_ReadPin(DHT11_GPIO_Port,DHT11_Pin)&&cnt<100) // ????DHT11???????? 
	{
		cnt++;
		delay_us(1);
	}
	if (cnt>=100)
			return 1; // DHT11??????
	
	cnt=0;
	while(!HAL_GPIO_ReadPin(DHT11_GPIO_Port,DHT11_Pin)&&cnt<100) // ????DHT11???????? 
	{	
		cnt++;
		delay_us(1);
	}
	if (cnt>=100)
		return 1; // DHT11??????
	
	return 0; // DHT11?????????? ????????????
}

rt_int8_t DHT11_READ_BIT()  // ????????Bit
{
	while(HAL_GPIO_ReadPin(DHT11_GPIO_Port,DHT11_Pin))
		;  // ??????????????????
	while(!HAL_GPIO_ReadPin(DHT11_GPIO_Port,DHT11_Pin))
		;  // ??????????
	
	delay_us(35);            // ??????????27-28us????0 ????70us????1 ????????????35us????????????????????
	if(HAL_GPIO_ReadPin(DHT11_GPIO_Port,DHT11_Pin))
		return 1;
	else
		return 0;
}

rt_uint8_t DHT11_READ_BYTE() // ????????Byte
{
	rt_uint8_t mask,data;
	data=0;
	for(mask=0x80;mask!=0;mask>>=1)
	{
		if(DHT11_READ_BIT())
			data |=mask;
		else
			data &=~mask;
	}
	return data;
}

rt_uint8_t DHT11_GET_DATA(rt_uint8_t *humidity, rt_uint8_t *temperature) // ??????????????
{
	rt_uint8_t i;
	rt_uint8_t buf[5];
	if(DHT11_Reset() == 0)
	{
		for (i=0; i<5; i++)
		{
			buf[i]=DHT11_READ_BYTE();
		}
		if( (buf[0]+buf[1]+buf[2]+buf[3]) == buf[4])
		{
			*humidity = buf[0];
			*temperature = buf[2];
			rt_kprintf("DTH11_DATA read successfully!\n");
//			rt_kprintf("temperature:%u humidity:%u \n",buf[2], buf[0]);
			return 0;
		}
		else
		{
			rt_kprintf("check failed!\n");
			rt_kprintf("temperature:%u humidity:%u \n",buf[2], buf[0]);
			return 1;
		}
	}
	else
	{
		rt_kprintf("DHT11 failed to init!\n");
		return 1;
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void sensor_thread_entry(void* parameter)  // ?????????? ????????????????
{
	rt_kprintf("Sensor detection start\n");
	while(1)
{
	char temperature_string[20] = {0};      //  ??????????????????????
	char humidity_string[20] = {0};
	char light_string[20] = {0};
	DHT11_GET_DATA(&humidity, &temperature); // ??????????
  rt_kprintf("---Temperature:%u 'C Humidity:%u%---\n", temperature, humidity/2);
	HAL_ADC_Start(&hadc1);     //????????????ADC????
	HAL_ADC_PollForConversion(&hadc1, 100);   //??????????????100??????????????????????ms
 
 if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
 {
  ADC_Value = HAL_ADC_GetValue(&hadc1);      //????AD??
  rt_kprintf("ADC1 Reading:%u\n", ADC_Value);
	voltage = ADC_Value*3.3/4096;   // ??????????
	Lux = (3300-(unsigned int)(voltage*1000))/2;  // ????????????
  rt_kprintf("---Light sensor:%u Lux---\n", Lux);
 }

	sprintf(temperature_string,"t0.txt=\"%u'C\"", temperature);   //  ??????????????????????
	sprintf(humidity_string,"t1.txt=\"%u%%\"", humidity/2);
	sprintf(light_string,"t2.txt=\"%ulx\"", Lux);
 	UART5_output(temperature_string);   // ??????????
	UART5_output(humidity_string);
	UART5_output(light_string);
	
	rt_thread_mdelay(3000);  // ????????3000ms
	}
}
MSH_CMD_EXPORT(sensor_thread_entry, Sensors start to detect);

static void L610_thread_entry(void* parameter)  // ??????????
{
	char connect[] = "AT+CLOUDAUTH=\"a1iKWF07Pbb\",\"ADP_L610\",\"7727d7aad67ccb07a259e5b27ba4b6e1\",\"iot-as-mqtt.cn-shanghai.aliyuncs.com\"\r\n";
	char keep_alive[] = "AT+CLOUDCONN=80,0,4\r\n";
	char success[] = "AT+CLOUDPUB=\"/a1iKWF07Pbb/ADP_L610/user/get\",1,\"MCU connection sucess\"\r\n";
	char Data_string[100] = {0};      //  ????L610??aT??????????
	
	
	rt_thread_mdelay(4000);  // ????L610????
	USART1_output(connect);  // ??????????
	rt_thread_mdelay(1000);
	USART1_output(keep_alive); // ????
	rt_thread_mdelay(1000);
	USART1_output(success);
	rt_kprintf("AT+CLOUDPUB=\"/a1iKWF07Pbb/ADP_L610/user/get\",1,\"MCU connection sucess\"\n");
	rt_thread_mdelay(1000);
	while(1)
	{
		sprintf(Data_string,"AT+CLOUDPUB=\"/a1iKWF07Pbb/ADP_L610/user/get\",1,\"Temperature:%u Humidity:%u Lux:%u CO2:%u\"\r\n", temperature, humidity, Lux, sgp30_data.co2);
		USART1_output(Data_string);  // ????????????
		rt_kprintf("Send message to Ali_Cloud");
		rt_thread_mdelay(30000);
	}
}

static void watchdog_thread_entry(void* parameter) // ??????????
{
	while(1)
	{
		if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)  // ????????????????????
    {
       cloudmessage = huart1.Instance->RDR & 0xff;
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			 rt_thread_mdelay(2000);
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    }
		else
				__HAL_UART_CLEAR_OREFLAG(&huart1);
		
		if(Lux<1000)  // ????????????????
		{
			for(int i=0;i<10;i++)   
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
				rt_thread_mdelay(100);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				rt_thread_mdelay(100);
			}
		}
		rt_thread_mdelay(500);
	}
}

static void led_thread_entry(void* parameter)   // LED????????
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       // ??????LED??GPIO 
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	rt_kprintf("LED blink start\n");
	while(1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		rt_thread_mdelay(1000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		rt_thread_mdelay(1000);
	}
}

static void SGP30_thread_entry(void* parameter)   // SGP30????
{
	MX_I2C2_Init();
	rt_kprintf("SGP30 start!\n");
	if (-1 == sgp30_init())
			{
					printf("sgp30 init fail\r\n");
					rt_kprintf("SGP30 init failed!\n");
					UART5_output("SGP30 init failed!");
			}
	rt_kprintf("SGP30 init success(it takes about 15s to get the data since start)\n");
	char CO2_string[20] = {0};
	 while(1) // SGP30????????
	{
		if( -1 == sgp30_read()) 
		{
			rt_kprintf("sgp30 read fail\n");
			break;
		}
		else
		{
			if(sgp30_data.co2 == 400 )
		  {
				rt_kprintf("SGP30 is reading data...\n");
			}
			else
			{
				rt_kprintf("---SGP30 read success, CO2:%4d ppm, TVOC:%4d ppd---\n", sgp30_data.co2, sgp30_data.tvoc);
				sprintf(CO2_string,"t3.txt=\"CO2:%uppm\"", sgp30_data.co2);
				UART5_output(CO2_string);
				break;
			}
		}
		rt_thread_mdelay(3000);
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);  // DHT11??????????
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);  // ??????????????
	HAL_ADCEx_Calibration_Start(&hadc1, 100); // ADC??????
	UART5_output("t0.txt=\"26'C\"");  // ????????????????
	UART5_output("t1.txt=\"50%\"");
	UART5_output("t2.txt=\"1000lx\"");
	UART5_output("t3.txt=\"CO2:400ppm\"");
	rt_thread_mdelay(1000);
	
	rt_err_t rst;  // ????????????
	rst = rt_thread_init(&led_thread,
						"ledshine",
						led_thread_entry,
						RT_NULL,
						&led_thread_stack[0],
						sizeof(led_thread_stack),
						RT_THREAD_PRIORITY_MAX-2,
						20);
	if(rst == RT_EOK)
	{
		rt_thread_startup(&led_thread);  // ????led????
	}
	
	rst = rt_thread_init(&sensor_thread,
						"sensors detect",
						sensor_thread_entry,
						RT_NULL,
						&sensor_thread_stack[0],
						sizeof(sensor_thread_stack),
						RT_THREAD_PRIORITY_MAX-2,
						20);
	if(rst == RT_EOK)
	{
		rt_thread_startup(&sensor_thread); // ??????????????
	}
	
	rst = rt_thread_init(&SGP30_thread,
						"SGP30 thread",
						SGP30_thread_entry,
						RT_NULL,
						&SGP30_thread_stack[0],
						sizeof(SGP30_thread_stack),
						RT_THREAD_PRIORITY_MAX-2,
						10);
	if(rst == RT_EOK)
	{
		rt_thread_startup(&SGP30_thread); // ????SGP30????
	}

	
	rst = rt_thread_init(&L610_thread,
						"L610 thread",
						L610_thread_entry,
						RT_NULL,
						&L610_thread_stack[0],
						sizeof(L610_thread_stack),
						RT_THREAD_PRIORITY_MAX-2,
						5);
	if(rst == RT_EOK)
	{
		rt_thread_startup(&L610_thread); // ??????????
	}
	
	rst = rt_thread_init(&watchdog_thread,
						"watchdog thread",
						watchdog_thread_entry,
						RT_NULL,
						&watchdog_thread_stack[0],
						sizeof(watchdog_thread_stack),
						RT_THREAD_PRIORITY_MAX-2,
						5);
	if(rst == RT_EOK)
	{
		rt_thread_startup(&watchdog_thread);   // ??????????
	}
	
	rt_kprintf("RT-Thread start successfully!\n");  
	return 0;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
	
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
