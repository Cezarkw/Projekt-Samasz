/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd4bit.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PA0 *((volatile unsigned long*) 0x42210100)
#define PA1 *((volatile unsigned long*) 0x42210104)	
#define PA2 *((volatile unsigned long*) 0x42210108)
#define PA3 *((volatile unsigned long*) 0x4221010C)
	



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */



char RXBuf[10];
char TXBuf[10]={"SW_0"};
char Z;
int KOM=0;
int Lr=0;
int NN;
int size;
Lcd_PortType ports[] = {GPIOC, GPIOC, GPIOC, GPIOC};
Lcd_PinType pins[] = {GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0};
Lcd_HandleTypeDef lcd;
char NMEA_Buff[256];
int NMEA_Size =0;
int crcCheck();
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);


/* USER CODE BEGIN PFP */
void USB_DEVICE_MasterHardReset(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART3_UART_Init();
	USB_DEVICE_MasterHardReset();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	lcd = Lcd_create(ports, pins, GPIOC, GPIO_PIN_12, GPIOC, GPIO_PIN_10, LCD_4_BIT_MODE);
	Lcd_string(&lcd,"Hello!");
	HAL_UART_Receive_IT(&huart3,(uint8_t*) &Z,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (PA0==GPIO_PIN_RESET)
			{
			TXBuf[3]='0';	
		  TXBuf[4]=13;
			CDC_Transmit_FS((uint8_t*)TXBuf,5);
			while(PA0==GPIO_PIN_RESET);
			};
		if (PA1==GPIO_PIN_RESET)
			{
			TXBuf[3]='1';
			TXBuf[4]=13;
			CDC_Transmit_FS((uint8_t*)TXBuf,5);
			while(PA1==GPIO_PIN_RESET);
			};
		if (PA2==GPIO_PIN_RESET)
			{
			TXBuf[3]='2';
			TXBuf[4]=13;
			CDC_Transmit_FS((uint8_t*)TXBuf,5);
			while(PA2==GPIO_PIN_RESET);
			};
		if (PA3==GPIO_PIN_RESET)
			{
			TXBuf[3]='3';
			TXBuf[4]=13;
			CDC_Transmit_FS((uint8_t*)TXBuf,5);
			while(PA3==GPIO_PIN_RESET);
			};
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void USB_DEVICE_MasterHardReset(void)

{
     GPIO_InitTypeDef GPIO_InitStruct;
     GPIO_InitStruct.Pin = GPIO_PIN_12;
     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
     GPIO_InitStruct.Pull = GPIO_PULLDOWN;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,0);
     HAL_Delay(1000);
 }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		static int NMEA_ReadInit =0;
		static int crRead =0;
		static int NMEA_Ready =0;
		
		if(!NMEA_ReadInit && Z=='$')
				{
					NMEA_ReadInit =1;
					NMEA_Ready =0;
					NMEA_Size =0;;
				}
			else if(!NMEA_ReadInit)
				{
					NMEA_Ready =0;
					NMEA_Size =0;
				}
			else
				{
					if(Z =='\r')
						{
						if(!crRead)
							crRead =1;
						else
							{
							NMEA_ReadInit =0;
							NMEA_Ready =0;
							NMEA_Size =0;
							}
						}
					else if(crRead && Z =='\n')
							{
							if(crcCheck())
								{
									NMEA_Ready =1;
								}
							else
								{
									NMEA_Ready =0;
									NMEA_Size =0;;
								}
							NMEA_ReadInit =0;
							crRead =0;
							}
					else if(crRead && Z!='\n')
							{
								crRead =0;
								NMEA_ReadInit =0;
								NMEA_Ready =0;
								NMEA_Size =0;
							}
					else
							{
								NMEA_Size++;
								NMEA_Buff[NMEA_Size -1]=Z;
							}
				}	
		HAL_UART_Receive_IT(&huart3,(uint8_t*) &Z,1);
	};
	

	
	int crcCheck()
	{
		char strCrc[3]={'\0','\0','\0'};
		if(NMEA_Buff[NMEA_Size-3]!='*')
			return 0;
		strCrc[0]=NMEA_Buff[NMEA_Size-2];
		strCrc[1]=NMEA_Buff[NMEA_Size-1];
		unsigned char crc =0;
		int declaredCrc =strtol(strCrc,NULL,16);
		for(int i =0;i <NMEA_Size-3;i++)
			crc ^=NMEA_Buff[i];
		return	crc ==declaredCrc;
		}
/* USER CODE END 4 */

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
