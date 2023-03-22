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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	CDC_Transmit_FS((uint8_t*)&ch,1);
  return ch;
}
	
	
struct NMEA_Token
		{
		char*begin;
		char*end;
		};
struct NMEA_Token tokens[20];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint8_t byte;
char NMEA_Buff[256];
int NMEA_Ready =0;
int NMEA_Size =0;
int token_id_szer;
int	token_id_N;
int token_id_dlug;
int token_id_E;
double szerokosc;
double dlugosc;
double predkosc;
char gps_head[10];
double czas;
uint8_t fix;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
double extractNMEA_Value(struct NMEA_Token*token);
void clearNMEA(void);
int crcCheck(void);
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
  MX_USB_DEVICE_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart3,(uint8_t*) &byte,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//strcat(DataTx11,"$");
	//strcat(DataTx11,"fix");
	//strcat(DataTx11,"*");
	//strcat(DataTx22,"$");
	//strcat(DataTx22,"nofix");
	//strcat(DataTx22,"*");
	
  while (1)
  {
	//uruchomienie / zatrzymanie pomiarow przyciskiem
	
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(NMEA_Ready)
				{
				NMEA_Ready =0;
				int currentToken =0;
				tokens[currentToken].begin =NMEA_Buff;
				for(int i =0;i <NMEA_Size;i++)
					{
						if(NMEA_Buff[i]==',' && currentToken<20)
							{
								tokens[currentToken].end =NMEA_Buff +i;
								currentToken++;
								tokens[currentToken].begin =NMEA_Buff +i +1;
							}
						if (currentToken>=20)
							break;
					};
				tokens[currentToken].end =NMEA_Buff+NMEA_Size;
				strncpy(gps_head, tokens[0].begin, 5);
				if (!strcmp(gps_head,"GPGGA") || !strcmp(gps_head,"GNRMC"))
					{
					if (!strcmp(gps_head,"GPGGA"))
						{
						token_id_szer=2;
						token_id_N=3;
						token_id_dlug=4;
						token_id_E=5;
						fix=(int) *(tokens[6].begin)-48;
						};
					if (!strcmp(gps_head,"GNRMC"))
						{
						token_id_szer=3;
						token_id_N=4;
						token_id_dlug=5;
						token_id_E=6;
						};
					if (fix>0)
						{
						szerokosc=extractNMEA_Value(&tokens[token_id_szer]);
						dlugosc=extractNMEA_Value(&tokens[token_id_dlug]);	
						czas=extractNMEA_Value(&tokens[1]);
						switch(*(tokens[token_id_N].begin))
							{
							case 'S':szerokosc =-szerokosc;break;
							case 'N':break;
							default:szerokosc =0.0;break;
							};
						switch(*(tokens[token_id_E].begin))
							{
							case 'W':dlugosc =-dlugosc;break;
							case 'E':break;
							default:dlugosc=0.0;break;
							}
						printf("Dlugosc geograficzna: %f2.6 \n",dlugosc);
						printf("Szerokosc geograficzna: %f2.6 \n",szerokosc);	
						printf("Czas UTC: %f2.6 \n", czas);
						}
					};	
					if (!strcmp(gps_head,"GPVTG"))
					{
						if (fix>0)
							{	
							predkosc=extractNMEA_Value(&tokens[7]);
							printf("Predkosc: %f2.6 \n",predkosc);
							}
						else
							predkosc=0.0;
					}
		
		}
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		static int NMEA_ReadInit =0;
		static int crRead =0;
		if(!NMEA_ReadInit && byte=='$')
				{
					NMEA_ReadInit =1;
					clearNMEA();
				}
			else if(!NMEA_ReadInit)
					clearNMEA();
			else
				{
					if(byte =='\r')
						{
						if(!crRead)
							crRead =1;
						else
							{
							NMEA_ReadInit =0;
							clearNMEA();
							}
						}
					else if(crRead && byte =='\n')
							{
							if(crcCheck())
								{
									NMEA_Ready =1;
								}
							else
									clearNMEA();
							NMEA_ReadInit =0;
							crRead =0;
							}
					else if(crRead && byte!='\n')
							{
								crRead =0;
								NMEA_ReadInit =0;
								clearNMEA();
							}
					else
							{
								NMEA_Size++;
								NMEA_Buff[NMEA_Size -1]=byte;
							}
				}	
		HAL_UART_Receive_IT(&huart3,(uint8_t*) &byte,1);
	}
	
	
	void clearNMEA()
	{
		NMEA_Ready =0;
		NMEA_Size =0;
	}
	
	
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
	
		
	double extractNMEA_Value(struct NMEA_Token* token)
		{
		double wynikNMEA;
		int dot,comma;
		char tmpStr[30];
		char cal[30],ulam[30];
		for(int i =0;i<30;i++)
			{
			tmpStr[i]=0;
			cal[i]=0;
			ulam[i]=0;
			}
		for(int i =0;;i++)
			{
			tmpStr[i]=*(token->begin+i);
			if((token->begin +i)==(token->end))
				{
				break;
				};
			}
		dot=30;comma=30;
		for(int i =0;i<30;i++)
			{
			if (i<dot)
				cal[i]=tmpStr[i];
			if (tmpStr[i]=='.')
				{
					dot=i;
					cal[i]=0x00;
				}
			if (tmpStr[i]==',')
				{
					comma=i;
					tmpStr[i]=0x00;
					break;
				}
			if (i>dot && i<comma && dot<30)
				ulam[i-dot-1]=tmpStr[i];
			}	
		wynikNMEA=((float)atoi(ulam))/(pow(10,(comma-dot-1)))+(float) atoi(cal);;
		return wynikNMEA;
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
