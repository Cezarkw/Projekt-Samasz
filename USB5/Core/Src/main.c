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
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PB8 *((volatile unsigned long*) 0x422181A0)
#define PB9 *((volatile unsigned long*) 0x422181A4)
#define PA0 *((volatile unsigned long*) 0x42210100)	
#define PA1 *((volatile unsigned long*) 0x42210104)	
	

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
double sensor_gap[6]; //cm
double sensor_gap2[7]; //cm
double distance;  //cm
double distance_array[7];  //cm
double distance_array2[7];  //cm
double wartosc;
double kat_tab[7]={36.87, 26.57, 14.04, 0.00, 14.04, 26.57, 36.87};
int offset[7]={20 ,15,12,9,8,13,20};
double Area_t=0.0; //cm2
double szer;
volatile double Volume=0.0; //cm3
uint8_t byte;
double wysokosc[2]; //cm
char DataTx44[50];
char DataTx33[50];
char DataTx[50];
int czujnik=0;
int checksum;
double calc;
int size;
int komunikat=0;
int start=0;
int scan_result=0;
int usb_tx3, usb_tx4=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
int calculate_CheckSum(char*, int);
int pomiar(uint8_t adr);
int scan(void);
void zeruj(void);
void zeruj2(char * tab, int n);
int rozmiar(char * tab, int n);
void USB_tx(void);
void dane(void);
void obliczenia(void);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//strcat(DataTx11,"$");
	//strcat(DataTx11,"fix");
	//strcat(DataTx11,"*");
	//strcat(DataTx22,"$");
	//strcat(DataTx22,"nofix");
	//strcat(DataTx22,"*");
	strcat(DataTx33,"$");	
	strcat(DataTx33,"fault");
	strcat(DataTx33,"*");
  while (1)
  {
	//uruchomienie / zatrzymanie pomiarow przyciskiem
	if (PA0==0)
		{
		start=start^1;
		HAL_Delay(500);
		while (PA0==0);
		}
	//przelaczenie danych z pola przekroju na pomiar 1 czujnika - przyciskiem
	if (PA1==0)
		{
		czujnik=czujnik^1;
		HAL_Delay(500);
		while (PA1==1);
		}
	//pomiar z 7 czujnik�w i obliczenia
	obliczenia();	
	//jesli start i nie ma bledow pomiaru to ptzygotuj dane do wyslania
	if (scan_result!=-1 && start)
		dane();
	//komunikat o bledzie odczytu dowolnego czujnika
	if ((komunikat==0) && (scan_result==-1))
					{
						komunikat=1;
						zeruj();
						//dlugosc[0]=dlugosc[1];
						//szerokosc[0]=szerokosc[1];	
						scan_result=0;
						usb_tx3=1;
					}	
	//wyslanie danych po USB
	if (start)
		USB_tx();
	
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_MultiProcessor_Init(&huart1, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB11 PB12 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


		int calculate_CheckSum(char * tab, int n)
		{
			int checksum=tab[0];
			for(int i=1;i<n;i++)
					{
					checksum=checksum^tab[i];
					}		
			return checksum;
		}
		
	int pomiar(uint8_t adr)
			{
				uint8_t value1=0x04;
				uint8_t value2;
				uint8_t low, high=0x00;

				uint16_t wynik;
				HAL_StatusTypeDef OK;
				//wyslanie komendy pomiaru
				OK=HAL_I2C_Mem_Write(&hi2c1, adr<<1, 0x00, I2C_MEMADD_SIZE_8BIT, &value1, 0x1, 100);
				if (OK!=HAL_OK)
					{return -1;};	
				do
					{
					//odcyztywanie flagi zakonczenie pomiaru
					OK=HAL_I2C_Mem_Read(&hi2c1, adr<<1, 0x01, I2C_MEMADD_SIZE_8BIT, &value2, 0x1, 100);
					if (OK!=HAL_OK)
						{return -1;};
					}
				while((value2 & 0x01)!=0x00);
				//odczytanie mlodszego bajtu wyniku
				OK=HAL_I2C_Mem_Read(&hi2c1, adr<<1, 0x10, I2C_MEMADD_SIZE_8BIT, &low, 0x1, 100);
				if (OK!=HAL_OK)
					{return -1;};
				//odczytanie starszego bajtu wyniku
				OK=HAL_I2C_Mem_Read(&hi2c1, adr<<1, 0x11, I2C_MEMADD_SIZE_8BIT, &high, 0x1, 100);
				if (OK!=HAL_OK)
					{	return -1;};
				wynik= (((uint16_t) high)<<8)+ (uint16_t) low+16;
				return wynik;
			}
			
		int scan (void)
			{
				int value;
				int index;
				int blad=0;
				uint8_t i=0;
				for(i=0x63;i<0x6A;i++)
					{
					//odczytanie pomiaru z czujnika
					value=pomiar(i);
					index=i-0x63;
					if (value==-1)
						{
						blad=1;
						distance_array[index]=0;	
						}
					else
						//dodatnie do pomiaru odleglosci czujnika od srodka okregu krzywizny glowicy
						distance_array[index]=(double)value+offset[index];
					};
					if (blad==1)
						return -1;
				else
						return 0;
			}
			
		void zeruj(void)
			{
				Area_t=0;
				Volume=0.0;
			}
			
		void zeruj2(char * tab, int n)
			{
				for(int i=0;i<n;i++)
					tab[i]=0;
			}
			
			
			
		int rozmiar(char * tab, int n)
		{
			int i;
			for(i=0;i<n;i++)
				if (tab[i]=='*')
					return (i+1);
			return i;
		}
		
			
		void USB_tx(void)
		{
			volatile int R;
			PB9=PB9^1;
				if (usb_tx3)
					{
					R=rozmiar(DataTx33,50);
					if (R<50)
						{
						checksum=calculate_CheckSum(DataTx33, R);
						strcat(DataTx33,(char*) &checksum);
						DataTx33[R+1]=0x0A;
						CDC_Transmit_FS((uint8_t*)DataTx33,R+2);
						usb_tx3=0;
						}
					}
				if (usb_tx4)
					{
					R=rozmiar(DataTx44,50);
					if (R<50)
						{
						checksum=calculate_CheckSum(DataTx44, R);
						strcat(DataTx44,(char*) &checksum);
						DataTx44[R+1]=0x0A;
						CDC_Transmit_FS((uint8_t*)DataTx44,R+2);
						usb_tx4=0;
						}
					}			
		}
		
		void dane(void)
		{
			int c;
			usb_tx4=0;
			zeruj2(DataTx44,50);
			zeruj2(DataTx,50);
			DataTx44[0]='$';
			c=snprintf(DataTx,50,"%.2f",fabs(Volume));
			if (c<0 || c>20)
				return;	
			if (DataTx[0]=='-')
				return;	
			strcat(DataTx44,DataTx);
			strcat(DataTx44,"*");
			usb_tx4=1;
			return;		
		};
		
		
void obliczenia (void)
	{
			double alfa;
				//jesli start to wykonaj pomiar i oblicz
				if (start)
					{
					//wykoannie odczytu z 7 czujnikow
					scan_result=scan();
					//jesli nie ma bledow odczytu to oblicz
						if (scan_result!=-1)
							{
							if (czujnik==1)
								Volume=distance_array[3];
							komunikat=0;
							alfa=0.017453277*kat_tab[0]; //kat alfa
				      for(int i=0;i<2;i++)
								wysokosc[i]=cos(alfa)*(distance_array[6*i]); //wysokosc H1 i H7 z 1 i 7 czujnika
							szer=(wysokosc[0]-wysokosc[1])/(sin(alfa)*(distance_array[0]+distance_array[6])); //odleglosc punktu pomiaru 1 i 7 czujnika od osi symetrii
							for(int i=0;i<6;i++)
								sensor_gap[i]=fabs(distance_array[i]*sin(0.01745327*kat_tab[i])-distance_array[i+1]*sin(0.01745327*kat_tab[i+1])); //odleglosci pomiedzy punktami pomiaru w kierunku prostopadlym do osi symetrii
							for(int i=1;i<7;i++)
								sensor_gap2[i]=sensor_gap2[i-1]+sensor_gap[i-1]; //odleglosci pomiedzy punktami pomiaru w kierunku prostopadlym do osi symetrii narastajaco
							for(int i=0;i<7;i++)
								{
								wartosc=wysokosc[0]-sensor_gap2[i]*szer-cos(0.01745327*kat_tab[i])*distance_array[i]; //obliczenie wysokosci sianokoasu w punktach pomiaru
								distance_array2[i]=fabs(wartosc);
								}
							Area_t=0;	
							for(int i=0;i<6;i++)
								{
									calc=fabs(distance_array2[i+1]-distance_array2[i]); //odliczenie przyblizonego pola trapezu lub trojkata dla skrajnych pomiarow
									Area_t=Area_t+sensor_gap[i]*(distance_array2[i]+0.5*calc);			
								}		
							if (czujnik==0)	
								Volume=fabs(Area_t); //wartosc wysylana - pole przekroju
						}
					}
				else
				{
				zeruj();
				}
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
		PB8=PB8^1;
		HAL_Delay(1000);
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
