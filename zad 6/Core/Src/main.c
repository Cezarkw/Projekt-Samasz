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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd4bit.h"
#include "stdio.h"
#include "./FATFs/ff.h"
#include "./FATFs/diskio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STAN_PROG_OCZEKIWANIE     0
#define STAN_PROG_GOTOWOSC_ZAPISU 1
#define STAN_PROG_JEST_KARTA      2
#define STAN_PROG_BRAK_KARTY      3
#define STAN_PROG_BLAD_KARTY      4
#define STAN_PROG_BRAK_PLIKU      5
#define STAN_PROG_ZAPIS_PLIKU     7
#define STAN_PROG_BLAD_ZAPISU     8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
char stanProg=STAN_PROG_OCZEKIWANIE;
static FATFS g_sFatFs; //obiekt FATFs
bool sekunda=FALSE;
const char stopienSymbol[8] = {0x06, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00,0};   //symbol stopnia
FRESULT fresult;
FIL plik;
UINT bajtowZapisanych=0;
bool zapis=FALSE;
unsigned long int t=0;
char tTekst[8]={"0"};
char poprzedniStanGPIOA8=0;
char obecnyStanGPIOA8=0;
uint16_t temp;
float temp_f;
char text[10];
Lcd_PortType ports[] = {GPIOC, GPIOC, GPIOC, GPIOC};
Lcd_PinType pins[] = {GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0};
Lcd_HandleTypeDef lcd;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	lcd = Lcd_create(ports, pins, GPIOC, GPIO_PIN_12, GPIOC, GPIO_PIN_10, LCD_4_BIT_MODE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	obecnyStanGPIOA8=(1-HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8));//zanegowanie rzeczywistego stanu GPIOA8 spowoduje, ze stan obecny i stan poprzedni beda rozne -> wymusi to testowanie obecnosci karty przy pierwszym wejsciu w STAN_PROG_OCZEKIWANIE
  HAL_Delay(5);
  while (1)
  {
		HAL_GPIO_WritePin( GPIOB, GPIO_PIN_14, GPIO_PIN_RESET );
		HAL_SPI_Receive(&hspi2,(uint8_t*) &temp,1,HAL_MAX_DELAY);
		HAL_GPIO_WritePin( GPIOB, GPIO_PIN_14, GPIO_PIN_SET );
		temp_f=(temp>>3)*0.0625;
		sprintf(text,"%2.2f  C",temp_f);
		Lcd_cursor(&lcd, 0,0);
		Lcd_string(&lcd,"Temp.:");
		Lcd_cursor(&lcd, 0,8);
		Lcd_string(&lcd,text);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		poprzedniStanGPIOA8=obecnyStanGPIOA8;
	  obecnyStanGPIOA8=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
		 switch (stanProg){
	     case STAN_PROG_OCZEKIWANIE : {
	                                     if (obecnyStanGPIOA8==poprzedniStanGPIOA8){
	                                       HAL_Delay(100);
	                                     } else {
	                                       if ((obecnyStanGPIOA8==SET)&&(poprzedniStanGPIOA8==RESET)){
	                                         stanProg=STAN_PROG_BRAK_KARTY;
	                                       } else {
	                                         stanProg=STAN_PROG_JEST_KARTA;
	                                       }
	                                     }
	                                   } break;
	      case STAN_PROG_GOTOWOSC_ZAPISU : {
	                                     if (obecnyStanGPIOA8==poprzedniStanGPIOA8){ //brak zmiany stanu oznacza, ze nie wyjeto karty w czasie zapisu
		                                   HAL_Delay(100);

	                                       if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){//SW0 - wlacz zapis
	                                         zapis=TRUE;
	                                         t=0;
	                                       }
	                                       if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)){//SW1 - wylacz zapis
	                                         zapis=FALSE;
	                                       }

	                                       if (zapis) {                              //trwa zapis
																					 Lcd_cursor(&lcd,1,0);
	                                         Lcd_string(&lcd,"Zapis pliku");
	                                         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	                                         if (sekunda) {                          //zapis kolejnych wartosci tylko w chwilach, gdy uplynela kolejna sekunda
	                                           stanProg=STAN_PROG_ZAPIS_PLIKU;
	                                         }
	                                       } else {                                  //brak zapisu, ale karta zainicjalizowana i gotowa do zapisu
																					 Lcd_cursor(&lcd,1,0);
	                                         Lcd_string(&lcd,"Gotowy do zapisu");
	                                         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	                                       }
	                                     } else {                                    //nastapila zmiana stanu linii DETECT
	                                       if ((obecnyStanGPIOA8==SET)&&(poprzedniStanGPIOA8==RESET)){ //wyjeto karte
	                                         stanProg=STAN_PROG_BRAK_KARTY;
	                                       }
	                                     }
	                                   } break;
	      case STAN_PROG_BRAK_KARTY : {
																			 Lcd_cursor(&lcd,1,0);
	                                     Lcd_string(&lcd,"Brak karty");
	                                     stanProg=STAN_PROG_OCZEKIWANIE;
	                                     zapis=FALSE;                                //jesli brak karty - zapis jest anulowany
	                                     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	                                   } break;
	      case STAN_PROG_BLAD_KARTY : {  //blad inicjalizacji (np. blad komunikacji z karta)
																			 Lcd_cursor(&lcd,1,0);
	                                     Lcd_string(&lcd,"Blad karty");
	                                     stanProg=STAN_PROG_OCZEKIWANIE;
	                                   } break;
	      case STAN_PROG_JEST_KARTA : {
	                                     fresult = f_mount(&g_sFatFs,"/",1);
	                                     if (fresult==0) {
	                                        stanProg=STAN_PROG_GOTOWOSC_ZAPISU;
	                                     } else {
	                                        stanProg=STAN_PROG_BLAD_KARTY;
	                                     }
	                                  } break;
	      case STAN_PROG_BLAD_ZAPISU : {
																			 Lcd_cursor(&lcd,1,0);
	                                     Lcd_string(&lcd,"Blad zapisu");
	                                     zapis=FALSE;                                //jesli byl blad - zapis jest anulowany
	                                     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);   //brak zapisu
	                                     stanProg=STAN_PROG_OCZEKIWANIE;
	                                   } break;
	      case STAN_PROG_ZAPIS_PLIKU : {
	    	                             HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);     //rozpoczeto zapis
	                                     fresult = f_open (&plik, "dane.txt", FA_WRITE | FA_OPEN_ALWAYS);
	                                     if (fresult) {stanProg=STAN_PROG_BLAD_ZAPISU; break;}
	                                     if (t>0) {                                  //jesli czas >0, dopisujemy na koncu pliku nowa linie danych
	                                       fresult = f_lseek(&plik, plik.obj.objsize);
	                                     } else {                                    //jesli czas ==0, to oznacza to, ze zapis dopiero sie zaczyna i nalezy plik "obciac" i zaczac zapis od nowa
	                                       fresult = f_lseek(&plik, 0);
	                                       fresult = f_truncate(&plik);
	                                     }
	    		                         sprintf(tTekst, "%5ld  ", t);                                                    //kolumna czasu
	                                     fresult = f_write (&plik, (const void*)text, sizeof(text), &bajtowZapisanych);
	                                     if (fresult) {stanProg=STAN_PROG_BLAD_ZAPISU; break;}
	                                     fresult = f_write (&plik, (const void*)text, sizeof(text), &bajtowZapisanych);	 //wartosc temperatury, zapis 4 bajtow czyli tylko wartosci, bez symbolu stC
	                                     if (fresult) {stanProg=STAN_PROG_BLAD_ZAPISU; break;}
	                                     fresult = f_write (&plik, (const void*)"\r\n", 2, &bajtowZapisanych);	           //nowa linia  (wg. standardu Windows)
	                                     if (fresult) {stanProg=STAN_PROG_BLAD_ZAPISU; break;}
	                                   	 fresult = f_close (&plik);
	                                   	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);   //koniec zapisu
	                                     stanProg=STAN_PROG_GOTOWOSC_ZAPISU;
	                                     t+=1;                                       //czas dla nastepnej wartosci
	                                   } break;

	      default : stanProg=STAN_PROG_OCZEKIWANIE;
	 }
	 sekunda=FALSE;
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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA3 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
extern void disk_timerproc (void);

void HAL_SYSTICK_Callback(void){
  static int licznik=0;

  disk_timerproc();
  licznik++;
  if (licznik==1000) {
    sekunda=TRUE;
    licznik=0;
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
