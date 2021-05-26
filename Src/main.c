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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_ts.h"
#include "math.h"
#include "achtergrond_data.h"
#include "-100_data.h"
#include "-75_data.h"
#include "-50_data.h"
#include "-25_data.h"
#include "0_data.h"
#include "25_data.h"
#include "50_data.h"
#include "75_data.h"
#include "100_data.h"

#define SCREENSAVER_DELAY 10000
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define aantal_samples 65536
#define aantal_samples_d2 aantal_samples/2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c1;

LTDC_HandleTypeDef hltdc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t N = aantal_samples;


float Cte = 0;

uint32_t i = 0;
uint8_t j = 0;
uint16_t k = 0;

uint16_t terts[]={25,31,40,50,63,80,100,125,160,200,250,315,400,500,630,800,1000,1250,1600,2000,2500,3150,4000,5000,6300,8000,10000,12500,16000,20000};
//uint16_t terts[]={63,16456};


uint8_t * pointer = (uint8_t *)0xC0000000;

float * sample = (float *)0xC0411000;

float * even   = (float *)0xC0101000;
float * oneven = (float *)0xC0201000;
float * han    = (float *)0xC0301000;
uint8_t m = 32;
uint16_t LedState = 0;
uint8_t val[32];


TS_StateTypeDef TS_State;
volatile uint8_t Bass = 4;
volatile uint8_t Treble = 4;
volatile uint8_t change = 0;
volatile uint8_t AD5236_ADDR = 0x58; // Use 8-bit address
volatile uint8_t REG_INS1 = 0b00000000;
volatile uint8_t REG_INS2 = 0b00100000;
volatile uint8_t REG_INS3 = 0b01000000;
volatile uint8_t REG_INS4 = 0b01100000;
uint8_t REG_DATAb = 128;
uint8_t REG_DATAt = 128;
//uint8_t pointer[50];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LTDC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

int _write(int file, char *ptr, int len) {
    HAL_StatusTypeDef xStatus;
    switch (file) {
    case STDOUT_FILENO: /*stdout*/
		xStatus = HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
		if (xStatus != HAL_OK) {
			errno = EIO;
			return -1;
		}
        break;
    case STDERR_FILENO: /* stderr */
		xStatus = HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
		if (xStatus != HAL_OK) {
			errno = EIO;
			return -1;
		}
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return len;
}
volatile uint8_t ready = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Conversion Complete & DMA Transfer Complete As Well
    // So The AD_RES Is Now Updated & Let's Move IT To The PWM CCR1
    // Update The PWM Duty Cycle With Latest ADC Conversion Result

	HAL_ADC_Stop_DMA(&hadc3);
	ready++;
	  if(ready == 1){
		  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&pointer[aantal_samples_d2], aantal_samples_d2);
	  }
//	for(int i = 0; i<20; i++){
//		printf("int%d: %d \n\r",i, pointer[i]);
//	}
//	memset(pointer,'\0',20);
//	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&pointer[0], 20);
}

void SysTickDelayCount(unsigned long ulCount){
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    while(DWT->CYCCNT < ulCount);
}

HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	uint8_t buf[2];
	BSP_TS_GetState(&TS_State);
	if(TS_State.touchDetected){ //zien of er een aanraking was
		//bepalen waar de vinger is
		uint16_t Y = TS_State.touchY[0];
		uint16_t X = TS_State.touchX[0];
		if (X <= 254 && X >= 220) { //- knop bass
			printf("Bass: %d \n\r", Bass );
			if(Y >= 30 && Y <= 81){
				if(Bass != 0){
					Bass--;
					change = 1;
					printf("Bass: %d \n\r", Bass );
				}
			}
			if(Y >= 190 && Y <= 241){ //-knop treble
				if(Treble != 0){
					Treble--;
					change = 1;
					printf("Treble: %d \n\r", Treble );
				}
			}
		}
		if (X >= 420 && X <= 450) {
			printf("Bass: %d \n\r", Bass );
			if(Y >= 30 && Y <= 81){ //+knop bass
				if(Bass != 8){
					Bass++;
					change = 1;
					printf("Bass: %d \n\r", Bass );
				}
			}
			if(Y >= 190 && Y <= 241){ //+knop treble
				if(Treble != 8){
					Treble++;
					change = 1;
					printf("Treble: %d \n\r", Treble );
				}
			}
		}
		if(change == 1){
			change = 0;
			//bepalen welke figuur er op komt
			if(Bass == 0){
				WDA_LCD_DrawBitmap(n100_DATA,254,30,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAb = 255;
			}
			if(Bass == 1){
				WDA_LCD_DrawBitmap(n75_DATA,254,30,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAb = 223;
			}
			if(Bass == 2){
				WDA_LCD_DrawBitmap(n50_DATA,254,30,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAb = 191;
			}
			if(Bass == 3){
				WDA_LCD_DrawBitmap(n25_DATA,254,30,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAb = 159;
			}
			if(Bass == 4){
				WDA_LCD_DrawBitmap(_0_DATA,254,30,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAb = 128;
			}
			if(Bass == 5){
				WDA_LCD_DrawBitmap(_25_DATA,254,30,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAb = 112;
			}
			if(Bass == 6){
				WDA_LCD_DrawBitmap(_50_DATA,254,30,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAb = 96;
			}
			if(Bass == 7){
				WDA_LCD_DrawBitmap(_75_DATA,254,30,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAb = 80;
			}
			if(Bass == 8){
				WDA_LCD_DrawBitmap(_100_DATA,254,30,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAb = 64;
			}
			if(Treble == 0){
				WDA_LCD_DrawBitmap(n100_DATA,254,190,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAt = 64;
			}
			if(Treble == 1){
				WDA_LCD_DrawBitmap(n75_DATA,254,190,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAt = 80;
			}
			if(Treble == 2){
				WDA_LCD_DrawBitmap(n50_DATA,254,190,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAt = 96;
			}
			if(Treble == 3){
				WDA_LCD_DrawBitmap(n25_DATA,254,190,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAt = 112;
			}
			if(Treble == 4){
				WDA_LCD_DrawBitmap(_0_DATA,254,190,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAt = 128;
			}
			if(Treble == 5){
				WDA_LCD_DrawBitmap(_25_DATA,254,190,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAt = 159;
			}
			if(Treble == 6){
				WDA_LCD_DrawBitmap(_50_DATA,254,190,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAt = 191;
			}
			if(Treble == 7){
				WDA_LCD_DrawBitmap(_75_DATA,254,190,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAt = 223;
			}
			if(Treble == 8){
				WDA_LCD_DrawBitmap(_100_DATA,254,190,145,51,LTDC_PIXEL_FORMAT_ARGB1555);
				REG_DATAt = 255;
			}
			buf[0] = REG_INS1;
			buf[1] = REG_DATAb;
			HAL_I2C_Master_Transmit(&hi2c1, AD5236_ADDR, buf, 2, HAL_MAX_DELAY); //data sturen naar pot 1
			buf[0] = REG_INS2;
			buf[1] = REG_DATAb;
			HAL_I2C_Master_Transmit(&hi2c1, AD5236_ADDR, buf, 2, HAL_MAX_DELAY); //data sturen naar pot 2
			buf[0] = REG_INS3;
			buf[1] = REG_DATAt;
			HAL_I2C_Master_Transmit(&hi2c1, AD5236_ADDR, buf, 2, HAL_MAX_DELAY); //data sturen naar pot 3
			buf[0] = REG_INS4;
			buf[1] = REG_DATAt;
			HAL_I2C_Master_Transmit(&hi2c1, AD5236_ADDR, buf, 2, HAL_MAX_DELAY); //data sturen naar pot 4
		}
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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

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
  MX_DMA_Init();
  MX_LTDC_Init();
  MX_USART1_UART_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LWIP_Init();
  MX_ADC3_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* LCD Initialization */
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(1,LCD_FB_START_ADDRESS);//adres instellen voor externe ram
    /* Enable the LCD */
    BSP_LCD_DisplayOn(); //lcd aan
    /* Select the LCD Background Layer  */
    BSP_LCD_SelectLayer(1); //laag 1 van lcd kiezen
    /* Touch veld instellen*/
    BSP_TS_Init(480,272);
    WDA_LCD_DrawBitmap(ACHTERGROND_DATA,0,0,ACHTERGROND_DATA_X_PIXEL,ACHTERGROND_DATA_Y_PIXEL,ACHTERGROND_DATA_FORMAT); //achtergrond tekenen
    WDA_LCD_DrawBitmap(_0_DATA,254,30,145,51,LTDC_PIXEL_FORMAT_ARGB1555); //afbeelding 0 tekenen
    WDA_LCD_DrawBitmap(_0_DATA,254,190,145,51,LTDC_PIXEL_FORMAT_ARGB1555); //afbeelding 0 tekenen

    HAL_TIM_Base_Start_IT(&htim2);//timer 2 starten

    //printf("DMA test3\r\n");


    HAL_StatusTypeDef stat;
    stat = HAL_ADC_Start_DMA(&hadc3, (uint32_t*) &pointer[0], aantal_samples_d2);//dma starten en kijken of het werkt
    if(stat != HAL_OK){
  	  printf("HAL_ADC_Start_DMA faalt\r\n");
    }

    stat = HAL_TIM_Base_Start_IT(&htim8);//timer voor dma starten en kijken of het werkt
    if(stat != HAL_OK){
  	  printf("HAL_TIM_Base_Start_IT faalt\r\n");
    }

    HAL_GPIO_WritePin(OE_GPIO_Port,OE_Pin,SET); //leds uit
	HAL_GPIO_WritePin(SER_GPIO_Port,SER_Pin,SET); //ingang hoog
	HAL_GPIO_WritePin(Latch_GPIO_Port,Latch_Pin,RESET); //clk naar beneden


	for (i = 0; i < aantal_samples; i++) { //hann window constante al uitrekenen
		han[i] = (1 - cos(2*M_PI*i/aantal_samples))/2;
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(ready == 2){
		//printf("ready \n\r");
		ready = 0;
		HAL_GPIO_WritePin(OE_GPIO_Port,OE_Pin,SET);//leds uit
		//printf("ready: %d\n\r",ready);

		N = aantal_samples;

		//printf("adc: %d \n\r",pointer[0]);

		for (i = 0; i < aantal_samples; i++) { //alles verplaatsen naar float en hann window toepassen
			sample[i] = ((pointer[i] -98) /98.0000)*han[i];
		}

		//splitsen
		while(N != 2){
		      for(i = 0; i < aantal_samples_d2; i++){
		         even[i]= sample[i*2];
		         oneven[i]= sample[i*2+1];
		      }
		      N = N/2;
		      for(i = 0; i < aantal_samples_d2; i++){
		        sample[i] = even[i];
		        sample[aantal_samples_d2+i] = oneven[i];
		      }


		    }

		  //samenvoegen
		  for (j = 0; j < 30; j++) {
			  N = aantal_samples;
			  k = terts[j];
			    while( N != 1){

			        Cte = (-M_PI*N*k)/aantal_samples;
			        Cte = cos(Cte)+sin(Cte);

			        N = N/2;
			        for(i=0; i < N;i++){
			          sample[2*i]=sample[2*i]+Cte*sample[2*i+1];
			          //printf("sample: %f\n\r",sample[2*i]);
			        }

			        if (N > 2) {
				        for(i=0; i < (N/2); i = i +2){
				          sample[i+1]=sample[i+N];
				        }
					}
			        if (N == 1) {
						sample[1] = sample[2];
					}

			    }

			    sample[0] = abs(sample[0]);
			    val[j] = 0;

			    //waarde bepalen voor op leds
			    if(((uint8_t)sample[0] / 1) != 0){
			    	val[j] = 1;
			    }
			    if(((uint8_t)sample[0] / 2) != 0){
					val[j] = 1;
				}
			    if(((uint8_t)sample[0] / 4) != 0){
					val[j] = 3;
				}
			    if(((uint8_t)sample[0] / 8) != 0){
					val[j] = 7;
				}
			    if(((uint8_t)sample[0] / 16) != 0){
					val[j] = 15;
				}
			    if(((uint8_t)sample[0] / 32) != 0){
					val[j] = 31;
				}
			    if(((uint8_t)sample[0] / 64) != 0){
					val[j] = 63;
				}
			    if(((uint8_t)sample[0] / 128) != 0){
					val[j] = 127;
				}
			    if(((uint8_t)sample[0] / 255) != 0){
					val[j] = 255;
				}

			    //printf("freq: %dHz: %f\n\r",terts[j],sample[0]);


			    for(i = 0; i < aantal_samples_d2; i++){
			            sample[i] = even[i];
			            sample[aantal_samples_d2+i] = oneven[i];
			    }//alle waardes terug resetten om volgende frequentie te berekenen
		  }

		HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&pointer[0], aantal_samples_d2);//dma starten
	  }

	  //ledmatrix
	  HAL_GPIO_WritePin(OE_GPIO_Port,OE_Pin,RESET); // als 32 pulsen zijn geweest, 1 op ingang zetten
	  if( m == 32){
		  m = 0;
		  HAL_GPIO_WritePin(SER_GPIO_Port,SER_Pin,SET);
	  }
	  else{
		  HAL_GPIO_WritePin(SER_GPIO_Port,SER_Pin,RESET);
	  }



	  HAL_GPIO_WritePin(SRCLK_GPIO_Port,SRCLK_Pin,SET); //SRCLK is latch tegenovergestelde clk
	  HAL_GPIO_WritePin(Latch_GPIO_Port,Latch_Pin,RESET);
	  SysTickDelayCount(10000); //wachttijd om leds aan te passen

 	  LedState = GPIOI -> ODR; //zien wat in register zit
 	  LedState = LedState & 0xFFF0; //masker plaatsen
	  LedState = LedState+((~val[m] & 0xF0)>>4); //waarde op juiste pinnen zetten
	  GPIOI -> ODR = LedState; //4 eerste leds aansturen

	  LedState = GPIOF -> ODR; //zien wat in register zit
	  LedState = LedState & 0xFC3F;//masker plaatsen
	  LedState = LedState+((~val[m] & 0x0F)<<6); //waarde op juiste pinnen zetten
	  GPIOF -> ODR = LedState; //4 laatste leds aansturen

	  HAL_GPIO_WritePin(SRCLK_GPIO_Port,SRCLK_Pin,RESET);
	  HAL_GPIO_WritePin(Latch_GPIO_Port,Latch_Pin,SET);
	  SysTickDelayCount(10000);//wachttijd om leds aan te passen

	  m = m + 1;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_8B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

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
  hi2c1.Init.Timing = 0x00C0EAFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 40;
  hltdc.Init.VerticalSync = 9;
  hltdc.Init.AccumulatedHBP = 53;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 533;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 565;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 255;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB1555;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 480;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 272;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB1555;
  pLayerCfg1.Alpha = 255;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 480;
  pLayerCfg1.ImageHeight = 272;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim8.Init.Period = 1524;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|LCD_DISP_Pin
                          |GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OE_Pin|SER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SRCLK_GPIO_Port, SRCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_9|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : Latch_Pin */
  GPIO_InitStruct.Pin = Latch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Latch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PI3 PI2 PI1 LCD_DISP_Pin
                           PI0 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|LCD_DISP_Pin
                          |GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OE_Pin SER_Pin */
  GPIO_InitStruct.Pin = OE_Pin|SER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SRCLK_Pin */
  GPIO_InitStruct.Pin = SRCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SRCLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF6 PF9 PF8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_9|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
