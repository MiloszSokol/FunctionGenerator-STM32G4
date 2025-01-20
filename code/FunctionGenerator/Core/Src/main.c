/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_MAX_VALUE 4095
#define SAMPLES 100
//#define DMA_CHANNEL_FOR_DAC1 DMA1_Channel3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */
uint16_t sine_wave[SAMPLES] = {
    2047, 2175, 2302, 2427, 2550, 2670, 2787, 2900, 3010, 3114,
    3214, 3308, 3396, 3478, 3553, 3622, 3684, 3738, 3785, 3825,
    3856, 3880, 3896, 3904, 3904, 3896, 3880, 3856, 3825, 3785,
    3738, 3684, 3622, 3553, 3478, 3396, 3308, 3214, 3114, 3010,
    2900, 2787, 2670, 2550, 2427, 2302, 2175, 2047, 1918, 1791,
    1666, 1543, 1423, 1306, 1193, 1083,  979,  879,  785,  697,
     615,  540,  471,  409,  355,  308,  268,  237,  213,  197,
     189,  189,  197,  213,  237,  268,  308,  355,  409,  471,
     540,  615,  697,  785,  879,  979, 1083, 1193, 1306, 1423,
    1543, 1666, 1791, 1918
};

uint16_t triangle_wave[SAMPLES] = {
    0,   82,  165,  247,  330,  412,  495,  577,  660,  742,
    825,  907,  990, 1072, 1155, 1237, 1320, 1402, 1485, 1567,
    1650, 1732, 1815, 1897, 1980, 2062, 2145, 2227, 2310, 2392,
    2475, 2557, 2640, 2722, 2805, 2887, 2970, 3052, 3135, 3217,
    3300, 3382, 3465, 3547, 3630, 3712, 3795, 3877, 3960, 4042,
    4042, 3960, 3877, 3795, 3712, 3630, 3547, 3465, 3382, 3300,
    3217, 3135, 3052, 2970, 2887, 2805, 2722, 2640, 2557, 2475,
    2392, 2310, 2227, 2145, 2062, 1980, 1897, 1815, 1732, 1650,
    1567, 1485, 1402, 1320, 1237, 1155, 1072,  990,  907,  825,
    742,  660,  577,  495,  412,  330,  247,  165,   82,    0
};

volatile uint32_t sample_index = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void TIM6_Init(void);
void DAC1_Init(void);
//void DMA_Init(uint16_t *waveform, uint16_t size);
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

  //DMA_Init(sine_wave, SAMPLES);
  DAC1_Init();
  TIM6_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
}

/* USER CODE BEGIN 4 */


void TIM6_Init()
{
	RCC -> APB1ENR1 |= RCC_APB1ENR1_TIM6EN;					// clock on

	TIM6 -> CR1 &= ~(TIM_CR1_CEN);							// TIM6 off

	uint16_t prescaler = 16;								// (170MHz / 10MHz) - 1
	TIM6 -> PSC = prescaler;
	TIM6 -> ARR = (100 - 1);

	//TIM6 -> DIER |= TIM_DIER_UIE;
	TIM6 -> DIER |= TIM_DIER_UDE;
	TIM6->CR2 &= ~TIM_CR2_MMS;
	TIM6 -> CR2 |= 0b010 << TIM_CR2_MMS_Pos;
	TIM6 -> CR1 |= TIM_CR1_CEN;								// TIM6 on

//	NVIC_SetPriority(TIM6_DAC_IRQn, 2);
//	NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

//void TIM6_DAC_IRQHandler(void)
//{
//	if(TIM6 -> SR & TIM_SR_UIF)
//	{
//		TIM6 -> SR &= ~(TIM_SR_UIF);
//		DAC1 -> DHR12R1 = sine_wave[sample_index];
//		sample_index = (sample_index + 1) % SAMPLES;
//	}
//}

void DMA_Init(uint16_t *waveform, uint16_t size)
{
	RCC -> AHB1ENR |= RCC_AHB1ENR_DMA2EN;						// DMA clock on

	DMA2_Channel3 -> CCR &= ~(DMA_CCR_EN);						// DMA off

	DMA2_Channel3 -> CMAR = (uint32_t)waveform;
	DMA2_Channel3 -> CPAR = (uint32_t)&(DAC1 -> DHR12R1);
	DMA2_Channel3 -> CNDTR = size;

	DMA2_Channel3 -> CCR |= DMA_CCR_CIRC;     					// circual mode
	DMA2_Channel3 -> CCR |= DMA_CCR_MINC;             			// increment memory
	DMA2_Channel3 -> CCR |= DMA_CCR_DIR;          				// memory to peripheral

	//DMA1_Channel3 -> CCR |= 0b01 << DMA_CCR_MSIZE_Pos;
	//DMA1_Channel3 -> CCR |= 0b01 << DMA_CCR_PSIZE_Pos;
	DMA2_Channel3->CCR |= (0b01 << DMA_CCR_MSIZE_Pos);  		// MSIZE = 16-bit
	DMA2_Channel3->CCR |= (0b01 << DMA_CCR_PSIZE_Pos);  		// PSIZE = 16-bit

	DMA2_Channel3 -> CCR |= DMA_CCR_EN;   						// DMA on
}

void DAC1_Init(void)
{
	RCC -> AHB2ENR |= RCC_AHB2ENR_DAC1EN;					// clock on

	DAC1 -> CR &= ~DAC_CR_EN1;         						// DAC off
	DAC1 -> MCR &= ~DAC_MCR_MODE1; 							// Output bufor on

	DAC1 -> CR |= DAC_CR_DMAEN1;							// DMA for DAC1 enable

	DAC1 -> CR |= DAC_CR_TEN1;								// trigger enable for DAC1
	DAC1 -> CR |= (0b0111 << DAC_CR_TSEL1_Pos);				// DAC triggered from TIM6 (dac_ch1_trg7)
	//DAC1 -> CR |= DAC_CR_DMAUDRIE1;

	DAC1 -> CR |= DAC_CR_EN1;								// DAC on

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
