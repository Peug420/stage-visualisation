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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "sai.h"
#include "tim.h"
#include "gpio.h"
//#include "arm_math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "Codec.h"
#include "dsp/filtering_functions.h"
//#include "iir_break"	// iir filter coefficients
#include "iir_kick.h"
//#include "iir_melody"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RECORD_BUFFER_SIZE 512
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//float32_t state_L_break[2*STAGES];	// state arrays
//float32_t state_R_break[2*STAGES];
float32_t state_L_kick[2*STAGES];
float32_t state_R_kick[2*STAGES];
//float32_t state_L_melody[2*STAGES];
//float32_t state_R_melody[2*STAGES];

//arm_biquad_cascade_df2T_instance_f32 IIR_L_break;	 // create IIR instances
//arm_biquad_cascade_df2T_instance_f32 IIR_R_break;
arm_biquad_cascade_df2T_instance_f32 IIR_L_kick;
arm_biquad_cascade_df2T_instance_f32 IIR_R_kick;
//arm_biquad_cascade_df2T_instance_f32 IIR_L_melody;
//arm_biquad_cascade_df2T_instance_f32 IIR_R_melody;

int16_t RecordBuffer[RECORD_BUFFER_SIZE];
int16_t PlaybackBuffer[RECORD_BUFFER_SIZE];
int16_t left_channel[RECORD_BUFFER_SIZE];
int16_t right_channel[RECORD_BUFFER_SIZE];
static volatile int16_t *RecordBufferPtr = &RecordBuffer;
static volatile int16_t *PlaybackBufferPtr = &PlaybackBuffer;

uint8_t dataReadyFlag;

/*Buffer and Variables ADC1 Channel on Pins A0, A1, A2*/
uint16_t AdcValues[3];
uint16_t AdcChannel1, AdcChannel2, AdcChannel3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void ProcessData();
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
	arm_biquad_cascade_df2T_init_f32(&IIR_L_kick, STAGES, ba_coeff, state_L_kick);
	arm_biquad_cascade_df2T_init_f32(&IIR_R_kick, STAGES, ba_coeff, state_R_kick);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C4_Init();
  MX_SAI1_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*) PlaybackBuffer, RECORD_BUFFER_SIZE);
  HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*) RecordBuffer, RECORD_BUFFER_SIZE);

  HAL_TIM_Base_Start_IT(&htim5);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) AdcValues, 3);

  Codec();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(dataReadyFlag){
		  ProcessData();
	  }
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
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 344;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 7;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai){

	RecordBufferPtr = &RecordBuffer[0];
	PlaybackBufferPtr = &PlaybackBuffer[0];

	dataReadyFlag = 1;

}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){

	RecordBufferPtr = &RecordBuffer[RECORD_BUFFER_SIZE/2];
	PlaybackBufferPtr = &PlaybackBuffer[RECORD_BUFFER_SIZE/2];

	dataReadyFlag = 1;

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	AdcChannel1 = AdcValues[0];
	AdcChannel2 = AdcValues[1];
	AdcChannel3 = AdcValues[2];
}

void ProcessData(){

	HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_5, GPIO_PIN_SET);
	 volatile static float leftIn, rightIn;
	 volatile static float leftOut, rightOut;

	 for (uint16_t i =0; i<(RECORD_BUFFER_SIZE/2)-1 ; i+=2){


	 		/*Audio Input convert to float Left Channel*/
		 	leftIn = (1.0f/32768.0f)* RecordBufferPtr[i];

		 	/*Clip sample values > 1.0f*/
	 		if (leftIn > 1.0f){
	 			leftIn -= 2.0f;
	 		}

	 		/*Audio Input convert to float Right Channel*/
	 		rightIn = (1.0f/32768.0f)* RecordBufferPtr[i+1];

	 		/*Clip sample values > 1.0f*/
	 		if (rightIn > 1.0f){
	 			rightIn -= 2.0f;
	 		 }

	 		/*Do some processing*/
	 		//arm_biquad_cascade_df2T_f32(&IIR_L_kick, &leftIn, &leftOut, 1);
	 		//arm_biquad_cascade_df2T_f32(&IIR_R_kick, &rightIn, &rightOut, 1);

	 		leftOut = leftIn;
	 		rightOut = rightIn;

	 		/*Convert back to int16*/
	 		PlaybackBufferPtr[i] =(int16_t) (32768.0f * leftOut);
	 		PlaybackBufferPtr[i+1] = (int16_t) (32768.0f * rightOut);

	 }

	 HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_5, GPIO_PIN_RESET);
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
