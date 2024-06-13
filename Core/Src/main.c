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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "Codec.h"
#include "dsp/filtering_functions.h"
//#include "iir_break"	// iir filter coefficients
//#include "iir_kick_96.h"			//#################################
#include "iir_filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RECORD_BUFFER_SIZE 4096 // 1024*4=4096 bei 96khz (x2stereo + x2 double buffer = x4)                 (512*4=2048 bei 48khz)
#define MIN_ENERGY 1000 	//2000
#define NUM_SHORT_AVR 4 	//4
#define NUM_BREAK_AVR 100 	//100
#define NUM_BEAT_HIST 15 	//15
#define KICK_THRESH 4.5 	//4.5
#define KICK_VAL 7000		//7000   //old 9000
#define KICK_MAX 20000 		//20000
#define BREAK_DIF 0.8 		//0.8
#define BREAK_THRESH 7000 	//7000
#define BREAK_SLOPE 1000 		//10
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//float32_t state_L_break[2*STAGES];	// state arrays
//float32_t state_R_break[2*STAGES];
//float32_t state_L_kick[2*STAGES];
//float32_t state_R_kick[2*STAGES];
//float32_t state_kick[2*STAGES];//old with iir_kick_96							//#################################
//float32_t state_L_melody[2*STAGES];
//float32_t state_R_melody[2*STAGES];

float32_t state_break[2*STAGES];
float32_t state_kick[4*STAGES];//4*5Stages (2*10stages)

//arm_biquad_cascade_df2T_instance_f32 IIR_L_break;	 // create IIR instances
//arm_biquad_cascade_df2T_instance_f32 IIR_R_break;
//arm_biquad_cascade_df2T_instance_f32 IIR_L_kick;
//arm_biquad_cascade_df2T_instance_f32 IIR_R_kick;
//arm_biquad_cascade_df2T_instance_f32 IIR_L_melody;
//arm_biquad_cascade_df2T_instance_f32 IIR_R_melody;
arm_biquad_cascade_df2T_instance_f32 IIR_break; 			//#################################
arm_biquad_cascade_df2T_instance_f32 IIR_kick; 			//#################################

//variables
uint16_t 	kick_ema = 0;
uint16_t 	break_ema = 0;
int16_t  	slope_ema = 0;
uint16_t 	slope_old_energy = 0;
uint32_t 	kick_value = 0;
uint32_t 	break_value = 0;
uint16_t 	kick_avr = 0;
uint16_t    break_avr = 0;
uint16_t 	kick_transformed = 0;

//arrays
uint16_t 	break_avr_buffer[NUM_BREAK_AVR];
uint16_t 	short_avr_buffer[NUM_SHORT_AVR];
uint8_t		beat_hist_buffer[NUM_BEAT_HIST];

int16_t RecordBuffer[RECORD_BUFFER_SIZE];
int16_t PlaybackBuffer[RECORD_BUFFER_SIZE];
int16_t left_channel[RECORD_BUFFER_SIZE];
int16_t right_channel[RECORD_BUFFER_SIZE];
static volatile int16_t *RecordBufferPtr = &RecordBuffer;
static volatile int16_t *PlaybackBufferPtr = &PlaybackBuffer;

volatile uint8_t dataReadyFlag;

/*Buffer and Variables ADC1 Channel on Pins A0, A1, A2*/
uint16_t AdcValues[3];
uint16_t AdcChannel1, AdcChannel2, AdcChannel3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void ProcessData();

uint16_t cal_short_avr(uint16_t energy);
uint16_t transform_exp(uint16_t value, uint16_t max_value, uint16_t exp);
uint16_t cal_ema(uint16_t new_value, float alpha, uint16_t previous_ema);
uint8_t beat_history(uint8_t is_beat);
uint16_t cal_break_avr(uint16_t energy);
int16_t cal_slope(uint16_t energy);

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
	//arm_biquad_cascade_df2T_init_f32(&IIR_L_kick, STAGES, ba_coeff, state_L_kick);
	//arm_biquad_cascade_df2T_init_f32(&IIR_R_kick, STAGES, ba_coeff, state_R_kick);
	//arm_biquad_cascade_df2T_init_f32(&IIR_kick, STAGES, ba_coeff, state_kick); //old with iir_kick_96	   //#################################
	arm_biquad_cascade_df2T_init_f32(&IIR_break, STAGES,   ba_coeff_break, state_break);
	arm_biquad_cascade_df2T_init_f32(&IIR_kick,  2*STAGES, ba_coeff_kick,  state_kick);
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
	  		  dataReadyFlag = 0;
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

	//HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_13, GPIO_PIN_SET);//LD1
	HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_5, GPIO_PIN_SET);	//LD2
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);//LD3

	 volatile static float leftIn, rightIn;
	 volatile static float monoIn;
	 volatile static float kickOut;
	 volatile static float breakOut;
	 volatile static float leftOut, rightOut;
	 volatile static float kick_leftOut, kick_rightOut;
	// uint16_t kick = 0;
	 //uint16_t right = 0;

	 //volatile static float break_leftOut, break_rightOut;
	 kick_value = 0;
	 break_value = 0;

	 for (uint16_t i =0; i<(RECORD_BUFFER_SIZE/2)-1 ; i+=2){


	 		/*Audio Input convert to float Left Channel*/
		 	leftIn = (1.0f/32768.0f)* RecordBufferPtr[i];

		 	/*Clip sample values > 1.0f*/
	 		if (leftIn > 1.0f){
	 			leftIn -= 2.0f;
	 			//HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_13, GPIO_PIN_SET);
	 		}

	 		/*Audio Input convert to float Right Channel*/
	 		rightIn = (1.0f/32768.0f)* RecordBufferPtr[i+1];

	 		/*Clip sample values > 1.0f*/
	 		if (rightIn > 1.0f){
	 			rightIn -= 2.0f;
	 			//HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_13, GPIO_PIN_SET);
	 		 }

	 		monoIn = (1.0f/32768.0f) * ((RecordBufferPtr[i] + RecordBufferPtr[i+1]) / 2);

	 		/*Do some processing*/
	 		//arm_biquad_cascade_df2T_f32(&IIR_L_kick, &leftIn, &kick_leftOut, 1);
	 		//arm_biquad_cascade_df2T_f32(&IIR_R_kick, &rightIn, &kick_rightOut, 1);

	 		arm_biquad_cascade_df2T_f32(&IIR_kick, &monoIn, &kickOut, 1);			//#################################
	 		arm_biquad_cascade_df2T_f32(&IIR_break, &monoIn, &breakOut, 1);

	 		//kick_leftOut  = fabs(leftOut);
	 		//kick_rightOut = fabs(rightOut);

	 		//left  = (uint16_t) (65536.0f * kick_leftOut);
	 		//right = (uint16_t) (65536.0f * kick_rightOut);

	 		//left  = (32768.0f * kick_leftOut);
	 		//left  = (32768.0f * fabs(kickOut));
	 		//right  = (32768.0f * fabs(breakOut));


	 		//kick_value += (left + right) / 2;
	 		//kick_value += (left + right);
	 		kick_value 	+= ((32768.0f * fabs(kickOut)) 	* 2);
	 		break_value += ((32768.0f * fabs(breakOut)) * 2);
	 		//break_value += (uint16_t)((fabs(break_leftOut) + fabs(break_rightOut)) / 2);

	 		//leftOut = leftIn;
	 		//rightOut = rightIn; //return audio in to audio out

	 		/*Convert back to int16*/
	 		PlaybackBufferPtr[i] 	= (int16_t) (1000.0f * leftIn);  //(1000.0f * leftIn)
	 		PlaybackBufferPtr[i+1] 	= (int16_t) (1000.0f * rightIn); //(32768.0f * rightIn)

	 		//kick_value = fabs(PlaybackBufferPtr[i]) + fabs(PlaybackBufferPtr[i+1]);


	 }

	 kick_value = kick_value / (RECORD_BUFFER_SIZE/4);
	 break_value = break_value / (RECORD_BUFFER_SIZE/4);

	 kick_avr  			= cal_short_avr(kick_value);
	 kick_transformed  	= transform_exp(kick_avr,KICK_VAL,4); //with old max energy: transform_exp(kick_avr,9000,4)
	 kick_ema          	= cal_ema(kick_transformed, 0.01, kick_ema);


	 if ((KICK_VAL < kick_transformed) && (kick_transformed > (kick_ema * KICK_THRESH))){ //and not (break_energy_value < break_threshold): # detect beat
		 if(beat_history(1) == 1){
		 	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);//LD3
		 }
		 else{
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		 }
	 }
	 else{
		 beat_history(0);
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	 }

	 break_avr         	= cal_break_avr(break_value);
	 break_ema         	= cal_ema(break_avr, 0.06, break_ema);

	 //if ((((BREAK_DIF < (break_avr / break_ema)) && (10 < cal_slope(break_ema))) || (break_ema > BREAK_THRESH)) && (!(break_ema < MIN_ENERGY) || !(BREAK_DIF > (break_avr/break_ema)))){
	 //if (break_ema > BREAK_THRESH){
	 if ((((BREAK_DIF < (break_avr / break_ema)) && (BREAK_SLOPE < cal_slope(break_ema))) || (break_ema > BREAK_THRESH)) && (!(break_ema < MIN_ENERGY))){
		 HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_13, GPIO_PIN_SET);
	 }
	 else{
		 HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_13, GPIO_PIN_RESET);
	 }

	 //HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_13, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_5, GPIO_PIN_RESET);
	 //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}

uint16_t cal_short_avr(uint16_t energy){
	for (uint8_t val = NUM_SHORT_AVR; val > 0; val--) {
		if (val > 1) {
			short_avr_buffer[val - 1] = short_avr_buffer[val - 2]; //move value to next position
	    }
		else {
	    	short_avr_buffer[val - 1] = energy; // add new value to first position
	    }
	}
	uint16_t kick_average = 0;
	for (uint8_t n = 0; n < NUM_SHORT_AVR; n++) {
		kick_average += ((1.0f/NUM_SHORT_AVR) * short_avr_buffer[n]);
	}
	return kick_average;
}

uint16_t cal_break_avr(uint16_t energy){
	for (uint8_t val = NUM_BREAK_AVR; val > 0; val--) {
		if (val > 1) {
			break_avr_buffer[val - 1] = break_avr_buffer[val - 2]; //move value to next position
	    }
		else {
			break_avr_buffer[val - 1] = energy; // add new value to first position
	    }
	}
	uint16_t break_average = 0;
	for (uint8_t n = 0; n < NUM_BREAK_AVR; n++) {
		break_average += ((1.0f/NUM_BREAK_AVR) * break_avr_buffer[n]);
	}
	return break_average;
}

uint16_t transform_exp(uint16_t value, uint16_t max_value, uint16_t exp){
	uint16_t transformed_value = (pow(value, exp) / pow(max_value, (exp-1)));
	if(transformed_value > KICK_MAX){
		transformed_value = KICK_MAX;
	}
	return transformed_value;
}

uint16_t cal_ema(uint16_t new_value, float alpha, uint16_t previous_ema){
	uint16_t ema = alpha * new_value + (1 - alpha) * previous_ema;
	return ema;
}

int16_t cal_slope(uint16_t energy){
    int16_t slope = energy - slope_old_energy;
    slope_old_energy = energy;
    slope_ema = cal_ema(slope, 0.06, slope_ema);
    return slope_ema;
}

uint8_t beat_history(uint8_t is_beat) {
    // Überprüfen, ob is_beat gültig ist
    if (is_beat != 0 && is_beat != 1) {
        is_beat = 0;
    }

    // Pufferwerte verschieben und neuen Wert hinzufügen
    for (uint8_t val = NUM_BEAT_HIST; val > 0; val--){
        if (val > 1) {
            if (beat_hist_buffer[val - 2] != 0) {
            	is_beat = 0;
            }
            beat_hist_buffer[val - 1] = beat_hist_buffer[val - 2]; // Wert zur nächsten Position verschieben
        }
        else {
            beat_hist_buffer[val - 1] = is_beat; // Neuen Wert an erster Position hinzufügen
        }
    }

    return is_beat; // Rückgabe des aktuellen beat_hist
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

