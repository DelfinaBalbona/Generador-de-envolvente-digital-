/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	attack,
	decay,
	sustain,
	release,
	off
}etapas_t;

typedef enum
{
	lineal,
	exponencial,
}modo_t;

typedef enum
{
	envolvente,
	filtro
}tipo_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

// variables para env lineal
float t_a = 0;
float t_d = 0;
float v_s = 0;
float t_r = 0;
int flag_lineal = 0;
// variables para env exp
float r_a = 0;
float r_d = 0;
//float v_s = 0;
float r_r = 0;
int flag_exponencial = 0;

uint16_t adcValues[4];
etapas_t etapasL = off;
etapas_t etapasE = off;
modo_t modo = lineal;
tipo_t tipo = envolvente;

float b_a = 0;
float b_d = 0;
float b_r = 0;

int Gate = 0;

float x = 0;
float y = 0;

float env = 0;
int flag = 0;
float pendiente_d = 0;

//filtro
float fc = 0.0;
float k = 0.0;
float fs = 500000.0;
float wcd = 0.0;
float wca = 0.0;
float g = 0.0;
float ya = 0.0;
float ya_ant = 0.0;
float yb = 0.0;
float yb_ant = 0.0;
float yc = 0.0;
float yc_ant = 0.0;
float yd = 0.0;
float yd_ant1 = 0.0;
float yd_ant2 = 0.0;
float ys = 0.0;
int flag_filtro = 0.0;
float in = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
float generador_rectas(float, float);
float generador_exp(float tau, float ini, char c);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  	HAL_TIM_Base_Start_IT(&htim1);
  	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 4);

  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
  	{
  	    tipo = envolvente;

  	    // Cambiar la frecuencia a 500 Hz
  	    __HAL_TIM_SET_AUTORELOAD(&htim1, 2000-1);    // Ajusta el periodo para 500 Hz

  	    // Reinicia el contador para aplicar los cambios
  	    __HAL_TIM_SET_COUNTER(&htim1, 0);
  	}
  	else
  	{
  	    tipo = filtro;

  	    // Cambiar la frecuencia a 500,000 Hz
  	    __HAL_TIM_SET_AUTORELOAD(&htim1, 2-1);    // Ajusta el periodo para 500,000 Hz

  	    // Reinicia el contador para aplicar los cambios
  	    __HAL_TIM_SET_COUNTER(&htim1, 0);
  	}


  	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET)
  	{
  		modo = lineal;
  	}
  	else
  	{
  		modo = exponencial;
  	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(tipo == envolvente)
	  {

		  if(modo == lineal)
		  {
			  switch(etapasL)
			  {
			  case attack:

				  float pendiente_a = 3.3/t_a;
				  b_a = 0;

				  if(flag == 1 && Gate == 1 )
				  {
					  env = generador_rectas(pendiente_a, b_a);

					  if(env >= 3.3)
					  {
						  x=0;
						  etapasL = decay;
					  }
					  else
					  {
						  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, env*4095/3.3);
						  flag = 0;
					  }
				  }
				  if(flag == 1 && Gate == 0)
				  {
					  x=0;
					  v_s = env;
					  etapasL = release;
				  }

				  break;
			  case decay:

				  float pendiente_d = (v_s - 3.3)/t_d;
				  b_d = v_s - pendiente_d * t_d;

				  if(flag == 1 && Gate == 1)
				  {
					  env = generador_rectas(pendiente_d, b_d);

					  if(env <= v_s)
					  {
						  etapasL = sustain;
						  x = 0;
					  }
					  else
					  {
						  flag = 0;
						  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, env*4095/3.3);
					  }
				  }
				  if(flag == 1 && Gate == 0)
				  {
					  x=0;
					  v_s = env;
					  etapasL = release;
				  }

				  break;
			  case sustain:

				  if(flag == 1)
				  {
					  if(Gate == 0)
					  {
						  etapasL = release;
						  x = 0;
					  }
					  else
					  {
						  //HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adcValues, 4);  // 4 canales
						  env = v_s;
						  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, env*4095/3.3);
						  flag = 0;
					  }
				  }

				  break;
			  case release:

				  float pendiente_r = (0 - v_s)/t_r;
				  b_r = - pendiente_r * t_r;

				  if(flag == 1)
				  {
					  env = generador_rectas(pendiente_r, b_r);

					  if(env <=0)
					  {
						  etapasL = off;
						  x = 0;
						  if (flag_exponencial == 1)
						  {
							  flag_exponencial = 0;
							  modo = exponencial;
						  }
					  }
					  else
					  {
						  flag = 0;
						  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, env*4095/3.3);
					  }
				  }
				  break;
			  case off:

				  if(Gate == 1)
				  {
					  etapasL = attack;
					  x = 0;
					  env = 0;
				  }
				  else
				  {
					  etapasL = off;
				  }

				  if (flag_exponencial == 1)
				  {
					  flag_exponencial = 0;
					  modo = exponencial;
				  }

				  break;
			  }
		  }
		  else if(modo == exponencial)
		  {
			  switch(etapasE)
				  {
				  case attack:

					  float tau_a = (2.2e-6)*r_a;

					  if(flag == 1 && Gate == 1)
					  {
						  env = generador_exp(tau_a, 0, 'a');

						  if(env >= 3.3)
						  {
							  etapasE = decay;
							  x = 0;
						  }
						  else
						  {
							  flag = 0;
							  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, env*4095/3.3);
						  }
					  }
					  if(flag == 1 && Gate == 0)
					  {
						  x = 0;
						  v_s = env;
						  etapasE = release;
					  }

					  break;
				  case decay:

					  float tau_d = (2.2e-6)*r_d;

					  if(flag == 1)
					  {
						  env = generador_exp(tau_d, 3.3 ,'d');

						  if(env <= v_s)
						  {
							  etapasE = sustain;
							  x = 0;
						  }
						  else
						  {
							  flag = 0;
							  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, env*4095/3.3);
						  }
					  }
					  if(flag == 1 && Gate == 0)
					  {
						  x = 0;
						  v_s = env;
						  etapasE = release;
					  }

					  break;
				  case sustain:

					  if(Gate == 0)
					  {
						  etapasE = release;
						  x = 0;
					  }
					  else
					  {
						  //HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adcValues, 4);  // 4 canales
						  env = v_s;
						  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, env*4095/3.3);
						  etapasE = sustain;
					  }

					  break;
				  case release:

					  float tau_r = (2.2e-6)*r_r;

					  if(flag == 1)
					  {
						  env = generador_exp(tau_r, v_s, 'r');

						  if(env <= 0.005)
						  {
							  etapasE = off;
							  x = 0;
							  if (flag_lineal == 1)
							  {
								  flag_lineal = 0;
								  modo = lineal;
							  }
						  }
						  else
						  {
							  flag = 0;
							  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, env*4095/3.3);
						  }
					  }

					  break;
				  case off:

					  if(Gate == 1)
					  {
						  etapasE = attack;
						  x = 0;
						  env = 0;
					  }
					  else
					  {
						  etapasE = off;
					  }

					  if (flag_lineal == 1)
					  {
						  flag_lineal = 0;
						  modo = lineal;
					  }

					  break;
				  }
		  }
	  }
	  else if(tipo == filtro)
	  {
		  if(flag_filtro == 1)
		  {
			  flag_filtro = 0;

			  fc = adcValues[0]*(20000.0/4095);
			  k = adcValues[1]*(3.99/4095);
			  in = adcValues[2]*(3.3/4095);
			  wcd = 2*M_PI*fc;
			  wca = 2*fs*tan(((1/fs)/2)*wcd);
			  g = 1 - exp(-wca/fs);

			  ya = ya_ant + g*(in - k*(yd_ant1 + yd_ant2)/2 - ya_ant);
			  yb = yb_ant + g*(ya - yb_ant);
			  yc = yc_ant + g*(yb - yc_ant);
			  yd = yd_ant1 + g*(yc - yd_ant1);
			  ys = yd * (1+k);

			  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, ys*4095/3.3);

			  ya_ant = ya;
			  yb_ant = yb;
			  yc_ant = yc;
			  yd_ant2 = yd_ant1;
			  yd_ant1 = yd;
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 80-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

float generador_rectas(float m, float b)
{

	y = m*x+b;

	return y;
}

float generador_exp(float tau, float ini, char c)
{

	if(c == 'a')
	{
		y = 4.95*(1-exp(-(1/tau)*x));
	}
	else if(c == 'd')
	{
		y = v_s - (v_s - ini)*exp(-(1/tau)*x);
	}
	else
	{
		y = ini*exp(-(1/tau)*x);
	}

	return y;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if (GPIO_Pin == GPIO_PIN_4)
	{

		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET)
		{
			Gate = 1;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);    //GATE (A5)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);  //TRIG (A6)
			HAL_Delay(10);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		}
		else
		{
			Gate = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		}

		//HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adcValues, 4);  // 4 canales

	}

	if(GPIO_Pin == GPIO_PIN_1)
	{
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET)
		{
			flag_lineal = 1;
			flag_exponencial = 0;
		}
		else
		{
			flag_exponencial = 1;
			flag_lineal = 0;
		}
	}

	if(GPIO_Pin == GPIO_PIN_6)
		{
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
			{
				tipo = envolvente;
				//HAL_TIM_Base_Stop_IT(&htim2);
				//HAL_TIM_Base_Start_IT(&htim1);

		  	    // Cambiar la frecuencia a 500 Hz
		  	    __HAL_TIM_SET_AUTORELOAD(&htim1, 2000-1);    // Ajusta el periodo para 500 Hz

		  	    // Reinicia el contador para aplicar los cambios
		  	    __HAL_TIM_SET_COUNTER(&htim1, 0);
			}
			else
			{
				tipo = filtro;
				//HAL_TIM_Base_Stop_IT(&htim1);
				//HAL_TIM_Base_Start_IT(&htim2);

		  	    // Cambiar la frecuencia a 500,000 Hz
		  	    __HAL_TIM_SET_AUTORELOAD(&htim1, 2-1);    // Ajusta el periodo para 500,000 Hz

		  	    // Reinicia el contador para aplicar los cambios
		  	    __HAL_TIM_SET_COUNTER(&htim1, 0);
			}
		}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
    	flag = 1;
        x += 0.002;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
    	if(tipo == envolvente)
    	{
    		if (modo == lineal)
    		{
    			t_a = adcValues[0]*(3.02/4095);  // attack
    			if(t_a <= 0.4)
    			{
    				t_a = 0.4;
    			}
    			t_d = adcValues[1]*(9.5/4095);  // decay
    			if(t_d <= 0.4)
    			{
    				t_d = 0.4;
    			}
    			v_s = adcValues[2]*(3.3/4095);  // sustain
    			t_r = adcValues[3]*(11.0/4095);  // Release
    			if(t_r <= 0.7)
    			{
    				t_r = 0.7;
    			}
    		}
    		if (modo == exponencial)
    		{
    			r_a = adcValues[0]*(1400000.0/4095);  // attack
    			if(r_a <= 33100.0)
    			{
    				r_a = 33100.0;
    			}
    			r_d = adcValues[1]*(1000000.0/4095);  // decay
    			if(r_d <= 20000.0)
    			{
    				r_d = 20000.0;

    			}
    			v_s = adcValues[2]*(3.3/4095);  // sustain
    			r_r = adcValues[3]*(1600000.0/4095);  // Release
    			if(r_r <= 1000.0)
    			{
    				r_r = 1000.0;
    			}
    		}
    	}
    	if (tipo == filtro)
    	{
    		flag_filtro = 1;
    	}
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
