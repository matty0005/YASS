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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "ili9163.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RTP_CS_GPIO GPIOB
#define LCD_CS_GPIO GPIOB
#define RTP_PIN GPIO_PIN_12
#define LCD_PIN GPIO_PIN_9

#define PID_KP 4.0f
#define PID_KI 0.045f
#define PID_KD 0.0475f

#define PID_TAU 0.02f
#define PID_SAMPLE_TIME 0.12f

#define SLEEP_TEMP 150

#define SAFETY_COUNTER 5


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

/* Definitions for tempMeasure */
osThreadId_t tempMeasureHandle;
const osThreadAttr_t tempMeasure_attributes = {
  .name = "tempMeasure",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ironControll */
osThreadId_t ironControllHandle;
const osThreadAttr_t ironControll_attributes = {
  .name = "ironControll",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t safetyCutoffHandle;
const osThreadAttr_t safetyCutoff_attributes = {
  .name = "safetyCutoff",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for screen */
osThreadId_t screenHandle;
const osThreadAttr_t screen_attributes = {
  .name = "screen",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for controls */
osThreadId_t controlsHandle;
const osThreadAttr_t controls_attributes = {
  .name = "controls",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for zeroCrossingTas */
osThreadId_t zeroCrossingTasHandle;
const osThreadAttr_t zeroCrossingTas_attributes = {
  .name = "zeroCrossingTas",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for measureTimerTas */
osThreadId_t measureTimerTasHandle;
const osThreadAttr_t measureTimerTas_attributes = {
  .name = "measureTimerTas",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for zeroCrossingSemaphore */
osSemaphoreId_t zeroCrossingSemaphoreHandle;
const osSemaphoreAttr_t zeroCrossingSemaphore_attributes = {
  .name = "zeroCrossingSemaphore"
};
/* USER CODE BEGIN PV */

uint8_t SPI_DMA_FL = 0;
uint32_t SPI_DMA_CNT=1;
uint16_t setTemp = 350;
bool canSwitch = false;
bool heaterOn = false;
bool measuringTemp = false;
bool measureInProgress = false;
bool sleepMode = false;
uint16_t currentTemp = 0;
uint16_t lastTemp = 0;


uint16_t powerLevel = 0;

PIDController ironPID;

uint16_t a = 0;
uint16_t b = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_DMA_Init(void);
void tempMeas(void *argument);
void ironControl(void *argument);
void safetyCutoff(void *argument);
void screenTask(void *argument);
void controlTask(void *argument);
void zeroCrossing(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) // Your TxCpltCallback
{
    SPI_DMA_CNT--;
    if (SPI_DMA_CNT == 0) {
        HAL_SPI_DMAStop(&hspi2);
        SPI_DMA_CNT = 1;
        SPI_DMA_FL = 1;
    }
}


uint8_t map(uint8_t value, uint8_t max, uint8_t newMax) {
	return ((value * newMax) / max);
}

uint16_t getTemp() {
	return (uint16_t) ((float)currentTemp * 1000 / 1241) * 0.106 + 13;
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
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */

  ILI9163_init(0);

  PID_init(&ironPID);

  ironPID.max = 100;
  ironPID.min = 0;
  ironPID.Kp = PID_KP;
  ironPID.Ki = PID_KI;
  ironPID.Kd = PID_KD;
  ironPID.tau = PID_TAU;
  ironPID.T = PID_SAMPLE_TIME;



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of zeroCrossingSemaphore */
  zeroCrossingSemaphoreHandle = osSemaphoreNew(1, 1, &zeroCrossingSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of tempMeasure */
  tempMeasureHandle = osThreadNew(tempMeas, NULL, &tempMeasure_attributes);

  /* creation of ironControll */
  ironControllHandle = osThreadNew(ironControl, NULL, &ironControll_attributes);

  safetyCutoffHandle = osThreadNew(safetyCutoff, NULL, &safetyCutoff_attributes);


  /* creation of screen */
  screenHandle = osThreadNew(screenTask, NULL, &screen_attributes);

  /* creation of controls */
  controlsHandle = osThreadNew(controlTask, NULL, &controls_attributes);

  /* creation of zeroCrossingTas */
  zeroCrossingTasHandle = osThreadNew(zeroCrossing, NULL, &zeroCrossingTas_attributes);


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA3 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_tempMeas */
/**
  * @brief  Function implementing the tempMeasure thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_tempMeas */
void tempMeas(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	  measuringTemp = true;
	  osDelay(20);

	  if (heaterOn) {
		  osDelay(1);
		  while (heaterOn) {
			  measuringTemp = true;
			  osDelay(1);
		  }
	  }

	  uint32_t sum = 0;
	  uint8_t avgCount = 10;

	  for (int i = 0; i < avgCount; i++) {
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, 100);
		  sum += HAL_ADC_GetValue(&hadc1);
	  }



	  currentTemp = sum / avgCount;

	  measuringTemp = false;
	  osDelay(100);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ironControl */
/**
* @brief Function implementing the ironControll thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ironControl */
void ironControl(void *argument)
{
  /* USER CODE BEGIN ironControl */

	// Add startup delay.
	osDelay(3000);

	// If temperature not reading correctly, reboot
	if (getTemp() > 100) {
		HAL_NVIC_SystemReset();
	}
  /* Infinite loop */
	for (;;) {
		powerLevel = (uint16_t)PID_update(&ironPID, sleepMode ? SLEEP_TEMP : setTemp, getTemp());

		// Turn off heater if measuring temperature
		if (measuringTemp) {
			heaterOn = false;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			osDelay(1);
		} else {

			// Wait for zero crossing
			while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) != GPIO_PIN_SET) {
				;
			}
			osDelay(8);

			heaterOn = true;
			uint16_t powerDelay = powerLevel / 10;

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			osDelay(powerDelay* 10);


			while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) != GPIO_PIN_SET) {
				;
			}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

			osDelay(100 - (powerDelay * 10));

		}
	}
  /* USER CODE END ironControl */
}


/* USER CODE BEGIN Header_safetyCutoff */
/**
* @brief Function implementing the safetyCutoff thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_safetyCutoff */
void safetyCutoff(void *argument)
{
  /* USER CODE BEGIN ironControl */
  /* Infinite loop */
	for (;;) {


		// If heater is on at full power for over a second without a temperature change....

		if (powerLevel < 90) { // full power >= 90
			osDelay(100);
			continue;
		}

		// Power level is over 90%
		lastTemp =  getTemp();
		osDelay(50);

		// Check to make sure temp is changing
		uint8_t sameCounter = 0;

		for (uint8_t i = 0; i < SAFETY_COUNTER; i++) {

			if (abs(lastTemp - getTemp()) <= 1 && powerLevel >= 90) {
				sameCounter++;
			}

			lastTemp =  getTemp();
			osDelay(100);
		}

		// reset the MCU if that is the case.
		if (sameCounter >= (SAFETY_COUNTER - 1)) {
			// turn off iron
			heaterOn = false;
			measuringTemp = true;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

			// Shutdown
			HAL_NVIC_SystemReset();
		}
	}
  /* USER CODE END ironControl */
}



/* USER CODE BEGIN Header_screenTask */
/**
* @brief Function implementing the screen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_screenTask */
void screenTask(void *argument)
{
  /* USER CODE BEGIN screenTask */
  /* Infinite loop */
  for(;;)
  {

	  HAL_GPIO_WritePin(RTP_CS_GPIO, RTP_PIN, 1);



	  ILI9163_newFrame();
	  ILI9163_fillRect(0,  ILI9163_HEIGHT - 20, map(powerLevel, 100, ILI9163_WIDTH), ILI9163_HEIGHT, ORANGE);
	  ILI9163_drawRect(0,  ILI9163_HEIGHT - 20, ILI9163_WIDTH, ILI9163_HEIGHT, 1, ORANGE);


	  if (sleepMode) {
		  ILI9163_fillRect(0,  0, (4 * ILI9163_WIDTH) / 10, 28, GREEN);
		  ILI9163_drawStringF(4, 5, Font_11x18, WHITE, "%s", "SLEEP");
	  }



	  ILI9163_drawStringF(ILI9163_WIDTH - 80, 10, Font_7x10, BLACK, "Set: %hu C", setTemp);

//	  ILI9163_drawStringF(80, 80, Font_7x10, BLACK, "%humV", currentTemp * 1000 / 1241);
//	  ILI9163_drawStringF(80, 60, Font_7x10, BLACK, "%hu%", powerLevel);
//	  ILI9163_drawStringF(100, 60, Font_7x10, BLACK, "%hu%", (powerLevel / 10)*10);

	  ILI9163_drawStringF(45, 55, Font_16x26, BLACK, "%huC", getTemp());


	  ILI9163_render();

	  osDelay(5);
  }
  /* USER CODE END screenTask */
}

/* USER CODE BEGIN Header_controlTask */
/**
* @brief Function implementing the controls thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_controlTask */
void controlTask(void *argument)
{
  /* USER CODE BEGIN controlTask */
  /* Infinite loop */
  for(;;)
  {
	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET) {
	  		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_RESET) {
	  			while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_RESET){};
	  			while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET) {};

	  			setTemp -= 5;
	  			osDelay(1);



	  		}
	  		else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET) {
	  			while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET) {};
				while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET) {};
				while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_RESET) {};

				setTemp += 5;



				osDelay(1);


			}
	  	}
	  if (setTemp >= 450) {
		  setTemp = 450;
	  } else if (setTemp <= 0) {
		  setTemp = 0;
	  }

    osDelay(1);
  }
  /* USER CODE END controlTask */
}

/* USER CODE BEGIN Header_zeroCrossing */
/**
* @brief Function implementing the zeroCrossingTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_zeroCrossing */
void zeroCrossing(void *argument)
{
  /* USER CODE BEGIN zeroCrossing */
  /* Infinite loop */
  for(;;)
  {
	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) {
		  sleepMode = true;
	  } else {
		  sleepMode = false;
	  }
//	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET) {
//		  // Can switch here
//		  canSwitch = true;
////		  osSemaphoreRelease(zeroCrossingSemaphoreHandle);
//		  a++;
//
//	  } else {
//		  canSwitch = false;
////		  osSemaphoreAcquire(zeroCrossingSemaphoreHandle, osWaitForever);
//		  b++;
//		  // Cannot switch here
//	  }
	  osDelay(5);
  }
  /* USER CODE END zeroCrossing */
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
