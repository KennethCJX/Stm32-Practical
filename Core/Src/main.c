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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RCC.h"
#include "GPIO.h"
#include "Timer.h"
#include "NVIC.h"
#include "Adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Practical_Experiment1();
void Practical_Experiment2();
void Practical_Experiment3();
void Practical_Experiment4();
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
  rccResetAndUnresetGpio(Rcc_GPIOB);
  rccUnresetAndEnableTimer(RCC_TIM4);
  gpioConfigurePin(gpioB, 8, GPIO_ALT_FUNC |GPIO_PUSH_PULL| AF_2);

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Practical_Experiment1();
	  Practical_Experiment2();
	  //Practical_Experiment3();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : UserButton_Pin */
  GPIO_InitStruct.Pin = UserButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UserButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Practical_Experiment1(){
	  rccResetAndUnresetGpio(Rcc_GPIOB);
  	  rccUnresetAndEnableTimer(RCC_TIM4);
	  gpioConfigurePin(gpioB, 6, GPIO_ALT_FUNC | AF_2);		 					//tim4_ch1(button)
	  gpioConfigurePin(gpioB, 8, GPIO_ALT_FUNC |GPIO_PUSH_PULL| AF_2);			//tim4_ch3(LED)
	  tmrConfigure(tmr4, TMR_CEN_EN | TMR_ARP_EN | TMR_TS_TI1FP1 | TMR_SLAVE_GATED);
	  tmrConfigureEventGeneration(tmr4, TMR_UG_EN);
	  //tmrConfigureCaptureCompare(tmr4, 3, TMR_CC3S_OUTPUT | TMR_OC3M_PWM_MODE1 | TMR_CC3_OUTPUT_ON | TMR_CC3P_OC1_HIGH | TMR_OC3_PRELOAD_EN);
	  CaptureCompare_config_P1(tmr4);
	  set_PWM_Freq(tmr4, 1);
	  set_PWM_Duty_Cycle(tmr4, 3, 80);
	  //set_PWM_Duty_Cycle(tmr4, 3, 20);
}

void Practical_Experiment2(){
	 volatile int buttonstate;
	 buttonstate = gpioReadPin(gpioB, 6);
	  if(buttonstate)
		  //tmrConfigureCaptureCompare(tmr4, 3, TMR_CC3S_OUTPUT | TMR_OC3M_FORCE_ACTIVE | TMR_CC3_OUTPUT_ON | TMR_CC3P_OC1_HIGH);
		  CaptureCompare_config_P2_state1(tmr4);
	  else
		  //tmrConfigureCaptureCompare(tmr4, 3, TMR_CC3S_OUTPUT | TMR_OC3M_FORCE_INACTIVE | TMR_CC3_OUTPUT_ON | TMR_CC3P_OC1_HIGH);
		  CaptureCompare_config_P2_state2(tmr4);
}

void Practical_Experiment3(){
	static int counter = 0;
	rccUnresetAndEnableGpio(Rcc_GPIOB);
	rccUnresetAndEnableTimer(RCC_TIM4);

	gpioConfigurePin(gpioB, 8, GPIO_ALT_FUNC |GPIO_PUSH_PULL| GPIO_HIGH_SPEED | AF_2);		//tim4_ch3(LED)
	tmrConfigure(tmr4, TMR_URS | TMR_UIE_EN);
	tmrConfigureEventGeneration(tmr4, TMR_UG_EN);
	//tmrConfigureCaptureCompare(tmr4, 3, TMR_CC3S_OUTPUT | TMR_OC3M_TOGGLE | TMR_CC3_OUTPUT_ON | TMR_CC3P_OC1_HIGH);
	CaptureCompare_config_P3(tmr4);
	nvicEnableIrq(30);																		//Timer 4 interrupt number is 30

	set_PWM_Freq(tmr4, 1);														//1Hz
	set_PWM_Duty_Cycle(tmr4, 3, 50);											//50% duty cycle

	set_PWM_Freq(tmr4, 2);														//2Hz
	set_PWM_Duty_Cycle(tmr4, 3, 50);											//50% duty cycle

	set_PWM_Freq(tmr4, 4);														//4Hz
	set_PWM_Duty_Cycle(tmr4, 3, 50);											//50% duty cycle

}

void Practical_Experiment4(){
	  /*
	  rccUnresetAndEnableADC(RCC_ADC1);
	  Adc_setSampleTime(ADC1, 0, SAMPLE_3_CYCLE);										//channel 0, sampling time 3 cycle
	  Adc_setRegularChannel(ADC1, TIMER4_CC4_EVENT_REG  | TRIG_DET_RISING_REG);
	  Adc_setChannelSequence(ADC1, 0, 1);												//channel 0, scan only 1 channel
	  nvicEnableIrq(18);																//Enable ADC interrupt
	  Adc_Configuration(ADC1, EOC_INTERRUPT_EN| RESOLUTION_12BIT | ADC_ON | CONT_SINGLE_CONV  | RIGHT_ALIGN);
	  */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
