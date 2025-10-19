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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "x_nucleo_ihmxx.h"
#include "x_nucleo_ihm03a1_stm32f4xx.h"
#include "powerstep01.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 128
#define MAX_MOTOR_STEPS 10000L  // Define maximum allowed steps
#define MIN_MOTOR_STEPS -10000L // Define minimum allowed steps
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile uint16_t gLastError;
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t tx_buffer[TX_BUFFER_SIZE];
static volatile uint32_t rx_index = 0;
static volatile uint8_t command_ready = 0;

static volatile int32_t motor0_target = 0;
static volatile int32_t motor1_target = 0;
static uint8_t uart_rx_byte;
static uint8_t command_buffer[RX_BUFFER_SIZE];
static volatile uint32_t cmd_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void MyBusyInterruptHandler(void);
static void MyFlagInterruptHandler(void);
static void MyErrorHandler(uint16_t error);

static void UART_SendString(const char *str);
static void UART_ReceiveChar(uint8_t ch);
static void ParseCommand(void);
static void SendMotorStatus(uint8_t motor_id);
static void SendError(uint8_t motor_id, const char *error_type, const char *description);
static void MoveMotorsToPosition(int32_t motor0_pos, int32_t motor1_pos);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, 2);
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, NULL);
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, NULL);
  BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);
  BSP_MotorControl_AttachBusyInterrupt(MyBusyInterruptHandler);
  BSP_MotorControl_AttachErrorHandler(MyErrorHandler);
  BSP_MotorControl_CmdResetPos(0);
  BSP_MotorControl_CmdResetPos(1);
  HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);

  UART_SendString("SYSTEM,READY\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (command_ready)
	      {
	        command_ready = 0;
	        ParseCommand();
	      }

	      // Check motor status
	      SendMotorStatus(0);
	      SendMotorStatus(1);

	      HAL_Delay(1000);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STBY_RESET_Pin_GPIO_Port, STBY_RESET_Pin_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_Pin_GPIO_Port, CS_Pin_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : STBY_RESET_Pin_Pin */
  GPIO_InitStruct.Pin = STBY_RESET_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STBY_RESET_Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLAG_Pin_Pin */
  GPIO_InitStruct.Pin = FLAG_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FLAG_Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin_Pin */
  GPIO_InitStruct.Pin = CS_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_Pin_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void UART_SendString(const char *str)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

void UART_ReceiveChar(uint8_t ch)
{
  // Check for buffer overflow BEFORE writing
  if (cmd_index >= RX_BUFFER_SIZE - 1)
  {
    // Buffer full - reset and report error
    cmd_index = 0;
    SendError(0, "BUFFER_OVERFLOW", "Command too long");
    return;
  }

  if (ch == '\n' || ch == '\r')
  {
    if (cmd_index > 0)  // Only process if we have data
    {
      command_buffer[cmd_index] = '\0';
      // Copy to rx_buffer for processing
      memcpy(rx_buffer, command_buffer, cmd_index + 1);
      command_ready = 1;
    }
    cmd_index = 0;
  }
  else
  {
    command_buffer[cmd_index++] = ch;
  }
}

void SendMotorStatus(uint8_t motor_id)
{
  uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(motor_id);
  int32_t position = BSP_MotorControl_GetPosition(motor_id);

  // Extract status flags
  uint8_t is_moving = ((statusRegister & POWERSTEP01_STATUS_BUSY) == 0) ? 1 : 0;
  uint8_t has_error = 0;

  // Check for any error conditions
  if (((statusRegister & POWERSTEP01_STATUS_UVLO) == 0) ||
      ((statusRegister & POWERSTEP01_STATUS_OCD) == 0) ||
      ((statusRegister & POWERSTEP01_STATUS_STALL_A) == 0) ||
      ((statusRegister & POWERSTEP01_STATUS_STALL_B) == 0) ||
      (statusRegister & POWERSTEP01_STATUS_CMD_ERROR) ||
      (statusRegister & POWERSTEP01_STATUS_TH_STATUS))
  {
    has_error = 1;
  }

  // Send compact status
  snprintf((char *)tx_buffer, TX_BUFFER_SIZE,
           "STATUS,%d,%ld,%d,%d,0x%04X\n",
           motor_id, position, is_moving, has_error, statusRegister);
  UART_SendString((const char *)tx_buffer);
}

void SendError(uint8_t motor_id, const char *error_type, const char *description)
{
  snprintf((char *)tx_buffer, TX_BUFFER_SIZE, "ERROR,%d,%s,%s\n", motor_id, error_type, description);
  UART_SendString((const char *)tx_buffer);
}

void ParseCommand(void)
{
  char *cmd = (char *)rx_buffer;

  // Ignore empty commands
  if (strlen(cmd) == 0)
  {
    return;
  }

  // Debug: Echo back what was received (safely truncated)
  snprintf((char *)tx_buffer, TX_BUFFER_SIZE, "DEBUG,RX:[%.20s]\n", cmd);
  UART_SendString((const char *)tx_buffer);

  if (strncmp(cmd, "MOVE", 4) == 0)
  {
    int32_t motor0_steps, motor1_steps;
    if (sscanf(cmd, "MOVE,%ld,%ld", &motor0_steps, &motor1_steps) == 2)
    {
      // Validate step values
      if (motor0_steps < MIN_MOTOR_STEPS || motor0_steps > MAX_MOTOR_STEPS ||
          motor1_steps < MIN_MOTOR_STEPS || motor1_steps > MAX_MOTOR_STEPS)
      {
        SendError(0, "RANGE_ERROR", "Step value out of range");
        return;
      }

      // Queue both motors
      motor0_target = motor0_steps;
      motor1_target = motor1_steps;
      MoveMotorsToPosition(motor0_target, motor1_target);

      // Send confirmation with actual values
      snprintf((char *)tx_buffer, TX_BUFFER_SIZE, "OK,MOVE,%ld,%ld\n",
               motor0_steps, motor1_steps);
      UART_SendString((const char *)tx_buffer);
    }
    else
    {
      SendError(0, "PARSE_ERROR", "Invalid MOVE format");
    }
  }
  else if (strncmp(cmd, "STOP", 4) == 0)
  {
    UART_SendString("OK,STOPPING\n");
    BSP_MotorControl_CmdSoftStop(0);
    BSP_MotorControl_CmdSoftStop(1);
  }
  else if (strncmp(cmd, "HOME", 4) == 0)
  {
    // Move to position 0,0
    MoveMotorsToPosition(0, 0);
    UART_SendString("OK,HOMING\n");
  }
  else if (strncmp(cmd, "STATUS", 6) == 0)
  {
    // Check for CLEAR flag
    if (strstr(cmd, "CLEAR") != NULL)
    {
      BSP_MotorControl_CmdGetStatus(0); // Clear error flags
      BSP_MotorControl_CmdGetStatus(1);
      UART_SendString("OK,STATUS_CLEARED\n");
    }
    SendMotorStatus(0);
    SendMotorStatus(1);
  }
  else if (strncmp(cmd, "RESET", 5) == 0)
  {
    BSP_MotorControl_CmdGetStatus(0); // Clear error flags
    BSP_MotorControl_CmdGetStatus(1);
    BSP_MotorControl_CmdResetPos(0);
    BSP_MotorControl_CmdResetPos(1);
    UART_SendString("OK,RESET_COMPLETE\n");
  }
  else
  {
    // Safely truncate unknown command in error message
    snprintf((char *)tx_buffer, TX_BUFFER_SIZE, "ERROR,UNKNOWN,%.20s\n", cmd);
    UART_SendString((const char *)tx_buffer);
  }
}

void MoveMotorsToPosition(int32_t motor0_pos, int32_t motor1_pos)
{
	BSP_MotorControl_QueueCommands(0,POWERSTEP01_GO_TO, motor0_pos);
	BSP_MotorControl_QueueCommands(1,POWERSTEP01_GO_TO, motor1_pos);
	BSP_MotorControl_SendQueuedCommands();
}

void CheckMotorErrors(uint8_t motor_id)
{
  uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(motor_id);

  if ((statusRegister & POWERSTEP01_STATUS_HIZ) == POWERSTEP01_STATUS_HIZ)
  {
    SendError(motor_id, "HIZ_STATE", "Power bridges disabled");
  }

  if ((statusRegister & POWERSTEP01_STATUS_CMD_ERROR) == POWERSTEP01_STATUS_CMD_ERROR)
  {
    SendError(motor_id, "CMD_ERROR", "Command could not be performed");
  }

  if ((statusRegister & POWERSTEP01_STATUS_UVLO) == 0)
  {
    SendError(motor_id, "UVLO", "Undervoltage lock-out");
  }

  if ((statusRegister & POWERSTEP01_STATUS_UVLO_ADC) == 0)
  {
    SendError(motor_id, "UVLO_ADC", "ADC undervoltage lock-out");
  }

  if ((statusRegister & POWERSTEP01_STATUS_TH_STATUS) != 0)
  {
    SendError(motor_id, "THERMAL", "Thermal warning or shutdown");
  }

  if ((statusRegister & POWERSTEP01_STATUS_OCD) == 0)
  {
    SendError(motor_id, "OCD", "Overcurrent detection");
  }

  if ((statusRegister & POWERSTEP01_STATUS_STALL_A) == 0)
  {
    SendError(motor_id, "STALL_A", "Stall on bridge A");
  }

  if ((statusRegister & POWERSTEP01_STATUS_STALL_B) == 0)
  {
    SendError(motor_id, "STALL_B", "Stall on bridge B");
  }
}

void MyFlagInterruptHandler(void)
{
  // Check errors for both motors
  CheckMotorErrors(0);
  CheckMotorErrors(1);
}

void MyBusyInterruptHandler(void)
{
   if (BSP_MotorControl_CheckBusyHw())
   {

   }
   else
   {

   }
}

void MyErrorHandler(uint16_t error)
{
  gLastError = error;
  snprintf((char *)tx_buffer, TX_BUFFER_SIZE, "ERROR,0,SYSTEM_ERROR,0x%04X\n", error);
  UART_SendString((const char *)tx_buffer);

  while(1)
  {
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    UART_ReceiveChar(uart_rx_byte);
    // Re-enable interrupt for next byte
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
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
#ifdef USE_FULL_ASSERT
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
