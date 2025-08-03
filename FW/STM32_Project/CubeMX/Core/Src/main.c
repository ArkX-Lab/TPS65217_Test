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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MCU_GPIO_PA_X_Pin		MCU_GPIO_PA5_Pin
#define I2C1_SLAVE_ADDR 		(0x24 << 1)
#define REG_ADDR_CHIP_ID		(0x00)
#define REG_ADDR_PASSWD			(0x0B)
#define REG_ADDR_ENABLE			(0x16)
#define REG_ADDR_SEQ6			(0x1E)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
uint8_t tx_buffer = 0;
uint8_t rx_buffer = 0;
uint8_t	chip_id = 0;
uint8_t reg_enable = 0;
uint8_t reg_seq6 = 0;

uint8_t reg_passwd = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_GPIO_WritePin(MCU_GPIO_PA5_GPIO_Port, MCU_GPIO_PA5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCU_GPIO_PA5_GPIO_Port, MCU_GPIO_PA6_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCU_GPIO_PA5_GPIO_Port, MCU_GPIO_PA7_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCU_GPIO_PA5_GPIO_Port, MCU_GPIO_PA8_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(MCU_GPIO_PA5_GPIO_Port, MCU_GPIO_PA5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCU_GPIO_PA5_GPIO_Port, MCU_GPIO_PA6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCU_GPIO_PA5_GPIO_Port, MCU_GPIO_PA7_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCU_GPIO_PA5_GPIO_Port, MCU_GPIO_PA8_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);

	//////////// 1. Read Chip ID
	HAL_I2C_Mem_Read(&hi2c1, I2C1_SLAVE_ADDR, REG_ADDR_CHIP_ID, 1, &rx_buffer, 1, HAL_MAX_DELAY);
	chip_id = rx_buffer;

	//////////// 2. Read Enable Register
	HAL_I2C_Mem_Read(&hi2c1, I2C1_SLAVE_ADDR, REG_ADDR_ENABLE, 1, &rx_buffer, 1, HAL_MAX_DELAY);
	reg_enable = rx_buffer;

	//////////// 3. Read SEQ6 Register
	HAL_I2C_Mem_Read(&hi2c1, I2C1_SLAVE_ADDR, REG_ADDR_SEQ6, 1, &rx_buffer, 1, HAL_MAX_DELAY);
	reg_seq6 = rx_buffer;
	HAL_Delay(1);

	//////////// 4. Initialize SEQ Down
	// 4-1. Password Register Write
	reg_passwd = REG_ADDR_SEQ6 ^ 0x7D;
	tx_buffer = reg_passwd;
	HAL_I2C_Mem_Write(&hi2c1, I2C1_SLAVE_ADDR, REG_ADDR_PASSWD, 1, &tx_buffer, 1, HAL_MAX_DELAY);

	// 4-2. Set SEQDWN bit
	tx_buffer = reg_seq6 | 0x02;
	HAL_I2C_Mem_Write(&hi2c1, I2C1_SLAVE_ADDR, REG_ADDR_SEQ6, 1, &tx_buffer, 1, HAL_MAX_DELAY);
	HAL_Delay(100);


	//////////// 5. Enable All Rails
	// 5-1. Password Register Write
	reg_passwd = REG_ADDR_ENABLE ^ 0x7D;
	tx_buffer = reg_passwd;
	HAL_I2C_Mem_Write(&hi2c1, I2C1_SLAVE_ADDR, REG_ADDR_PASSWD, 1, &tx_buffer, 1, HAL_MAX_DELAY);

	// 5-2. Enable Register Write
	tx_buffer = 0x7F;
	HAL_I2C_Mem_Write(&hi2c1, I2C1_SLAVE_ADDR, REG_ADDR_ENABLE, 1, &tx_buffer, 1, HAL_MAX_DELAY);

	HAL_Delay(100);
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x0090194B;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MCU_GPIO_PA5_Pin|MCU_GPIO_PA6_Pin|MCU_GPIO_PA7_Pin|MCU_GPIO_PA8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MCU_GPIO_PA5_Pin MCU_GPIO_PA6_Pin MCU_GPIO_PA7_Pin MCU_GPIO_PA8_Pin */
  GPIO_InitStruct.Pin = MCU_GPIO_PA5_Pin|MCU_GPIO_PA6_Pin|MCU_GPIO_PA7_Pin|MCU_GPIO_PA8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
