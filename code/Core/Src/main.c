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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint16_t GPIO_Output = 0; // global variable

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
void GPIO_Init_FullAdder(void); // custom GPIO initialization
void GPIO_Init_SevenSegment(void); // custom GPIO initialization for Seven Segment Display
void Seven_Segment_Digit(unsigned char digit, unsigned char hex_char); // custom function
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
  MX_I2S3_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */

  // Configure necessary GPIOs
  GPIO_Init_FullAdder();
  GPIO_Init_SevenSegment(); // new port E config

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    uint32_t pc = GPIOC->IDR;

    uint8_t A   = (pc >> 12) & 1;
    uint8_t B   = (pc >> 13) & 1;
    uint8_t Cin = (pc >> 14) & 1;

    uint8_t sum  = A ^ B ^ Cin;  // XOR
    uint8_t Cout = (A & B) | (A & Cin) | (B & Cin);

    uint16_t leds = 0;
    leds |= (A    << 0); // PD0
    leds |= (B    << 1); // PD1
    leds |= (Cin  << 2); // PD2
    leds |= (sum  << 3); // PD3
    leds |= (Cout << 4); // PD4

    GPIOD->ODR = leds;

    // show each signal on the 7 segment display
    Seven_Segment_Digit(0, A);    // digit 0 -> A (0 or 1)
    Seven_Segment_Digit(1, B);    // digit 1 -> B
    Seven_Segment_Digit(2, Cin);  // digit 2 -> Cin
    Seven_Segment_Digit(3, sum);  // digit 3 -> Sum (optional)
    Seven_Segment_Digit(4, Cout); // digit 4 -> Cout (optional)

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief GPIO Full Adder Logic Initialization Function
 * @param NONE
 * @retval NONE
 */
void GPIO_Init_FullAdder(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  // Enable Port C clock (inputs)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // Enable Port D clock (LEDs output)

	// configure GPIO port C pins (PC12-PC14) as inputs
	// Note: each pin has 2 bits
	// int r = 0b11;
	GPIOC->MODER &= ~(
		(0b11 << (12 * 2)) |
		(0b11 << (13 * 2)) |
		(0b11 << (14 * 2))
	);	

	// disable any pull-up or pull-down resistors
	GPIOC->PUPDR &= ~(
		(0b11 << (12 * 2)) |
		(0b11 << (13 * 2)) |
		(0b11 << (14 * 2))
	);	

  /* ------------------------------
   * Configure outputs (PD0–PD4)
   * ------------------------------ */
  // For each pin: MODER = 01 (output)
  GPIOD->MODER &= ~(
      (3u << (0 * 2)) |
      (3u << (1 * 2)) |
      (3u << (2 * 2)) |
      (3u << (3 * 2)) |
      (3u << (4 * 2))
  )
  GPIOD->MODER |= (
      (1u << (0 * 2)) |
      (1u << (1 * 2)) |
      (1u << (2 * 2)) |
      (1u << (3 * 2)) |
      (1u << (4 * 2))
  )
  // Push-pull (default, OTYPER = 0)
  GPIOD->OTYPER &= ~(
      (1u << 0) |
      (1u << 1) |
      (1u << 2) |
      (1u << 3) |
      (1u << 4)
  )
  // No pull-up/pull-down on output pins
  GPIOD->PUPDR &= ~(
      (3u << (0 * 2)) |
      (3u << (1 * 2)) |
      (3u << (2 * 2)) |
      (3u << (3 * 2)) |
      (3u << (4 * 2))
  );
}

/**
 * @brief GPIO Seven Segment Display Initialization Function
 * @param None
 * @retval None
 */
void GPIO_Init_SevenSegment(void)
{
    // Enable clock for Port E
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    // PE8..PE15 = outputs
    GPIOE->MODER &= ~(
        (3u << (8 * 2))  |
        (3u << (9 * 2))  |
        (3u << (10 * 2)) |
        (3u << (11 * 2)) |
        (3u << (12 * 2)) |
        (3u << (13 * 2)) |
        (3u << (14 * 2)) |
        (3u << (15 * 2))
    );
    GPIOE->MODER |= (
        (1u << (8 * 2))  |
        (1u << (9 * 2))  |
        (1u << (10 * 2)) |
        (1u << (11 * 2)) |
        (1u << (12 * 2)) |
        (1u << (13 * 2)) |
        (1u << (14 * 2)) |
        (1u << (15 * 2))
    );

    // Open-drain on PE8..PE15 (required by board)
    GPIOE->OTYPER |= 0xFF00;

    // No pull-ups/pull-downs
    GPIOE->PUPDR &= ~(
        (3u << (8 * 2))  |
        (3u << (9 * 2))  |
        (3u << (10 * 2)) |
        (3u << (11 * 2)) |
        (3u << (12 * 2)) |
        (3u << (13 * 2)) |
        (3u << (14 * 2)) |
        (3u << (15 * 2))
    );
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief Custom Function to output to Seven Segment Display
 * @param digit     The position/index of the seven-segment display
 * @param hex_char  The hexadecimal character (0-15) to display
 * @retval NONE
 */
void Seven_Segment_Digit (unsigned char digit, unsigned char hex_char)
{
/*******************************************************************************
Code to mask and bit shift 0-7 value of digit and 0-15 value of hex_char
to output correct bit pattern to GPIO_Output
*******************************************************************************/

// Limit inputs to valid ranges
    digit    &= 0x07;   // only 0-7
    hex_char &= 0x0F;   // only 0-15

    // Invert lower 2 bits of digit (required by the board)
    unsigned char low2      = digit & 0x03;
    unsigned char inv_digit = (~low2) & 0x03;

    // Start building the output pattern
    // Clear the bits we care about: PE8-PE15
    GPIO_Output &= ~((uint16_t)0xFF00);

    // Put hex_char on PE8..PE11
    GPIO_Output |= ((uint16_t)hex_char << 8);

    // Put inverted digit select on PE12-PE13
    GPIO_Output |= ((uint16_t)inv_digit << 12);

    // Set both chip select bits high (PE14, PE15 = 1 -> inactive)
    GPIO_Output |= (1u << 14) | (1u << 15);

    // Write pattern to GPIOE
    GPIOE->ODR = GPIO_Output;

    HAL_Delay(1); // small delay

    // Pulse the appropriate chip select low
    if (digit > 3)
    {
        // upper 4 digits use PE15
        GPIO_Output &= ~(1u << 15);   // PE15 = 0 (active)
    }
    else
    {
        // lower 4 digits use PE14
        GPIO_Output &= ~(1u << 14);   // PE14 = 0 (active)
    }

    GPIOE->ODR = GPIO_Output;

    HAL_Delay(1); // let the latch see the low pulse

    // Return both chip selects high (inactive)
    GPIO_Output |= (1u << 14) | (1u << 15);
    GPIOE->ODR = GPIO_Output;

    return;
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
     example: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
