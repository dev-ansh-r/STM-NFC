/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdbool.h"
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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);   // RST

  // 0x2F: Command Message, komplett Message, GID = 1111b
  // 0x02 = OID
  uint8_t CORE_RESET_CMD[] = {0x20, 0x00, 0x01, 0x00};
  HAL_I2C_Master_Transmit(&hi2c1, (0x28<<1), CORE_RESET_CMD , sizeof(CORE_RESET_CMD), HAL_MAX_DELAY);
  HAL_Delay(500); // Delay for 500 milisecond between readings

  // Read data from NFC Click via I2C
  uint8_t dataBufferREST[32]; // Buffer to store the received data
  if(HAL_GPIO_ReadPin( GPIOB, GPIO_PIN_5) == GPIO_PIN_SET)
  {
    HAL_I2C_Master_Receive(&hi2c1, (0x28 << 1), dataBufferREST, sizeof(dataBufferREST), HAL_MAX_DELAY);
    // Print the received data to the serial monitor
    for(int i = 0; i < sizeof(dataBufferREST); i++)
    {
      char uartBuffer[8];  // Buffer to store the converted data

      // Convert the data to ASCII and print it
      sprintf(uartBuffer, "%02X ", dataBufferREST[i]);
      HAL_UART_Transmit(&huart2, (uint8_t*) uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, HAL_MAX_DELAY);  // Print a new line
  }

  uint8_t CORE_INIT_CMD[] = {0x20, 0x01, 0x00};
  HAL_I2C_Master_Transmit(&hi2c1, (0x28<<1), CORE_INIT_CMD , sizeof(CORE_INIT_CMD), HAL_MAX_DELAY);
  HAL_Delay(500); // Delay for 500 milisecond between readings
  if(HAL_GPIO_ReadPin( GPIOB, GPIO_PIN_5) == GPIO_PIN_SET)
  {
    // Read data from NFC Click via I2C
    uint8_t dataBufferINT[32]; // Buffer to store the received data
    HAL_I2C_Master_Receive(&hi2c1, (0x28 << 1), dataBufferINT, sizeof(dataBufferINT), HAL_MAX_DELAY);
    // Print the received data to the serial monitor
    for(int i = 0; i < sizeof(dataBufferINT); i++)
    {
      char uartBuffer[8];  // Buffer to store the converted data

      // Convert the data to ASCII and print it
      sprintf(uartBuffer, "%02X ", dataBufferINT[i]);
      HAL_UART_Transmit(&huart2, (uint8_t*) uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, HAL_MAX_DELAY);  // Print a new line
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN 3 */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
/*    HAL_StatusTypeDef ret;
    GPIO_PinState a = HAL_GPIO_ReadPin( GPIOB, GPIO_PIN_5) ;
    if (a == GPIO_PIN_SET ){
      HAL_UART_Transmit(&huart2, "SET\r\n", sizeof("SET\r\n"), HAL_MAX_DELAY);
    }else{
      HAL_UART_Transmit(&huart2, "UNSET\r\n", sizeof("UNSET\r\n"), HAL_MAX_DELAY);
    }
    HAL_Delay(501);
    if(HAL_I2C_IsDeviceReady(&hi2c1, (0x28<<1), 2, 100) == HAL_OK)
    {
        HAL_GPIO_WritePin( GPIOB, GPIO_PIN_3,  GPIO_PIN_SET);  //Set LED

        uint8_t stat[] = "Device is Ready, and LED  is ON\r\n";
        HAL_UART_Transmit(&huart2, stat, sizeof(stat), HAL_MAX_DELAY);



        if (ret == HAL_OK)
        {
            uint8_t stat[] = "Transmit to NFCC is Ok\r\n";
            HAL_UART_Transmit(&huart2, stat, sizeof(stat), HAL_MAX_DELAY);

        //    uint8_t dataBuffer[32] = {0x4F, 0x02, 0x05};
         //   HAL_I2C_Master_Transmit(&hi2c1, (0x29<<1), dataBuffer, sizeof(dataBuffer), HAL_MAX_DELAY);

            if(HAL_GPIO_ReadPin( GPIOB, GPIO_PIN_5) == GPIO_PIN_SET)
            {
                uint8_t stat[] = "INT/IQR is triggered \r\n";
                HAL_UART_Transmit(&huart2, stat, sizeof(stat), HAL_MAX_DELAY);

                // Read data from NFC Click via I2C
                uint8_t dataBufferRes[32]; // Buffer to store the received data

                HAL_I2C_Master_Receive(&hi2c1, (0x29<<1), dataBufferRes, sizeof(dataBufferRes), HAL_MAX_DELAY);
                // Print the received data to the serial monitor
                for (int i = 0; i < sizeof(dataBufferRes); i++)
                {
                    char uartBuffer[8];  // Buffer to store the converted data

                    // Convert the data to ASCII and print it
                    sprintf(uartBuffer, "%02X ", dataBufferRes[i]);
                    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
                 }
                 HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);  // Print a new line
          }
        }
        else
        {
            HAL_GPIO_WritePin( GPIOB, GPIO_PIN_3,  GPIO_PIN_RESET);
            uint8_t stat[] = "Transmit is not OK, and LED is off\r\n";
            HAL_UART_Transmit(&huart2, stat, sizeof(stat), HAL_MAX_DELAY);
        }
    }
    else
    {
        HAL_GPIO_WritePin( GPIOB, GPIO_PIN_3,  GPIO_PIN_RESET);
        uint8_t stat[] = "Device is not Ready, and LED is Off\r\n";
        HAL_UART_Transmit(&huart2, stat, sizeof(stat), HAL_MAX_DELAY);
        HAL_Delay(500);  // Delay for 1 second between readings
    }
*/
    /* USER CODE BEGIN 3 */

    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c1.Init.Timing = 0x00707CBB;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD3_Pin RST_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
