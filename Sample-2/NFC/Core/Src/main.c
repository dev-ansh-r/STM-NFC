/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* Constants */
#define NFC_CLICK_ADDRESS 0xAA  // Replace with the correct address of NFC Click

/* Function to send NCI command to NFC Click */
uint8_t sendNCICommand(uint8_t *command, uint8_t commandSize, uint8_t *response, uint16_t *responseSize)
{
  if (HAL_I2C_Master_Transmit(&hi2c1, NFC_CLICK_ADDRESS, command, commandSize, HAL_MAX_DELAY) != HAL_OK)
    return NFC_ERROR;

  if (HAL_I2C_Master_Receive(&hi2c1, NFC_CLICK_ADDRESS, response, *responseSize, HAL_MAX_DELAY) != HAL_OK)
    return NFC_ERROR;

  return NFC_SUCCESS;
}

/* Function to delay in milliseconds */
void nfc_hal_delay(uint32_t milliseconds)
{
  HAL_Delay(milliseconds);
}

int initialize_core(void)
{
  uint8_t answer[ANSWER_MAX_SIZE] = {0};
  uint16_t answer_size = 0;
  int i = 10;
  uint8_t CORE_RESET_CMD[] = {0x20, 0x00, 0x01, 0x01};
  uint8_t CORE_INIT_CMD[] = {0x20, 0x01, 0x00};

#if defined(DEBUG)
  printf("\r\n***** Resetting and Initializing Core *****\r\n");
#endif

  /* Reset the core */
  while (sendNCICommand(CORE_RESET_CMD, sizeof(CORE_RESET_CMD), answer, &answer_size))
  {
    if (!i--)
      return NFC_ERROR;

    nfc_hal_delay(500);
  }

  /* Once reset, initialize the core */
  if (sendNCICommand(CORE_INIT_CMD, sizeof(CORE_INIT_CMD), answer, &answer_size))
    return NFC_ERROR;
  else if ((answer[0] != 0x40) || (answer[1] != 0x01) || (answer[3] != 0x00))
    return NFC_ERROR;

  return NFC_SUCCESS;
}


int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /*Initialize core*/
  if (initialize_core() != NFC_SUCCESS)
  {
    printf("Error initializing core\r\n");
    return NFC_ERROR;
  }

  /* Infinite loop SOURCE CODE */
  /* OUR CODE BEGIN WHILE */
  while (1)
    {
      uint8_t dataBuffer[32];  // Buffer to store the received data

      if(HAL_I2C_IsDeviceReady(&hi2c1, (0x28<<1), 2, 100) == HAL_OK)
      {
      HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
      HAL_Delay(1000);
      // Read data from NFC Click via I2C
      if (HAL_I2C_Master_Receive(&hi2c1, 0x51, dataBuffer, sizeof(dataBuffer), HAL_MAX_DELAY) == HAL_OK)
      {
        // Print the received data to the serial monitor
        for (int i = 0; i < sizeof(dataBuffer); i++)
        {
          char uartBuffer[8];  // Buffer to store the converted data

          // Convert the data to ASCII and print it
          sprintf(uartBuffer, "%02X ", dataBuffer[i]);
          HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
        }
        HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);  // Print a new line
      }else{
            char uartBuffer[8];
            sprintf(uartBuffer, "%02X ", 0);
            HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);  // Print a new line
          }

      HAL_Delay(1000);  // Delay for 1 second between readings
    }else{
    	char i2cBuffer[8];
    	sprintf(i2cBuffer, "%02X", 2);
    	HAL_UART_Transmit(&huart2,(uint8_t*)i2cBuffer, strlen(i2cBuffer), HAL_MAX_DELAY);
    	HAL_UART_Transmit(&huart2,(uint8_t*)i2cBuffer, strlen(i2cBuffer), HAL_MAX_DELAY);
    }

  }
}

/*  System Clock Configuration  */
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

/** I2C1 Initialization Function */
static void MX_I2C1_Init(void)
{

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

}

/* USART2 Initialization Function   */
static void MX_USART2_UART_Init(void)
{

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

}

/* GPIO Initialization Function   */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* This function is executed in case of error occurrence */
void Error_Handler(void)
{
  /* Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
