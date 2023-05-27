#include "main.h"
#include "stm32f3xx_hal.h"
#include "nfc/nfc.h"
#include "nfc/nfc-types.h"
#include <string.h>

#define NFC_TAG_TYPE NFC_ISO14443A
#define CMD_SET_LED "set led"

nfc_context *context;
nfc_device *pnd;
uint8_t led_state = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  // Initialize the NFC context
  nfc_init(&context);
  if (context == NULL) {
    printf("Unable to init libnfc (malloc)\n");
    return 1;
  }

  // Open the NFC device
  pnd = nfc_open(context, NULL);
  if (pnd == NULL) {
    printf("Unable to open NFC device\n");
    nfc_exit(context);
    return 1;
  }

  // Set the device to target mode
  if (nfc_target_init(pnd, NFC_TAG_TYPE, NULL, 0, NULL) < 0) {
    nfc_perror(pnd, "nfc_target_init");
    nfc_close(pnd);
    nfc_exit(context);
    return 1;
  }

  // Wait for a command from an initiator
  while(1) {
    uint8_t command[64];
    size_t command_len = sizeof(command);
    int res = nfc_target_receive_bytes(pnd, command, &command_len, 0);

    if (res < 0) {
      nfc_perror(pnd, "nfc_target_receive_bytes");
      continue;
    }

    if (command_len == strlen(CMD_SET_LED) && memcmp(command, CMD_SET_LED, command_len) == 0) {
      // Set LED on
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
      led_state = 1;
    } else {
      // Set LED off
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
      led_state = 0;
    }

    // Send response back to initiator
    uint8_t response[] = { led_state };
    size_t response_len = sizeof(response);
    res = nfc_target_send_bytes(pnd, response, response_len, 0);
    if (res < 0) {
      nfc_perror(pnd, "nfc_target_send_bytes");
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
