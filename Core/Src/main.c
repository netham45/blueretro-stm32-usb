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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer))
/* Size of Reception buffer */
#define RXBUFFERSIZE                      (COUNTOF(aRxBuffer))
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
__IO uint32_t     Transfer_Direction = 0;
__IO uint32_t     Xfer_Complete = 0;
#define NOKEY 0
#define L_CONTROL 0x81
#define L_SHIFT 0x82
#define L_ALT 0x84
#define L_META 0x88
#define R_CONTROL 0x90
#define R_SHIFT 0xA0
#define R_ALT 0xC0
#define R_META 0x80

struct gameport_kb_data {
	uint8_t magic;
	uint8_t key;
	uint8_t pressed;
};

/**
 * USB HID Keyboard scan codes as per USB spec 1.11
 * plus some additional codes
 *
 * Created by MightyPork, 2016
 * Public domain
 *
 * Adapted from:
 * https://source.android.com/devices/input/keyboard-devices.html
 */

/**
 * Scan codes - last N slots in the HID report (usually 6).
 * 0x00 if no key pressed.
 *
 * If more than N keys are pressed, the HID reports
 * KEY_ERR_OVF in all slots to indicate this condition.
 */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Buffer used for transmission */
uint8_t aTxBuffer[4];

/* Buffer used for reception */
uint8_t aRxBuffer[32];

/* Keyboard state */
typedef struct
{
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
}subKeyBoard;

subKeyBoard keyBoardHIDsub = {0,0,0,0,0,0,0,0};

uint8_t buttons[255] = { 0 };
uint32_t debounce[255] = { 0 };
uint8_t USBReport[9] = { 0 };
uint8_t lastUSBReport[9] = { 0 };
uint8_t modifier = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
uint8_t rxBuffer[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void pressReleaseButton(uint8_t button, uint8_t pressed)
{
	if ( pressed )
	{
	  if (!buttons[button])
	  {
		buttons[button] = 1;
		debounce[button] = HAL_GetTick();
		if (button & 0x80) //Modifier key
		{
			uint8_t modifierKey = button & 0x7F;
			if (modifierKey == 0)
				modifierKey = 0x80;
			USBReport[0] |= modifierKey;
		}
		else
		{
			for (int k=3;k<8;k++)
			{
				if (USBReport[k] == 0)
				{
					USBReport[k] = button;
					break;
				}
			}
		}
	  }
	}
	else
	{
	  if (buttons[button])
	  {
		buttons[button] = 0;
		debounce[button] = HAL_GetTick();
		if (button & 0x80) //Modifier key
		{
			uint8_t modifierKey = button & 0x7F;
			if (modifierKey == 0)
				modifierKey = 0x80;
			USBReport[0] ^= modifierKey;
		}
		else
		{
			for (int k=2;k<8;k++)
			{
				if (USBReport[k] == button)
				{
					for (int l=k;l<7;l++)
					{
						USBReport[l] = USBReport[l+1];
					}
					USBReport[7] = 0;
				}
			}
		}
	  }
	}
	if (memcmp(USBReport,lastUSBReport,8) != 0)
	{
		//USBReport[0] = 2;
		memcpy(lastUSBReport,USBReport,8);
		USBD_HID_SendReport(&hUsbDeviceFS, USBReport, 8);

	}
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
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
  {
    /* Transfer error in reception process */
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (Xfer_Complete)
	  {
	   if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
	   {
		 /* Transfer error in reception process */
		 //Error_Handler();
		   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	   }
	   Xfer_Complete = 0;
	  }


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  /* OTG_FS_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 2;
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
  HAL_GPIO_WritePin(PIN_LED_GPIO_Port, PIN_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PIN_LED_Pin */
  GPIO_InitStruct.Pin = PIN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle.
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Toggle LED4: Transfer in transmission process is correct */

  Xfer_Complete = 1;
  aTxBuffer[0]++;
  aTxBuffer[1]++;
  aTxBuffer[2]++;
  aTxBuffer[3]++;
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}


/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Toggle LED4: Transfer in reception process is correct */
  if (aRxBuffer[0] == 'k') // Keyboard
  {
	  struct gameport_kb_data *data = (struct gameport_kb_data*)aRxBuffer;
	  pressReleaseButton(data->key, data->pressed);
  }
  else if (aRxBuffer[0] == 'm') // Mouse
  {

  }
  //printf("Received: %s\n",aRxBuffer);
  Xfer_Complete = 1;
}



/**
  * @brief  Slave Address Match callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  TransferDirection: Master request Transfer Direction (Write/Read), value of @ref I2C_XferOptions_definition
  * @param  AddrMatchCode: Address Match Code
  * @retval None
  */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  Transfer_Direction = TransferDirection;
  if (Transfer_Direction == 0)
  {
     /*##- Start the transmission process #####################################*/
  /* While the I2C in reception process, user can transmit data through
     "aTxBuffer" buffer */
  if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, (uint8_t *)aTxBuffer, 4, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)

    {
    /* Transfer error in transmission process */
    Error_Handler();
  }
  else
  {

  }

  }
  else
  {

      /*##- Put I2C peripheral in reception process ###########################*/
  if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, (uint8_t *)aRxBuffer, sizeof(struct gameport_kb_data), I2C_FIRST_AND_LAST_FRAME) != HAL_OK)
    {
    /* Transfer error in reception process */
    Error_Handler();
  }
  else
  {
	  HAL_I2C_SlaveRxCpltCallback(&hi2c1);
	  HAL_I2C_EnableListen_IT(&hi2c1);
  }

  }

}

/**
  * @brief  Listen Complete callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
}

/**
  * @brief  I2C error callbacks.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  /** Error_Handler() function is called when error occurs.
    * 1- When Slave doesn't acknowledge its address, Master restarts communication.
    * 2- When Master doesn't acknowledge the last data transferred, Slave doesn't care in this example.
    */
  if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
  {
    Error_Handler();
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
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
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
