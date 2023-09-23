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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus.h"
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
rs485info_ rs485info;
uint16_t adcDatas = 0;
uint16_t adcDatas2 = 0;
extern uint8_t rs485_uart1[512];
extern uint16_t count;

bool saveFlag = false;



////////////////////////////////////////////********MODBUS KONTROL FONKSIYONLARI START************////////////////////////////


/**
 * Handel Read Holding Registers (FC=03)
 * write back the values from eeprom (holding registers).
 */
uint8_t readMemory(uint8_t fc, uint16_t address, uint16_t length) {
    uint16_t analogValue = 0;
    /// read holding
    // read program memory.
	int i;
    for (i = 0; i < length; i++)
	{
        switch(address + i) // read holding register
		{

			case 0: // rs485 baudrate 0-14 arasi
					// 1:1200, 2:2400, 3:4800, 4:5800, 5:9600, 6:14400,
				    // 7:19200, 8:28800, 9:33600, 10:38400, 11:57600, 12:115200, 13: 230400, 14:256700
					// 5:9600 DEFAULT
			{
				analogValue = adcDatas2; //rs485info.baudrate;
				break;
			}
			case 1: // rs485 parity,
					// 1:NONE, 2:EVEN, 3:ODD
					// 1:NONE DEFAULT
			{
				analogValue = rs485info.parity;
				break;
			}
			case 2: // rs485 stopbit,
					// 1:ONE, 2:TWO
					// 1:ONE DEFAULT
			{
				analogValue = rs485info.stopbits;
				break;
			}
			case 3: // rs485 bytesize
					// 1:8B, 2:9B
					// 1:8B DEFAULT
			{
				analogValue = rs485info.bytesize;
				break;
			}
			case 4: // rs485 slaveId
					// 1-255
					// 1:1 DEFAULT
			{
				analogValue = rs485info.slaveId;
				break;
			}

			default:
			{
				analogValue = 0xffff;
			}

		}

		writeRegisterToBuffer(i, analogValue);
    }

    return STATUS_OK;
}

/**
 * Handle Write Holding Register(s) (FC=06, FC=16)
 * write data into eeprom.
 */
uint8_t writeMemory(uint8_t fc, uint16_t address, uint16_t length) {
    uint16_t analogValue;
    int i;
    // write to eeprom.
    saveFlag = false;
    for (i = 0; i < length; i++)
	{
		// get uint16_t value from the request buffer.
        analogValue = readRegisterFromBuffer(i);
		switch(address + i)
		{
			case 0: // rs485 baudrate 0-14 arasi
					// 1:1200, 2:2400, 3:4800, 4:5800, 5:9600, 6:14400,
				    // 7:19200, 8:28800, 9:33600, 10:38400, 11:57600, 12:115200, 13: 230400, 14:256700
					// 5:9600 DEFAULT
			{
				adcDatas2 = analogValue;
				/*if(analogValue <BAUDRATE_INDEX && analogValue > 0)
				{
					rs485info.baudrate = analogValue;
					saveFlag = true;
				}*/
				break;
			}
			case 1: // rs485 parity,
					// 1:NONE, 2:EVEN, 3:ODD
					// 1:NONE DEFAULT
			{
				if(analogValue < PARITY_INDEX &&  analogValue > 0)
				{
					rs485info.parity = analogValue;
					saveFlag = true;
				}
				break;
			}
			case 2: // rs485 stopbit,
					// 1:ONE, 2:TWO
					// 1:ONE DEFAULT
			{
				if(analogValue < STOPBIT_INDEX &&  analogValue > 0)
				{
					rs485info.stopbits = analogValue;
					saveFlag = true;
				}
				break;
			}
			case 3: // rs485 bytesize
					// 1:8B, 2:9B
					// 1:8B DEFAULT
			{
				if(analogValue < BYTESIZE_INDEX &&  analogValue > 0)
				{
					rs485info.bytesize = analogValue;
					saveFlag = true;
				}
				break;
			}
			case 4: // rs485 slaveId
					// 1-255
					// 1:1 DEFAULT
			{
				if(analogValue <= SLAVE_ID_MAX &&  analogValue > 0)
				{
					rs485info.slaveId = analogValue;
					saveFlag = true;
				}
				break;
			}



			/*default:
			{
				analogValue = 0xffff;
			}*/
		}
    }
    if(saveFlag == true)
	{
		// bir array olustur, protection, ve rs485 datalarini iceren, sonra kaydet.
		//SaveDatas();
	}
    return STATUS_OK;
}

/**
 * Handel Read Coils (FC=01)
 * write back the values from digital in pins (input status).
 */
uint8_t readCoils(uint8_t fc, uint16_t address, uint16_t length) {
    // read coils state
	GPIO_PinState pinState;
	int i ;
    for (i = 0; i < length; i++) {
        // write one boolean (1 bit) to the response buffer.
		switch(address + i)
		{
			case 0:
			{
				pinState = HAL_GPIO_ReadPin(DO1_GPIO_Port, DO1_Pin);
				break;
			}
			default:
			{
				pinState = GPIO_PIN_RESET;
			}
		}
        writeCoilToBuffer(i, pinState);
    }

    return STATUS_OK;
}

/**
 * Handle Force Single Coil (FC=05) and Force Multiple Coils (FC=15)
 * set digital output pins (coils).
 */
uint8_t writeCoils(uint8_t fc, uint16_t address, uint16_t length) {
    // set digital pin state(s).
	int i;
    for (i = 0; i < length; i++) {
    	switch(address + i)
    	{
			case 0:
			{
				switch(readCoilFromBuffer(i))
				{
					case HIGH:
					{
						HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);
						break;
					}
					case LOW:
					{
						HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);
						break;
					}
					case ERROR:
						break;
				}

				break;
			}
    	}
    }
    return STATUS_OK;
}

/**
 * Handel Read Input Status (FC=02)
 * write back the values from digital in pins (input status).
 *
 * handler functions must return void and take:
 *      uint8_t  fc - function code.
 *      uint16_t address - first register/coil address.
 *      uint16_t length/status - length of data / coil status.
 */
uint8_t readDigitalIn(uint8_t fc, uint16_t address, uint16_t length)
{
	GPIO_PinState pinState;
	int i;
    // read digital input
    for (i = 0; i < length; i++) {
        // write one boolean (1 bit) to the response buffer.
		switch(address + i)
		{
			case 0:
			{
				pinState = HAL_GPIO_ReadPin(DI1_GPIO_Port, DI1_Pin);
				break;
			}
			default:
			{
				pinState = GPIO_PIN_RESET;
			}
		}
        writeCoilToBuffer(i, pinState);
    }

    return STATUS_OK;
}

/**
 * Handel Read Input Registers (FC=04)
 * write back the values from analog in pins (input registers).
 */
uint8_t readAnalogIn(uint8_t fc, uint16_t address, uint16_t length) {
    // read analog input
	int i;
	uint16_t analogValue;
    for (i = 0; i < length; i++) {
        // write uint16_t value to the response buffer.
       // writeRegisterToBuffer(i, analogRead(address + i));
	   switch(address + i)
		{
			case 0:
			{
				analogValue = adcDatas++;  // 0-10V 1
				break;
			}
			default:
			{
				analogValue = 0xffff;
			}

		}
        writeRegisterToBuffer(i, analogValue);
    }
    return STATUS_OK;
}




///////////////////////////////////////////*********MODBUS KONTROL FONKSIYONLARI END**************///////////////////////////////////

uint8_t returnAnswer(uint8_t *data, uint16_t length)
{
	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
	while(GPIO_PIN_SET != HAL_GPIO_ReadPin(RS485_EN_GPIO_Port, RS485_EN_Pin));

	HAL_UART_Transmit(&huart1, data, length, 100);

	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
	while(GPIO_PIN_RESET != HAL_GPIO_ReadPin(RS485_EN_GPIO_Port, RS485_EN_Pin));

	if(saveFlag == true)
	{
		//MX_USART1_UART_Init_Re();
		saveFlag = false;
	}
}

void FabrikaAyarlari()
{
	rs485info.slaveId 	 	= 1;
	rs485info.baudrate    	= 5;
	rs485info.parity      	= 1;
	rs485info.stopbits   	= 1;
	rs485info.bytesize    	= 1;

	//SaveDatas();
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
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  FabrikaAyarlari();

	cbVector[CB_READ_COILS] = readCoils;
	cbVector[CB_READ_DISCRETE_INPUTS] = readDigitalIn;
	cbVector[CB_READ_HOLDING_REGISTERS] = readMemory;
	cbVector[CB_READ_INPUT_REGISTERS] = readAnalogIn;
	cbVector[CB_WRITE_COILS] = writeCoils;
	cbVector[CB_WRITE_REGISTERS] = writeMemory;

	rVector[RETURN_FUNCTION] = returnAnswer;

	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);

	 GPIO_PinState rs485En;
  memset(rs485_uart1, 0x00, 512);
  	count = 0;

  	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  rs485En = HAL_GPIO_ReadPin(RS485_EN_GPIO_Port, RS485_EN_Pin);
	  if(count>0)
	  {
	    int countTemp = count;
		while(1)
		{
			countTemp = count;
			HAL_Delay((8000/huart1.Init.BaudRate)+3);
			if(countTemp == count) break;
		}
			 //time2 = HAL_GetTick() -time1;
		modbusAnalyser(rs485_uart1, count);
		//time2 = HAL_GetTick() -time1;
		count=0;
		memset(rs485_uart1, 0x00, 512);

	  }
	  //HAL_UART_Transmit(&huart1, (uint8_t*)"data", 4, 100);
	  HAL_Delay(1);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DO1_Pin */
  GPIO_InitStruct.Pin = DO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DI1_Pin */
  GPIO_InitStruct.Pin = DI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DI1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_EN_Pin */
  GPIO_InitStruct.Pin = RS485_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_EN_GPIO_Port, &GPIO_InitStruct);

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
