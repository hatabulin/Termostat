
/**
  ******************************************************************************
	Termostat

	Author: SergikLutsk, https://github.com/hatabulin/

	Use HAL STM32 CubeMX driver.

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "OneWire.h"
#include "string.h"
#include "stdbool.h"
#include "eeprom.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t devices;
uint32_t pDelay = 300;
uint8_t sensor;
uint8_t i;
OneWire ow;
Temperature t;
char *crcOK;
char str[100];
uint8_t profile_num = 0;
uint8_t common_array_number ;
float hyst1_min_temp = 25.5;
float hyst1_max_temp = 26.5;
bool flag_hyst1;
uint16_t timer2_counter;
bool flag_timer2_counter;
hysteresis hyst[MAX_HYST];
uint8_t flag_usb;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void ReadEEprom() {
	uint32_t data_;
	EE_Read(EEP_CHECK_ADDR,&data_);
	if (data_!= CHECK_DATA)	{
		EE_Format();
		WriteDefaultsConstantToEEP();
		EE_Write(EEP_CHECK_ADDR,CHECK_DATA);
		ReadConstantsFromEEP();
	}
	else ReadConstantsFromEEP();
}

void WriteDefaultsConstantToEEP() {
	for (uint8_t i=0;i<MAX_HYST;i++) {
		EE_Write(EEP_HYST_MIN + i * 5, 25); //
		EE_Write(EEP_HYST_MAX + i * 5, 30); //
		EE_Write(EEP_HYST_POWER + i * 5, 1); //
	}
}

void WriteValuesToEEprom() {
	for (uint8_t i=0;i<MAX_HYST;i++) {
		EE_Write(EEP_HYST_MIN + i * 5, hyst[i].min_temp);
		EE_Write(EEP_HYST_MAX + i * 5, hyst[i].max_temp);
		EE_Write(EEP_HYST_POWER + i * 5, hyst[i].power);
	}
}

void ReadConstantsFromEEP(void) {
	uint32_t _data;

	for (uint8_t i=0;i<MAX_HYST;i++) {
		EE_Read(EEP_HYST_MIN + i*5, &_data);hyst[i].min_temp = _data;
		EE_Read(EEP_HYST_MAX + i*5, &_data);hyst[i].max_temp = _data;
		EE_Read(EEP_HYST_POWER + i*5, &_data);hyst[i].power = _data;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init(115200);
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim2);

  snprintf(str,sizeof(str),"Kernel started...\n\rget_ROMid function executed...\n\r");
  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
  get_ROMid();
  ReadEEprom();

  for (uint8_t i = 0; i < devices; i++) {

	  LED_ONOFF();
	  switch ((ow.ids[i]).family) {
      case DS18B20:
    	  t = readTemperature(&ow, &ow.ids[i], 1);
    	  snprintf(str,sizeof(str),"DS18B20 N_%2u , Temperature: %3u.%2uC\n\r",i,t.inCelsus, t.frac);
    	  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
    	  break;
      case DS18S20:
    	  t = readTemperature(&ow, &ow.ids[i], 1);
    	  snprintf(str,sizeof(str),"DS18S20 N_%2u , Temperature: %3u.%2uC\n\r",i,t.inCelsus, t.frac);
    	  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
    	  break;
      case 0x00:
    	  break;
      default:
    	  snprintf(str,sizeof(str),"UNKNOWN Family:%x (SN: %x%x%x%x%x%x\n\r)",
    			  (ow.ids[i]).family, (ow.ids[i]).code[0],(ow.ids[i]).code[1],(ow.ids[i]).code[2],(ow.ids[i]).code[3], (ow.ids[i]).code[4], (ow.ids[i]).code[5]);
    	  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
    	  break;
	  }
  }

  snprintf(str,sizeof(str),"get_ROMid function complete.\n\r");
  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  if (flag_timer2_counter) {
		  if (timer2_counter > MAX_UPDATE_CNT) {
			  t = readTemperature(&ow, &ow.ids[0], 1);
			  snprintf(str,sizeof(str),"DS18B20 N_%2u , Temp: %3u.%2uC\n\r",0,t.inCelsus, t.frac);
			  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
			  CDC_Transmit_FS(str, strlen(str));
			  timer2_counter = 0;
			  CheckAndSetPower((float)t.inCelsus+t.frac/10);
		  }
		  flag_timer2_counter = false;
	  }

	  if (flag_usb == SAVE_CFG) {
		  flag_usb = 0;
		  snprintf(str,sizeof(str),"Settings stored to EEP !\n\r");
		  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
		  CDC_Transmit_FS(str, strlen(str));
		  WriteValuesToEEprom();
	  }

	  if (flag_usb == GET_CFG) {
		  flag_usb = 0;
		  snprintf(str,sizeof(str),"Hysteresis EEP values:\n\rprofile_num:%u\n\r",profile_num);
		  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
		  CDC_Transmit_FS(str, strlen(str));

		  for (uint8_t i = 0;i<MAX_HYST;i++) {
			  snprintf(str,sizeof(str),"hyst[%u]_min:%2u, hyst[%u]_max:%2u, hyst[%u]_power:%1u\n\r",
					  i,(uint8_t)hyst[i].min_temp,i,(uint8_t)hyst[i].max_temp,i, hyst[i].power);
			  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
			  CDC_Transmit_FS(str, strlen(str));
		  }

		  snprintf(str,sizeof(str),"\n\r--====--\n\rLutsk,UA, 2019 Build:051119 (by }{aTa6, Email:dragosha2000@gmx.net)\n\r\n\r\n\r");
		  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
		  CDC_Transmit_FS(str, strlen(str));
	  }

	  if (flag_usb == SET_PROFILE) {
		  flag_usb = 0;
		  snprintf(str,sizeof(str),"Profile changed to %u\n\r", profile_num);
		  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
		  CDC_Transmit_FS(str, strlen(str));
	  }

	  if (flag_usb == SET_VALUES) {
		  flag_usb = 0;
		  snprintf(str,sizeof(str),"values changed !\n\rhyst[%u]_min:%2u, hyst[%u]_max:%2u, hyst[%u]_power:%1u\n\r",
				  common_array_number,(uint8_t)hyst[common_array_number].min_temp,common_array_number,
				  (uint8_t)hyst[common_array_number].max_temp,common_array_number, hyst[common_array_number].power);

		  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
		  CDC_Transmit_FS(str, strlen(str));
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUTTON_ONOFF_Pin|BUTTON_UP_Pin|BUTTON_DOWN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_ONOFF_Pin BUTTON_UP_Pin BUTTON_DOWN_Pin */
  GPIO_InitStruct.Pin = BUTTON_ONOFF_Pin|BUTTON_UP_Pin|BUTTON_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void button_press(uint8_t button) {
	switch (button)	{
	case ONOFF_BUTTON:
		HAL_GPIO_WritePin(BUTTON_ONOFF_GPIO_Port,BUTTON_ONOFF_Pin,SET);
		HAL_Delay(DELAY1_MS);
		HAL_GPIO_WritePin(BUTTON_ONOFF_GPIO_Port,BUTTON_ONOFF_Pin,RESET);
		HAL_Delay(DELAY1_MS);
		break;
	case DOWN_BUTTON:
		HAL_GPIO_WritePin(BUTTON_DOWN_GPIO_Port,BUTTON_DOWN_Pin,SET);
		HAL_Delay(DELAY1_MS);
		HAL_GPIO_WritePin(BUTTON_DOWN_GPIO_Port,BUTTON_DOWN_Pin,RESET);
		HAL_Delay(DELAY1_MS);
		break;
	}
}

void SetPower(uint8_t power) {
	for (uint8_t i = 0; i< 6 - power;i++) {
		button_press(DOWN_BUTTON);
		HAL_Delay(DELAY1_MS);
	}
}

void get_ROMid (void) {

	if (owResetCmd() != ONEWIRE_NOBODY)	{    // is anybody on the bus?
		devices = owSearchCmd(&ow);        // получить ROMid всех устройст на шине или вернуть код ошибки
		if (devices <= 0) {
			snprintf(str,sizeof(str),"Error has happened!");
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
			while (1) {
				pDelay = 1000000;
				for (i = 0; i < pDelay * 1; i++){}    /* Wait a bit. */
           // __asm__("nop");
			}
		}

		snprintf(str,sizeof(str),"found %d devices on 1-wire bus", devices);
		HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);

		for (i = 0; i < devices; i++) {
			RomCode *r = &ow.ids[i];
			uint8_t crc = owCRC8(r);
			crcOK = (crc == r->crc)?"CRC OK":"CRC ERROR!";
			snprintf(str,sizeof(str),"\n\rdevice %d (SN: %02X/%02X%02X%02X%02X%02X%02X/%02X) ", i,
					r->family, r->code[5], r->code[4], r->code[3],r->code[2], r->code[1], r->code[0], r->crc);
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
			if (crc != r->crc) {
				snprintf(str,sizeof(str),"CRCfailedk\n\r");
				HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
			} else {
				snprintf(str,sizeof(str),"CRCok\n\r");
				HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
			}
		}
	}
}

void CheckAndSetPower(float celsium) {

	if (celsium < hyst[profile_num].min_temp + 0.1) {
		if (!flag_hyst1) {
			snprintf(str,sizeof(str),"Set power to %d\n\r",hyst[profile_num].power);
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
			CDC_Transmit_FS(str, strlen(str));

			button_press(ONOFF_BUTTON);
			SetPower(hyst[profile_num].power);
			flag_hyst1 = true;
		}
	}

	if ((celsium > hyst[profile_num].max_temp - 0.1) & (flag_hyst1)) {
		snprintf(str,sizeof(str),"Power off.\n\r");
		HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str), HAL_MAX_DELAY);
		CDC_Transmit_FS(str, strlen(str));

		button_press(ONOFF_BUTTON);
		flag_hyst1 = false;
	}
}

void MX_USART2_UART_Init(uint32_t baud) {

  huart2.Instance = USART2;
  huart2.Init.BaudRate = baud;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
