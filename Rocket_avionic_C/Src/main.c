/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

#include "bmp280.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "lwgps/lwgps.h"
//#include "mpu6050.h"

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
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//########################!!!!!!!!!!!!!!!!########################
int minimum_altitude_for_safety = 20;

int recovery_system_trigger_altitude = 30;
//########################!!!!!!!!!!!!!!!!########################
char datax[120];
int wing_start_signal_counter = 1;
int deployment_start_signal_counter = 1;
//--------------GPS İÇİN -----------------//

lwgps_t gps;

uint8_t rx_buffer[128];
uint8_t rx_index = 0;
uint8_t rx_data = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);

		if (rx_data != '\n' && rx_index < sizeof(rx_buffer)) {
			rx_buffer[rx_index++] = rx_data;
		} else {
			lwgps_process(&gps, rx_buffer, rx_index + 1);
			rx_index = 0;
			rx_data = 0;
		}
	}

}

//--------------GPS İÇİN -----------------//

BMP280_HandleTypedef bmp280;

float pressure, temperature, humidity;
float first_pressure, altitude, first_altitude;

uint16_t size;
uint8_t Data[256];
uint8_t Rx_data[150];
uint8_t gps_saat, gps_dakika, gps_saniye;

//########################################################################
//------PAKETLİK STRİNG VERİLER-----//
char paket_kanat_durumu[2];
char paket_ayrilma_durumu[2];
char paket_altitude[10];
char paket_lng[10];
char paket_lat[10];
char paket_saat[10];

char paket[58]; // lora ile gönderilecek olan veri paketi

//########################################################################
//------STATE'LER-----//
//########################################################################

//------state management variables-----//
bool state_of_bmp280;
bool state_of_eeprom;
bool state_of_gps;
bool state_of_wings;
bool state_of_deployment;
//------state management variables-----//
//########################################################################
//------EEPROM VERİLER-----//
uint8_t eeprom_paket[62]; // lora ile gönderilecek olan veri paketi

int eeprom_paket_sayac = 1;
uint8_t eeprom_offset;

uint8_t dataRead[150]; //eepromdan veri okumak için
uint8_t dataWrite[100]; //eeproma veri yazmak için

uint8_t eeprom_read_for_transmit[62];
uint8_t eeprom_page = 1;
uint8_t eeprom_page_preview;
//########################################################################

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_I2C2_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(GPIOB, deployment_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, deployment_2_Pin, GPIO_PIN_RESET);

	HAL_Delay(10);
	//--------------GPS İÇİN -----------------//
	lwgps_init(&gps);
	HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	//--------------GPS İÇİN -----------------//

	//state_buzzer_led_init(); //timerları başlatır

	//timer_value = __HAL_TIM_GET_COUNTER(&htim1); //get current timer

	void bmp280_first_pressure_calibrate(float *first_pressure_average,
			int number_of_samples) {
		float temporary_pressure, temporary_temperature, temporary_humidty;
		float first_pressure_datas[number_of_samples], sum_first_pressure_datas;
		int calibrate_time = (((number_of_samples * 50)
				+ (number_of_samples * 10)) / 1000);

		HAL_UART_Transmit(&huart1, (uint8_t*) datax,
				sprintf(datax, "%d sample data reading in %d second...\n",
						number_of_samples, calibrate_time), 1000);
		HAL_Delay(100);

		HAL_GPIO_WritePin(GPIOA, sl_red_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, sl_yellow_Pin, GPIO_PIN_RESET);
		for (int var = 0; var < number_of_samples; ++var) {
			bmp280_read_float(&bmp280, &temporary_temperature,
					&temporary_pressure, &temporary_humidty);
			HAL_Delay(50);
			first_pressure_datas[var] = temporary_pressure;

			HAL_GPIO_TogglePin(GPIOA, sl_red_Pin);

			HAL_GPIO_TogglePin(GPIOA, sl_yellow_Pin);

			HAL_GPIO_TogglePin(GPIOB, buzzer_Pin);

			if (var % 20 == 0) {
				calibrate_time = calibrate_time - 1;
				HAL_UART_Transmit(&huart1, (uint8_t*) datax,
						sprintf(datax, "calibration will end after %d s\n",
								calibrate_time), 1000);
				HAL_GPIO_TogglePin(GPIOA, sl_red_Pin);

				HAL_GPIO_TogglePin(GPIOA, sl_yellow_Pin);

				HAL_GPIO_TogglePin(GPIOB, buzzer_Pin);

			}

		}
		HAL_Delay(500);
		for (int var = 0; var < number_of_samples; ++var) {
			sum_first_pressure_datas = sum_first_pressure_datas
					+ first_pressure_datas[var];
			HAL_Delay(10);

			if (var % 20 == 0) {
				if (calibrate_time > 0) {
					calibrate_time = calibrate_time - 1;
					HAL_UART_Transmit(&huart1, (uint8_t*) datax,
							sprintf(datax, "calibration will end after %d s\n",
									calibrate_time), 1000);
					HAL_GPIO_TogglePin(GPIOA, sl_red_Pin);

					HAL_GPIO_TogglePin(GPIOA, sl_yellow_Pin);

					HAL_GPIO_TogglePin(GPIOB, buzzer_Pin);
				}
			}
		}
		*first_pressure_average = sum_first_pressure_datas / number_of_samples;

		HAL_UART_Transmit(&huart1, (uint8_t*) datax,
				sprintf(datax, "first pressure average: %2.f\n",
						*first_pressure_average), 1000);
		HAL_Delay(50);

	}

	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c2;

	while (!bmp280_init(&bmp280, &bmp280.params)) {
		state_of_bmp280 = false;
		size = sprintf((char*) Data, "BMP280 initialization failed\n");
		HAL_UART_Transmit(&huart1, Data, size, 1000);
		HAL_Delay(2000);
	}
	bool bme280p = bmp280.id == BME280_CHIP_ID;
	size = sprintf((char*) Data, "BMP280: found %s\n",
			bme280p ? "BME280" : "BMP280");
	HAL_UART_Transmit(&huart1, Data, size, 1000);

	while (!bmp280_read_float(&bmp280, &temperature, &first_pressure, &humidity)) {
		size = sprintf((char*) Data, "Altitude reading failed\n");
		HAL_UART_Transmit(&huart1, Data, size, 1000);
		HAL_Delay(2000);
	}

	bool apogee_detecting(float *first_altitude, float *second_altitude) {
		float fark;
		fark = *second_altitude - *first_altitude;
		if (*second_altitude >= minimum_altitude_for_safety) {
			if (fark <= -1.0) {
				return true;
			} else {
				return false;
			}
		} else {
			return false;
		}
	}

	bmp280_first_pressure_calibrate(&first_pressure, 100);
	HAL_Delay(200);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		gps_saat = gps.hours == 55 ? 0 : gps.hours;
		gps_dakika = gps.minutes == 87 ? 0 : gps.minutes;
		gps_saniye = gps.seconds == 247 ? 0 : gps.seconds;

		sprintf(paket_lat, "%f", gps.latitude);
		sprintf(paket_lng, "%f", gps.longitude);
		sprintf(paket_saat, "%d:%d:%d", gps_saat+3, gps_dakika, gps_saniye);
		state_of_gps = gps.fix_mode == '1' ? false : true; //gps'in bulduğu uydu sayısına göre gps durumunu günceller

		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {

			HAL_UART_Transmit(&huart1, (uint8_t*) datax,
					sprintf(datax, "Temperature/pressure reading failed\n"),
					1000);

			HAL_Delay(300);
		}

		first_altitude = altitude;
		HAL_Delay(10);
		bmp280_read_altitude(&bmp280, &first_pressure, &pressure, &altitude);
		sprintf(paket_altitude, "%.2f", altitude);
		HAL_Delay(100);

		HAL_GPIO_TogglePin(GPIOA, sl_red_Pin);

		HAL_GPIO_TogglePin(GPIOA, sl_yellow_Pin);

		HAL_GPIO_TogglePin(GPIOB, buzzer_Pin);

		if (apogee_detecting(&first_altitude, &altitude)
				&& wing_start_signal_counter < 20) {
			sprintf((char*) paket_kanat_durumu, "+");
			HAL_GPIO_WritePin(GPIOB, deployment_1_Pin, GPIO_PIN_SET);
			wing_start_signal_counter++;
			state_of_wings = true;

		} else {
			sprintf((char*) paket_kanat_durumu, "-");
			//state_of_wings = false;

			HAL_GPIO_WritePin(GPIOB, deployment_1_Pin, GPIO_PIN_RESET);
		}

		if (state_of_wings == true
				&& altitude < recovery_system_trigger_altitude
				&& deployment_start_signal_counter < 20) {
			HAL_Delay(5);
			sprintf((char*) paket_ayrilma_durumu, "+");
			HAL_GPIO_WritePin(GPIOB, deployment_2_Pin, GPIO_PIN_SET);
			deployment_start_signal_counter++;
			state_of_deployment = true;

		} else {
			sprintf((char*) paket_ayrilma_durumu, "-");
			//state_of_deployment = false;

			HAL_GPIO_WritePin(GPIOB, deployment_2_Pin, GPIO_PIN_RESET);
		}

		size = sprintf(paket, "%s,%s,%s,%s m,%s,%s\n", paket_saat, paket_lat,
				paket_lng, paket_altitude, paket_kanat_durumu,
				paket_ayrilma_durumu);

		HAL_Delay(300);
		HAL_UART_Transmit(&huart1, (uint8_t*) paket, size, HAL_MAX_DELAY);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	sl_red_Pin | sl_yellow_Pin | GPIO_PIN_8 | GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, deployment_1_Pin | deployment_2_Pin | buzzer_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC14 PC15 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : sl_red_Pin sl_yellow_Pin */
	GPIO_InitStruct.Pin = sl_red_Pin | sl_yellow_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : user_button_Pin PB8 PB9 */
	GPIO_InitStruct.Pin = user_button_Pin | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PA8 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : deployment_1_Pin deployment_2_Pin buzzer_Pin */
	GPIO_InitStruct.Pin = deployment_1_Pin | deployment_2_Pin | buzzer_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
