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

/*
 * Chaque slice a 10 potards
 * Il y a 8 slices, donc 80 potards
 *
 * Solution 1
 * ==========
 * Chaque slice reçoit et transmet le tableau complet
 * On peut bosser avec des DMA sans trop se poser de questions
 * On transmet un tableau de taille 81 (1 + 80 potards)
 * La première case contient le numéro de la slice (FF pour le master)
 * Le master envoie FF 00 00 00 ... à la première slice
 * Elle construit son tableau à partir des données de l'ADC
 * Elle envoie 00 XX XX XX ... à la deuxième slide
 * Etc...
 * À la fin, le master reçoit 07 XX XX XX... le tableau plein
 *
 * 81 * 9 = 729 octets transmis
 * = ~15Hz à 115200 bauds
 * = ~729000 bauds pour 100Hz
 *
 * Solution 2
 * ==========
 * On ne transmet pas le même nombre d'octets pour gagner du temps
 * Le master envoie juste une valeur à la première slice
 * Elle construit son tableau (taille 10) et l'envoie à la deuxième slice
 * Etc...
 *
 * Besoin de savoir qui est qui au début
 *
 * 1 + 10 + 20 + 30 ... + 80 = 361 octets transmis
 * = ~31Hz à 115200 bauds
 * = ~361000 bauds pour 100Hz
 *
 * Solution 3
 * ==========
 * Le plus rapide, je sais pas comment faire
 *
 * -> En mode registre à décalage
 *
 * 80 octets transmis
 * = ~144Hz
 * = ~80000 bauds pour 100Hz
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spatioport.h"
#include "usbd_midi.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef OLDCODE
static uint8_t buffUsbReport[MIDI_EPIN_SIZE] = {0};
extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t uart_transmit_complete = 1;

static uint8_t uart_tx_buf[CC_LENGTH*SLICE_NUMBER];
static uint8_t uart_rx_buf0[CC_LENGTH*SLICE_NUMBER];
static uint8_t uart_rx_buf1[CC_LENGTH*SLICE_NUMBER];
static uint8_t uart_which_buf = 0;

static uint32_t rx_timouts = 0;
static uint32_t tx_errors = 0;
#endif
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
	MX_USB_Device_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_TIM7_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	__HAL_DBGMCU_FREEZE_TIM7();
	spatioport_init();

#ifdef OLDCODE
	uint32_t tickstart = 0;
	for (int tx_it = 0 ; tx_it < CC_LENGTH*SLICE_NUMBER ; tx_it++)
	{
		uart_tx_buf[tx_it] = 0;
		uart_rx_buf0[tx_it] = 0;
		uart_rx_buf1[tx_it] = 0;
	}
#endif

	while (1)
	{
		spatioport_usb_loop();
#ifdef OLDCODE
		// Wait 10ms
		while ((HAL_GetTick() - tickstart) < 10)
		{
		}
		tickstart = HAL_GetTick();
#endif

#ifdef OLDCODE
		// Get the values
		for (int i = 0 ; i < CC_LENGTH*SLICE_NUMBER ; i++) {
			if (HAL_UART_Transmit(&huart2, &uart_tx_buf[i], 1, HAL_MAX_DELAY) != HAL_OK)
			{
				tx_errors++;
			}
			if (uart_which_buf == 0)
			{
				while(HAL_UART_Receive(&huart2, &uart_rx_buf0[i], 1, 10) == HAL_TIMEOUT)
				{
					HAL_UART_Transmit(&huart2, &uart_tx_buf[i], 1, HAL_MAX_DELAY);
					rx_timouts++;
				}
			}
			else
			{
				while(HAL_UART_Receive(&huart2, &uart_rx_buf1[i], 1, 10) == HAL_TIMEOUT)
				{
					HAL_UART_Transmit(&huart2, &uart_tx_buf[i], 1, HAL_MAX_DELAY);
					rx_timouts++;
				}
			}
			//			HAL_Delay(1);
		}
#endif

#ifdef OLDCODE
		// Send if different
		for (int i = 0 ; i < CC_LENGTH*SLICE_NUMBER ; i++)
		{
			uint8_t cc = i;
			uint8_t val;

			// Check difference
			if (uart_rx_buf0[i] != uart_rx_buf1[i])
			{
				if (uart_which_buf == 0)
				{
					val = uart_rx_buf0[i];
				}
				else
				{
					val = uart_rx_buf1[i];
				}

				while (USBD_MIDI_GetState(&hUsbDeviceFS) != MIDI_IDLE)
				{
				}

				buffUsbReport[0] = 0x0B;	// 4 MSB = ??? (maybe wire number?) ; 4 LSB Seems to be the kind of message (B = CC message)
				buffUsbReport[1] = 0xB0;	// 4 MSB = message ; 4 LSB = channel
				buffUsbReport[2] = cc;		// Param 1 (for CC = CC number)
				buffUsbReport[3] = val;	// Param 2 (for CC = CC value)

				USBD_MIDI_SendReport(&hUsbDeviceFS, buffUsbReport, 4);

				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

			}

		}

		uart_which_buf = !uart_which_buf;
#endif

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
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	if (TIM7 == htim->Instance)
	{
		spatioport_timer_irq_handler();
	}
	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
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
