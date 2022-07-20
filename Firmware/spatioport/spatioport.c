/*
 * spatioport.c
 *
 *  Created on: Jul 18, 2022
 *      Author: laurentf
 */

#include "spatioport.h"

#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

#include "usbd_midi.h"

#define IS_DEBUG 0

#define _HEADER_SIZE 4
#define _RX_BUF_SIZE CC_LENGTH*SLICE_NUMBER

static spatioport_fsm_t spatioport_fsm = SPATIOPORT_STATE_HEADER;

static bool ok_to_send = false;
const uint8_t header_tx[_HEADER_SIZE] = "spat";
static uint32_t header_tx_counter = 0;

// Store 2 buffers to compare new to old values
static uint8_t uart_rx_buf[_RX_BUF_SIZE];
static uint8_t uart_mem_buf[_RX_BUF_SIZE];

static uint8_t uart_counter = 0;

static uint8_t buffer_is_full = 0;

static uint8_t buffUsbReport[MIDI_EPIN_SIZE] = {0};
extern USBD_HandleTypeDef hUsbDeviceFS;

static uint32_t nok_to_send = 0;

void spatioport_init(void)
{
	HAL_Delay(1000);

	LL_USART_EnableIT_RXNE_RXFNE(USART2);

	for (int it = 0 ; it < _RX_BUF_SIZE ; it++)
	{
		uart_mem_buf[it] = 0;
	}

	HAL_TIM_Base_Start_IT(&htim7);

	ok_to_send = true;
}

void spatioport_timer_irq_handler(void)
{
	if (ok_to_send)
	{
		switch (spatioport_fsm)
		{
		case SPATIOPORT_STATE_HEADER:
			ok_to_send = false;		// We want to be sure to get a data before sending the next one
			LL_USART_TransmitData8(USART2, header_tx[header_tx_counter]);

			break;
		case SPATIOPORT_STATE_FORWARD_CC:
//			ok_to_send = false;		// It's a bit slow with that, so I don't care... It seems to work anyway
			LL_USART_TransmitData8(USART2, 0xFF);
			break;
		}
	}
	else
	{
		nok_to_send++;
	}
}

void spatioport_uart_irq_handler(void)
{
	uint8_t ch_in = LL_USART_ReceiveData8(USART2);

	switch (spatioport_fsm)
	{
	case SPATIOPORT_STATE_HEADER:
		if (header_tx[header_tx_counter] == ch_in)
		{
			// Header character as expected, let's continue
			header_tx_counter++;
		}
		else
		{
			// We didn't get the header as expected
			// It usually happens during debug : you need to reset the slices
			// If it happens during release, we're screwed I guess
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			header_tx_counter = 0;
		}

		if (_HEADER_SIZE == header_tx_counter)
		{
			// Header transmission complete, let's get that juicy CC values
			header_tx_counter = 0;
			spatioport_fsm = SPATIOPORT_STATE_FORWARD_CC;
		}

		ok_to_send = true;
		break;
	case SPATIOPORT_STATE_FORWARD_CC:
		uart_rx_buf[uart_counter] = ch_in;

		uart_counter++;

		// First buffer is full
		if (_RX_BUF_SIZE == uart_counter)
		{
			buffer_is_full = 1;
			uart_counter = 0;
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		}

		ok_to_send = true;
		break;
	}
}

void spatioport_usb_loop(void)
{
	if (buffer_is_full != 0)
	{
		buffer_is_full = 0;

		// Send if different
		for (int i = 0 ; i < _RX_BUF_SIZE ; i++)
		{
			uint8_t cc = i;
			uint8_t val;

			/* Check difference
			 */
			int8_t diff = uart_rx_buf[i] - uart_mem_buf[i];

			if (diff != 0)
			{
				if ((uart_rx_buf[i] == 0) || (uart_rx_buf[i] == 127))
				{
					diff = 10;
				}
			}
			if ((diff < -1) || (diff > 1))
			{
				uart_mem_buf[i] = uart_rx_buf[i];
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

				val = uart_rx_buf[i];

				while (USBD_MIDI_GetState(&hUsbDeviceFS) != MIDI_IDLE)
				{
				}

				buffUsbReport[0] = 0x0B;	// 4 MSB = ??? (maybe wire number?) ; 4 LSB Seems to be the kind of message (B = CC message)
				buffUsbReport[1] = 0xB0;	// 4 MSB = message ; 4 LSB = channel
				buffUsbReport[2] = cc;		// Param 1 (for CC = CC number)
				buffUsbReport[3] = val;	// Param 2 (for CC = CC value)

				USBD_MIDI_SendReport(&hUsbDeviceFS, buffUsbReport, 4);
			}
		}
	}
}
