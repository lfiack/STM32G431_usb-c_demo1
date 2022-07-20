/*
 * spatioport.h
 *
 *  Created on: Jul 18, 2022
 *      Author: laurentf
 */

#ifndef SPATIOPORT_SPATIOPORT_H_
#define SPATIOPORT_SPATIOPORT_H_

#include <stdint.h>
#include <stdbool.h>

#define CC_LENGTH 10
#define SLICE_NUMBER 8

typedef enum
{
	SPATIOPORT_STATE_HEADER,
	SPATIOPORT_STATE_FORWARD_CC
} spatioport_fsm_t;

void spatioport_init(void);
void spatioport_uart_irq_handler(void);
void spatioport_timer_irq_handler(void);
void spatioport_usb_loop(void);

#endif /* SPATIOPORT_SPATIOPORT_H_ */
