/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include <inttypes.h>
#include <stdio.h>
#include "clock_config.h"
#include "MKL27Z644.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
static void delay(volatile uint32_t nof);

#define SEC 2400000

#define BOTTOM_CAR_RED 0x0004
#define BOTTOM_CAR_ORRANGE 0x0001
#define BOTTOM_CAR_GREEN_1 0x0002
#define BOTTOM_CAR_GREEN_2 0x0002
#define BOTTON_3 0x1000000

#define BOTTOM_MEN_RED 0x2000000
#define BOTTOM_MEN_GREEN 0x1000

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void BOARD_InitPins(void) {
	static const gpio_pin_config_t LED_configOutput_on = { kGPIO_DigitalOutput, /* use as output pin */
	1, /* initial value */
	};

	static const gpio_pin_config_t LED_configOutput_off = { kGPIO_DigitalOutput, /* use as output pin */
	0, /* initial value */
	};

//	static const gpio_pin_config_t Button_input = { kGPIO_DigitalInput, };
//
//	static const port_interrupt_t Button_config = { kPORT_InterruptFallingEdge,
//			kPORT_PullUp, kPORT_InterruptLogicZero, };

	SIM->COPC = SIM_COPC_COPT(0x00);

	/* Port A Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortA);
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_PortD);
	CLOCK_EnableClock(kCLOCK_PortE);

//  BOTTOM CAR SEMAPHORE
	PORT_SetPinMux(PORTA, 2U, kPORT_MuxAsGpio); // RED
	PORT_SetPinMux(PORTB, 0U, kPORT_MuxAsGpio); // ORRANGE
	PORT_SetPinMux(PORTB, 1U, kPORT_MuxAsGpio); // GREEN 1
	PORT_SetPinMux(PORTA, 1U, kPORT_MuxAsGpio); // GREEN 2

	GPIO_PinInit(GPIOA, 2U, &LED_configOutput_on); // RED
	GPIO_PinInit(GPIOB, 0U, &LED_configOutput_off); // ORRANGE
	GPIO_PinInit(GPIOB, 1U, &LED_configOutput_off); // GREEN 1
	GPIO_PinInit(GPIOA, 1U, &LED_configOutput_off); // GREEN 2

//	ZEBRA 3 SEMAPHORES + BOTTON
	PORT_SetPinMux(PORTE, 25U, kPORT_MuxAsGpio); // RED
	PORT_SetPinMux(PORTA, 12U, kPORT_MuxAsGpio); // GREEN

	GPIO_PinInit(GPIOE, 25U, &LED_configOutput_on); // RED
	GPIO_PinInit(GPIOA, 12U, &LED_configOutput_off); // GREEN

//	S3 Button
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[24] = (PORT_PCR_ISF(0x01) /* Nuluj ISF (Interrupt Status Flag) */
	| PORT_PCR_IRQC(0x0A) /* Interrupt enable on failing edge */
	| PORT_PCR_MUX(0x01) /* Pin Mux Control to GPIO */
	| PORT_PCR_PE(0x01) /* Pull resistor enable... */
	| PORT_PCR_PS(0x01));
    NVIC_ClearPendingIRQ(PORTB_PORTC_PORTD_PORTE_IRQn); /* Nuluj priznak preruseni od portu B */
	NVIC_EnableIRQ(PORTB_PORTC_PORTD_PORTE_IRQn);

//  Red LEDs for car semaphores
    PORT_SetPinMux(PORTA, 5U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 7U, kPORT_MuxAsGpio);

    GPIO_PinInit(GPIOA, 5U, &LED_configOutput_on);
    GPIO_PinInit(GPIOC, 7U, &LED_configOutput_on);

//  Yellow LEDs for car semaphores
	PORT_SetPinMux(PORTC, 4U, kPORT_MuxAsGpio);
	PORT_SetPinMux(PORTC, 5U, kPORT_MuxAsGpio);

	GPIO_PinInit(GPIOC, 4U, &LED_configOutput_on);
	GPIO_PinInit(GPIOC, 5U, &LED_configOutput_on);

//  Green LEDs for car semaphores
	PORT_SetPinMux(PORTC, 6U, kPORT_MuxAsGpio);
	PORT_SetPinMux(PORTD, 6U, kPORT_MuxAsGpio);

	GPIO_PinInit(GPIOC, 6U, &LED_configOutput_on);
	GPIO_PinInit(GPIOD, 6U, &LED_configOutput_on);

//  Red LEDs for zebras semaphores
    PORT_SetPinMux(PORTE, 20U, kPORT_MuxAsGpio);
	PORT_SetPinMux(PORTC, 9U, kPORT_MuxAsGpio);

	GPIO_PinInit(GPIOE, 20U, &LED_configOutput_on);
    GPIO_PinInit(GPIOC, 9U, &LED_configOutput_on);

//  Green LEDs for zebras semaphores
	PORT_SetPinMux(PORTE, 31U, kPORT_MuxAsGpio);
	PORT_SetPinMux(PORTD, 7U, kPORT_MuxAsGpio);

	PORT_SetPinMux(PORTC, 8U, kPORT_MuxAsGpio);
	PORT_SetPinMux(PORTC, 0U, kPORT_MuxAsGpio);

	GPIO_PinInit(GPIOE, 31U, &LED_configOutput_on);
	GPIO_PinInit(GPIOD, 7U, &LED_configOutput_on);
	GPIO_PinInit(GPIOC, 8U, &LED_configOutput_on);
	GPIO_PinInit(GPIOC, 0U, &LED_configOutput_on);
}

static void delay(volatile uint32_t nof) {
	while (nof != 0) {
		__asm("NOP");
		nof--;
	}
}

void change_red_orange_bottom(void){
	GPIO_ClearPinsOutput(GPIOA, BOTTOM_CAR_RED);
	GPIO_SetPinsOutput(GPIOB, BOTTOM_CAR_ORRANGE);
}

void change_orange_green_bottom(uint8_t arrows){
	GPIO_ClearPinsOutput(GPIOB, BOTTOM_CAR_ORRANGE);
	switch(arrows){
	case 0:
		GPIO_SetPinsOutput(GPIOB, BOTTOM_CAR_GREEN_1);
		GPIO_SetPinsOutput(GPIOA, BOTTOM_CAR_GREEN_2);
		break;
	case 1:
		GPIO_SetPinsOutput(GPIOB, BOTTOM_CAR_GREEN_1);
		break;
	case 2:
		GPIO_SetPinsOutput(GPIOA, BOTTOM_CAR_GREEN_2);
		break;
	case 3:
		break;
	default:
		break;
	}
}

void change_green_orange_bottom(void){
	GPIO_ClearPinsOutput(GPIOB, BOTTOM_CAR_GREEN_1);
	GPIO_ClearPinsOutput(GPIOA, BOTTOM_CAR_GREEN_2);
	GPIO_SetPinsOutput(GPIOB, BOTTOM_CAR_ORRANGE);
}

void change_orange_red_bottom(void){
	GPIO_ClearPinsOutput(GPIOB, BOTTOM_CAR_ORRANGE);
	GPIO_SetPinsOutput(GPIOA, BOTTOM_CAR_RED);
}

void change_red_green_bottom(void){
	GPIO_ClearPinsOutput(GPIOE, BOTTOM_MEN_RED);
	GPIO_SetPinsOutput(GPIOA, BOTTOM_MEN_GREEN);
}

void change_green_red_bottom(void){
	GPIO_SetPinsOutput(GPIOE, BOTTOM_MEN_RED);
	GPIO_ClearPinsOutput(GPIOA, BOTTOM_MEN_GREEN);
}

void PORTB_PORTC_PORTD_PORTE_IRQHandler(void){
	static const uint64_t timeout = 300;
	delay(timeout);
	if (PORTE->ISFR & BOTTON_3){
		if (GPIO_ReadPinInput(GPIOE, 24U) == 0){
			PORTE->PCR[24] |= PORT_PCR_ISF(1);
			if (GPIO_ReadPinInput(GPIOA, BOTTOM_CAR_RED) == 1){
				delay(SEC*2);
				change_red_orange_bottom();
				delay(SEC*2);
				change_orange_green_bottom(1);
				change_red_green_bottom();
				delay(SEC*6);
				change_green_orange_bottom();
				change_red_green_bottom();
				delay(SEC*2);



			}
//			else if(GPIO_ReadPinInput(GPIOA, BOTTOM_CAR_RED)){
//
//			} else if(GPIO_ReadPinInput(GPIOA, BOTTOM_CAR_RED)){
//
//			}

//			GPIO_ClearPinsOutput(GPIOE, BOTTOM_MEN_RED);
//			GPIO_SetPinsOutput(GPIOA, BOTTOM_MEN_GREEN);
//			delay(SEC*6);
//			GPIO_SetPinsOutput(GPIOE, BOTTOM_MEN_RED);
//			GPIO_ClearPinsOutput(GPIOA, BOTTOM_MEN_GREEN);

		}
	}
}

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */

int main(void) {
	/* Init board hardware. */
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	for (;;) {
		delay(SEC * 6);

		change_red_orange_bottom();
		delay(SEC * 3);

		change_orange_green_bottom(0);
		delay(SEC * 6);

		change_green_orange_bottom();
		delay(SEC * 2);

		change_orange_red_bottom();

	}

	while (1) {
	}
}
