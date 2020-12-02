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
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRTOPTION) HOWEVER CAUSED AND ON
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
#include <stdbool.h>
#include "clock_config.h"
#include "MKL27Z644.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SEC 2400000

// ---------------- BOTTOM ----------------
#define BOTTOM_CAR_RED 0x00000004
#define BOTTOM_CAR_ORRANGE 0x00000001
#define BOTTOM_CAR_GREEN_1 0x00000002
#define BOTTOM_CAR_GREEN_2 0x00000002

#define BOTTOM_ZEBRA_RED 0x00100000
#define BOTTOM_ZEBRA_GREEN 0x00000001

#define BOTTOM_BUTTON 0x00200000

// ---------------- CENTRAL ----------------
#define CENTRAL_CAR_RED 0x00000020
#define CENTRAL_CAR_ORRANGE 0x00000010
#define CENTRAL_CAR_GREEN_1 0x00000040
#define CENTRAL_CAR_GREEN_2 0x80000000

#define CENTRAL_ZEBRA_RED 0x02000000
#define CENTRAL_ZEBRA_GREEN 0x00001000

#define CENTRAL_BUTTON 0x01000000


// ---------------- TOP----------------
#define TOP_CAR_RED 0x00000080
#define TOP_CAR_ORRANGE 0x00000020
#define TOP_CAR_GREEN_1 0x00000040
#define TOP_CAR_GREEN_2 0x00000080

#define TOP_ZEBRA_RED 0x00000200
#define TOP_ZEBRA_GREEN 0x00000100

#define TOP_BUTTON 0x00010000

static void delay(volatile uint32_t nof);
volatile bool bottom_button;
volatile bool central_button;
volatile bool top_button;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void BOARD_InitPins(void) {
	static const gpio_pin_config_t LED_configOutput_on = {
		kGPIO_DigitalOutput, /* use as output pin */
		1, /* initial value */
	};

	static const gpio_pin_config_t LED_configOutput_off = {
		kGPIO_DigitalOutput, /* use as output pin */
		0, /* initial value */
	};

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

//  CENTRAL CAR SEMAPHORE
	PORT_SetPinMux(PORTA, 5U, kPORT_MuxAsGpio); // RED
	PORT_SetPinMux(PORTC, 4U, kPORT_MuxAsGpio); // ORANGE
	PORT_SetPinMux(PORTC, 6U, kPORT_MuxAsGpio); // GREEN 1
	PORT_SetPinMux(PORTE, 31U, kPORT_MuxAsGpio); // GREEN 2

	GPIO_PinInit(GPIOA, 5U, &LED_configOutput_off); // RED
	GPIO_PinInit(GPIOC, 4U, &LED_configOutput_off); // ORANGE
	GPIO_PinInit(GPIOC, 6U, &LED_configOutput_on); // GREEN 1
	GPIO_PinInit(GPIOE, 31U, &LED_configOutput_on); // GREEN 2

// TOP CAR SEMAPHORE
	PORT_SetPinMux(PORTC, 7U, kPORT_MuxAsGpio); // RED
	PORT_SetPinMux(PORTC, 5U, kPORT_MuxAsGpio); // ORANGE
	PORT_SetPinMux(PORTD, 6U, kPORT_MuxAsGpio); // GREEN 1
	PORT_SetPinMux(PORTD, 7U, kPORT_MuxAsGpio); // GREEN 2

	GPIO_PinInit(GPIOC, 7U, &LED_configOutput_on); // RED
	GPIO_PinInit(GPIOC, 5U, &LED_configOutput_off); // ORANGE
	GPIO_PinInit(GPIOD, 6U, &LED_configOutput_off); // GREEN 1
	GPIO_PinInit(GPIOD, 7U, &LED_configOutput_off); // GREEN 2

//  BOTTOM ZEBRA SEMAPHORE
	PORT_SetPinMux(PORTE, 20U, kPORT_MuxAsGpio); // RED
	PORT_SetPinMux(PORTC, 0U, kPORT_MuxAsGpio); // GREEN

	GPIO_PinInit(GPIOE, 20U, &LED_configOutput_on); // RED
	GPIO_PinInit(GPIOC, 0U, &LED_configOutput_off); // GREEN

//	CENTRAL ZEBRA SEMAPHORES
	PORT_SetPinMux(PORTE, 25U, kPORT_MuxAsGpio); // RED
	PORT_SetPinMux(PORTA, 12U, kPORT_MuxAsGpio); // GREEN

	GPIO_PinInit(GPIOE, 25U, &LED_configOutput_on); // RED
	GPIO_PinInit(GPIOA, 12U, &LED_configOutput_off); // GREEN


//  TOP ZEBRA SEMAPHORE
    PORT_SetPinMux(PORTC, 9U, kPORT_MuxAsGpio); // RED
    PORT_SetPinMux(PORTC, 8U, kPORT_MuxAsGpio); // GREEN

    GPIO_PinInit(GPIOC, 9U, &LED_configOutput_on); // RED
    GPIO_PinInit(GPIOC, 8U, &LED_configOutput_off); // GREEN


//	CENTRAL BUTTON
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[24] = (PORT_PCR_ISF(0x01) /* Nuluj ISF (Interrupt Status Flag) */
		| PORT_PCR_IRQC(0x0A) /* Interrupt enable on failing edge */
		| PORT_PCR_MUX(0x01) /* Pin Mux Control to GPIO */
		| PORT_PCR_PE(0x01) /* Pull resistor enable... */
		| PORT_PCR_PS(0x01));

	PORTE->PCR[21] = (PORT_PCR_ISF(0x01) /* Nuluj ISF (Interrupt Status Flag) */
		| PORT_PCR_IRQC(0x0A) /* Interrupt enable on failing edge */
		| PORT_PCR_MUX(0x01) /* Pin Mux Control to GPIO */
		| PORT_PCR_PE(0x01) /* Pull resistor enable... */
		| PORT_PCR_PS(0x01));

	PORTE->PCR[16] = (PORT_PCR_ISF(0x01) /* Nuluj ISF (Interrupt Status Flag) */
		| PORT_PCR_IRQC(0x0A) /* Interrupt enable on failing edge */
		| PORT_PCR_MUX(0x01) /* Pin Mux Control to GPIO */
		| PORT_PCR_PE(0x01) /* Pull resistor enable... */
		| PORT_PCR_PS(0x01));


	NVIC_ClearPendingIRQ(PORTB_PORTC_PORTD_PORTE_IRQn); /* Nuluj priznak preruseni od portu B */
	NVIC_EnableIRQ(PORTB_PORTC_PORTD_PORTE_IRQn);


}

static void delay(volatile uint32_t nof) {
	while (nof != 0) {
		__asm("NOP");
		nof--;
	}
}

// ---------------- BOTTOM ----------------
void change_red_orange_bottom(void){
	GPIO_ClearPinsOutput(GPIOA, BOTTOM_CAR_RED);
	GPIO_SetPinsOutput(GPIOB, BOTTOM_CAR_ORRANGE);
}


void change_orange_green_bottom(uint8_t arrows){
	GPIO_ClearPinsOutput(GPIOA, BOTTOM_CAR_RED);
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
	GPIO_ClearPinsOutput(GPIOE, BOTTOM_ZEBRA_RED);
	GPIO_SetPinsOutput(GPIOC, BOTTOM_ZEBRA_GREEN);
}

void change_green_red_bottom(void){
	GPIO_SetPinsOutput(GPIOE, BOTTOM_ZEBRA_RED);
	GPIO_ClearPinsOutput(GPIOC, BOTTOM_ZEBRA_GREEN);
}


// ---------------- CENTRAL ----------------
void change_red_orange_central(void){
	GPIO_ClearPinsOutput(GPIOA, CENTRAL_CAR_RED);
	GPIO_SetPinsOutput(GPIOC, CENTRAL_CAR_ORRANGE);
}

void change_orange_green_central(uint8_t arrows){
	GPIO_ClearPinsOutput(GPIOA, CENTRAL_CAR_RED);
	GPIO_ClearPinsOutput(GPIOC, CENTRAL_CAR_ORRANGE);
	switch(arrows){
	case 0:
		GPIO_SetPinsOutput(GPIOC, CENTRAL_CAR_GREEN_1);
		GPIO_SetPinsOutput(GPIOE, CENTRAL_CAR_GREEN_2);
		break;
	case 1:
		GPIO_SetPinsOutput(GPIOC, CENTRAL_CAR_GREEN_1);
		break;
	case 2:
		GPIO_SetPinsOutput(GPIOE, CENTRAL_CAR_GREEN_2);
		break;
	case 3:
		break;
	default:
		break;
	}
}

void change_green_orange_central(void){
	GPIO_ClearPinsOutput(GPIOC, CENTRAL_CAR_GREEN_1);
	GPIO_ClearPinsOutput(GPIOE, CENTRAL_CAR_GREEN_2);
	GPIO_SetPinsOutput(GPIOC, CENTRAL_CAR_ORRANGE);
}

void change_orange_red_central(void){
	GPIO_ClearPinsOutput(GPIOC, CENTRAL_CAR_ORRANGE);
	GPIO_SetPinsOutput(GPIOA, CENTRAL_CAR_RED);
}

void change_red_green_central(void){
	GPIO_ClearPinsOutput(GPIOE, CENTRAL_ZEBRA_RED);
	GPIO_SetPinsOutput(GPIOA, CENTRAL_ZEBRA_GREEN);
}

void change_green_red_central(void){
	GPIO_SetPinsOutput(GPIOE, CENTRAL_ZEBRA_RED);
	GPIO_ClearPinsOutput(GPIOA, CENTRAL_ZEBRA_GREEN);
}

// ---------------- TOP ----------------
void change_red_orange_top(void){
	GPIO_ClearPinsOutput(GPIOC, TOP_CAR_RED);
	GPIO_SetPinsOutput(GPIOC, TOP_CAR_ORRANGE);
}

void change_orange_green_top(uint8_t arrows){
	GPIO_ClearPinsOutput(GPIOC, TOP_CAR_RED);
	GPIO_ClearPinsOutput(GPIOC, TOP_CAR_ORRANGE);
	switch(arrows){
	case 0:
		GPIO_SetPinsOutput(GPIOD, TOP_CAR_GREEN_1);
		GPIO_SetPinsOutput(GPIOD, TOP_CAR_GREEN_2);
		break;
	case 1:
		GPIO_SetPinsOutput(GPIOD, TOP_CAR_GREEN_1);
		break;
	case 2:
		GPIO_SetPinsOutput(GPIOD, TOP_CAR_GREEN_2);
		break;
	case 3:
		break;
	default:
		break;
	}
}

void change_green_orange_top(void){
	GPIO_ClearPinsOutput(GPIOD, TOP_CAR_GREEN_1);
	GPIO_ClearPinsOutput(GPIOD, TOP_CAR_GREEN_2);
	GPIO_SetPinsOutput(GPIOC, TOP_CAR_ORRANGE);
}

void change_orange_red_top(void){
	GPIO_ClearPinsOutput(GPIOC, TOP_CAR_ORRANGE);
	GPIO_SetPinsOutput(GPIOC, TOP_CAR_RED);
}

void change_red_green_top(void){
	GPIO_ClearPinsOutput(GPIOC, TOP_ZEBRA_RED);
	GPIO_SetPinsOutput(GPIOC, TOP_ZEBRA_GREEN);
}

void change_green_red_top(void){
	GPIO_SetPinsOutput(GPIOC, TOP_ZEBRA_RED);
	GPIO_ClearPinsOutput(GPIOC, TOP_ZEBRA_GREEN);
}

void handler_top_only(void){
	top_button = false;

	delay(SEC * 6);

	change_red_orange_bottom();
	change_green_orange_central();
	delay(SEC * 2);

	change_orange_green_bottom(2);
	change_orange_red_central();
	change_orange_green_central(1);
	change_red_green_top();
	delay(SEC * 6);

	change_red_orange_bottom();
	change_green_orange_central();
	change_red_orange_top();
	change_green_red_top();
	delay(SEC * 2);

	change_orange_green_bottom(0);
	change_orange_red_central();
	change_orange_green_top(0);

}

void handler_bottom_only(void){
	bottom_button = false;

	delay(SEC * 6);

	change_red_orange_top();
	change_green_orange_central();
	delay(SEC * 2);

	change_orange_green_top(1);
	change_orange_red_central();
	change_orange_green_central(2);
	change_red_green_bottom();
	delay(SEC * 6);

	change_red_orange_bottom();
	change_green_red_bottom();
	change_green_orange_central();
	change_red_orange_top();
	delay(SEC * 2);

	change_orange_green_bottom(0);
	change_orange_red_central();
	change_orange_green_top(0);
}

void handler_central_only(void){
	central_button = false;

	delay(SEC * 6);

	change_red_orange_bottom();
	change_green_orange_central();
	change_red_orange_top();
	delay(SEC*2);

	change_orange_green_bottom(1);
	change_red_green_central();
	change_orange_red_central();
	change_orange_green_top(2);
	delay(SEC*6);

	change_green_orange_bottom();
	change_green_red_central();
	change_green_orange_top();
	change_red_orange_central();
	delay(SEC*2);

	change_orange_red_bottom();
	change_orange_green_central(0);
	change_orange_red_top();


}

 void PORTB_PORTC_PORTD_PORTE_IRQHandler(void){

	delay(300);
	if (PORTE->ISFR & TOP_BUTTON){
		PORTE->PCR[16] |= PORT_PCR_ISF(1);
		if (GPIO_ReadPinInput(GPIOE, 16U) == 0){
			top_button = true;
		}
	}

	if (PORTE->ISFR & CENTRAL_BUTTON){
		PORTE->PCR[24] |= PORT_PCR_ISF(1);
		if (GPIO_ReadPinInput(GPIOE, 24U) == 0){
			central_button = true;
		}
	}

	if (PORTE->ISFR & BOTTOM_BUTTON){
		PORTE->PCR[21] |= PORT_PCR_ISF(1);
		if (GPIO_ReadPinInput(GPIOE, 21U) == 0){
			bottom_button = true;
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
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	bottom_button = false;
	central_button = false;
	top_button = false;
	for (;;) {
		if (bottom_button){
			handler_bottom_only();
		}
		else if (top_button){
			handler_top_only();
		}
		else if (central_button){
			handler_central_only();
		}
		delay(SEC * 6);

		change_red_orange_bottom();
		change_green_orange_central();
		change_red_orange_top();
		delay(SEC * 3);


		change_orange_green_bottom(0);
		change_orange_green_top(0);
		change_orange_red_central();
		delay(SEC * 6);

		change_green_orange_bottom();
		change_red_orange_central();
		change_green_orange_top();
		delay(SEC * 2);

		change_orange_red_bottom();
		change_orange_green_central(0);
		change_orange_red_top();
	}

	while (1) {
	}
}
