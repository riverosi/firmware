/* Copyright 2018, Eduardo Filomena - Gonzalo Cuenca
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Blinking Bare Metal example source file
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "mi_proyecto.h"       /* <= own header */
#include "systemclock.h"
#include <string.h>
/*=====[Inclusions of function dependencies]=================================*/

/*=====[Definition macros of private constants]==============================*/
#define SISTICK_CALL_FREC	1000  /*call SysTick every 1ms 1/1000Hz*/
#define ARRAY_SIZE 8
/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/
void printString(uint16_t* data){
	uint8_t  arr2[sizeof(uint16_t)*ARRAY_SIZE] = {0};
	memcpy(&arr2, data, sizeof(uint16_t)*ARRAY_SIZE);
	SendStringRs485(arr2, sizeof(uint16_t)*ARRAY_SIZE);
}

/*=======================[SysTick_Handler]===================================*/
void SysTick_Handler(void) {

}
/*=====[Main function, program entry point after power on or reset]==========*/
int main(void) {

	/* perform the needed initialization here */
	SystemClockInit();
	fpuInit();
	StopWatch_Init();
	Init_Leds();
	Init_Uart_Rs485();
	SysTick_Config(SystemCoreClock / SISTICK_CALL_FREC);/*call systick every 1ms*/
	uint16_t data_array[ARRAY_SIZE] = {0xFFFF,0,0xFFFF,0,0xFFFF,0,0xFFFF,0};
	// ----- Repeat for ever -------------------------
	while (TRUE) {
		Led_Toggle(RGB_B_LED);
		printString(data_array);
		StopWatch_DelayMs(100);
		__WFI();
	}

	// YOU NEVER REACH HERE, because this program runs directly or on a
	// microcontroller and is not called by any Operating System, as in the
	// case of a PC program.

	return 0;

}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

