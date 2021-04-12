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
#include "led.h"
void interruption_tec_2(void){
	GPIOToggle(LED1);
	StopWatch_DelayMs(500);
	GPIOToggle(LED1);
}
void interruption_tec_3(void){
	GPIOToggle(LED2);
	StopWatch_DelayMs(500);
	GPIOToggle(LED2);
}
void interruption_tec_4(void){
	GPIOToggle(LED3);
	StopWatch_DelayMs(500);
	GPIOToggle(LED3);
}

int main(void) {
	/* perform the needed initialization here */
	SystemClockInit();
	fpuInit();
	StopWatch_Init();
	Init_Uart_Ftdi(115200);
	Init_Leds();

	GPIOInit(TEC_2, GPIO_INPUT);
	GPIOActivInt( GPIOGP0 , TEC_2 , interruption_tec_2 , IRQ_EDGE_FALL);

	GPIOInit(TEC_3, GPIO_INPUT);
	GPIOActivInt( GPIOGP1 , TEC_3 , interruption_tec_3, IRQ_EDGE_RISE);

	GPIOInit(TEC_4, GPIO_INPUT);
	GPIOActivInt( GPIOGP2 , TEC_4 , interruption_tec_4, IRQ_LEVEL_LOW);

	while (TRUE) {
		__WFI();
	};
	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado por ningun S.O. */

	return 0;

}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

