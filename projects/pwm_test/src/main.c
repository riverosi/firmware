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
/*=====[Inclusions of function dependencies]=================================*/
#define SISTICK_CALL_FREC	1000
/*=====[Definition macros of private constants]==============================*/

/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/
/** Variable used for SysTick Counter */
static volatile uint32_t cnt = 0;
/** Variable used for update dutyCycle */
static volatile int dutyCycle_led1 = 0;
/*=====[Definitions of private global variables]=============================*/
void interruption_tec_2(void) {
	/* Increment duty */
	dutyCycle_led1 = dutyCycle_led1 + 8;
	if (dutyCycle_led1 > 255) {
		dutyCycle_led1 = 255;
	}
}
void interruption_tec_3(void) {
	/* Decrement duty */
	dutyCycle_led1 = dutyCycle_led1 - 8;
	if (dutyCycle_led1 < 0) {
		dutyCycle_led1 = 0;
	}
}
void interruption_tec_4(void){
	dutyCycle_led1 = 0;
}

void SysTick_Handler(void) {
	if ((cnt % 50) == 0) {
		pwmWrite(PWM7, (uint8_t)dutyCycle_led1);// LED1 -> not inverter
		pwmWrite(PWM0, ~(uint8_t)dutyCycle_led1);// T_FIL1 -> Not mask is used
	}
	cnt++;
}

/*=====[Main function, program entry point after power on or reset]==========*/
int main(void) {

	/* perform the needed initialization here */
	SystemClockInit();
	fpuInit();
	StopWatch_Init();
	Init_Leds();
	pwmInit(0,PWM_ENABLE); // Enable pwm
	pwmInit(PWM0, PWM_ENABLE_OUTPUT);/* T_FIL1 */
	pwmInit(PWM7, PWM_ENABLE_OUTPUT);/*LED1 PWM*/
	pwmInit(PWM9, PWM_ENABLE_OUTPUT);/*LED3 PWM*/
	GPIOInit(TEC_2, GPIO_INPUT);
	GPIOActivInt(GPIOGP0, TEC_2, interruption_tec_2, IRQ_LEVEL_LOW);
	GPIOInit(TEC_3, GPIO_INPUT);
	GPIOActivInt(GPIOGP1, TEC_3, interruption_tec_3, IRQ_LEVEL_LOW);
	GPIOInit(TEC_4, GPIO_INPUT);
	GPIOActivInt(GPIOGP2, TEC_4, interruption_tec_4, IRQ_LEVEL_LOW);
	SysTick_Config(SystemCoreClock / SISTICK_CALL_FREC);/*call systick every 1ms*/
	// ----- Repeat for ever -------------------------
	while (TRUE) {
		uint8_t var;
		for (var = 0; var < 127; var++) {
			pwmWrite(PWM9, var);
			StopWatch_DelayMs(10);
		}

		for (var = 127; var > 0 ; var--) {
			pwmWrite(PWM9, var);
			StopWatch_DelayMs(10);
		}
		pwmWrite(PWM9, 0);
		StopWatch_DelayMs(500);
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

