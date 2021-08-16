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
// DSP libs
#define ARM_MATH_CM4
#define __FPU_PRESENT 1
#include "arm_math.h"
#include "arm_const_structs.h"

/*=====[Definition macros of private constants]==============================*/
#define SISTICK_CALL_FREC	1000  // call SysTick every 1/1000Hz
#define MAX_BLOCKSIZE   63

/*=====[Definitions of extern global variables]==============================*/
float32_t testInput_f32[MAX_BLOCKSIZE] = {
		0,    0.0998,    0.1987,    0.2955,    0.3894,    0.4794,    0.5646,    0.6442,    0.7174,    0.7833,    0.8415,    0.8912,    0.9320,    0.9636,    0.9854,    0.9975,    0.9996,    0.9917,    0.9738,    0.9463,    0.9093,    0.8632,    0.8085,    0.7457,    0.6755,    0.5985,    0.5155,    0.4274,    0.3350,    0.2392,    0.1411,    0.0416,   -0.0584,   -0.1577,   -0.2555,   -0.3508,   -0.4425,   -0.5298,   -0.6119,   -0.6878,   -0.7568,   -0.8183,   -0.8716,   -0.9162,   -0.9516,   -0.9775,   -0.9937,   -0.9999,   -0.9962,   -0.9825,   -0.9589,   -0.9258,   -0.8835,   -0.8323,   -0.7728,   -0.7055,   -0.6313,   -0.5507,   -0.4646,   -0.3739,   -0.2794,   -0.1822,   -0.0831
};
uint32_t blockSize = MAX_BLOCKSIZE;
/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

/*=====[Main function, program entry point after power on or reset]==========*/

/*=======================[SysTick_Handler]===================================*/
void SysTick_Handler(void) {

}
int main(void) {

	/* perform the needed initialization here */
	SystemClockInit();
	fpuInit();
	StopWatch_Init();
	Init_Uart_Ftdi(115200);
	Init_Leds();
	SysTick_Config(SystemCoreClock / SISTICK_CALL_FREC);/*call systick every 1ms*/
	float rms, power, max, min, mean;
	uint32_t idmax, idmin;
	// ----- Repeat for ever -------------------------
	while (TRUE) {
		Led_Toggle(RGB_R_LED);
		StopWatch_DelayMs(500);
		arm_rms_f32(&testInput_f32, blockSize, &rms);
		arm_power_f32(&testInput_f32, blockSize, &power);
		arm_max_f32(&testInput_f32, blockSize, &max, &idmax);
		arm_min_f32(&testInput_f32, blockSize, &min, &idmin);
		arm_mean_f32(&testInput_f32, blockSize, &mean);
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

