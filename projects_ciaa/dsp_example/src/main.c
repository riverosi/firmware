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
#include "signal.h"
#include "systemclock.h"

/*=====[Inclusions of function dependencies]=================================*/

/*=====[Definition macros of private constants]==============================*/
#define SISTICK_CALL_FREC	1000  // call SysTick every 1/1000Hz
#define BLOCKSIZE  128
#define UART_BAUDRATE 115200

/*=====[Definitions of extern global variables]==============================*/
extern const float32_t testInput_f32[BLOCKSIZE];
uint32_t blockSize = BLOCKSIZE;

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

/*=====[Main function, program entry point after power on or reset]==========*/

/*=======================[SysTick_Handler]===================================*/
static volatile uint32_t cnt = 0;/** Variable used for SysTick Counter */
void SysTick_Handler(void) {
	cnt++;
	if ((cnt) % 500 == 0) {
		Led_Toggle(RGB_B_LED);
	}
}
int main(void) {

	/* perform the needed initialization here */
	SystemClockInit();
	fpuInit();
	StopWatch_Init();
	Init_Uart_Ftdi(UART_BAUDRATE);
	Init_Leds();
	SysTick_Config(SystemCoreClock / SISTICK_CALL_FREC);/*call systick every 1ms*/
	float rms, power, ptp, iemg;

	union {/** Union data for stream in UART*/
		uint32_t cycles_enlapsed;
		uint8_t rxBuff[4];
	} data_union;

	// ----- Repeat for ever -------------------------
	while (TRUE) {

		DWTStart();
		for (int var = 0; var < 1000; ++var) {
			dsp_emg_rms_f32(testInput_f32, blockSize, &rms);
		}
		data_union.cycles_enlapsed = DWTStop();
		Chip_UART_SendBlocking(USB_UART, &data_union.rxBuff, 4);

		DWTStart();
		for (int var = 0; var < 1000; ++var) {
			dsp_emg_power_f32(testInput_f32, blockSize, &power);
		}
		data_union.cycles_enlapsed = DWTStop();
		Chip_UART_SendBlocking(USB_UART, &data_union.rxBuff, 4);

		DWTStart();
		for (int var = 0; var < 1000; ++var) {
			dsp_emg_ptp_f32(testInput_f32, blockSize, &ptp);
		}
		data_union.cycles_enlapsed = DWTStop();
		Chip_UART_SendBlocking(USB_UART, &data_union.rxBuff, 4);

		DWTStart();
		for (int var = 0; var < 1000; ++var) {
			dsp_emg_iemg_f32(testInput_f32, blockSize, &iemg);
		}
		data_union.cycles_enlapsed = DWTStop();
		Chip_UART_SendBlocking(USB_UART, &data_union.rxBuff, 4);

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

