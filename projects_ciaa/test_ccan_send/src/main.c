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
#include "../../test_gpio/inc/mi_proyecto.h"       /* <= own header */
#include "systemclock.h"
/*=====[Inclusions of function dependencies]=================================*/

/*=====[Definition macros of private constants]==============================*/
#define SISTICK_CALL_FREC	1000  /*call SysTick every 1ms 1/1000Hz*/
/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/
void Init_ccan(void) {
	Chip_CCAN_Init(LPC_C_CAN0);
	/* Set CCAN peripheral clock under 50Mhz for working stable */
	Chip_Clock_SetBaseClock(CLK_BASE_APB3, CLKIN_IDIVC, TRUE, FALSE);
	Chip_CCAN_Init(LPC_C_CAN0);
	Chip_CCAN_SetBitRate(LPC_C_CAN0, 500000); //500Khz

	Chip_SCU_PinMux(3, 2, MD_PDN, FUNC2); /* P3_2: CAN_TD */
	Chip_SCU_PinMux(3, 1, MD_PLN | MD_EZI | MD_ZI, FUNC2); /* P3_1: CAN_RD */
}
/*==================[Init_Hardware]==========================================*/
void Init_Hardware(void) {
	fpuInit();
	StopWatch_Init();
	uint8_t var;
	for (var = 0; var < 8; var++) {
		GPIOInit(CIAA_DO0 + var, GPIO_OUTPUT);
		GPIOInit(CIAA_DI0 + var, GPIO_INPUT);
	}
	Init_ccan();
}
/*=======================[SysTick_Handler]===================================*/
static uint32_t cnt = 0;
void SysTick_Handler(void) {
	if (cnt == 500) {
		GPIOToggle(CIAA_DO7);
		cnt = 0;
	}
	cnt++;
}
/*=====[Main function, program entry point after power on or reset]==========*/

int main(void) {

	/* perform the needed initialization here */
	SystemClockInit();
	Init_Hardware();
	SysTick_Config(SystemCoreClock / SISTICK_CALL_FREC);/*call systick every 1ms*/

	CCAN_MSG_OBJ_T send_obj; // CCAN object

	// ----- Repeat for ever -------------------------
	while (TRUE) {
		send_obj.id = 0x200; //CCAN_TX_MSG_ID 0x200
		send_obj.dlc = 1;
		send_obj.data[0] = 'O';
		Chip_CCAN_Send(LPC_C_CAN0, CCAN_MSG_IF1, FALSE, &send_obj);
		Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_TXOK);
		StopWatch_DelayMs(500);
		send_obj.data[0] = 'F';
		Chip_CCAN_Send(LPC_C_CAN0, CCAN_MSG_IF1, FALSE, &send_obj);
		Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_TXOK);
		StopWatch_DelayMs(500);
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

