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
#include "../../../examples/test_ccan_read/inc/mi_proyecto.h"       /* <= own header */
#include "systemclock.h"
/*=====[Inclusions of function dependencies]=================================*/

/*=====[Definition macros of private constants]==============================*/
#define SISTICK_CALL_FREC	1000  /*call SysTick every 1ms 1/1000Hz*/
#define CCAN_RX_MSG_ID (0x200) //CCAN RX Message ID
/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

void Init_ccan(void){
	Chip_CCAN_Init(LPC_C_CAN0);
	/* Set CCAN peripheral clock under 50Mhz for working stable */
	Chip_Clock_SetBaseClock(CLK_BASE_APB3, CLKIN_IDIVC, TRUE, FALSE);
	Chip_CCAN_Init(LPC_C_CAN0);
	Chip_CCAN_SetBitRate(LPC_C_CAN0, 500000);//500Khz

	Chip_SCU_PinMux(3, 2, MD_PDN, FUNC2); /* P3_2: CAN_TD */
	Chip_SCU_PinMux(3, 1, MD_PLN | MD_EZI | MD_ZI, FUNC2); /* P3_1: CAN_RD */

	Chip_CCAN_EnableInt(LPC_C_CAN0, (CCAN_CTRL_IE | CCAN_CTRL_SIE | CCAN_CTRL_EIE));
	Chip_CCAN_AddReceiveID(LPC_C_CAN0, CCAN_MSG_IF1, CCAN_RX_MSG_ID);
	NVIC_SetPriority(C_CAN0_IRQn, 5);
	NVIC_EnableIRQ(C_CAN0_IRQn);
}
/*=======================[C_CAN0_IRQn_Handler]===============================*/
void CAN0_IRQHandler(void)
{
	CCAN_MSG_OBJ_T msg_buf;
	uint32_t can_int, can_stat, i;
	while ( (can_int = Chip_CCAN_GetIntID(LPC_C_CAN0)) != 0 )
	{
		if ((1 <= CCAN_INT_MSG_NUM(can_int)) && (CCAN_INT_MSG_NUM(can_int) <= 0x20))
		{
			Chip_CCAN_GetMsgObject(LPC_C_CAN0, CCAN_MSG_IF1, can_int, &msg_buf);
			switch (msg_buf.data[0]) {
				case 'O':
					Led_On(RGB_G_LED);
					break;
				case 'F':
					Led_Off(RGB_G_LED);
					break;

				default:
					break;
			}
		}
	}
}
/*=======================[SysTick_Handler]===================================*/
static uint32_t cnt = 0;
void SysTick_Handler(void) {
	if (cnt == 500) {
		Led_Toggle(RGB_B_LED);
		cnt = 0;
	}
	cnt++;
}
/*=====[Main function, program entry point after power on or reset]==========*/
int main(void) {
	/* perform the needed initialization here */
	SystemClockInit();
	fpuInit();
	StopWatch_Init();
	Init_Uart_Ftdi(115200);
	Init_Leds();
	Init_ccan();
	SysTick_Config(SystemCoreClock / SISTICK_CALL_FREC);/*call systick every 1ms*/
	// ----- Repeat for ever -------------------------
	while (TRUE) {
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

