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
#define CCAN_TX_MSG_ID (0x200)
#define CCAN_RX_MSG_ID (0x100)
#define CCAN_TX_MSG_REMOTE_ID (0x300)
/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/
uint8_t msg_received_counter = 0;
/*=====[Definitions of private global variables]=============================*/
void Init_ccan(void) {

	Chip_SCU_PinMuxSet(0x3, 1, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC2)); /* CAN RD */
	Chip_SCU_PinMuxSet(0x3, 2, (SCU_MODE_INACT | SCU_MODE_FUNC2)); /* CAN TD */

	uint32_t freq;
	freq = Chip_Clock_GetBaseClocktHz(CLK_BASE_APB3);//68Mhz

	/* Set CCAN peripheral clock under 100Mhz for working stable */
	Chip_Clock_EnableBaseClock(CLK_BASE_APB3);
	Chip_Clock_SetBaseClock(CLK_BASE_APB3, CLKIN_IDIVC, TRUE, FALSE);
	freq = Chip_Clock_GetBaseClocktHz(CLK_BASE_APB3); // Frequency verification

	Chip_CCAN_Init(LPC_C_CAN0);
	Chip_CCAN_SetBitRate(LPC_C_CAN0, 125000); //125000Khz
	Chip_CCAN_EnableInt(LPC_C_CAN0, (CCAN_CTRL_IE | CCAN_CTRL_SIE | CCAN_CTRL_EIE));
	Chip_CCAN_DisableAutoRetransmit(LPC_C_CAN0);
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
void CAN0_IRQHandler(void)
{
	CCAN_MSG_OBJ_T msg_buf;
	uint32_t can_int, can_stat, i;
	while ( (can_int = Chip_CCAN_GetIntID(LPC_C_CAN0)) != 0 ) {
		if (can_int & CCAN_INT_STATUS) {
			can_stat = Chip_CCAN_GetStatus(LPC_C_CAN0);
			// TODO with error or TXOK, RXOK
			if (can_stat & CCAN_STAT_EPASS) {
				return;
			}
			if (can_stat & CCAN_STAT_EWARN) {
				return;
			}
			if (can_stat & CCAN_STAT_BOFF) {
				return;
			}
			Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_TXOK);
			Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_RXOK);
		}
		else if ((1 <= CCAN_INT_MSG_NUM(can_int)) && (CCAN_INT_MSG_NUM(can_int) <= 0x20)) {
			// Process msg num canint
			Chip_CCAN_GetMsgObject(LPC_C_CAN0, CCAN_MSG_IF1, can_int, &msg_buf);
			switch (msg_buf.id) {
			case CCAN_RX_MSG_ID:
				msg_buf.id += 1;
				Chip_CCAN_Send(LPC_C_CAN0, CCAN_MSG_IF1, false, &msg_buf);
				break;

			case CCAN_TX_MSG_ID:
				break;

			case CCAN_TX_MSG_REMOTE_ID:
				msg_received_counter++;
				if (msg_received_counter == 2) {
					Chip_CCAN_DeleteReceiveID(LPC_C_CAN0, CCAN_MSG_IF1, CCAN_TX_MSG_REMOTE_ID);
				}
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
	CCAN_MSG_OBJ_T send_obj;

	send_obj.id = CCAN_TX_MSG_ID;
	send_obj.dlc = 8;
	send_obj.data[0] = 'C';
	send_obj.data[1] = 'I';
	send_obj.data[2] = 'A';
	send_obj.data[3] = 'A';
	send_obj.data[4] = '-';
	send_obj.data[5] = 'N';
	send_obj.data[6] = 'X';
	send_obj.data[7] = 'P';

	Chip_CCAN_Send(LPC_C_CAN0, CCAN_MSG_IF1, false, &send_obj);
	Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_TXOK);

	Chip_CCAN_AddReceiveID(LPC_C_CAN0, CCAN_MSG_IF1, CCAN_RX_MSG_ID);

	NVIC_EnableIRQ(C_CAN0_IRQn);
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

