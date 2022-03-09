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
#include "../../test_rs485/inc/mi_proyecto.h"       /* <= own header */
#include "systemclock.h"
/*=====[Inclusions of function dependencies]=================================*/

/*=====[Definition macros of private constants]==============================*/
#define SISTICK_CALL_FREC	1000  /* call SysTick every 1ms 1/1000Hz */
#define ARRAY_SIZE 8
#define BUFFLEN 128
/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/
RINGBUFF_T rbRx;
uint8_t rxBuff[BUFFLEN];
static volatile Bool uart_flag = FALSE;

/*=====[Definitions of private global variables]=============================*/


void app_rs485_irq_config(void) {
	/* UART0 (RS485/Profibus) Only work with this configuration */
	Chip_UART_Init(LPC_USART0);
	Chip_UART_SetBaudFDR(LPC_USART0, 921600);
	Chip_UART_SetupFIFOS(LPC_USART0, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_TRG_LEV3));

	Chip_UART_ReadByte(LPC_USART0);
	Chip_UART_TXEnable(LPC_USART0);

	Chip_SCU_PinMux(9, 5, MD_PDN, FUNC7); /* P9_5: UART0_TXD */
	Chip_SCU_PinMux(9, 6, MD_PLN | MD_EZI | MD_ZI, FUNC7); /* P9_6: UART0_RXD */
	Chip_UART_SetRS485Flags(LPC_USART0,	UART_RS485CTRL_DCTRL_EN | UART_RS485CTRL_OINV_1);
	Chip_SCU_PinMux(6, 2, MD_PDN, FUNC2); /* P6_2: UART0_DIR */
	Chip_UART_IntEnable(LPC_USART0, (UART_IER_RBRINT | UART_IER_RLSINT));
	NVIC_SetPriority(USART0_IRQn, 5);
	NVIC_EnableIRQ(USART0_IRQn);
}
/*==================[Init_Hardware]==========================================*/
void Init_Hardware(void) {
	fpuInit();
	StopWatch_Init();
	Init_Uart_Ftdi(115200);
	uint8_t var;
	for (var = 0; var < 8; var++) {
		GPIOInit(CIAA_DO0 + var, GPIO_OUTPUT);
		GPIOInit(CIAA_DI0 + var, GPIO_INPUT);
	}
	app_rs485_irq_config();
}
/*=======================[UART0_IRQHandler]==================================*/
void UART0_IRQHandler(void) {
	Chip_UART_RXIntHandlerRB(LPC_USART0, &rbRx); //pone los datos que se mandan por UART en el ring buffer
	Chip_UART_ReadRB(LPC_USART0, &rbRx, rxBuff, ARRAY_SIZE);
	uart_flag = TRUE;
}
/*=======================[SysTick_Handler]===================================*/
static uint32_t cnt = 0;
void SysTick_Handler(void) {
	if (cnt == 250) {
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
	RingBuffer_Init(&rbRx, rxBuff, 1, BUFFLEN);
	RingBuffer_Flush(&rbRx);
	SysTick_Config(SystemCoreClock / SISTICK_CALL_FREC);/*call systick every 1ms*/
	// ----- Repeat for ever -------------------------
	while (TRUE) {
		if (uart_flag) {
			Chip_UART_SendBlocking(USB_UART, rxBuff, ARRAY_SIZE);
			uart_flag = FALSE;
		}
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

