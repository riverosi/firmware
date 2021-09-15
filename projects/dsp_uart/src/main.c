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
#define BLOCKSIZE  128
#define UART_BAUDRATE 460800
#define BUFF_UART_LEN 4*BLOCKSIZE

/*=====[Definitions of extern global variables]==============================*/
uint32_t blockSize = BLOCKSIZE;
bool dataReady = FALSE;
RINGBUFF_T rbRx;

/*=====[Definitions of public global variables]==============================*/
static union {
	/** Union data for stream in UART*/
	float32_t testInput_f32[BLOCKSIZE];
	uint8_t rxBuff[BUFF_UART_LEN];
} data_union;

/*=====[Definitions of private global variables]=============================*/

void uart_init_intact(void) {
	Chip_SCU_PinMuxSet(7, 1, SCU_MODE_PULLDOWN | SCU_MODE_FUNC6);
	Chip_SCU_PinMuxSet(7, 2,
	SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC6);
	Chip_UART_Init( LPC_USART2);
	Chip_UART_ConfigData( LPC_USART2,
	UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS);
	Chip_UART_SetBaud( LPC_USART2, UART_BAUDRATE);
	Chip_UART_SetupFIFOS( LPC_USART2,
			( UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS
					| UART_FCR_TRG_LEV3));
	Chip_UART_IntEnable( LPC_USART2, ( UART_IER_RBRINT | UART_IER_RLSINT));
	NVIC_EnableIRQ(USART2_IRQn);
	Chip_UART_TXEnable( LPC_USART2);
}

void UART2_IRQHandler(void) {
	Chip_UART_RXIntHandlerRB(LPC_USART2, &rbRx);//pone los datos que se mandan por UART en el ring buffer
	if ( RingBuffer_IsFull(&rbRx) )
	{
		Chip_UART_ReadRB( LPC_USART2, &rbRx, &data_union.rxBuff, BUFF_UART_LEN);
		RingBuffer_Flush(&rbRx);
		dataReady = TRUE;
	}
}

/*=======================[SysTick_Handler]===================================*/
static volatile uint32_t cnt = 0;/** Variable used for SysTick Counter */
void SysTick_Handler(void) {
	cnt++;
	if ((cnt)%500 == 0) {
		Led_Toggle(RGB_R_LED);
	}
}
/*=====[Main function, program entry point after power on or reset]==========*/
int main(void) {

	/* perform the needed initialization here */
	SystemClockInit();
	fpuInit();
	StopWatch_Init();
	uart_init_intact();
	Init_Leds();
	GPIOInit(LCD1 , GPIO_OUTPUT);
	SysTick_Config(SystemCoreClock / SISTICK_CALL_FREC);/*call systick every 1ms*/
	RingBuffer_Init(&rbRx, data_union.rxBuff, 1, BUFF_UART_LEN);
	RingBuffer_Flush(&rbRx);
	float rms, power, max, min, mean;
	uint32_t idmax, idmin;
	// ----- Repeat for ever -------------------------
	for(;;) {
		if (dataReady)
		{
			Led_Toggle(RED_LED);
			GPIOOn(LCD1);
			uint16_t var = 0;
			uint32_t cycles_enlapsed;
			DWTStart();
			for (var = 0; var < 1000; ++var) {
				arm_rms_f32(&data_union.testInput_f32, blockSize, &rms);
				//arm_power_f32(&data_union.testInput_f32, blockSize, &power);
				//power = power/BLOCKSIZE;//read the documentation the power is not normalized
				//arm_max_f32(&data_union.testInput_f32, blockSize, &max, &idmax);
				//arm_min_f32(&data_union.testInput_f32, blockSize, &min, &idmin);
				//arm_mean_f32(&data_union.testInput_f32, blockSize, &mean);
			}
			cycles_enlapsed = DWTStop();
			GPIOOff(LCD1);
			dataReady = FALSE;
		}
		//__WFI(); //uncomment for low power apps
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

