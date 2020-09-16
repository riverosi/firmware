/*
 * Cátedra: Electrónica Programable
 * FIUNER - 2018
 * Autor/es:
 * JMReta - jmreta@ingenieria.uner.edu.ar
 *
 *
 *
 * Revisión:
 * 07-02-18: Versión inicial
 * 01-04-19: V1.1 SM
 *
 * All rights reserved.
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
/*==================[inclusions]=============================================*/
#include "../../app_9250_M0/inc/app_9250.h"       /* <= own header */

#include "systemclock.h"
#include "chip.h"
#include "cr_start_m0.h"
#include <stdint.h>


/*==================[macros and definitions]=================================*/
#define BUFFER_SIZE 21 /*UART BUFFER DATA*/
/*==================[internal data definition]===============================*/
static volatile uint32_t counter_systick = 0;
/*==================[internal functions declaration]=========================*/

/**
 * @brief convert float in array of bytes
 * @param bytes_temp
 * @param float_variable
 */
void float2Bytes(uint8_t *bytes_temp , float float_variable){
  memcpy(bytes_temp, (uint8_t*) (&float_variable), 4);
};
/*==================[external data definition]===============================*/

/*==================[external functions definition]==========================*/

/* void Interruption_Trigger(void){
		GPIOToggle(LEDRGB_B);
		/Send zeros for trigger signal/
		uint8_t buffer_trigger[BUFFER_SIZE+1],var;
		for (var = 0; var < BUFFER_SIZE; ++var) {
			buffer_trigger[var] = 0;
		}
		buffer_trigger[BUFFER_SIZE] = '\n';
		Chip_UART_SendBlocking(USB_UART, buffer_trigger, sizeof(buffer_trigger));
};
*/
/*Sistick Handler*/
void SysTick_Handler(void){
	counter_systick++;
	if ( counter_systick == 500 ){
		Led_Toggle(RGB_R_LED);
		mpu9250Read();
		uint8_t buffer_data_out[BUFFER_SIZE+1];
		buffer_data_out[BUFFER_SIZE] = '\n';
		mpu9250GetDataBuffer(buffer_data_out);
		Chip_UART_SendBlocking(USB_UART, buffer_data_out, sizeof(buffer_data_out));
		/*Counter Systick Reset*/
		counter_systick = 0;
	}
}
//---------------------------------------------------------------------------------------------------
int main(void)
{
	SystemClockInit();
	SysTick_Config(SystemCoreClock/1000);//llamada systick cada 1ms
	cr_start_m0(SLAVE_M0APP, (uint8_t *)0x1B000000);
	fpuInit();
	StopWatch_Init();
	Init_Uart_Ftdi(460800);
	Init_Leds();
	MPU9250_address_t addr = MPU9250_ADDRESS_0; // If MPU9250 AD0 pin is connected to GND
	int8_t status;
	status = mpu9250Init( addr );
	if( status == 1){
		Led_On(GREEN_LED);
	}
	else{
		Led_On(RED_LED);
	}

	while(TRUE){
		__WFI ();
	};

	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
	   por ningun S.O. */
	return 0;
}

/*==================[end of file]============================================*/

