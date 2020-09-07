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
#include "../../uart_example/inc/uart_example.h"       /* <= own header */

#include "systemclock.h"
#include "chip.h"

/*==================[macros and definitions]=================================*/
uint16_t counter_systick=0;
uint8_t buffer_uart[] = {128,255,0,0,1,1,127,127,0,0,1,1,255,255,127,127,0,0};
/*==================[internal data definition]===============================*/



/*==================[internal functions declaration]=========================*/

/*==================[external data definition]===============================*/

/*==================[external functions definition]==========================*/


//---------------------------------------------------------------------------------------------------
/*Sistick Handler*/
void  SysTick_Handler(void){
	counter_systick++;
	if (counter_systick==500){
		GPIOToggle(LED1);
		Chip_UART_SendBlocking(USB_UART, buffer_uart, sizeof(buffer_uart));
		Chip_UART_SendBlocking(USB_UART,"\n",1);
		counter_systick=0;
	}
}
//---------------------------------------------------------------------------------------------------
int main(void)
{
	SystemClockInit();
	fpuInit();
	StopWatch_Init();
	Init_Uart_Ftdi(460800);
	LedsInit();
	LedOff(LED1);
	SysTick_Config(SystemCoreClock/1000);/*llamada systick cada 1ms*/
	 while(TRUE){
		__WFI ();
	};
	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
	   por ningun S.O. */

	return 0;
}

/*==================[end of file]============================================*/

