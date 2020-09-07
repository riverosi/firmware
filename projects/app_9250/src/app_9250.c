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
#include "../inc/app_9250.h"       /* <= own header */
#include "systemclock.h"


/*==================[macros and definitions]=================================*/
#define RISING 1
#define FALLING 0
#define BUFFER_SIZE 14 /*UART BUFFER DATA*/
/*==================[internal data definition]===============================*/
uint16_t counter_systick=0;
uint8_t buffer_trigger[BUFFER_SIZE];

/*==================[internal functions declaration]=========================*/


/*==================[external data definition]===============================*/

/*==================[external functions definition]==========================*/
void MPU1Init(void)
{
	StopWatch_DelayMs(100);
	MPU9250InitI2C(400000,MPU9250_ADDRESS_AD0L);
	/*Si hay error al iniciar imu enciende led RGD rojo, sino led verde*/
	if((MPU9250Begin() & MPU9250SetAccelRange(ACCEL_RANGE_2G) &	MPU9250SetGyroRange(GYRO_RANGE_250DPS)) == TRUE){
		GPIOOn(LED3);
	}
	else{
		GPIOOn(LED2);
	}
};
//---------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------
/*GPIOGP0 Interruption Handler*/
void Interruption_Trigger(void){
		GPIOToggle(LEDRGB_B);
		uint8_t i=0;
		/*Send zeros for trigger signal*/
		while(i<BUFFER_SIZE){
			Chip_UART_SendByte(USB_UART, buffer_trigger[i]);
			i++;
		};
		Chip_UART_SendByte(USB_UART, '\n');
};
//---------------------------------------------------------------------------------------------------
/*Sistick Handler*/
void SysTick_Handler(void){
		counter_systick++;
		if (counter_systick==50){
			/*
			 * Routine*/
			GPIOToggle(LEDRGB_G);
			uint8_t buffer_data_out[14];
			uint8_t buf[2];
			buf[0] = 0x3B;
			I2CRead( MPU9250_ADDRESS_AD0L , buf , 1 , buffer_data_out , 14);
			Chip_UART_SendBlocking(USB_UART, buffer_data_out, sizeof(buffer_data_out));
			Chip_UART_SendBlocking(USB_UART, "\n" , 1);
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
	MPU1Init();
	GPIOInit(TEC_1, GPIO_INPUT);
	GPIOActivInt( GPIOGP0 , TEC_1 , Interruption_Trigger , FALLING);
	SysTick_Config(SystemCoreClock/1000);/*llamada systick cada 1ms*/
	while(TRUE){
		__WFI ();
	};
	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
	   por ningun S.O. */
	return 0;
}

/*==================[end of file]============================================*/

