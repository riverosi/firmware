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
/** @section wiring Wiring
 * ##Transmitter NRF24L01 ##
 *
 * | NRF24L01 pins (PTX) | EDU-CIAA pins |
 * |:-------------------:|:-------------:|
 * |         VCC         |    +3.3V      |
 * |         GND         |    GND        |
 * |         CSN         |    GPIO1      |
 * |         CE	         |    GPIO3      |
 * |         SCK         |    SPI_SCK    |
 * |         MOSI        |    SPI_MOSI   |
 * |         IRQ         |    GPIO5      |
 * |         MISO        |    SPI_MISO   |
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "../../app_nrf24l01/inc/mi_proyecto.h"       /* <= own header */
#include "systemclock.h"
#include "chip.h"
/*==================[Definitions]=============================================*/
/* Change 1 to activate */
#define asTX 1
/*==================[Init_Hardware]=============================================*/
void Init_Hardware(void){
	fpuInit();
	StopWatch_Init();
	Init_Switches();
	Init_Leds();
	Init_Uart_Ftdi(460800);
}
/*==================[SystickHandler]=============================================*/
void SysTick_Handler(void){
	static uint32_t cnt = 0;
	cnt = cnt % 200;

	if (cnt == 0){

		GPIOToggle(LED1);
#if asTX
		Nrf24TxTick();
#endif

	}
	cnt ++;
}

int main(void)
{ 

/* perform the needed initialization here */
	SystemClockInit();
	Init_Hardware();

/*	Select mode of NRF	*/
#if asTX

	nrf24l01_t TX;
	TX.spi.cfg = nrf24l01_spi_default_cfg;
	TX.cs = GPIO1;
	TX.ce = GPIO3;
	TX.irq = GPIO5;
	TX.mode = PTX;
	TX.en_ack_pay = TRUE;

	if ( Nrf24Init(&TX) == NRF24_SUCCESS) {
		GPIOOn(LEDRGB_G);
	}
	else {
		GPIOOn(LEDRGB_R);
	}

	/* Enable ack payload */
	Nrf24EnableFeatureAckPL(&TX);

	uint8_t ucRxAddr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };// Set RX Address
	uint8_t ucTxAddr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };// Set TX Address

	Nrf24SetRxAddress( &TX , NRF24_PIPE0 ,  ucRxAddr );
	Nrf24SetTXAddress( &TX , ucTxAddr );

	/* Set buffer data */
	snd_to_PRX[0] = 1;
	Nrf24EnableTxMode(&TX);
	Nrf24PrimaryDevISRConfig(&TX);


#endif



	SysTick_Config(SystemCoreClock/1000);/*call systick every 1ms*/


    //Variables
   
	while(TRUE){
			__WFI ();
		};
	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado por ningun S.O. */

	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

