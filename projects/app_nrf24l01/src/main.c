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
#include "../../app_nrf24l01/inc/mi_proyecto.h"       /* <= own header */
#include "systemclock.h"
#include "chip.h"
/*==================[Definitions]=============================================*/
/*
 * Uncomment the select mode*/
#define asRX 1
#define asTX 1

void GPIO4_IRQHandler(void){
	NVIC_DisableIRQ( PIN_INT4_IRQn);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH(4));
	GpioInterrupt(4);
	NVIC_EnableIRQ( PIN_INT4_IRQn);
}

void GPIO5_IRQHandler(void){
	NVIC_DisableIRQ( PIN_INT5_IRQn);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH(5));
	GpioInterrupt(5);
	NVIC_EnableIRQ( PIN_INT5_IRQn);
}
/*==================[SystickHandler]=============================================*/
void SysTick_Handler(void){
	static uint32_t cnt = 0;
	cnt = cnt % 500;

	if (cnt == 0){
		Led_Toggle(RGB_R_LED);
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
	fpuInit();
	StopWatch_Init();
	Init_Switches();
	Init_Uart_Ftdi(460800);
	Init_Leds();

/*	Select mode of NRF	*/
#if asTX
	nrf24l01_t TX;
	TX.spi.cfg=nrf24l01_spi_default_cfg;
	TX.cs.n=GPIO_0;
	TX.ce.n=GPIO_1;
	TX.irq.n=GPIO_4;
	TX.mode=PTX;
	TX.en_ack_pay=TRUE;
	TX.pin_int_num=4;

	Nrf24Init(&TX);

	/* Enable ack payload */
	Nrf24EnableFeatureAckPL(&TX);

	Nrf24PrimaryDevISRConfig(&TX);
#endif



#if asRX
	nrf24l01_t RX;
	RX.spi.cfg=nrf24l01_spi_default_cfg;
	RX.cs.n=GPIO_2;
	RX.ce.n=GPIO_3;
	RX.irq.n=GPIO_5;
	RX.mode=PRX;
	RX.en_ack_pay=TRUE;
	RX.pin_int_num=5;

	Nrf24Init(&RX);

	/* Enable ack payload */
	Nrf24EnableFeatureAckPL(&RX);

	/* Set the first ack payload */
	uint8_t first_ack[21] = "First ack received!!!";
	Nrf24SetAckPayload(&RX,first_ack, 0x00, 21);

	/* Enable RX mode */
	Nrf24EnableRxMode(&RX);

	Nrf24SecondaryDevISRConfig(&RX); // GPIO5_IRQHandler
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

