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
 * ##Transmitter - Reciber NRF24L01 ##
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
/** @section wiring Wiring
 * ##Transmitter - Reciber NRF24L01 ##
 *
 * | NRF24L01 pins (PTX) | CIAA pins |
 * |:-------------------:|:-------------:|
 * |         VCC         |    +3.3V      |
 * |         GND         |    GND        |
 * |         CSN         |    T_FIL0     |
 * |         CE	         |    T_FIL2     |
 * |         SCK         |    SPI_SCK    |
 * |         MOSI        |    SPI_MOSI   |
 * |         IRQ         |    T_FIL3     |
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
/*
 * In this test connect two modules and debug
 */
#define asTX 1
#define asRX 1

/*==================[Init_Hardware]=============================================*/
void Init_Hardware(void) {
	fpuInit();
	StopWatch_Init();
	Init_Uart_Ftdi(115200);
}
/*==================[SystickHandler]=============================================*/
uint32_t cnt = 0;
void SysTick_Handler(void) {


	if (cnt == 20) {

#if asTX
		GPIOToggle(LEDRGB_R);
		Nrf24TxTick();
#endif
		GPIOToggle(LEDRGB_R);
		cnt = 0;
	}
	cnt+=10;
}

int main(void) {

	/* perform the needed initialization here */
	SystemClockInit();
	Init_Hardware();
	Init_Switches();
	Init_Leds();
	/*	Select mode of NRF	*/
#if asTX

	nrf24l01_t TX;
	TX.spi.cfg = nrf24l01_spi_default_cfg;
	TX.cs = GPIO1;
	TX.ce = GPIO3;
	TX.irq = GPIO5;
	TX.mode = PTX;
	TX.en_ack_pay = TRUE;

	Nrf24Init(&TX);
	/* Enable ack payload */
	Nrf24EnableFeatureAckPL(&TX);

	Nrf24PrimaryDevISRConfig(&TX);

	uint8_t tx_config = Nrf24RegisterRead8(&TX , NRF24_CONFIG);
#endif

#if asRX

	nrf24l01_t RX;
	RX.spi.cfg = nrf24l01_spi_default_cfg;
	RX.cs = T_FIL0;
	RX.ce = T_FIL2;
	RX.irq = T_FIL3;
	RX.mode = PRX;
	RX.en_ack_pay = TRUE;

	Nrf24Init(&RX);

	/* Enable ack payload */
	Nrf24EnableFeatureAckPL(&RX);

	/* Set the first ack payload */
	uint8_t first_ack[21] = "First ack received!!!";
	Nrf24SetAckPayload(&RX,first_ack, 0x00, 21);

	/* Enable RX mode */
	Nrf24EnableRxMode(&RX);
	Nrf24SecondaryDevISRConfig(&RX); // GPIO5_IRQHandler

	uint8_t rx_config = Nrf24RegisterRead8(&RX , NRF24_CONFIG);

#endif

	SysTick_Config(SystemCoreClock / 100);/*call systick every 1ms*/

	uint8_t key = 0;

	//Variables

	while (TRUE) {

#if asTX
		//Read state of switches and save in buffer for send to PRX
		key = Read_Switches();
		snd_to_PRX[0] = key;
#endif

#if asRX
		/* Turns on led associated with button if data is received from PTX */
		if(rcv_fr_PTX[0]==1) {
			GPIOOn(LED1);
			StopWatch_DelayMs(100);
			GPIOOff(LED1);
		}
		if(rcv_fr_PTX[0]==2) {
			GPIOOn(LED2);
			StopWatch_DelayMs(100);
			GPIOOff(LED2);
		}
		if(rcv_fr_PTX[0]==4) {
			GPIOOn(LED3);
			StopWatch_DelayMs(100);
			GPIOOff(LED3);
		}

#endif

		__WFI();
	};
	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado por ningun S.O. */

	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

