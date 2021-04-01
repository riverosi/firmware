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
 **
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

typedef struct {
	float force_node; /*flag 0x01 in first byte payload*/
	float time_node;
	bool data_ready;
} nrf24l01p_data;

/*==================[Init_Hardware]=============================================*/
void Init_Hardware(void);
void print_serial_data(void);

int main(void) {

	/* perform the needed initialization here */
	SystemClockInit();
	Init_Hardware();

	nrf24l01_t RX;
	RX.spi.cfg = nrf24l01_spi_default_cfg;
	RX.cs = GPIO1;
	RX.ce = GPIO3;
	RX.irq = GPIO5;
	RX.mode = PRX;
	RX.en_ack_pay = FALSE;

	Nrf24Init(&RX);
	Nrf24SetRXPacketSize(&RX, 0x00, 32); // Set length of pipe 0 in 32
	Nrf24SetRXPacketSize(&RX, 0x01, 32); // Set length of pipe 1 in 32
	Nrf24EnableRxMode(&RX); /* Enable RX mode */
	Nrf24SecondaryDevISRConfig(&RX); /*Config ISR (only use one module in PRX mode on board)*/

	SysTick_Config(SystemCoreClock / 100);/*call systick every 10ms*/
	float float_data = 0.0f;

	nrf24l01p_data RX_data[2] = { 0 };


	while (TRUE) {

		memcpy(&float_data, &rcv_fr_PTX[1], sizeof(float_data));

		/*Pedal L*/
		if (rcv_fr_PTX[0] == 0x01) {
			RX_data[0].data_ready = true;
			RX_data[0].force_node = float_data;
			rcv_fr_PTX[0] = 0x00;
			print_serial_data();
			if (float_data > 0.2) {
				GPIOOn(LEDRGB_R);
			} else {
				GPIOOff(LEDRGB_R);
			}
		} else {
			RX_data[0].data_ready = false;/*Clear the data flags*/
		}
		/*Pedal R*/
		if (rcv_fr_PTX[0] == 0x02) {
			RX_data[1].data_ready = true;
			RX_data[1].force_node = float_data;
			rcv_fr_PTX[0] = 0x00;
			if (float_data > 0.2) {
				GPIOOn(LED1);
			} else {
				GPIOOff(LED1);
			}
		} else {
			RX_data[0].data_ready = false;/*Clear the data flags*/
			RX_data[1].data_ready = false;/*Clear the data flags*/
		}

		__WFI();
	};
	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado por ningun S.O. */

	return 0;
}
// FUNCTIONS DECLARATIONS
void Init_Hardware(void) {
	fpuInit();
	StopWatch_Init();
	Init_Uart_Ftdi(115200);
	Init_Switches();
	Init_Leds();
}

void print_serial_data(void) {
	/* Protocol Serial
	 * lenght: 32 bytes
	 * |0xFF|data_byte|....|data_byte|
	 */
	Chip_UART_SendByte(USB_UART, 0xff); // serial init frame 0xFF
	Chip_UART_SendBlocking(USB_UART, &rcv_fr_PTX[1], sizeof(float));
}

/*==================[SystickHandler]=============================================*/
static uint32_t cnt = 0;
void SysTick_Handler(void) {

	if (cnt == 50) {
		GPIOToggle(LED3);
		cnt = 0;
	}
	cnt++;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

