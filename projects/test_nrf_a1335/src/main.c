/* Copyright 2021, Riveros Ignacio
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
#include "mi_proyecto.h"       /* <= own header */
#include "systemclock.h"
#include "chip.h"
/*==================[Definitions]============================================*/
#define SYSTICK_CALL_FREC	100  /*call SysTick every 10ms 1/100Hz*/
#define ANGLE_I2C_CLK 100000


typedef struct {
	float force_node; /** <= force in [Kg]*/
	float time_node; /** <= time in [ms]*/
	float battery_voltage; /** <= voltage in battery [Volts]*/
	bool data_ready; /** <= data ready flag*/
} nrf24l01p_pedal_data;


/*=====[Inclusions of function dependencies]=================================*/

/*=====[Definition macros of private constants]==============================*/

/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global functions]==============================*/
void Init_Hardware(void) {
	fpuInit();
	StopWatch_Init();
	Init_Uart_Ftdi(115200);
	Init_Switches();
	Init_Leds();
	StopWatch_DelayMs(100);
	angle_i2cDriverInit(ANGLE_I2C_CLK, 0x0C);
	angle_setConfig(
			_ANGLE_CDS_NO_CHANGLE | _ANGLE_HDR_RESET_1 | _ANGLE_SFR_RESET_1
					| _ANGLE_CSR_STA_1 | _ANGLE_CXE_1 | _ANGLE_CER_1);
	dacInit(DAC_ENABLE);
}
void print_serial_data(nrf24l01p_pedal_data *rx_buffer) {
	/* Protocol Serial:
	 * length: 1 + 4*N bytes
	 * |0xFF|4 x data_float|4 x data_float|
	 */
	Chip_UART_SendByte(USB_UART, 0xff); // serial init frame is 0xFF
	Chip_UART_SendBlocking(USB_UART, &(rx_buffer)->force_node, sizeof(float));
	Chip_UART_SendBlocking(USB_UART, &(rx_buffer + 1)->force_node,
			sizeof(float));
}
void clear_array(void) {
	memset(rcv_fr_PTX, 0x00, 32); //Set array of data input whit zeros
}
/*=====[Definitions of public global variables]=============================*/
/** Variable used for SysTick Counter */
static volatile uint32_t cnt = 0;
/** Flag for print data in serial port*/
static volatile bool flag_serial_data_print = FALSE;
/*==================[SystickHandler]=========================================*/
void SysTick_Handler(void) {
	if ((cnt % 5) == 0) { /*flag change every 50 ms*/
		flag_serial_data_print = TRUE;
	}
	if (cnt == 50) { /*LED3 toggle every 500 ms*/
		Led_Toggle(GREEN_LED);
		cnt = 0;
	}
	cnt++;
}
/*=====[Main function, program entry point after power on or reset]==========*/
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
	Nrf24SetRXPacketSize(&RX, 0x00, 32); // Set length of pipe 0 in 32 (used for the Pedal Left)
	Nrf24SetRXPacketSize(&RX, 0x01, 32); // Set length of pipe 1 in 32 (used for the Pedal Right)
	Nrf24EnableRxMode(&RX); /* Enable RX mode */
	Nrf24SecondaryDevISRConfig(&RX); /* Config ISR (only use one module in PRX mode on board)*/

	SysTick_Config(SystemCoreClock / SYSTICK_CALL_FREC);

	float float_data = 0.0f;
	/* RX_data[0] Store Pedal Left data
	 * RX_data[1] Store Pedal Rigth data
	 */
	nrf24l01p_pedal_data RX_data[2] = { 0 };
	uint16_t angle;

	for(;;) {

		angle = angle_getAngle();

		dacWrite(angle);

		memcpy(&float_data, &rcv_fr_PTX[1], sizeof(float_data)); /*Convert array data to float data*/

		if (rcv_fr_PTX[0] == 0x01) {/*Pedal L*/
			RX_data[0].data_ready = true;
			RX_data[0].force_node = float_data;
			if (float_data > 0.2) {
				Led_On(RGB_B_LED);
			} else {
				Led_Off(RGB_B_LED);
			}
		}
		if (rcv_fr_PTX[0] == 0x02) {/*Pedal R*/
			RX_data[1].data_ready = true;
			RX_data[1].force_node = float_data;
			if (float_data > 0.2) {
				Led_On(RED_LED);
			} else {
				Led_Off(RED_LED);
			}
		}

		if (flag_serial_data_print) {
			print_serial_data(RX_data);
			flag_serial_data_print = FALSE;/*Clear the flag*/
		}

		clear_array();

		__WFI();

	};
	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado por ningun S.O. */

	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

