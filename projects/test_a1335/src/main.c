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
//FPU dependences
#define ARM_MATH_CM4
#define __FPU_PRESENT 1
#include "arm_math.h"
#include "arm_const_structs.h"

/*=====[Inclusions of function dependencies]=================================*/

/*=====[Definition macros of private constants]==============================*/
#define ANGLE_SENSOR_I2C_CLK 100000
#define ANGLE_SENSOR_I2C_ADR 0x0C

/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/
typedef struct {
	uint16_t Angle;
	uint16_t dacValue;
} data_angle_t;

static union {
	/** Union data for stream in UART*/
	data_angle_t data_a1335;
	uint8_t buffer_string[4];
} data_union;

/*=====[Definitions of private global variables]=============================*/
/**
 * Read sensors
 */
void readInputs(void) {
	data_union.data_a1335.Angle = angle_getAngle();
}
/**
 * Add calculates here
 */
void calculate(void) {
	data_union.data_a1335.dacValue = data_union.data_a1335.Angle;
}
/**
 * Set outputs
 */
void setOutputs(void) {
	dacWrite(data_union.data_a1335.dacValue);
}
/**
 * Send data, using a union predefinition.
 */
void printDataUART(void) {
	Chip_UART_SendBlocking(USB_UART, data_union.buffer_string, 4);
}

/* ----------------------------- SysTick Handler----------------------------*/
static volatile uint32_t cnt = 0; /** SysTick Counter variable*/
void SysTick_Handler(void) {
	if (cnt == 50) {
		GPIOToggle(LED1);
		cnt = 0;
	}
	cnt++;
}
/*=====[Main function, program entry point after power on or reset]==========*/
int main(void) {
	/* perform the needed initialization here */
	SystemClockInit();
	fpuInit(); //FPU hardware on
	StopWatch_Init();
	Init_Uart_Ftdi(115200); //115200
	Init_Leds();
	dacInit(DAC_ENABLE);
	angle_i2cDriverInit(ANGLE_SENSOR_I2C_CLK, ANGLE_SA0SA1_00);
	angle_setConfig(
			_ANGLE_CDS_NO_CHANGLE | _ANGLE_HDR_RESET_1 | _ANGLE_SFR_RESET_1
					| _ANGLE_CSR_STA_1 | _ANGLE_CXE_1 | _ANGLE_CER_1);
	SysTick_Config(SystemCoreClock / 100); /*call systick every 10 ms*/

	// ----- Repeat for ever -------------------------
	while (TRUE) {
		//read sensors data
		readInputs();
		//Compute
		calculate();
		//Update outputs
		setOutputs();
		//print UART values
		printDataUART();
		//delay
		StopWatch_DelayMs(20);
		__WFI();
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

