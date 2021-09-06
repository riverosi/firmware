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
#define BUFFLEN 16
#define UART_BAUDRATE 230400
#define SAMPLE_PERIOD_US 1000
#define PI 3.14159265358
#define W_MAX 0.337837837838 // 75/3.7 the dc motor operates at most at 75 rpm and it has a reduccion relation of 3.7
/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/
typedef struct {
	float Angle;
	float dacValue;
	float omega;
} data_angle_t;

static union {
	/** Union data for stream in UART*/
	data_angle_t data_a1335;
	uint8_t buffer_string[12];
} data_union;

static float prev_angle = 0;
RINGBUFF_T rbRx; //ring buffer
uint8_t rxBuff[BUFFLEN]; //array data for ring buffer

/*=====[Definitions of private global variables]=============================*/
/**
 * Computes angular velocity considering the sign
 */
void computeAngularVelocity(void){
	data_union.data_a1335.omega  = data_union.data_a1335.Angle - prev_angle;
	if (abs(data_union.data_a1335.omega )> W_MAX /(1000000/SAMPLE_PERIOD_US)) // terminar de corregir aca
	{
		if(data_union.data_a1335.Angle>prev_angle)
			data_union.data_a1335.omega = -PI+data_union.data_a1335.Angle-prev_angle;
		else
			data_union.data_a1335.omega = PI+data_union.data_a1335.Angle-prev_angle;
	}

}

/**
 * Read sensors
 */
void readInputs(void) {
	prev_angle = data_union.data_a1335.Angle;
	data_union.data_a1335.Angle = angle_getAngleRad();
	computeAngularVelocity();
}

/**
 * Add calculates here
 */
void calculate(void) {
	//Is recommended use floats for calculations because the FPU is active
}
/**
 * Set outputs
 */
void setOutputs(void) {
	pwmWrite(PWM5, (uint8_t)data_union.data_a1335.dacValue);/*T_COL1 PWM*/
	pwmWrite(PWM9, (uint8_t)data_union.data_a1335.dacValue);/*LED3 PWM*/
}
/**
 * Send data, using a union predefinition.
 */
void printDataUART(void) {
	Chip_UART_SendBlocking(USB_UART, data_union.buffer_string, 12);
}

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

/* ----------------------------- UART2_IRQHandler----------------------------*/
void UART2_IRQHandler(void) {
	Chip_UART_RXIntHandlerRB(LPC_USART2, &rbRx);//pone los datos que se mandan por UART en el ring buffer
	uint8_t data_array[2] = { 0 };
	Chip_UART_ReadRB( LPC_USART2, &rbRx, &data_array, 1);
	data_union.data_a1335.dacValue = (float)data_array[0];
}
/* ----------------------------- SysTick Handler----------------------------*/
static volatile uint32_t cnt = 0; /** SysTick Counter variable*/
/**
 * Only for blinky
 */
void SysTick_Handler(void) {
	if (cnt == 20) {
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
	uart_init_intact();
	RingBuffer_Init(&rbRx, rxBuff, 1, BUFFLEN);
	Init_Leds();
	GPIOInit(GPIO1,GPIO_OUTPUT);
	pwmInit(0,PWM_ENABLE); // Enable pwm
	pwmInit(PWM5, PWM_ENABLE_OUTPUT);/*T_COL1 PWM*/

	pwmInit(PWM9, PWM_ENABLE_OUTPUT);/*LED3 PWM*/
	angle_i2cDriverInit(ANGLE_SENSOR_I2C_CLK, ANGLE_SA0SA1_00);
	SysTick_Config(SystemCoreClock / 1000); /*call systick every 10 ms*/

	// ----- Repeat for ever -------------------------
	while (TRUE) {
		//read sensors data
		GPIOOn(GPIO1);
		readInputs(); // at most 2kHz.
		//Compute
		calculate();
		//Update outputs
		setOutputs();
		//print UART values
		printDataUART();
		GPIOOff(GPIO1);
		//delay
		//StopWatch_DelayMs(20);
		StopWatch_DelayUs(SAMPLE_PERIOD_US);
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

