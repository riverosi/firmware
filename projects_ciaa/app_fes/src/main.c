/* Copyright 2022 - Riveros Ignacio
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
 *	RI			Riveros Ignacio
 */

/*==================[inclusions]=============================================*/
#include "mi_proyecto.h"       /* <= own header */
#include "systemclock.h"
#include <string.h>
//FPU dependences
#define ARM_MATH_CM4
#define __FPU_PRESENT 1
#include "arm_math.h"
#include "arm_const_structs.h"
/*=====[Inclusions of function dependencies]=================================*/

/*=====[Definition macros of private constants]==============================*/
#define SISTICK_CALL_FREC	1000  /*call SysTick every 1ms 1/1000Hz*/
#define ARRAY_SIZE 8
#define BUFFLEN 32

#define CCAN_TX_MSG_ID (0x200)
#define CCAN_RX_MSG_ID (0x100)
#define CCAN_TX_MSG_REMOTE_ID (0x300)
/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/
RINGBUFF_T rbRx;
uint8_t rxBuff[BUFFLEN];
static volatile Bool uart_flag = FALSE;
nrf24l01_t RX;
uint8_t msg_received_counter = 0; //ccan counter msg

typedef struct {
	uint32_t header;
	float32_t angle;
	float32_t forceL;
	float32_t forceR;
	float32_t fatigue;
} appData_t;

/*=====[Definitions of private global variables]=============================*/

void ccan_send(void){
	CCAN_MSG_OBJ_T send_obj;
	send_obj.id = CCAN_TX_MSG_ID;
	send_obj.dlc = 8;
	send_obj.data[0] = 'C';
	send_obj.data[1] = 'I';
	send_obj.data[2] = 'A';
	send_obj.data[3] = 'A';
	send_obj.data[4] = '-';
	send_obj.data[5] = 'N';
	send_obj.data[6] = 'X';
	send_obj.data[7] = 'P';
	Chip_CCAN_Send(LPC_C_CAN0, CCAN_MSG_IF1, false, &send_obj);
	Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_TXOK);
}

void Init_ccan(void) {
	Chip_SCU_PinMuxSet(0x3, 1, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC2)); /* CAN RD */
	Chip_SCU_PinMuxSet(0x3, 2, (SCU_MODE_INACT | SCU_MODE_FUNC2)); /* CAN TD */

	Chip_Clock_GetBaseClocktHz(CLK_BASE_APB3);//68Mhz
	/* Set CCAN peripheral clock under 100Mhz for working stable */
	Chip_Clock_EnableBaseClock(CLK_BASE_APB3);
	Chip_Clock_SetBaseClock(CLK_BASE_APB3, CLKIN_IDIVC, TRUE, FALSE);
	Chip_Clock_GetBaseClocktHz(CLK_BASE_APB3); // Frequency verification

	Chip_CCAN_Init(LPC_C_CAN0);
	Chip_CCAN_SetBitRate(LPC_C_CAN0, 125000); //125000Khz is the speed of the port
	Chip_CCAN_EnableInt(LPC_C_CAN0, (CCAN_CTRL_IE | CCAN_CTRL_SIE | CCAN_CTRL_EIE));
	Chip_CCAN_DisableAutoRetransmit(LPC_C_CAN0); //Disable autoretransmit otherwise it doesn't work
	Chip_CCAN_AddReceiveID(LPC_C_CAN0, CCAN_MSG_IF1, CCAN_RX_MSG_ID);
	NVIC_EnableIRQ(C_CAN0_IRQn);
}

void app_nrf24_config(void) {
	RX.spi.cfg = nrf24l01_spi_default_cfg;
	RX.cs = CIAA_GPIO0;
	RX.ce = CIAA_GPIO3;
	RX.irq = CIAA_GPIO1;
	RX.mode = PRX;
	RX.en_ack_pay = FALSE;

	Nrf24Init(&RX);
	Nrf24SetRXPacketSize(&RX, 0x00, 32); // Set length of pipe 0 in 32 (used for the Pedal Left)
	Nrf24SetRXPacketSize(&RX, 0x01, 32); // Set length of pipe 1 in 32 (used for the Pedal Right)
	Nrf24EnableRxMode(&RX); /* Enable RX mode */
	Nrf24SecondaryDevISRConfig(&RX); /* Config ISR (only use one module in PRX mode on board)*/
}

void app_rs485_irq_config(void) {
	/* UART0 (RS485/Profibus) Only work with this configuration */
	Chip_UART_Init(LPC_USART0);
	Chip_UART_SetBaudFDR(LPC_USART0, 921600);
	Chip_UART_SetupFIFOS(LPC_USART0,
			(UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS
					| UART_FCR_TRG_LEV3));

	Chip_UART_ReadByte(LPC_USART0);
	Chip_UART_TXEnable(LPC_USART0);

	Chip_SCU_PinMux(9, 5, MD_PDN, FUNC7); /* P9_5: UART0_TXD */
	Chip_SCU_PinMux(9, 6, MD_PLN | MD_EZI | MD_ZI, FUNC7); /* P9_6: UART0_RXD */
	Chip_UART_SetRS485Flags(LPC_USART0,
	UART_RS485CTRL_DCTRL_EN | UART_RS485CTRL_OINV_1);
	Chip_SCU_PinMux(6, 2, MD_PDN, FUNC2); /* P6_2: UART0_DIR */
	Chip_UART_IntEnable(LPC_USART0, (UART_IER_RBRINT | UART_IER_RLSINT));
	NVIC_SetPriority(USART0_IRQn, 5);
	NVIC_EnableIRQ(USART0_IRQn);
}

/*==================[Init_Hardware]==========================================*/
/**
 * Init hardware
 */
void Init_Hardware(void) {
	fpuInit();
	StopWatch_Init();
	Init_Uart_Ftdi(115200);
	for (uint8_t var = 0; var < 8; var++) {
		GPIOInit(CIAA_DO0 + var, GPIO_OUTPUT);
		GPIOInit(CIAA_DI0 + var, GPIO_INPUT);
	}
	angle_i2cDriverInit(ANGLE_SA0SA1_00);
	dacInit(DAC_ENABLE);
	app_nrf24_config();
	app_rs485_irq_config();
	RingBuffer_Init(&rbRx, rxBuff, sizeof(uint8_t), BUFFLEN);
	RingBuffer_Flush(&rbRx);
	Init_ccan();
}

/*=======================[CAN0_IRQHandler]==================================*/
void CAN0_IRQHandler(void)
{
	CCAN_MSG_OBJ_T msg_buf;
	uint32_t can_int, can_stat;
	while ( (can_int = Chip_CCAN_GetIntID(LPC_C_CAN0)) != 0 ) {
		if (can_int & CCAN_INT_STATUS) {
			can_stat = Chip_CCAN_GetStatus(LPC_C_CAN0);
			// TODO with error or TXOK, RXOK
			if (can_stat & CCAN_STAT_EPASS) {
				return;
			}
			if (can_stat & CCAN_STAT_EWARN) {
				return;
			}
			if (can_stat & CCAN_STAT_BOFF) {
				return;
			}
			Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_TXOK);
			Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_RXOK);
		}
		else if ((1 <= CCAN_INT_MSG_NUM(can_int)) && (CCAN_INT_MSG_NUM(can_int) <= 0x20)) {
			// Process msg num canint
			Chip_CCAN_GetMsgObject(LPC_C_CAN0, CCAN_MSG_IF1, can_int, &msg_buf);
			switch (msg_buf.id) {
			case CCAN_RX_MSG_ID:
				msg_buf.id += 1;
				Chip_CCAN_Send(LPC_C_CAN0, CCAN_MSG_IF1, false, &msg_buf);
				break;

			case CCAN_TX_MSG_ID:
				break;

			case CCAN_TX_MSG_REMOTE_ID:
				msg_received_counter++;
				if (msg_received_counter == 2) {
					Chip_CCAN_DeleteReceiveID(LPC_C_CAN0, CCAN_MSG_IF1, CCAN_TX_MSG_REMOTE_ID);
				}
				break;

			default:
				break;
			}
		}
	}
}

/*=======================[UART0_IRQHandler]==================================*/
/**
 * Read data on the rs485 port and put in the buffer
 */
void UART0_IRQHandler(void) {
	Chip_UART_RXIntHandlerRB(LPC_USART0, &rbRx); //pone los datos que se mandan por UART en el ring buffer
	Chip_UART_ReadRB(LPC_USART0, &rbRx, rxBuff, sizeof(float32_t)); //count in bytes always
	uart_flag = TRUE;
}

/*=======================[SysTick_Handler]===================================*/
/**
 * Only Toggle led CIAA_DO7 and send data in can
 */
static volatile uint32_t cnt = 0;
void SysTick_Handler(void) {
	if (cnt == 500) {
		GPIOToggle(CIAA_DO7);
		ccan_send();
		cnt = 0;
	}
	cnt++;
}

/*=====[Main function, program entry point after power on or reset]==========*/
int main(void) {
	/* perform the needed initialization here */
	SystemClockInit();
	Init_Hardware();
	SysTick_Config(SystemCoreClock / SISTICK_CALL_FREC);/*call systick every 1ms*/
	//Init data frame
	appData_t appData = {0};
	appData.header = 0xFFFFFFFF;


	// ----- Repeat for ever -------------------------

	while (TRUE) {

		if (rcv_fr_PTX[0] == 0x01) {/*Pedal L address*/
			memcpy(&appData.forceL, &rcv_fr_PTX[1], sizeof(float32_t)); /*Convert array data to float data*/
		}
		if (rcv_fr_PTX[0] == 0x02) {/*Pedal R address*/
			memcpy(&appData.forceR, &rcv_fr_PTX[1], sizeof(float32_t)); /*Convert array data to float data*/
		}
		float32_t angle = angle_getAngleRad();
		memcpy(&appData.angle, &angle, sizeof(float32_t));
		memcpy(&appData.fatigue, rxBuff, sizeof(float32_t));
		//ccan_send();
		Chip_UART_SendBlocking(USB_UART, &appData , sizeof(appData_t));
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

