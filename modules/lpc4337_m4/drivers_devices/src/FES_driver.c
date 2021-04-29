/*
 * FES_driver.c
 *
 *  Created on: 28 abr. 2021
 *      Author: river
 */
/*=====[Inclusion of own header]=============================================*/

#include "FES_driver.h"
#include "chip.h"
#include <string.h>
/*=====[Inclusions of private function dependencies]=========================*/

/*=====[Definition macros of private constants]==============================*/
#define ANGLE_I2C_CFG_CLK 100000
/*=====[Private function-like macros]========================================*/

/*=====[Definitions of private data types]===================================*/

/*=====[Definitions of external public global variables]=====================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

/*=====[Prototypes (declarations) of private functions]======================*/

/*=====[Implementations of public functions]=================================*/
uint8_t channelEnable(muscule_t muscule, uint16_t startAngle, uint16_t angle,
		uint16_t endAngle) {
	if (startAngle < angle) {
		if (angle < endAngle) {
			return muscule;
		}
	}

	return FES_ERROR;
}

uint8_t sendDataFES(dataFES_t* data){

	uint16_t arr1[NUMBER_CHANNELS];
	memcpy(&arr1, &data->channelAmplitude, sizeof(uint16_t)*NUMBER_CHANNELS);
	uint8_t  arr2[sizeof(uint16_t)*NUMBER_CHANNELS] = {0};
	uint8_t* ptr = (uint8_t *) &arr1; //cast the 16bit pointer to an 8bit pointer
	for(int i=0; i<sizeof(uint16_t)*NUMBER_CHANNELS; i++)
	{
	 arr2[i] = *ptr; //pass data to other array
	 ptr++;          //move your pointer
	}
	SendStringRs485(arr2, sizeof(uint16_t)*NUMBER_CHANNELS);
	return FES_SUCCESS;
};

uint8_t initFES(void) {
	Init_Uart_Rs485();
	angle_i2cDriverInit(ANGLE_I2C_CFG_CLK, 0x0C);
	angle_setConfig(
			_ANGLE_CDS_NO_CHANGLE | _ANGLE_HDR_RESET_1 | _ANGLE_SFR_RESET_1
					| _ANGLE_CSR_STA_1 | _ANGLE_CXE_1 | _ANGLE_CER_1);
	return FES_SUCCESS;
}

uint8_t setFES(dataFES_t* data) {
	uint8_t channels = 0x00;
	uint16_t angle = angle_getAngle();
	channels |= channelEnable(RIGTH_GLUTEUS, RG_START, angle, RG_END);

	sendDataFES(data);
	return FES_SUCCESS;
}

uint8_t clearFES(void) {
	return FES_SUCCESS;
}

