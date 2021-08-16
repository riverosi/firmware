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
#define ANGLE_SENSOR_I2C_CLK 100000
#define ANGLE_SENSOR_I2C_ADR 0x0C
/*=====[Private function-like macros]========================================*/

/*=====[Definitions of private data types]===================================*/

/*=====[Definitions of external public global variables]=====================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

/*=====[Prototypes (declarations) of private functions]======================*/

/*=====[Implementations of public functions]=================================*/
uint16_t calculateAngleCorrection(void) {
	uint16_t var = (K * angle_getAngle()) / OMEGA_MAX;
	return var;
}

uint8_t channelsEnables(){
	uint8_t var = 0x00;
	uint16_t actualAngle  = angle_getAngle();

	if ((RQ_START < actualAngle) || (actualAngle< RQ_END)) {
		var |= RIGTH_QUADRICEPS;
	} else {var |= CLEAN_MASK;}

	return var;
}

uint8_t sendDataFES(dataFES_t* data) {
	uint8_t arr2[sizeof(uint8_t) * NUMBER_CHANNELS] = { 0 };
	memcpy(&arr2, &data->channelAmplitude, sizeof(uint8_t) * NUMBER_CHANNELS);
	/*definir la trama*/
	SendStringRs485(arr2, sizeof(uint8_t) * NUMBER_CHANNELS);
	return FES_SUCCESS;
}

uint8_t initFES(void) {
	Init_Uart_Rs485();
	angle_i2cDriverInit(ANGLE_SENSOR_I2C_CLK, ANGLE_SENSOR_I2C_ADR); /* TODO enum in class*/
	angle_setConfig(
			_ANGLE_CDS_NO_CHANGLE | _ANGLE_HDR_RESET_1 | _ANGLE_SFR_RESET_1
					| _ANGLE_CSR_STA_1 | _ANGLE_CXE_1 | _ANGLE_CER_1);
	/*TODO add flag error*/
	return FES_SUCCESS;
}

uint8_t setFES(dataFES_t* data) {
	sendDataFES(data); /*send data using 485*/
	return FES_SUCCESS;
}

uint8_t clearFES(void) {
	dataFES_t clear_data;
	memset(&clear_data, 0x00, sizeof(clear_data));
	sendDataFES(&clear_data);
	return FES_SUCCESS;
}

