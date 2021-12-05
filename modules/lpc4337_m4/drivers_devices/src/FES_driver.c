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

/*=====[Private function-like macros]========================================*/

/*=====[Definitions of private data types]===================================*/

/*=====[Definitions of external public global variables]=====================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

/*=====[Prototypes (declarations) of private functions]======================*/

/*=====[Implementations of public functions]=================================*/

uint8_t channelsEnables(){
	uint8_t var = 0x00;
	uint16_t actualAngle  = angle_getAngle();

	if ((RQ_START < actualAngle) || (actualAngle< RQ_END)) {
		var |= RIGTH_QUADRICEPS;
	} else {var |= CLEAN_MASK;}

	return var;
}

uint8_t sendDataFES(dataFES_t* data) {
	uint8_t arr2[PAYLOAD_SIZE];
	data->header = 0xFFFF;
	memcpy(&arr2, data, PAYLOAD_SIZE );
	SendStringRs485(arr2, PAYLOAD_SIZE);
	return FES_SUCCESS;
}

uint8_t initFES(void) {
	Init_Uart_Rs485();
	/*TODO add flag error*/
	return FES_SUCCESS;
}

uint8_t clearFES(void) {
	dataFES_t clear_data;
	memset(&clear_data, 0x00, sizeof(clear_data));
	sendDataFES(&clear_data);
	return FES_SUCCESS;
}

