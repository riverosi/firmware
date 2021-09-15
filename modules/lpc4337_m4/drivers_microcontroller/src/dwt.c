/*
 * dwt.c
 *
 *  Created on: 15 sept. 2021
 *      Author: river
 */

#include "dwt.h"

void DWTStart(void) {
	DWT->CTRL |= 1;
	DWT->CYCCNT = 0;
}

uint32_t DWTStop(void) {
	uint32_t cycles = DWT->CYCCNT;
	return (cycles);
}
