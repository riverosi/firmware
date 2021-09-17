/*
 * dsp_EMG.c
 *
 *  Created on: 15 sept. 2021
 *      Author: river
 */
#include "dsp_EMG.h"

void dsp_emg_rms_f32(float32_t* pSrc, uint32_t blockSize, float32_t* pResult) {
	arm_rms_f32(pSrc, blockSize, pResult);
}


void dsp_emg_power_f32(float32_t* pSrc, uint32_t blockSize, float32_t* pResult) {
	float aux;
	arm_power_f32(pSrc, blockSize, &aux);
	aux = aux/(blockSize);
	*pResult = aux;
}

void dsp_emg_ptp_f32(float32_t* pSrc, uint32_t blockSize, float32_t* pResult) {
	uint32_t idmax, idmin;
	float max, min;
	arm_max_f32(pSrc, blockSize, &max, &idmax);
	arm_min_f32(pSrc, blockSize, &min, &idmin);
	*pResult = fabsf(max - min);
}

void dsp_emg_iemg_f32(float32_t* pSrc, uint32_t blockSize, float32_t* pResult) {
	//The functions support in-place computation allowing the source and destination pointers to reference the same memory buffer.
	float result = 0;
	uint32_t i = 0;
	if (blockSize > 0) {
		while(i<blockSize){
			result += fabsf(pSrc[i]);
			i++;
		}
		*pResult = result;
	}
}
