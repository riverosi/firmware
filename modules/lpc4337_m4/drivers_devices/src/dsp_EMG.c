/*
 * dsp_EMG.c
 *
 *  Created on: 15 sept. 2021
 *      Author: river
 */
#include "dsp_EMG.h"
#include <stdlib.h>

#define APP_FFT_LEN 64
//Vector de frecuencia para una frecuencia de muestreo de 2khz y un buffer de 128 muestras.
float freq_vector[65] = { 0.00000000f, 15.62500000f, 31.25000000f, 46.87500000f,
		62.50000000f, 78.12500000f, 93.75000000f, 109.37500000f, 125.00000000f,
		140.62500000f, 156.25000000f, 171.87500000f, 187.50000000f,
		203.12500000f, 218.75000000f, 234.37500000f, 250.00000000f,
		265.62500000f, 281.25000000f, 296.87500000f, 312.50000000f,
		328.12500000f, 343.75000000f, 359.37500000f, 375.00000000f,
		390.62500000f, 406.25000000f, 421.87500000f, 437.50000000f,
		453.12500000f, 468.75000000f, 484.37500000f, 500.00000000f,
		515.62500000f, 531.25000000f, 546.87500000f, 562.50000000f,
		578.12500000f, 593.75000000f, 609.37500000f, 625.00000000f,
		640.62500000f, 656.25000000f, 671.87500000f, 687.50000000f,
		703.12500000f, 718.75000000f, 734.37500000f, 750.00000000f,
		765.62500000f, 781.25000000f, 796.87500000f, 812.50000000f,
		828.12500000f, 843.75000000f, 859.37500000f, 875.00000000f,
		890.62500000f, 906.25000000f, 921.87500000f, 937.50000000f,
		953.12500000f, 968.75000000f, 984.37500000f, 1000.00000000f };

void dsp_emg_rms_f32(float32_t* pSrc, uint32_t blockSize, float32_t* pResult) {
	arm_rms_f32(pSrc, blockSize, pResult);
}

void dsp_emg_power_f32(float32_t* pSrc, uint32_t blockSize, float32_t* pResult) {
	float aux;
	arm_power_f32(pSrc, blockSize, &aux);
	aux = aux / (blockSize);
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
		while (i < blockSize) {
			result += fabsf(pSrc[i]);
			i++;
		}
		*pResult = result;
	}
}

int compare(const void * a, const void * b) {
	float32_t fa = *(const float32_t*) a;
	float32_t fb = *(const float32_t*) b;
	return (fa > fb) - (fa < fb);
}

float32_t dsp_emg_mdf_f32(float32_t * pSrc, uint32_t blockSize,	float32_t * pResult) {
	float32_t median, aux;
	arm_rfft_fast_instance_f32 rfft_fast_instance;
	arm_rfft_fast_init_f32(&rfft_fast_instance, blockSize);
	arm_rfft_fast_f32(&rfft_fast_instance, pSrc, pSrc, 0);
	arm_cmplx_mag_f32(pSrc, pResult, APP_FFT_LEN);
	arm_scale_f32(pResult, 1.0f / blockSize, pResult, APP_FFT_LEN);
	arm_scale_f32(&pResult[1], 2.0f, pResult, APP_FFT_LEN - 1);//fft IS OK.
	arm_power_f32(pResult, APP_FFT_LEN, &aux);//Power spectrum IS OK

	//sort values
	qsort(pResult, APP_FFT_LEN, sizeof(float32_t), compare);
	//The median of a set of data sorted is the middle most number or center value in the set.
	median = (pResult[31] + pResult[32]) / 2.0f;
	return median;
}

float32_t dsp_emg_mnf_f32(float32_t * pSrc, uint32_t blockSize) {
	float32_t aux,aux1,aux2;
	arm_rfft_fast_instance_f32 rfft_fast_instance;
	arm_rfft_fast_init_f32(&rfft_fast_instance, blockSize);
	arm_rfft_fast_f32(&rfft_fast_instance, pSrc, pSrc, 0);
	arm_cmplx_mag_f32(pSrc, pSrc, APP_FFT_LEN);
	arm_scale_f32(pSrc, 1.0f / blockSize, pSrc, APP_FFT_LEN);
	arm_scale_f32(&pSrc[1], 2.0f, pSrc, APP_FFT_LEN - 1);//fft IS OK.
	arm_power_f32(pSrc, APP_FFT_LEN, &aux);//Power spectrum IS OK
	//calculate using the dot product
	arm_dot_prod_f32(pSrc, freq_vector, APP_FFT_LEN, &aux1);
	arm_power_f32(pSrc, blockSize, &aux2);
	aux2 = aux2 / blockSize;
	aux1 = aux1 / aux2;
	return aux1;
}
