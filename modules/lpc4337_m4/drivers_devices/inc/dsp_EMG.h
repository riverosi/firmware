/*
 * dsp_EMG.h
 *
 *  Created on: 15 sept. 2021
 *      Author: river
 */

#ifndef MODULES_LPC4337_M4_DRIVERS_DEVICES_INC_DSP_EMG_H_
#define MODULES_LPC4337_M4_DRIVERS_DEVICES_INC_DSP_EMG_H_

// DSP libs
#define ARM_MATH_CM4
#define __FPU_PRESENT 1
#include "arm_math.h"
#include "arm_const_structs.h"

/**
 * @brief  Root Mean Square of the elements of a floating-point vector.
 * @param[in]  pSrc       is input pointer
 * @param[in]  blockSize  is the number of samples to process
 * @param[out] pResult    is output value.
 */
void dsp_emg_rms_f32(float32_t * pSrc, uint32_t blockSize, float32_t * pResult);


/**
 * @brief  Sum of the squares of the elements of a floating-point vector.
 * @param[in]  pSrc       is input pointer
 * @param[in]  blockSize  is the number of samples to process
 * @param[out] pResult    is output value.
 */
void dsp_emg_power_f32(float32_t * pSrc, uint32_t blockSize, float32_t * pResult);

/**
 * @brief  Pick to pick amplitude of the elements of a floating-point vector.
 * @param[in]  pSrc       is input pointer
 * @param[in]  blockSize  is the number of samples to process
 * @param[out] pResult    is output value.
 */
void dsp_emg_ptp_f32(float32_t * pSrc, uint32_t blockSize, float32_t * pResult);

/**
 * @brief  Integral EMG of the elements of a floating-point vector.
 * @param[in]  pSrc       is input pointer
 * @param[in]  blockSize  is the number of samples to process
 * @param[out] pResult    is output value.
 */
void dsp_emg_iemg_f32(float32_t * pSrc, uint32_t blockSize, float32_t * pResult);

/**
 * @brief  MDF of the elements of a floating-point vector.
 * @param[in]  pSrc       is input pointer
 * @param[in]  blockSize  is the number of samples to process
 * @param[out] pResult    is output value.
 */
void dsp_emg_mdf_f32(float32_t * pSrc, uint32_t blockSize, float32_t * pResult);

#endif /* MODULES_LPC4337_M4_DRIVERS_DEVICES_INC_DSP_EMG_H_ */