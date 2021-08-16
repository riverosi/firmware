/*
 * FES_driver.h
 *
 *  Created on: 28 abr. 2021
 *      Author: riverosky ignacio
 */

#ifndef MODULES_LPC4337_M4_DRIVERS_DEVICES_INC_FES_DRIVER_H_
#define MODULES_LPC4337_M4_DRIVERS_DEVICES_INC_FES_DRIVER_H_

/*=====[Inclusions of public function dependencies]==========================*/
#include "UART.h"
#include "__angle_driver.h"

/*=====[C++ - begin]=========================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*=====[Definition macros of public constants]===============================*/
#define NUMBER_CHANNELS	6

#define FES_SUCCESS	1
#define FES_ERROR	0

#define	BIT0	1 << 0
#define	BIT1	1 << 1
#define	BIT2	1 << 2
#define	BIT3	1 << 3
#define	BIT4	1 << 4
#define	BIT5	1 << 5
#define	BIT6	1 << 6
#define	BIT7	1 << 7

#define CLEAN_MASK 0x00

/* Muscles range angles for excitation during a pedal stroke. Right and left quadriceps (RQ and LQ)
marked as red, right and left hamstrings (RH and LH) as blue, and right and left gluteus (RG and LG), as
green. The black contour marks the right leg.
Source: A Comparative Study on Control Strategies for FES Cycling Using a Detailed Musculoskeletal Model
*/
#define K 30.0
#define OMEGA_MAX 75.0	/*angular velocity in RPM*/

#define RQ_START	300
#define RQ_END		40
#define LQ_START	120
#define LQ_END		220
#define RH_START	70
#define RH_END		170
#define LH_START	250
#define LH_END		350
#define RG_START	30
#define RG_END		100
#define LG_START	210
#define LG_END		280

/*=====[Public function-like macros]=========================================*/

/*=====[Definitions of public data types]====================================*/
typedef struct {
	uint8_t channelEnable;/**!< mask of channels enables is FES*/
	uint8_t duty[NUMBER_CHANNELS];/**!< Duty Cycle for every channel*/
	uint8_t channelAmplitude[NUMBER_CHANNELS];/**!< Current Amplitude for every channel*/
	/* TODO: check*/
	uint8_t frecuency; /*Verificar frecuencia por si necesita cambiar para el reflejo de retirada*/
} dataFES_t;

typedef enum {
	RIGTH_GLUTEUS = BIT0,
	LEFT_GLUTEUS = BIT1,
	RIGTH_QUADRICEPS = BIT2,
	LEFT_QUADRICEPS = BIT3,
	RIGTH_HAMSTRINGS = BIT4,
	LEFT_HAMSTRINGS = BIT5
}muscule_t;
/*=====[Prototypes (declarations) of public functions]=======================*/
/**
 * @brief Initialize the drivers
 * @return Result of transmission.
 */
uint8_t initFES(void);

/**
 * @brief Brief description of the function
 *
 * Extended description
 *
 * @param[in]
 * @param[out]
 *
 * @return
 *
 */
uint8_t setFES(dataFES_t* data);

/**
 * @brief Brief description of the function
 *
 * Extended description
 *
 * @param[in]
 * @param[out]
 *
 * @return
 *
 */
uint8_t clearFES(void);

/*=====[Prototypes (declarations) of public interrupt functions]=============*/

/*=====[C++ - end]===========================================================*/

#ifdef __cplusplus
}
#endif

/*=====[Avoid multiple inclusion - end]======================================*/

#endif /* MODULES_LPC4337_M4_DRIVERS_DEVICES_INC_FES_DRIVER_H_ */
