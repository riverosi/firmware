/* Copyright 2018,
 * Sebastian Mateos
 * sebastianantoniomateos@gmail.com
 * Cátedra Electrónica Programable
 * Facultad de Ingeniería
 * Universidad Nacional de Entre Ríos
 * Argentina
 *
 * All rights reserved.
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

/** @addtogroup gpio
 *  @{
 *  @file gpio.h
 *  @brief Driver Bare Metal para los pulsadores y teclas de la EDU-CIAA.
 *
 *  Declaraciones de funciones y constantes simbólicas correspondientes con las teclas y los leds disponibles en la EDU-CIAA NXP
 *
 *  @section changeLog Historial de modificaciones (la mas reciente arriba)
 *
 *  20180922 v0.1 version inicial SM
 *
 *  Iniciales  |     Nombre
 * :----------:|:----------------
 *  SM		   | Sebastian Mateos
 *
 *  @date 22/09/2018
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_


/*==================[inclusions]=============================================*/
#include "bool.h"
#include <stdint.h>

/*==================[macros]=================================================*/
typedef enum {GPIO_INPUT = 0, GPIO_OUTPUT}io_t;
/*==================[typedef]================================================*/

/** @typedef gpio_t
 * @brief  Define los puertos de la EDU-CIAA
 */
typedef enum {
	TEC_1=0, /**< Mapeo GPIO0[4] en P1_0 */
	TEC_2, /**< Mapeo GPIO0[8] en P1_1 */
	TEC_3, /**< Mapeo GPIO0[9] en P1_2 */
	TEC_4, /**< Mapeo GPIO1[9] en P1_6 */
	LED1, /**< Mapeo GPIO0[14] en P2_10 */
	LED2, /**< Mapeo GPIO1[11] en P2_11 */
	LED3, /**< Mapeo GPIO1[12] en P2_12 */
	LEDRGB_R, /**< Mapeo GPIO5[0] en P2_0 */
	LEDRGB_G, /**< Mapeo GPIO5[1] en P2_1 */
	LEDRGB_B, /**< Mapeo GPIO5[2] en P2_2 */
	LCD1, /**< Mapeo GPIO2[4] en P4_4 */
	LCD2, /**< Mapeo GPIO2[5] en P4_5 */
	LCD3, /**< Mapeo GPIO2[6] en P4_6 */
	LCD4, /**< Mapeo GPIO5[14] en P4_10 */
	GPIO1, /**< Mapeo GPIO5[14] en P4_10 */
	GPIO3, /**< Mapeo GPIO5[14] en P4_10 */
	GPIO5, /**< Mapeo GPIO5[14] en P4_10 */
	LCD_RS, /**< Mapeo GPIO5[13] en P4_9 */
	LCD_EN, /**< Mapeo GPIO5[12] en P4_8 */
	T_COL0, /**< Mapeo GPIO1[8] en P1_5 */
	T_FIL0, /**< Mapeo GPIO2[2] en P4_0 */
	T_FIL2, /**< Mapeo GPIO2[2] en P4_2 */
	T_FIL3, /**< Mapeo GPIO2[3] en P4_3 */
	CIAA_GPIO1, /**< Mapeo para CIAA Board GPIO3[8] en P7_0 */
	CIAA_GPIO3, /**< Mapeo para CIAA Board GPIO5[15] en P6_7 */
	CIAA_GPIO0, /**< Mapeo para CIAA Board GPIO3[2] en P6_3 */
	CIAA_DO0,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DO1,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DO2,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DO3,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DO4,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DO5,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DO6,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DO7,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DI0,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DI1,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DI2,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DI3,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DI4,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DI5,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DI6,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
	CIAA_DI7,	/**< Mapeo para CIAA Board GPIO1[8] en P1_5 */
} gpio_t;
/**
 * @typedef gpiogp_t
 * @brief Pin interrupt mode
 */
typedef enum {
	IRQ_EDGE_RISE,
	IRQ_EDGE_FALL,
	IRQ_LEVEL_HIGH,
	IRQ_LEVEL_LOW
} gpioPinIrq_t;
/** @typedef gpiogp_t
 * @brief Define los grupos de interrupciones
 */
typedef enum {
	GPIOGP0=0, /**< Grupo de interrupciones GPIO0_IRQHandler 32 */
	GPIOGP1, /**< Grupo de interrupciones GPIO1_IRQHandler 33 */
	GPIOGP2, /**< Grupo de interrupciones GPIO2_IRQHandler 34 */
	GPIOGP3, /**< Grupo de interrupciones GPIO3_IRQHandler 35 */
	GPIOGP4, /**< Grupo de interrupciones GPIO4_IRQHandler 36 */
	GPIOGP5, /**< Grupo de interrupciones GPIO5_IRQHandler 37 */
	GPIOGP6, /**< Grupo de interrupciones GPIO6_IRQHandler 38 */
	GPIOGP7  /**< Grupo de interrupciones GPIO7_IRQHandler 39 */
} gpiogp_t;

/** @typedef gpiogroupgp_t
 * @brief Define los grupos de interrupciones
 */
typedef enum {
	GPIOGROUPGP0=0, /**< Grupo de interrupciones GPIO0_IRQHandler 32 */
	GPIOGROUPGP1, /**< Grupo de interrupciones GPIO1_IRQHandler 33 */
} gpiogroupgp_t;

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/
/** @fn void GPIOInit(void)
 * @brief Inicializacion de los puertos GPIO de la EDU-CIAA
 */
void GPIOInit(gpio_t pin, io_t io);

/** @fn void GPIOOn(gpio_t pin)
 * @brief Pone a 1 un pin de la EDU-CIAA
 * @param[in] pin Pin que se desea poner a 1
 */
void GPIOOn(gpio_t pin);

/** @fn void GPIOOff(gpio_t pin)
 * @brief Apaga un pin de la EDU-CIAA
 * @param[in] pin Pin que se desea poner a 0
 */
void GPIOOff(gpio_t pin);

/** @fn void GPIOState(gpio_t pin)
 * @brief Indica el estado de un pin de la EDU-CIAA
 * @param[in] pin Pin que se desea prender o apagar
 * @param[in] state Estado del pin (1: prendido o 0: apagado)
 */
void GPIOState(gpio_t pin, bool state);

/** @fn void GPIOToggle(gpio_t pin)
 * @brief Invierte el estado de un pin de la EDU-CIAA
 * @param[in] pin Pin que se desea invertir
 */
void GPIOToggle(gpio_t pin);

/** @fn bool GPIORead(gpio_t pin)
 * @brief Lectura de un pin de la EDU-CIAA
 * @param[in] pin Pin a leer el estado
 * @return Booleano que indica el estado del pin
 */
bool GPIORead(gpio_t pin);

/** @fn GPIOCleanInt(gp)
 * @brief Limpia la interrupcion de cada pin
 * @param[in] gp Indica de que pin va a limpiar la interrupcion
 * */
#define GPIOCleanInt(gp) \
Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(gp))

/** @brief Activa la interrupcion de un pin en particular y se la asigna a un grupo de interrupciones
 * definido en @ref gpiogp_t
 * @param[in] gp Indica a que grupo de interrupciones se asigna al pin
 * @param[in] pin Pin que va a interrumpir
 * @param[in] void *ptr_int_func Funcion a la que se va a llamar en la interrupcion
 * @param[in] gpioPinIrq_t edge pin interruption
 */
void GPIOActivInt(gpiogp_t gp, gpio_t pin, void *ptr_int_func, gpioPinIrq_t edge);

/** @brief Activate group interrupts and add to the interruption the chosen gpio through the vector
 * @param[in] group_gp
 * @param[in] pins
 * @param[in] n_pins
 * @param[in] ptrIntFunc Function to be called in the interruption.
 * @param[in] edge
 */
void GPIOActivGroupInt(gpiogroupgp_t group_gp, gpio_t *pins, uint8_t n_pins, void *ptr_int_func, bool edge);

/** @brief Deinicializacion de los puertos GPIO de la EDU-CIAA
 */
void GPIODeinit(void);

/*==================[end of file]============================================*/
#endif /* INC_GPIO_H_ */

/** @}*/
