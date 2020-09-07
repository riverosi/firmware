/* LIRINS
 * FIUNER - 2020
 * Autor/es:
 * JMReta - jmreta@ingenieria.uner.edu.ar
 * ICRiveros - riverosignacio138@gmail.com
 *
 *
 * Revisión:
 * 15-04-20: Versión inicial
 *
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
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USEhttps://www.linuxmint.com/start/tessa/, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*==================[inclusions]=============================================*/
#include "app.h"       /* <= own header */
#include "systemclock.h"
#include "chip.h"


/*==================[macros and definitions]=================================*/
#define RISING 1
#define FALLING 0
/*==================[internal data definition]===============================*/

uint16_t counter_systick=0;

/*==================[internal functions declaration]=========================*/

/*==================[external data definition]===============================*/
struct bno055_t bno055;
/* variable used to read the accel xyz data */
struct bno055_accel_t accel_xyz;
/* structure used to read the mag xyz data */
struct bno055_mag_t mag_xyz;
/* structure used to read the gyro xyz data */
struct bno055_gyro_t gyro_xyz;
/*==================[external functions definition]==========================*/

//---------------------------------------------------------------------------------------------------
/* GPIOGP0 Interruption Handler */
void interup(void){
	GPIOToggle(LEDRGB_G);
	char frase[]="Trigger;\n";
	Send_String_UART(frase , sizeof(frase));/*Send data array blocking*/
 }
//---------------------------------------------------------------------------------------------------
/* Sistick Handler */
void  SysTick_Handler(void){
	counter_systick++;
	if (counter_systick==500){
		GPIOToggle(LED3);
	    bno055_read_accel_xyz(&accel_xyz);
	    bno055_read_mag_xyz(&mag_xyz);
	    bno055_read_gyro_xyz(&gyro_xyz);
	    char buffer[5];
    	itoa(accel_xyz.x, buffer, 10);
    	Send_String_UART(buffer, 5);
    	Send_String_UART(";", 1);
		counter_systick=0;
	}
}
//---------------------------------------------------------------------------------------------------
int main(void)
{
	SystemClockInit();
	fpuInit();
	StopWatch_Init();
	Init_Uart_Ftdi(115200);
	LedsInit();
	/**/
	I2C_routine();
	s32 comres = BNO055_ERROR;

    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msek;
	/*
	 * BNO055 7-bit I2C slave address,
	 * which will be0x29 when GPIO1 is high and 0x28 when low.
	 */
    bno055.dev_addr = BNO055_I2C_ADDR1;
    comres = bno055_init(&bno055);
    /* set the power mode as NORMAL*/
    comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    /*  For reading sensor raw data it is required to set the
     * operation modes of the sensor
     * operation mode can set from the register
     * page - page0
     * register - 0x3D
     * bit - 0 to 3
     * for sensor data read following operation mode have to set
     * SENSOR MODE
     * 0x01 - BNO055_OPERATION_MODE_ACCONLY
     * 0x02 - BNO055_OPERATION_MODE_MAGONLY
     * 0x03 - BNO055_OPERATION_MODE_GYRONLY
     * 0x04 - BNO055_OPERATION_MODE_ACCMAG
     * 0x05 - BNO055_OPERATION_MODE_ACCGYRO
     * 0x06 - BNO055_OPERATION_MODE_MAGGYRO
     * 0x07 - BNO055_OPERATION_MODE_AMG
     * based on the user need configure the operation mode*/
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);

	GPIOInit(TEC_1, GPIO_INPUT);
	GPIOActivInt( GPIOGP0 , TEC_1 , interup , FALLING);
	SysTick_Config(SystemCoreClock/1000);/*llamada systick cada 1ms*/
    while(TRUE){
    	__WFI ();
    };
    /* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
       por ningun S.O. */
	return 0;
}
/*==================[end of file]============================================*/
