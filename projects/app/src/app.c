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
#include <stdint.h>
#include <string.h>
/*==================[macros and definitions]=================================*/
#define RISING 1
#define FALLING 0
/*==================[internal data definition]===============================*/
uint16_t counter_systick=0;
/*==================[internal functions declaration]=========================*/
// C++ version 0.4 char* style "itoa"
// Modified by Eric Pernia.
Bool uint32ToString( uint32_t value, char* result, uint8_t base )
{
   // check that the base if valid
   if( base < 2 || base > 36 ) {
      *result = '\0';
      return FALSE;
   }

   char* ptr = result, *ptr1 = result, tmp_char;
   uint32_t tmp_value;

   do {
      tmp_value = value;
      value /= (uint32_t)base;
      *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * (uint32_t)base)];
   } while ( value );

   // Apply negative sign
   if (tmp_value < 0) *ptr++ = '-';
   *ptr-- = '\0';
   while(ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr--= *ptr1;
      *ptr1++ = tmp_char;
   }
   return TRUE;
}
/*  \Brief: The API is used as I2C bus write
 *  \Return : Status of the I2C write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *      will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	u8 buffer[cnt+2];
	buffer[0] = (dev_addr << 1);
	buffer[1] = reg_addr;
	u8 j;
	for(j=0;j<cnt;j++){
		buffer[2+j] = *(reg_data+j);
	}

	if(Chip_I2CM_Write(LPC_I2C0, buffer, cnt+2) != (cnt+2))
		return FALSE;
	Chip_I2CM_SendStop(LPC_I2C0);

	return TRUE;
}
/*  \Brief: The API is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *  will data is going to be read
 *  \param reg_data : This data read from the sensor,
 *   which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{	u8 buf[2];
	buf[0] = reg_addr;
	I2CRead(dev_addr,buf, 1, reg_data, cnt);
     return TRUE;
}
void BNO055_delay_msek(u32 msek)
{
    /*Here you can write your own delay routine*/
	StopWatch_DelayMs(msek);
}
/*==================[external data definition]===============================*/
/*==================[external functions definition]==========================*/
void MPU1Init(void)
{
	StopWatch_DelayMs(100);
	MPU9250InitI2C(400000,MPU9250_ADDRESS_AD0H);
	/*Si hay error al iniciar imu enciende led RGD rojo, sino led verde*/
	if((MPU9250Begin() & MPU9250SetAccelRange(ACCEL_RANGE_2G) &	MPU9250SetGyroRange(GYRO_RANGE_250DPS)) == TRUE){
		GPIOOn(LED3);
		char frase[]="Initialize MPU AD0H\n";
		Send_String_UART(frase , sizeof(frase));/*Send data array blocking*/
	}
	else{
		GPIOOn(LED2);
		char frase[]="Error begin MPU AD0H\n";
		Send_String_UART(frase , sizeof(frase));/*Send data array blocking*/
		//while(TRUE){};
	}
};
void MPU2Init(void)
{
	StopWatch_DelayMs(100);
	MPU9250InitI2C(400000,MPU9250_ADDRESS_AD0L);
	/*Si hay error al iniciar imu enciende led RGD rojo, sino led verde*/
	if((MPU9250Begin() & MPU9250SetAccelRange(ACCEL_RANGE_2G) &	MPU9250SetGyroRange(GYRO_RANGE_250DPS)) == TRUE){
		GPIOOn(LED3);
		char frase[]="Initialize MPU AD0L\n";
		Send_String_UART(frase, sizeof(frase));/*Send data array blocking*/
	}
	else{
		GPIOOn(LED2);
		char frase[]="Error begin MPU AD0L\n";
		Send_String_UART(frase , sizeof(frase));/*Send data array blocking*/
		//while(TRUE){};
	}
};
void DataIMUsRead(void){
	uint8_t buffer_data_out[14];
	uint8_t buf[2];
	buf[0] = 0x3B;
	I2CRead( MPU9250_ADDRESS_AD0H , buf , 1 , buffer_data_out , 14);
	int16_t axcounts = (((int16_t)buffer_data_out[0]) << 8) | buffer_data_out[1];
	int16_t aycounts = (((int16_t)buffer_data_out[2]) << 8) | buffer_data_out[3];
	int16_t azcounts = (((int16_t)buffer_data_out[4]) << 8) | buffer_data_out[5];
//	int16_t gxcounts = (((int16_t)buffer_data_out[8]) << 8) | buffer_data_out[9];
//	int16_t gycounts = (((int16_t)buffer_data_out[10]) << 8) | buffer_data_out[11];
//	int16_t gzcounts = (((int16_t)buffer_data_out[12]) << 8) | buffer_data_out[13];
	Send_String_UART("MPU1;", 5);/*Send data array blocking*/
	char buffer_ticks[10];
	uint32ToString(StopWatch_TicksToMs(StopWatch_Start()), buffer_ticks, 10);
	Send_String_UART(buffer_ticks, 10);
	Send_String_UART(";", 1);
	char buffer[5];
	itoa(axcounts, buffer, 10);
	Send_String_UART(buffer, 5);
	Send_String_UART(";", 1);
	itoa(aycounts, buffer, 10);
	Send_String_UART(buffer, 5);
	Send_String_UART(";", 1);
	itoa(azcounts, buffer, 10);
	Send_String_UART(buffer, 5);
	Send_String_UART("\n", 1);
}

//---------------------------------------------------------------------------------------------------
/*GPIOGP0 Interruption Handler*/
void interup(void){
	GPIOToggle(LEDRGB_B);
	char frase[]="Trigger;\n";
	Send_String_UART(frase , sizeof(frase));/*Send data array blocking*/
 }
//---------------------------------------------------------------------------------------------------
/*Sistick Handler*/
void  SysTick_Handler(void){
	counter_systick++;
	if (counter_systick==500){
		GPIOToggle(LED1);
		//DataIMUsRead();
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
	MPU1Init();
//	MPU2Init();

	struct bno055_t myBNO;
	myBNO.bus_write = BNO055_I2C_bus_write;
	myBNO.bus_read = BNO055_I2C_bus_read;
	myBNO.delay_msec = BNO055_delay_msek;
	//BNO055 7-bit I2C slave address, which will be0x29 when GPIO1 is high and 0x28 when low.
	myBNO.dev_addr = BNO055_I2C_ADDR1;

	if(bno055_init(&myBNO)==FALSE){
		GPIOOn(LEDRGB_G);
	}
	else{
		GPIOOn(LEDRGB_R);
	}

	bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);
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
