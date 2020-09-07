/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file bno055_support.c
* @date 10/01/2020
* @version  2.0.6
*
*/

/*---------------------------------------------------------------------------*
*  Includes
*---------------------------------------------------------------------------*/
#include "chip.h"
#include "stopwatch.h"
#include "bno055.h"
#include "bno055_support.h"
/*----------------------------------------------------------------------------*
*  The following APIs are used for reading and writing of
*   sensor data using I2C communication
*----------------------------------------------------------------------------*/

#define BNO055_I2C_BUS_WRITE_ARRAY_INDEX ((u8)1)

/*-------------------------------------------------------------------------*
 *  By using bno055 the following structure parameter can be accessed
 *  Bus write function pointer: BNO055_WR_FUNC_PTR
 *  Bus read function pointer: BNO055_RD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *--------------------------------------------------------------------------*/
s8 I2C_routine(void)
{
	StopWatch_Init();
	// Configuracion de las lineas de SDA y SCL de la placa
	Chip_SCU_I2C0PinConfig( I2C0_STANDARD_FAST_MODE );
	// Inicializacion del periferico
	Chip_I2C_Init(I2C0);
	// Seleccion de velocidad del bus
	Chip_I2C_SetClockRate(I2C0, 400000 );
	// Configuracion para que los eventos se resuelvan por polliong
	Chip_I2C_SetMasterEventHandler(I2C0, Chip_I2C_EventHandlerPolling );

    return BNO055_INIT_VALUE;
}

/*-------------------------------------------------------------------*
 *
 *  This is a sample code for read and write the data by using I2C
 *  Use either I2C  based on your need
 *  The device address defined in the bno055.h file
 *
 *--------------------------------------------------------------------*/

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

    u8 array[I2C_BUFFER_LEN];
    u8 stringpos = BNO055_INIT_VALUE;

    array[BNO055_INIT_VALUE] = reg_addr;
    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] = *(reg_data + stringpos);
    }


	/*
	 * Please take the below APIs as your reference for
	 * write the data using I2C communication
	 * "BNO055_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	 * add your I2C write APIs here
	 * BNO055_iERROR is an return value of I2C read API
	 * Please select your valid return value
	 * In the driver BNO055_SUCCESS defined as 0
	 * and FAILURE defined as -1
	 * Note :
	 * This is a full duplex operation,
	 * The first read data is discarded, for that extra write operation
	 * have to be initiated. For that cnt+1 operation done
	 * in the I2C write string function
	 * For more information please refer data sheet SPI communication:
	 */

    I2CM_XFER_T i2cData;
	// Prepare the i2cData register
	i2cData.slaveAddr = dev_addr;
	i2cData.options   = 0;
	i2cData.status    = 0;
	i2cData.txBuff    = array;
	i2cData.txSz      = cnt+1;
	i2cData.rxBuff    = 0;
	i2cData.rxSz      = 0;

	/* Send the i2c data */
	if( Chip_I2CM_XferBlocking(LPC_I2C0, &i2cData ) == 0 )
	{
	return (s8)BNO055_ERROR;
	}
	return (s8)BNO055_SUCCESS;
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
{

    u8 array[I2C_BUFFER_LEN] = { BNO055_INIT_VALUE };
    u8 stringpos = BNO055_INIT_VALUE;

    array[BNO055_INIT_VALUE] = reg_addr;

	I2CM_XFER_T i2cData;
	// Prepare the i2cData register
	i2cData.slaveAddr = dev_addr;
	i2cData.options   = 0;
	i2cData.status    = 0;
	i2cData.txBuff    = array;
	i2cData.txSz      = 1;
	i2cData.rxBuff    = array;
	i2cData.rxSz      = cnt;

	/* Send the i2c data */
	if( Chip_I2CM_XferBlocking(LPC_I2C0, &i2cData ) == 0 )
	{
	return (s8)BNO055_ERROR;
	}
    /* Please take the below API as your reference
     * for read the data using I2C communication
     * add your I2C read API here.
     * "BNO055_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
     * ARRAY, ARRAY, 1, CNT)"
     * BNO055_iERROR is an return value of SPI write API
     * Please select your valid return value
     * In the driver BNO055_SUCCESS defined as 0
     * and FAILURE defined as -1
     */
    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        *(reg_data + stringpos) = array[stringpos];
    }

	return (s8)BNO055_SUCCESS;
}

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BNO055_delay_msek(u32 msek)
{
 	StopWatch_DelayUs(msek);
}


