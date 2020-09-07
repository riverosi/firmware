/*
 * bno055_support.h
 *
 *  Created on: 14 may. 2020
 *      Author: ignacio
 */


#ifndef MODULES_LPC4337_M4_DRIVERS_DEVICES_INC_BNO055_SUPPORT_H_
#define MODULES_LPC4337_M4_DRIVERS_DEVICES_INC_BNO055_SUPPORT_H_

/************** I2C buffer length******/

#define I2C_BUFFER_LEN 8

/*  \Brief: The API is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be read
 *  \param reg_data : This data read from the sensor,
 *   which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*  \Brief: The API is used as SPI bus write
 *  \Return : Status of the SPI write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *  will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BNO055_delay_msek(u32 msek);

/*
 * \Brief: I2C init routine
 */
s8 I2C_routine(void);


/********************End of I2C APIs declarations***********************/





#endif /* MODULES_LPC4337_M4_DRIVERS_DEVICES_INC_BNO055_SUPPORT_H_ */
