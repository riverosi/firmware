/*
 ******************************************************************************
 * @file    read_data_polling.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI159V1
 * - NUCLEO_F411RE + STEVAL-MKI159V11
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */


/* Includes ------------------------------------------------------------------*/
#include "systemclock.h"
#include "chip.h"
#include "../inc/lsm9d1_read_data_polling.h"
#include <string.h>



typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef struct {
	LPC_I2C_T* hbus;
	uint8_t i2c_address;
} sensbus_t;

#define I2C_BUFFER_LEN 		8
/* Magnetometer bus config */
static sensbus_t mag_bus = { I2C0 , LSM9DS1_MAG_I2C_ADD_H>>1 };
/* IMU bus config */
static sensbus_t imu_bus = { I2C0 , LSM9DS1_IMU_I2C_ADD_H>>1 };


/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            20 //ms


static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis3bit16_t data_raw_magnetic_field;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;
//implement a ring buffer
//static uint8_t tx_buffer[50];

/* Extern variables ----------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write_imu(void *handle, uint8_t reg, uint8_t *bufp,
                                  uint16_t len);
static int32_t platform_read_imu(void *handle, uint8_t reg, uint8_t *bufp,
                                 uint16_t len);
static int32_t platform_write_mag(void *handle, uint8_t reg, uint8_t *bufp,
                                  uint16_t len);
static int32_t platform_read_mag(void *handle, uint8_t reg, uint8_t *bufp,
                                 uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
int main(void)
{
	SystemClockInit();
	fpuInit();
	Init_Leds();
	Init_Uart_Ftdi(460800);

	stmdev_ctx_t dev_ctx_imu;
	stmdev_ctx_t dev_ctx_mag;

	/* Initialize inertial sensors (IMU) driver interface */
	dev_ctx_imu.write_reg = platform_write_imu;
	dev_ctx_imu.read_reg = platform_read_imu;
	dev_ctx_imu.handle = (void*)&imu_bus;

	/* Initialize magnetic sensors driver interface */
	dev_ctx_mag.write_reg = platform_write_mag;
	dev_ctx_mag.read_reg = platform_read_mag;
	dev_ctx_mag.handle = (void*)&mag_bus;

	/* Initialize platform specific hardware */
	platform_init();

	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Check device ID */
	lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);
	if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID){

	GPIOOn(LED2);

	while(1){
	  /* manage here device not found */
			}

	}

	GPIOOn(LED3);

	/* Restore default configuration */
	lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
	do {
	lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

	/* Set full scale */
	lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
	lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
	lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);

	/* Configure filtering chain - See datasheet for filtering chain details */
	/* Accelerometer filtering chain */
	lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
	lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
	lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
	/* Gyroscope filtering chain */
	lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
	lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
	lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);

	/* Set Output Data Rate / Power mode */
	lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
	lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);


  /* Read samples in polling mode (no int) */
  while(1)
  {
    /* Read device status register */
    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

    if ( reg.status_imu.xlda && reg.status_imu.gda )
    {
      /* Read imu data */
      memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
      memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));

      lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw_acceleration.u8bit);
      lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate.u8bit);

      acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]);
      acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]);
      acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]);

      angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
      angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
      angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

     // tx_com(tx_buffer, strlen((char const*)tx_buffer));
    }

    if ( reg.status_mag.zyxda )
    {
      /* Read magnetometer data */
      memset(data_raw_magnetic_field.u8bit, 0x00, 3 * sizeof(int16_t));

      lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field.u8bit);

      magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[0]);
      magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[1]);
      magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[2]);

      //tx_com(tx_buffer, strlen((char const*)tx_buffer));

    }
  }
}

/*
 * @brief  Write generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_imu(void *handle, uint8_t reg, uint8_t *bufp,
                                  uint16_t len)
{
	sensbus_t *sensbus = (sensbus_t*)handle;

	uint8_t array[I2C_BUFFER_LEN];
	uint8_t stringpos = 0;
	array[0] = reg;

	for (stringpos = 0; stringpos < len; stringpos++)
	{
		array[stringpos + 1] = *(bufp + stringpos);
	}

	I2CM_XFER_T i2cData;
	// Prepare the i2cData register
	i2cData.slaveAddr = sensbus->i2c_address;
	i2cData.options   = 0;
	i2cData.status    = 0;
	i2cData.txBuff    = array;
	i2cData.txSz      = len+1;
	i2cData.rxBuff    = 0;
	i2cData.rxSz      = 0;

	/* Send the i2c data */
	if( Chip_I2CM_XferBlocking( sensbus->hbus , &i2cData ) == 0 )
	{
	return -1;
	}

	return 0;
}

/*
 * @brief  Write generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_mag(void *handle, uint8_t reg, uint8_t *bufp,
                                  uint16_t len)
{
	sensbus_t *sensbus = (sensbus_t*)handle;

	uint8_t array[I2C_BUFFER_LEN];
	uint8_t stringpos = 0;
	array[0] = reg;

	for (stringpos = 0; stringpos < len; stringpos++)
	{
		array[stringpos + 1] = *(bufp + stringpos);
	}

	I2CM_XFER_T i2cData;
	// Prepare the i2cData register
	i2cData.slaveAddr = sensbus->i2c_address;
	i2cData.options   = 0;
	i2cData.status    = 0;
	i2cData.txBuff    = array;
	i2cData.txSz      = len+1;
	i2cData.rxBuff    = 0;
	i2cData.rxSz      = 0;

	/* Send the i2c data */
	if( Chip_I2CM_XferBlocking( sensbus->hbus , &i2cData ) == 0 )
	{
	return -1;
	}

	return 0;
}

/*
 * @brief  Read generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_imu(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
	sensbus_t *sensbus = (sensbus_t*)handle;

	I2CM_XFER_T i2cData;

	i2cData.slaveAddr = sensbus->i2c_address;
	i2cData.options   = 0;
	i2cData.status    = 0;
	i2cData.txBuff    = &reg;
	i2cData.txSz      = 1;
	i2cData.rxBuff    = bufp;
	i2cData.rxSz      = len;

	if( Chip_I2CM_XferBlocking( sensbus->hbus , &i2cData ) == 0 )
	{
	return -1;
	};

  return 0;
}

/*
 * @brief  Read generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_mag(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
	sensbus_t *sensbus = (sensbus_t*)handle;

  	I2CM_XFER_T i2cData;
  	i2cData.slaveAddr = sensbus->i2c_address;
  	i2cData.options   = 0;
  	i2cData.status    = 0;
  	i2cData.txBuff    = &reg;
  	i2cData.txSz      = 1;
  	i2cData.rxBuff    = bufp;
  	i2cData.rxSz      = len;

  	if( Chip_I2CM_XferBlocking( sensbus->hbus , &i2cData ) == 0 )
  	{
  		return -1;
  	}

  	return 0;

}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to trasmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  #ifdef NUCLEO_F411RE
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
  #endif
  #ifdef STEVAL_MKI109V3
  CDC_Transmit_FS(tx_buffer, len);
  #endif
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  StopWatch_DelayMs(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
	Chip_SCU_I2C0PinConfig(I2C0_STANDARD_FAST_MODE);
	Chip_I2C_Init(I2C0);
	Chip_I2C_SetClockRate(I2C0, 400000);
	Chip_I2C_SetMasterEventHandler(I2C0, Chip_I2C_EventHandlerPolling);
	StopWatch_Init();
}
