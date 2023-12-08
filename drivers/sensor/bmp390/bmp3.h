/**
* Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
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
* @file       bmp3.h
* @date       2022-04-01
* @version    v2.0.6
*
*/

/*!
 * @defgroup bmp3 BMP3
 */

#ifndef _BMP3_H
#define _BMP3_H

#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

/* Header includes */
#include "bmp3_defs.h"

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*bmp3_bus_ready_fn)(const struct device *dev);
typedef	int (*bmp3_read_fn)(const struct device *dev,uint8_t reg,void *data,size_t len);
typedef int (*bmp3_write_fn)(const struct device *dev,uint8_t reg,void *data,size_t len);

struct bmp390_sample 
{
	uint32_t press;
	uint32_t temp;
};

struct bmp390_bus_io 
{
  bmp3_bus_ready_fn ready;
  bmp3_read_fn      read;
  bmp3_write_fn     write;
};

struct bmp390_config 
{
  struct i2c_dt_spec i2c;
  const struct bmp390_bus_io *bus_io;
  uint8_t odr;
  uint8_t osr_pressure;
  uint8_t osr_temp;
  uint8_t iir;
};

struct bmp390_data 
{
  struct bmp3_settings settings;
	struct bmp3_calib_data calib_data;
  struct bmp3_data data;
  struct bmp390_sample sample;
  bool ready;
};

/**
 * \ingroup bmp3
 * \defgroup bmp3ApiInit Initialization
 * @brief Initialize the sensor and device structure
 */
/*!
 *  @brief This API is the entry point.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id and calibration data of the sensor.
 *
 *  @param[in,out] dev : Structure instance of bmp3_dev
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bmp3_init(const struct device *dev);

/**
 * \ingroup bmp3
 * \defgroup bmp3ApiSoftReset Soft reset
 * @brief Perform soft reset of the sensor
 */

/*!
 * \ingroup bmp3ApiSoftReset
 * \page bmp3_api_bmp3_soft_reset bmp3_soft_reset
 * \code
 * int8_t bmp3_soft_reset(const struct bmp3_dev *dev);
 * \endcode
 * @details This API performs the soft reset of the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_soft_reset(const struct device *dev);

/**
 * \ingroup bmp3
 * \defgroup bmp3ApiSensorS Sensor settings
 * @brief Get / Set sensor settings
 */

/*!
 * \ingroup bmp3ApiSensorS
 * \page bmp3_api_bmp3_set_sensor_settings bmp3_set_sensor_settings
 * \code
 * int8_t bmp3_set_sensor_settings(uint32_t desired_settings, struct bmp3_settings *settings, struct bmp3_dev *dev);
 * \endcode
 * @details This API sets the power control(pressure enable and
 * temperature enable), over sampling, odr and filter
 * settings in the sensor.
 *
 * @param[in] desired_settings : Variable used to select the settings which
 *                               are to be set in the sensor.
 * @param[in] settings         : Structure instance of bmp3_settings
 * @param[in] dev              : Structure instance of bmp3_dev.
 *
 * @note : Below are the macros to be used by the user for selecting the
 * desired settings. User can do OR operation of these macros for configuring
 * multiple settings.
 *
 *@verbatim
 * ---------------------|----------------------------------------------
 * desired_settings     |   Functionality
 * ---------------------|----------------------------------------------
 * BMP3_SEL_PRESS_EN    |   Enable/Disable pressure.
 * BMP3_SEL_TEMP_EN     |   Enable/Disable temperature.
 * BMP3_SEL_PRESS_OS    |   Set pressure oversampling.
 * BMP3_SEL_TEMP_OS     |   Set temperature oversampling.
 * BMP3_SEL_IIR_FILTER  |   Set IIR filter.
 * BMP3_SEL_ODR         |   Set ODR.
 * BMP3_SEL_OUTPUT_MODE |   Set either open drain or push pull
 * BMP3_SEL_LEVEL       |   Set interrupt pad to be active high or low
 * BMP3_SEL_LATCH       |   Set interrupt pad to be latched or nonlatched.
 * BMP3_SEL_DRDY_EN     |   Map/Unmap the drdy interrupt to interrupt pad.
 * BMP3_SEL_I2C_WDT_EN  |   Enable/Disable I2C internal watch dog.
 * BMP3_SEL_I2C_WDT     |   Set I2C watch dog timeout delay.
 * ---------------------|----------------------------------------------
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_set_sensor_settings(uint32_t desired_settings, struct bmp3_settings *settings, const struct device *dev);

/*!
 * \ingroup bmp3ApiSensorS
 * \page bmp3_api_bmp3_get_sensor_settings bmp3_get_sensor_settings
 * \code
 * int8_t bmp3_get_sensor_settings(struct bmp3_settings *settings, struct bmp3_dev *dev);
 * \endcode
 * @details This API gets the power control(power mode, pressure enable and
 * temperature enable), over sampling, odr, filter, interrupt control and
 * advance settings from the sensor.
 *
 * @param[out] settings       : Structure instance of bmp3_settings
 * @param[in] dev             : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_get_sensor_settings(struct bmp3_settings *settings, const struct device *dev);

/**
 * \ingroup bmp3
 * \defgroup bmp3ApiPowermode Power mode
 * @brief Set / Get power mode of the sensor
 */

/*!
 * \ingroup bmp3ApiPowermode
 * \page bmp3_api_bmp3_set_op_mode bmp3_set_op_mode
 * \code
 * int8_t bmp3_set_op_mode(struct bmp3_settings *settings, struct bmp3_dev *dev);
 * \endcode
 * @details This API sets the power mode of the sensor.
 *
 * @param[in] settings       : Structure instance of bmp3_settings
 * @param[in] dev            : Structure instance of bmp3_dev.
 *
 *@verbatim
 * ----------------------|-------------------
 * dev->settings.op_mode |   Macros
 * ----------------------|-------------------
 *     0                 | BMP3_MODE_SLEEP
 *     1                 | BMP3_MODE_FORCED
 *     3                 | BMP3_MODE_NORMAL
 * ----------------------|-------------------
 *@endverbatim
 *
 * @note : Before setting normal mode, valid odr and osr settings should be set
 * in the sensor by using 'bmp3_set_sensor_settings' function.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_set_op_mode(struct bmp3_settings *settings, const struct device *dev);

/*!
 * \ingroup bmp3ApiPowermode
 * \page bmp3_api_bmp3_get_op_mode bmp3_get_op_mode
 * \code
 * int8_t bmp3_get_op_mode(uint8_t *op_mode, const struct bmp3_dev *dev);
 * \endcode
 * @details This API gets the power mode of the sensor.
 *
 * @param[in] dev      : Structure instance of bmp3_dev.
 * @param[out] op_mode : Pointer variable to store the op-mode.
 *
 *@verbatim
 * ------------------------------------------
 *   op_mode             |   Macros
 * ----------------------|-------------------
 *     0                 | BMP3_MODE_SLEEP
 *     1                 | BMP3_MODE_FORCED
 *     3                 | BMP3_MODE_NORMAL
 * ------------------------------------------
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_get_op_mode(uint8_t *op_mode, const struct device *dev);

/**
 * \ingroup bmp3
 * \defgroup bmp3ApiData Sensor Data
 * @brief Get Sensor data
 */

/*!
 * \ingroup bmp3ApiData
 * \page bmp3_api_bmp3_get_sensor_data bmp3_get_sensor_data
 * \code
 * int8_t bmp3_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *data, struct bmp3_dev *dev);
 * \endcode
 * @details This API reads the pressure, temperature or both data from the
 * sensor, compensates the data and store it in the bmp3_data structure
 * instance passed by the user.
 *
 * @param[in] sensor_comp : Variable which selects which data to be read from
 * the sensor.
 *
 *@verbatim
 * sensor_comp |   Macros
 * ------------|-------------------
 *     1       | BMP3_PRESS
 *     2       | BMP3_TEMP
 *     3       | BMP3_ALL
 *@endverbatim
 *
 * @param[out] data : Structure instance of bmp3_data.
 * @param[in] dev   : Structure instance of bmp3_dev.
 *
 * @note : For fixed point the compensated temperature and pressure has a multiplication factor of 100.
 *         Units are degree celsius and Pascal respectively.
 *         ie. If temperature is 2426 then temperature is 24.26 deg C
 *         If press is 9528709, then pressure is 95287.09 Pascal.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *data, const struct device *dev);

/**
 * \ingroup bmp3
 * \defgroup bmp3ApiRegs Registers
 * @brief Read / Write data to the given register address
 */

/*!
 * \ingroup bmp3ApiRegs
 * \page bmp3_api_bmp3_set_regs bmp3_set_regs
 * \code
 * int8_t bmp3_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct bmp3_dev *dev);
 * \endcode
 * @details This API writes the given data to the register address
 * of the sensor.
 *
 *  @param[in] reg_addr  : Register address to where the data to be written.
 *  @param[in] reg_data  : Pointer to data buffer which is to be written
 *                         in the sensor.
 *  @param[in] len       : No. of bytes of data to write.
 *  @param[in] dev       : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint32_t len, const struct device *dev);

/*!
 * \ingroup bmp3ApiRegs
 * \page bmp3_api_bmp3_get_regs bmp3_get_regs
 * \code
 * int8_t bmp3_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t length, const struct bmp3_dev *dev);
 * \endcode
 * @details This API reads the data from the given register address of the sensor.
 *
 *  @param[in] reg_addr  : Register address from where the data to be read
 *  @param[out] reg_data : Pointer to data buffer to store the read data.
 *  @param[in] len       : No. of bytes of data to be read.
 *  @param[in] dev       : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval 0  -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Error
 */
int8_t bmp3_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, const struct device *dev);

/**
 * \ingroup bmp3
 * \defgroup bmp3ApiStatus Sensor Status
 * @brief Read status of the sensor
 */

/*!
 * \ingroup bmp3ApiStatus
 * \page bmp3_api_bmp3_get_status bmp3_get_status
 * \code
 * int8_t bmp3_get_status(struct bmp3_status *status, struct bmp3_dev *dev);
 * \endcode
 * @details This API gets the command ready, data ready for pressure and
 * temperature and interrupt (fifo watermark, fifo full, data ready) and
 * error status from the sensor.
 *
 * @param[out] status : Structure instance of bmp3_status
 * @param[in] dev     : Structure instance of bmp3_dev
 *
 * @return Result of API execution status.
 * @retval 0  -> Success
 * @retval <0 -> Error
 */
int8_t bmp3_get_status(struct bmp3_status *status, const struct device *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _BMP3_H */
