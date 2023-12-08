/* Bosch BMP390 pressure sensor
 *
 * Copyright (c) 2020 Facebook, Inc. and its affiliates
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds001.pdf
 */

#define DT_DRV_COMPAT bosch_bmp390

#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include "bmp390.h"

#ifdef CONFIG_BMP390_LOG_LEVEL
LOG_MODULE_REGISTER(BMP390, CONFIG_BMP390_LOG_LEVEL);
#else 
LOG_MODULE_REGISTER(BMP390, 0);
#endif

#define BMP390_SAMPLE_BUFFER_SIZE sizeof(struct bmp390_sample)

static bool bmp3_bus_ready_i2c(const struct device *dev) 
{
  const struct bmp390_config *cfg = dev->config;

  return device_is_ready(cfg->i2c.bus);
}

static int bmp3_read_i2c(const struct device *dev, uint8_t reg, void *buf, size_t len)
{
	const struct bmp390_config *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->i2c, reg, buf, len);
}

static int bmp3_write_i2c(const struct device *dev, uint8_t reg, void *buf, size_t len)
{
	const struct bmp390_config *cfg = dev->config;

	return i2c_burst_write_dt(&cfg->i2c, reg, buf, len);
}

static const struct bmp390_bus_io bmp3_bus_io_12c = 
{
  .ready = bmp3_bus_ready_i2c,
  .read = bmp3_read_i2c,
	.write = bmp3_write_i2c,
};

/* sensor interface commands */
static int bmp3_attr_set(const struct device *dev,
			   enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	int ret;

	switch (attr) 
  {
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int bmp3_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
  int8_t ret; 
 	struct bmp390_data *devdata = dev->data;
  uint8_t sensor_comp = BMP3_PRESS | BMP3_TEMP;

  __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

  ret =  bmp3_get_sensor_data(sensor_comp, &devdata->data, dev);

  return ret; 
}

static int bmp3_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
  struct bmp390_data *data = dev->data;

	switch (chan) 
  {
	case SENSOR_CHAN_PRESS:
		sensor_value_from_double(val, data->data.pressure);
		break;

	case SENSOR_CHAN_DIE_TEMP:
	case SENSOR_CHAN_AMBIENT_TEMP:
    sensor_value_from_double(val, data->data.temperature);
		break;

  case SENSOR_CHAN_ALL:
    sensor_value_from_double(&val[0], data->data.pressure);
    sensor_value_from_double(&val[1], data->data.temperature);
    break;

	default:
		LOG_DBG("Channel not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api bmp3_api = 
{
	.attr_set = bmp3_attr_set,
	.sample_fetch = bmp3_sample_fetch,
	.channel_get = bmp3_channel_get,
//	.trigger_set = bmp3_trigger_set,
};

static int bmp390_init(const struct device *dev)
{
	const struct bmp390_config *cfg = dev->config;
  struct bmp390_data *data = dev->data;
  uint16_t settings_sel;
  int8_t err = 0; 

	LOG_DBG("Initializing BMP390 device at %p", dev);

  if (!cfg->bus_io->ready(dev))
  {
    LOG_ERR("Bus not ready");
    return -EINVAL;
  }

  int i = 3;
  do 
  {
    k_busy_wait(BMP3_MS_TO_US(1));

    // For some reason it often doesn't work first time
    err = bmp3_init(dev);
  } while (err != BMP3_OK && i-- > 0);
  
  if (err != BMP3_OK)
  {
    LOG_ERR("init failed.");
    return -EINVAL;
  }

  /* Select the pressure and temperature sensor to be enabled */
  data->settings.press_en = BMP3_ENABLE;
  data->settings.temp_en = BMP3_ENABLE;

  /* Select the output data rate and oversampling settings for pressure and temperature */
  data->settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
  data->settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
  data->settings.odr_filter.odr = BMP3_ODR_50_HZ;
  //data->settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
  data->settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_7;

  /* Assign the settings which needs to be set in the sensor */
  settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN |
    BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_IIR_FILTER;

  err = bmp3_set_sensor_settings(settings_sel, &data->settings, dev);

  /* Set the power mode to normal mode */
  data->settings.op_mode = BMP3_MODE_NORMAL;
  err = bmp3_set_op_mode(&data->settings, dev);

  k_busy_wait(BMP3_MS_TO_US(20)); // wait before first read out

  return 0;
}

#define BMP3_DEFINE(inst) \
	static struct bmp390_data data_##inst;  \
	static const struct bmp390_config config_##inst = \
{                                         \
  .i2c = I2C_DT_SPEC_INST_GET(inst),  \
  .bus_io = &bmp3_bus_io_12c, \
  };								            \
	DEVICE_DT_INST_DEFINE(inst, bmp390_init, NULL, &data_##inst, &config_##inst,     \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,	       \
			      &bmp3_api);

DT_INST_FOREACH_STATUS_OKAY(BMP3_DEFINE)



