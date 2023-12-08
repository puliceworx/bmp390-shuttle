/* Bosch BMP390 pressure sensor
 *
 * Copyright (c) 2020 Facebook, Inc. and its affiliates
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf
 */

#ifndef __BMP390_H
#define __BMP390_H

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

#include "bmp3.h"


/**\name    Convert milliseconds to microseconds */
#define BMP3_MS_TO_US(X)                       UINT32_C(X * 1000)

int bmp390_trigger_mode_init(const struct device *dev);
int bmp390_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

#endif /* __BMP390_H */
