/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Synchronization demo using CMSIS RTOS V2 APIs.
 */

// system includes ------------------------------------------------------------
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

// local includes -------------------------------------------------------------
#include "sensors.h"

// library includes -----------------------------------------------------------

// Macros and Defines ---------------------------------------------------------
LOG_MODULE_REGISTER(MAIN, LOG_LEVEL_DBG);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

// enumerations ---------------------------------------------------------------

// structures -----------------------------------------------------------------

// global parameter declarations ----------------------------------------------

// local parameter declarations -----------------------------------------------
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// local function prototypes --------------------------------------------------

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&led)) 
  {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) 
  {
		return 0;
	}

  sensorsInit();

  while (1) 
	{
 		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) 
    {
			return 0;
		}

    k_msleep(SLEEP_TIME_MS);
  }

  return 0;
}


