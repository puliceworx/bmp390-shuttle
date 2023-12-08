/******************************************************************************
 * @file tof.c
 *
 * @brief time of flight controller implementation
 *
 * Version History
 * ======
 *
 * \addtogroup tof
 * @{
 *****************************************************************************/

// system includes ------------------------------------------------------------
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>
#include <cmsis_os2.h>
#include <math.h>
#include <errno.h>

#include <string.h>
#include <stdbool.h>

// local includes -------------------------------------------------------------
#include "worx_task_config.h"
#include "worx_types.h"
#include "sensors.h"

// library includes -----------------------------------------------------------

// Macros and Defines ---------------------------------------------------------

LOG_MODULE_REGISTER(SENSORS, CONFIG_BMP390_LOG_LEVEL);

#define SENSORS_READ_RATE_HZ            1000
#define SENSORS_STARTUP_TIME_MS         1000
#define SENSORS_READ_BARO_HZ            50
#define SENSORS_DELAY_BARO              (SENSORS_READ_RATE_HZ/SENSORS_READ_BARO_HZ)

// enumerations ---------------------------------------------------------------

// structures -----------------------------------------------------------------
struct bmp3_data
{
  /*! Compensated temperature */
  double temperature;

  /*! Compensated pressure */
  double pressure;
};

// global parameter declarations ----------------------------------------------

// local parameter declarations -----------------------------------------------
const struct device *const dev_sens = DEVICE_DT_GET(DT_ALIAS(press));

//static K_THREAD_STACK_DEFINE(sensorsTask_stack, SENSORS_TASK_STACKSIZE);
static K_THREAD_STACK_DEFINE(sensorsTask_stack, 2048);
static osThreadAttr_t sensorsTask_attr = {
		.name       = SENSORS_TASK_NAME,
		.stack_mem  = &sensorsTask_stack,
		//.stack_size = SENSORS_TASK_STACKSIZE,
    .stack_size = 2048,
		.priority   = SENSORS_TASK_PRI
};
osThreadId_t sensorsTaskId;

static bool isInit = false;

// local function prototypes --------------------------------------------------
static void sensorsTask(void *param);

bool sensorsInit(void)
{
  // true means there IS an error in initialization
  bool ret = true; 

  if(isInit) 
  {
    return false;
  }

  if (IS_ENABLED(CONFIG_BMP390))
  {
    if (!device_is_ready(dev_sens))
    {
      LOG_ERR("Error: Device %s is not ready", dev_sens->name);
      return ret;
    }

    ret = false; 
  }

  sensorsTaskId = osThreadNew(sensorsTask, NULL, &sensorsTask_attr);

  isInit = true;
  return ret;
}

static void sensorsScaleBaro(baro_t* baroScaled, float pressure,
                             float temperature)
{
  baroScaled->pressure = pressure*0.01f;
  baroScaled->temperature = temperature;
  baroScaled->asl = ((powf((1015.7f / baroScaled->pressure), 0.1902630958f)
      - 1.0f) * (25.0f + 273.15f)) / 0.0065f;
}

static int sensorsProcess(struct bmp3_data* dataOut)
{
  int err = 0;
  struct sensor_value data[2];

  err = sensor_sample_fetch(dev_sens);
  if (err != 0) 
  {
    LOG_ERR("Failed to fetch %d", err);
    return err;
  }

  err = sensor_channel_get(dev_sens, SENSOR_CHAN_ALL, data);
  if (err)
  {
    LOG_ERR("sample_fetch failed ret %d", err);
    return err;
  }

  dataOut->pressure = sensor_value_to_double(&data[0]);
  dataOut->temperature = sensor_value_to_double(&data[1]);
  
  return err;
}

static void sensorsTask(void *param)
{
  struct bmp3_data data;
  baro_t baroData;

  while (1)
  {
    osDelay(SENSORS_DELAY_BARO);
    //osDelay(2000);
    
    if (IS_ENABLED(CONFIG_BMP390))
    {
      sensorsProcess(&data);
      sensorsScaleBaro(&baroData, data.pressure, data.temperature);

      LOG_INF("asl %.3f, pressure %.3f, temp %.3f ",
              baroData.asl, baroData.pressure, baroData.temperature);
    }
  }
}

