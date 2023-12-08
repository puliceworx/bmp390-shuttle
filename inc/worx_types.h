#ifndef __WORX_TYPES_H__
#define __WORX_TYPES_H__

#include <stdint.h>
#include <stdbool.h>

typedef struct baro_s {
  float pressure;           // mbar
  float temperature;        // degree Celcius
  float asl;                // m (ASL = altitude above sea level)
} baro_t;

#endif /* __WORX_TYPES_H__ */
