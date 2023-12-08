/**
 *
 * For the flag. T_ means task. H_ means HAL module. U_ would means utils.
 */

#ifndef _WORX_TASK_CONFIG_H_
#define _WORX_TASK_CONFIG_H_
#include <cmsis_os2.h>

/* Note that Zephyr's implementation is backwards from
 * CMSIS OS documentation
 */
#define OS_QUEUE_NOTIMEOUT 0

#define CONFIG_BLOCK_ADDRESS    (2048 * (64-1))
#define MCU_ID_ADDRESS          0x1FFF7A10
#define MCU_FLASH_SIZE_ADDRESS  0x1FFF7A22
#ifndef WORX_HEAP_SIZE
  #define WORX_HEAP_SIZE      30000
#endif
#define WORX_MIN_STACK_SIZE 128 

/* 
 * FreeRTOS Config params
 * I saw no point in changing these everywhere 
*/
#define configTICK_RATE_HZ_RAW  1000
#define configTICK_RATE_HZ			( ( portTickType ) configTICK_RATE_HZ_RAW )
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) WORX_MIN_STACK_SIZE )
#define configTOTAL_HEAP_SIZE		( ( size_t ) ( WORX_HEAP_SIZE ) )
#define configMAX_TASK_NAME_LEN		( 10 )

//Milliseconds to OS Ticks
#if configTICK_RATE_HZ_RAW != 1000
  #error "Please review the use of M2T and T2M if there is not a 1 to 1 mapping between ticks and milliseconds"
#endif
#define M2T(X) ((unsigned int)(X))
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))
#define T2M(X) ((unsigned int)(X))

/* End FreeRTOS Config params */

/* 
 * Priorities Defined in main
 * To align with CMSIS style priorities 
 * these are using the real time selections. 
 *  
 * CMSIS to Zephyr 
 * (osPriorityISR (56) - c_prio) 
 *  
 * #define PRIORITY 7               == osPriorityRealtime1
 * #define PRIORITY_HI PRIORITY - 1 == osPriorityRealtime2
 */

// Task priorities
#define SENSORS_TASK_PRI              osPriorityRealtime2   // CMSIS 50, Zephyr 6, FreeRTOS Priority 4

// Task names
#define SENSORS_TASK_NAME             "SENSORS"

//Task stack sizes
#define STACK_SIZE_DBGLOG_MULTIPLIER  10

#define SENSORS_TASK_STACKSIZE        (STACK_SIZE_DBGLOG_MULTIPLIER * configMINIMAL_STACK_SIZE) 

#endif /* _WORX_TASK_CONFIG_H_ */
