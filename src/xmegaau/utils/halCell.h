/**
 * \file halCell.h
 * \author tom.M
 *
 * Provides functionality to interface with the Telit GL865 GSM chip
 */

#ifndef _HAL_CELL
#define _HAL_CELL

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

/************************************************************************/
/* Types.                                                               */
/************************************************************************/

/**
 * Enumerates valid request types.
 */
typedef enum {
	HAL_CELL_REQ_Reset,
    HAL_CELL_REQ_PowerEnable
}HAL_CELL_Req_t;


/************************************************************************/
/* Public prototypes.                                                   */
/************************************************************************/

/**
 * Initialized the gl865 chip.
 */
void 
HAL_CELL_Init
(
	void
);

/**
 * Performs gl865 chip-specific io requests.
 */
int
HAL_CELL_Ioctl
(
	const uint8_t req,
	const uint32_t arg,
	void * argp
);

/**
 * Write data to the gl865 chip via UART.
 */
int 
HAL_CELL_Write
(
	const char * str, 
	uint32_t str_len,
	uint32_t timeout
);

/**
 * Read data from the gl865 chip via UART.
 */
int 
HAL_CELL_Read
(
	char * str,
	uint32_t str_len,
	uint32_t timeout
);

/************************************************************************/
/* Public variables.                                                    */
/************************************************************************/

/************************************************************************/
/* Public Macros.                                                       */
/************************************************************************/

#endif  /* _HAL_CELL */