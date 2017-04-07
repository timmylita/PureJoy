/**
 * \file halCell.c
 * \author tom.m
 *
 * \see halCell.h.
 */

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

#include "project.h"
#include "timer.h"
#include "halCell.h"

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

#define HAL_CELL_USART                     &USARTD0
#define HAL_CELL_USART_BAUDRATE            115200
#define HAL_CELL_USART_CHAR_LENGTH         USART_CHSIZE_8BIT_gc
#define HAL_CELL_USART_PARITY              USART_PMODE_DISABLED_gc
#define HAL_CELL_USART_STOP_BIT            false

/************************************************************************/
/* Types.                                                               */
/************************************************************************/

/************************************************************************/
/* Private prototypes.                                                  */
/************************************************************************/

/************************************************************************/
/* Constants.                                                           */
/************************************************************************/

/************************************************************************/
/* Private variables.                                                   */
/************************************************************************/
/************************************************************************/
/* Macros.                                                              */
/************************************************************************/

/************************************************************************/
/* Implementations.                                                     */
/************************************************************************/

/*----------------------------------------------------------------------*/
void 
HAL_CELL_Init
(
	void
)
{
    static usart_rs232_options_t usart_options =
    {
        .baudrate = HAL_CELL_USART_BAUDRATE,
        .charlength = HAL_CELL_USART_CHAR_LENGTH,
        .paritytype = HAL_CELL_USART_PARITY,
        .stopbits = HAL_CELL_USART_STOP_BIT
    };
	
    ioport_configure_pin
    (
		CELL_PWR_EN, 
		IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH
    );
	
    ioport_configure_pin
    (
		CELL_RESET,
		IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW
    );
	
	ioport_configure_pin
	(
		CELL_RFTXDISABLE,
		IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW
	);
	
	ioport_configure_pin
	(
		CELL_RTS,
		IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW
	);
	
	ioport_configure_pin
	(
		CELL_STATUS,
		IOPORT_DIR_INPUT
	);
	
	ioport_configure_pin
	(
		CELL_RFTXMON,
		IOPORT_DIR_INPUT
	);

	ioport_configure_pin
	(
		CELL_CTS,
		IOPORT_DIR_INPUT
	);
		
	ioport_configure_pin
	(
		CELL_GPIO2,
		IOPORT_DIR_INPUT
	);	
	
    ioport_configure_pin
	(
		CELL_RXD, 
		IOPORT_DIR_INPUT
	);
	
	ioport_set_pin_dir
	(
		CELL_TXD,
		IOPORT_DIR_OUTPUT
	);
	
	usart_serial_init(HAL_CELL_USART, &usart_options);
}


/*----------------------------------------------------------------------*/
int
HAL_CELL_Ioctl
(
	const uint8_t req,
	const uint32_t arg,
	void * argp
)
{
	TIMER_t tmr;
	
	switch(req)
	{
		case HAL_CELL_REQ_Reset:
		{
			TIMER_Configure(&tmr, 0, 200);
			ioport_set_pin_low(CELL_RESET);
			while(!TIMER_Check(&tmr));
			ioport_set_pin_high(CELL_RESET);
			break;
		}
		case HAL_CELL_REQ_PowerEnable:
		{
			TIMER_Configure(&tmr, 0, 1000);
			ioport_set_pin_low(CELL_PWR_EN);
			while(!TIMER_Check(&tmr));
			TIMER_Mark(&tmr);
			ioport_set_pin_high(CELL_PWR_EN);
			while(!TIMER_Check(&tmr));
			
			break;
		}
		default:
		{
			break;
		}
	}
	
	return 0;
}

/*----------------------------------------------------------------------*/
int HAL_CELL_Write
(
	const char * str, 
	uint32_t str_len,
	uint32_t timeout
)
{
	uint32_t i = 0;
	TIMER_t tmr;
	
    if (str == NULL)
    {
	    return -1;
    }	
		
	TIMER_Configure(&tmr, timeout/1000, timeout%1000);
	
	do
	{
		if(i == str_len)
		{
			break;
		}
		if (usart_data_register_is_empty(HAL_CELL_USART)) 
		{
			usart_put(HAL_CELL_USART, (uint8_t)str[i]);
			i++;
			TIMER_Mark(&tmr);
		}	
	}while(!TIMER_Check(&tmr));
	
	return i;
}

/*----------------------------------------------------------------------*/
int HAL_CELL_Read
(
	char * str,
	uint32_t str_len,
	uint32_t timeout
)
{
	uint32_t i = 0;
	TIMER_t tmr;
	
	if (str == NULL)
	{
		return -1;
	}
	
	TIMER_Configure(&tmr, timeout/1000, timeout%1000);
	
	do
	{
		if(i == str_len)
		{
			break;
		}
		if (usart_rx_is_complete(HAL_CELL_USART))
		{
			str[i] =  usart_get(HAL_CELL_USART);
			i++;
			usart_clear_rx_complete(HAL_CELL_USART);
			TIMER_Mark(&tmr);
		}
	}while(!TIMER_Check(&tmr));
	
	return i;	
}