/**
 * \file srec.h
 * \author tom.m
 *
 * Functionality for parsing srec data.
 */
 
#ifndef _SREC_H
#define _SREC_H

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

#define SREC_BootSrecBlockSize                  0x40UL

/************************************************************************/
/* Types.                                                               */
/************************************************************************/

typedef enum
{
    SREC_ERROR_None,
    SREC_ERROR_BadChecksum,
    SREC_ERROR_BadDeviceType,
    SREC_ERROR_BadCharacter,
    SREC_ERROR_BadLength,
    SREC_ERROR_BadType,
    SREC_ERROR_BadCount,
    SREC_ERROR_BadJ,
    SREC_ERROR_BadI
} SREC_Error_t;

typedef enum
{
    SREC_TYPE_S0 = 0,
    SREC_TYPE_S1 = 1,
    SREC_TYPE_S2 = 2,
    SREC_TYPE_S3 = 3,
    SREC_TYPE_S5 = 5,
    SREC_TYPE_S7 = 7,
    SREC_TYPE_S8 = 8,
    SREC_TYPE_S9 = 9
} SREC_Type_t;

typedef struct
{
    uint8_t     type;
    uint8_t     count;
    uint32_t    address;
    uint8_t     data[SREC_BootSrecBlockSize];
    uint8_t     checksum;
} SREC_Format_t;

/************************************************************************/
/* Public prototypes.                                                   */
/************************************************************************/

SREC_Error_t
SREC_ParseLine
(
    const uint8_t *buffer
);

uint8_t
SREC_Asc2Int
(
    const uint8_t asc
);

uint8_t
SREC_AddressLen
(
    uint8_t type
);

/************************************************************************/
/* Public variables.                                                    */
/************************************************************************/

/************************************************************************/
/* Public Macros.                                                       */
/************************************************************************/

#endif // _SREC_H