/**
* \file srec.c
* \author tom.m
*
* \see srec.h.
*/

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

#include "project.h"
#include "srec.h"

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

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
SREC_Error_t
SREC_ParseLine
(
    const uint8_t *buffer
)
{
    SREC_Format_t
        srec;
        
    uint8_t
        address_length,
        i,
        j,
        k;
        
    uint32_t
        crc,
        byte;
    
    i = 0;
    
    memset(&srec, 0, sizeof(srec));

    if (buffer[i] != 's' && buffer[i] != 'S')
    {
        return SREC_ERROR_BadCharacter;
    }
    if (buffer[i+1] >= '0' && buffer[i+1] <= '9' && buffer[i+1] != '4' && buffer[i+1] != '6')
    {
        srec.type = buffer[i+1] - '0';
        address_length = SREC_AddressLen(srec.type);
        if(address_length == 0)
        {
            return SREC_ERROR_BadLength;
        }
        
    }
    else
    {
        return SREC_ERROR_BadType;
    }
    
    i += 2;

    srec.count = (SREC_Asc2Int (buffer[i]) << 4) + SREC_Asc2Int (buffer[i+1]);
    
    i += 2;

    if (srec.count <  (address_length+1))
    {
        return SREC_ERROR_BadCount;
    }
    
    srec.address = 0;
    
    for(k = 0; k < address_length; k++)
    {
        #if (UNTESTED)
        byte = (SREC_Asc2Int (buffer[i++]) << 4) + SREC_Asc2Int (buffer[i++]);
        byte <<= ((address_length-k-1)*8);
        srec.address += byte;
        #else
        byte = (SREC_Asc2Int (buffer[i++]) << 4);
        byte += SREC_Asc2Int (buffer[i++]);
        byte <<= ((address_length - k - 1) * 8);
        srec.address += byte;
        #endif
    }

    if (srec.count > (address_length+1))
    {
        memset(srec.data, 0, SREC_BootSrecBlockSize);

        for (j = i; j < (i + (srec.count - (address_length+1)) * 2); j += 2)
        {
            srec.data[(j - i) / 2] = (SREC_Asc2Int (buffer[j]) << 4) + SREC_Asc2Int (buffer[j + 1]);
        }
    }

    i += (srec.count - (address_length+1)) * 2;

    srec.checksum = (SREC_Asc2Int (buffer[i]) << 4) + SREC_Asc2Int (buffer[i+1]);
    
    crc = srec.count;
    crc += (uint8_t)(srec.address >> 16) & 0xff;
    crc += (uint8_t)(srec.address >> 8) & 0xff;
    crc += (uint8_t)(srec.address) & 0xff;

    for (i = 0; i < (srec.count - (address_length+1)); i++)
    {
        crc += srec.data[i];
    }
    crc = (~crc) & 0xff;
    
    if (crc != srec.checksum)
    {
        #if (DIAGNOSTICS)
        printf("#0x%02x:0x%02x\r\n", (uint8_t)(crc & 0xff), srec.checksum);
        #endif
        return SREC_ERROR_BadChecksum;
    }
    
    return SREC_ERROR_None;
}

/*----------------------------------------------------------------------*/
uint8_t
SREC_Asc2Int
(
    uint8_t asc
)
{
    if (asc >= '0' && asc <= '9')
    {
        return asc - '0';
    }
    else if (asc >= 'a' && asc <= 'f')
    {
        return 10 + asc - 'a';
    }
    else if (asc >= 'A' && asc <= 'F')
    {
        return 10 + asc - 'A';
    }
    else
    {
        return 0xff;
    }
}

/*----------------------------------------------------------------------*/
uint8_t
SREC_AddressLen
(
    uint8_t type
)
{
    uint8_t len;
    switch(type)
    {
        case SREC_TYPE_S0:
        case SREC_TYPE_S5:
        {
            len = 2;
            break;
        }
        case SREC_TYPE_S1:
        case SREC_TYPE_S2:
        case SREC_TYPE_S3:
        {
            len = type + 1;
            break;
        }
        case SREC_TYPE_S7:
        case SREC_TYPE_S8:
        case SREC_TYPE_S9:
        {
            len = 11 - type;
            break;
        }
        default:
        {
            len = 0;
            break;
        }
    }
    return len;
}
