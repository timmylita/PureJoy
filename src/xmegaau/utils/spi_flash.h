/**
 * \file spi_flash.h
 * \author tom.m
 *
 * Provides functionality for interoperating with the at25 family of
 * flash chips. All calls are blocking.
 */

#ifndef _SPI_FLASH
#define _SPI_FLASH

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

#define SPI_FLASH_PAGE_SIZE                                       (256UL)

#define SPI_FLASH_4KB_BLOCK_SIZE             (16UL * SPI_FLASH_PAGE_SIZE)

#define SPI_FLASH_32KB_BLOCK_SIZE        (8UL * SPI_FLASH_4KB_BLOCK_SIZE)

#define SPI_FLASH_64KB_BLOCK_SIZE       (2UL * SPI_FLASH_32KB_BLOCK_SIZE)

#define SPI_FLASH_SECTOR_SIZE                 (SPI_FLASH_64KB_BLOCK_SIZE)

#define SPI_FLASH_SECTOR_COUNT                                   (0x40UL)

#define SPI_FLASH_BLOCK_COUNT                                  (0x1000UL)

#define SPI_FLASH_CAPACITY                                              \
        (                                                               \
            SPI_FLASH_SECTOR_COUNT                                      \
            *                                                           \
            SPI_FLASH_SECTOR_SIZE                                       \
        )

#define SPI_FLASH_SIZE                                       (0x400000UL)

/************************************************************************/
/* Types.                                                               */
/************************************************************************/

/**
 * Enumerates valid request types.
 */
typedef enum {
    SPI_FLASH_REQ_SectorUnprotect = 0,
    SPI_FLASH_REQ_SectorProtect,
    SPI_FLASH_REQ_GlobalUnprotect,
    SPI_FLASH_REQ_GlobalProtect,
    SPI_FLASH_REQ_ReadStatus,
    SPI_FLASH_REQ_WriteStatus,
    SPI_FLASH_REQ_Block4kErase,
    SPI_FLASH_REQ_Block32kErase,
    SPI_FLASH_REQ_Block64kErase,
    SPI_FLASH_REQ_ChipErase,
    SPI_FLASH_REQ_DeviceId,
    SPI_FLASH_REQ_Reset,
    SPI_FLASH_REQ_SectorLockdown,
    SPI_FLASH_REQ_FreezeSectorLockdown
}SPI_FLASH_Req_t;

/**
 * Some information struct.
 */
typedef struct {
    uint8_t manufacturer_id;
    uint8_t device_id[2];
    uint8_t ext_device_info_string_length;
}SPI_FLASH_Info_t;

/************************************************************************/
/* Public prototypes.                                                   */
/************************************************************************/

/**
 * Initialized the spi flash interface.
 */
void SPI_FLASH_Init(void);

/**
 * Reads data from the flash chip.
 */
int
SPI_FLASH_Read
(
    const uint32_t address,
    void *buffer,
    const uint16_t data_len
);

/**
 * Write data to the flash chip.
 */
int
SPI_FLASH_Write
(
    const uint32_t address,
    const void *data,
    const uint16_t data_len
);

/**
 * Performs chip-specific io requests.
 */
int
SPI_FLASH_Ioctl
(
    const uint8_t req,
    const uint32_t arg,
    void * argp
);

/************************************************************************/
/* Public variables.                                                    */
/************************************************************************/

/************************************************************************/
/* Public Macros.                                                       */
/************************************************************************/

#endif  /* _SPI_FLASH */
