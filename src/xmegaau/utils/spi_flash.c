/**
 * \file spi_flash.c
 * \author tom.m
 *
 * \see spi_flash.h.
 */

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

#include "project.h"
#include "spi_flash.h"

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

#ifndef SPI_FLASH_SPI_INSTANCE
#define SPI_FLASH_SPI_INSTANCE      &SPIE
#endif

#ifndef SPI_FLASH_SPI_BAUDRATE
#define SPI_FLASH_SPI_BAUDRATE      500000UL
#endif

#ifndef SPI_FLASH_SPI_MODE
#define SPI_FLASH_SPI_MODE SPI_MODE_0
#endif

#define READY_BIT_MASK            0x01
#define DONT_CARE                 0
#define BLOCK_ALIGN_MASK_4K       0xFFFFF000UL
#define BLOCK_ALIGN_MASK_32K      0xFFFF8000UL
#define BLOCK_ALIGN_MASK_64K      0xFFFF0000UL

/************************************************************************/
/* Types.                                                               */
/************************************************************************/

/**
 * Enumerates the valid instructions for the at25df321a.
 */
enum
{
    SPI_FLASH_OPCODE_ReadArray          = 0x1B,
    SPI_FLASH_OPCODE_DeviceIdRead       = 0x9F,
    SPI_FLASH_OPCODE_WriteEnable        = 0x06,
    SPI_FLASH_OPCODE_WriteDisable       = 0x04,
    SPI_FLASH_OPCODE_ProgramPage        = 0x02,
    SPI_FLASH_OPCODE_WriteStatus1       = 0x01,
    SPI_FLASH_OPCODE_ChipErase          = 0x60,
    SPI_FLASH_OPCODE_Erase4kBlock       = 0x20,
    SPI_FLASH_OPCODE_Erase32kBlock      = 0x52,
    SPI_FLASH_OPCODE_Erase64kBlock      = 0xD8,
    SPI_FLASH_OPCODE_ReadStatus         = 0x05,
    SPI_FLASH_OPCODE_WriteStatusByte1   = 0x01,
    SPI_FLASH_OPCODE_WriteStatusByte2   = 0x31,
    SPI_FLASH_OPCODE_ProgramResume      = 0xD0,
    SPI_FLASH_OPCODE_ProtectSector      = 0x36,
    SPI_FLASH_OPCODE_UnprotectSector    = 0x39
};

/************************************************************************/
/* Private prototypes.                                                  */
/************************************************************************/

/************************************************************************/
/* Constants.                                                           */
/************************************************************************/

/************************************************************************/
/* Private variables.                                                   */
/************************************************************************/

struct spi_device SPI_FLASH_SPI_SLAVE =
{
    .id = SPIX_SS_FLASH
};

/************************************************************************/
/* Macros.                                                              */
/************************************************************************/

/************************************************************************/
/* Implementations.                                                     */
/************************************************************************/

/*----------------------------------------------------------------------*/
static uint8_t wait_ready(void)
{
  uint32_t count = 0;
  uint8_t ready_bit = 1;
  uint8_t command = SPI_FLASH_OPCODE_ReadStatus;
  
  spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
  
  do
  {
    spi_write_packet(SPI_FLASH_SPI_INSTANCE, &command, 1);    
    spi_read_packet(SPI_FLASH_SPI_INSTANCE, &ready_bit, 1);
    ready_bit = ready_bit & READY_BIT_MASK;
    count++;
  } while(( ready_bit == 1 ) && (count <= 0x7FFFFFFF));
  
  spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);

  return (ready_bit);
}

/*----------------------------------------------------------------------*/
void SPI_FLASH_Init(void)
{
  
    ioport_configure_pin(SPIX_SS_FLASH, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
    ioport_configure_pin(FLASH_WP, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);    
    ioport_configure_pin(SPIE_MOSI,IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
    ioport_configure_pin(SPIE_MISO, IOPORT_DIR_INPUT);
    ioport_configure_pin(SPIE_SCK,IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
  
    spi_master_init(SPI_FLASH_SPI_INSTANCE);
    spi_master_setup_device
    (
        SPI_FLASH_SPI_INSTANCE,
        &SPI_FLASH_SPI_SLAVE,
        SPI_FLASH_SPI_MODE,
        SPI_FLASH_SPI_BAUDRATE,
        0
    );
    spi_enable(SPI_FLASH_SPI_INSTANCE);
  
    SPI_FLASH_Ioctl(SPI_FLASH_REQ_WriteStatus,0x1000,NULL);
    SPI_FLASH_Ioctl(SPI_FLASH_REQ_GlobalUnprotect,0x0,NULL);
    
    #define DEBUG_SPI_REPORT 0
    #if (DEBUG_SPI_REPORT)
    dbg((PSTR("{\"spi_flash\":{\"page_size\":%u}}"), SPI_FLASH_PAGE_SIZE));
    dbg((PSTR("{\"spi_flash\":{\"4kb_size\":%u}}"), SPI_FLASH_4KB_BLOCK_SIZE));
    dbg((PSTR("{\"spi_flash\":{\"32kb_size\":%u}}"), SPI_FLASH_32KB_BLOCK_SIZE));
    dbg((PSTR("{\"spi_flash\":{\"64kb_size\":%lu}}"), SPI_FLASH_64KB_BLOCK_SIZE));
    dbg((PSTR("{\"spi_flash\":{\"sector_size\":%lu}}"), SPI_FLASH_SECTOR_SIZE));
    dbg((PSTR("{\"spi_flash\":{\"sector_count\":%u}}"), SPI_FLASH_SECTOR_COUNT));
    dbg((PSTR("{\"spi_flash\":{\"block_count\":%u}}"), SPI_FLASH_BLOCK_COUNT));
    dbg((PSTR("{\"spi_flash\":{\"capacity\":%llu}}"), SPI_FLASH_CAPACITY));
    #endif
  
}

/*----------------------------------------------------------------------*/
int SPI_FLASH_Read(const uint32_t address, void *buffer, const uint16_t data_len)
{
    uint8_t *data;
    data = (uint8_t *)buffer;
  uint8_t cmd_buf[6];
  
  if(wait_ready())
  {
    return -1;
  }
  
  cmd_buf[0] = SPI_FLASH_OPCODE_ReadArray;
  cmd_buf[1] = (uint8_t)((address >> 16) & 0xFF);
  cmd_buf[2] = (uint8_t)((address >> 8) & 0xFF);
  cmd_buf[3] = (uint8_t)(address & 0xFF);
  cmd_buf[4] = DONT_CARE;
  cmd_buf[5] = DONT_CARE;
  
  spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
  
  spi_write_packet(SPI_FLASH_SPI_INSTANCE, cmd_buf, 6);
  
  spi_read_packet(SPI_FLASH_SPI_INSTANCE, data, data_len);
  
  spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
  
  return 0;  
}

/*----------------------------------------------------------------------*/
int SPI_FLASH_Write(const uint32_t address, const void *buffer, const uint16_t data_len)
{
    uint8_t *data;
    data = (uint8_t *)buffer;
    
  uint8_t cmd_buf[4];

  uint32_t in_buffer_idx;
  uint32_t nb_bytes_to_write;
  uint32_t target_addr;
  uint32_t size_left;

  in_buffer_idx = 0;
  nb_bytes_to_write = data_len;
  target_addr = address;

  while ( in_buffer_idx < data_len )
  {
    if(wait_ready())
    {
      return -1;
    }
    
    cmd_buf[0] = SPI_FLASH_OPCODE_WriteEnable;
    
    spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
    
    spi_write_packet(SPI_FLASH_SPI_INSTANCE, cmd_buf, 1);
    
    spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);      
  
    cmd_buf[0] = SPI_FLASH_OPCODE_UnprotectSector;
    cmd_buf[1] = (address >> 16) & 0xFF;
    cmd_buf[2] = (address >> 8 ) & 0xFF;
    cmd_buf[3] = address & 0xFF;
    
    spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
  
    spi_write_packet(SPI_FLASH_SPI_INSTANCE, cmd_buf, 4);
    
    spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);    
    
    cmd_buf[0] = SPI_FLASH_OPCODE_WriteEnable;
    
    spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
    
    spi_write_packet(SPI_FLASH_SPI_INSTANCE, cmd_buf, 1);
    
    spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);    
    
    nb_bytes_to_write = 0x100 - (target_addr & 0xFF);
    
    // adjust max possible size to page boundary. 
    size_left = data_len - in_buffer_idx;
    if ( size_left < nb_bytes_to_write )
    {
      nb_bytes_to_write = size_left;
    }
    
    cmd_buf[0] = SPI_FLASH_OPCODE_ProgramPage;
    cmd_buf[1] = (target_addr >> 16) & 0xFF;
    cmd_buf[2] = (target_addr >> 8 ) & 0xFF;
    cmd_buf[3] = target_addr & 0xFF;
    
    spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
    
    spi_write_packet(SPI_FLASH_SPI_INSTANCE, cmd_buf, 4);
    
    spi_write_packet(SPI_FLASH_SPI_INSTANCE, &data[in_buffer_idx], nb_bytes_to_write);
 
    spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);

    target_addr += nb_bytes_to_write;
    in_buffer_idx += nb_bytes_to_write;
  }
  return 0;
}

/*----------------------------------------------------------------------*/
#if (1 == BOOTLOADER)
int SPI_FLASH_Ioctl(const uint8_t req, const uint32_t arg, void * argp)
{
    uint8_t tx_buf[8];
    
    if(wait_ready())
    {
        return -1;
    }
    
    switch(req)
    {
        case SPI_FLASH_REQ_GlobalProtect:
        case SPI_FLASH_REQ_GlobalUnprotect:
        {
            tx_buf[0] = SPI_FLASH_OPCODE_WriteEnable;
            
            spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
            
            spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
            
            spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
            
            tx_buf[0] = SPI_FLASH_OPCODE_WriteStatus1;
            tx_buf[1] = 0;
            
            spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
            
            spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 2);
            
            spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
            
            break;
        }
        case SPI_FLASH_REQ_WriteStatus:
        {
            tx_buf[0] = SPI_FLASH_OPCODE_WriteEnable;
            
            spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
            
            spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
            
            spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
            
            tx_buf[0] = SPI_FLASH_OPCODE_WriteStatusByte1;
            tx_buf[1] = arg & 0xFF;
            
            spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
            
            spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 2);
            
            spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
            
            tx_buf[0] = SPI_FLASH_OPCODE_WriteEnable;
            
            spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
            
            spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
            
            spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
            
            tx_buf[0] = SPI_FLASH_OPCODE_WriteStatusByte2;
            tx_buf[1] = (arg >> 8) & 0xFF;
            
            spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
            
            spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 2);
            
            spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
            
            break;
        }
        default:
        {
            break;
        }
    }
    return 0;
}
#else
int SPI_FLASH_Ioctl(const uint8_t req, const uint32_t arg, void * argp)
{
  uint8_t tx_buf[8];
  uint8_t rx_buf[8];
  
  if(wait_ready())
  {
    return -1;
  }
    
  switch(req)
  {
    case SPI_FLASH_REQ_DeviceId:
    {
      tx_buf[0] = SPI_FLASH_OPCODE_DeviceIdRead;
          
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
      
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
      
      spi_read_packet(SPI_FLASH_SPI_INSTANCE, rx_buf, 4);
      
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
      
      ((SPI_FLASH_Info_t *)argp)->manufacturer_id = rx_buf[0];
      ((SPI_FLASH_Info_t *)argp)->device_id[0] = rx_buf[1];
      ((SPI_FLASH_Info_t *)argp)->device_id[1] = rx_buf[2];
      ((SPI_FLASH_Info_t *)argp)->ext_device_info_string_length = rx_buf[3];
      
      break;
    }
    case SPI_FLASH_REQ_SectorProtect:
    {
      tx_buf[0] = SPI_FLASH_OPCODE_WriteEnable;
              
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
      
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
    
      tx_buf[0] = SPI_FLASH_OPCODE_ProtectSector;
      tx_buf[1] = (arg >> 16) & 0xFF;
      tx_buf[2] = (arg >> 8 ) & 0xFF;
      tx_buf[3] = arg & 0xFF;
    
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
      
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 4);
      
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      break;
    }
    case SPI_FLASH_REQ_SectorUnprotect:
    {
      tx_buf[0] = SPI_FLASH_OPCODE_WriteEnable;    
        
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
      
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
      
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      tx_buf[0] = SPI_FLASH_OPCODE_UnprotectSector;
      tx_buf[1] = (arg >> 16) & 0xFF;
      tx_buf[2] = (arg >> 8 ) & 0xFF;
      tx_buf[3] = arg & 0xFF;
      
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 4);
        
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      break;
    }        
    case SPI_FLASH_REQ_GlobalProtect:
    case SPI_FLASH_REQ_GlobalUnprotect:
    {
      tx_buf[0] = SPI_FLASH_OPCODE_WriteEnable;    
        
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
      
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      tx_buf[0] = SPI_FLASH_OPCODE_WriteStatus1;
      tx_buf[1] = 0;
      
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 2);
        
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      break;
    }
    case SPI_FLASH_REQ_ChipErase:
    {
      tx_buf[0] = SPI_FLASH_OPCODE_WriteEnable;
          
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);

      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
      
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      tx_buf[0] = SPI_FLASH_OPCODE_ChipErase;

      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
        
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      break;
    }    
    case SPI_FLASH_REQ_Reset:
    {
      tx_buf[0] = SPI_FLASH_OPCODE_WriteEnable;
          
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
        
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      break;
    }
    case SPI_FLASH_REQ_Block4kErase:
    {   
      
      tx_buf[0] = SPI_FLASH_OPCODE_WriteEnable;
          
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
      
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
      
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      tx_buf[0] = SPI_FLASH_OPCODE_Erase4kBlock;
      tx_buf[1] = ((arg & BLOCK_ALIGN_MASK_4K) >> 16) & 0xFF;
      tx_buf[2] = ((arg & BLOCK_ALIGN_MASK_4K) >> 8 ) & 0xFF;
      tx_buf[3] = (arg & BLOCK_ALIGN_MASK_4K) & 0xFF;
      
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);

      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 4);
        
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      break;
    }
    case SPI_FLASH_REQ_Block32kErase:
    {
      tx_buf[0] = SPI_FLASH_OPCODE_WriteEnable;    
      
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
      
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      tx_buf[0] = SPI_FLASH_OPCODE_Erase32kBlock;
      tx_buf[1] = ((arg & BLOCK_ALIGN_MASK_32K) >> 16) & 0xFF;
      tx_buf[2] = ((arg & BLOCK_ALIGN_MASK_32K) >> 8 ) & 0xFF;
      tx_buf[3] = (arg & BLOCK_ALIGN_MASK_32K) & 0xFF;
      
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
    
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 4);
        
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      break;
    }
    case SPI_FLASH_REQ_Block64kErase:
    {
      tx_buf[0] = SPI_FLASH_OPCODE_WriteEnable;
          
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
          
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
      
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      tx_buf[0] = SPI_FLASH_OPCODE_Erase64kBlock;
      tx_buf[1] = ((arg & BLOCK_ALIGN_MASK_64K) >> 16) & 0xFF;
      tx_buf[2] = ((arg & BLOCK_ALIGN_MASK_64K) >> 8 ) & 0xFF;
      tx_buf[3] = (arg & BLOCK_ALIGN_MASK_64K) & 0xFF;
      
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 4);
        
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      break;
    }
    case SPI_FLASH_REQ_ReadStatus:
    {
      tx_buf[0] = SPI_FLASH_OPCODE_ReadStatus;
          
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
        
      spi_read_packet(SPI_FLASH_SPI_INSTANCE, rx_buf, 1);
        
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      *((uint8_t *)argp) = rx_buf[0];
        
      break;
    }
    case SPI_FLASH_REQ_WriteStatus:
    {
      tx_buf[0] = SPI_FLASH_OPCODE_WriteEnable;
          
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
          
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
      
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      tx_buf[0] = SPI_FLASH_OPCODE_WriteStatusByte1;
      tx_buf[1] = arg & 0xFF;
          
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
      
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 2);
        
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
      
      tx_buf[0] = SPI_FLASH_OPCODE_WriteEnable;
          
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
          
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 1);
      
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
        
      tx_buf[0] = SPI_FLASH_OPCODE_WriteStatusByte2;
      tx_buf[1] = (arg >> 8) & 0xFF;
          
      spi_select_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
      
      spi_write_packet(SPI_FLASH_SPI_INSTANCE, tx_buf, 2);
        
      spi_deselect_device(SPI_FLASH_SPI_INSTANCE, &SPI_FLASH_SPI_SLAVE);
            
      break;
    }
    default:
    {
      break;
    }
  }
  return 0;
}
#endif
