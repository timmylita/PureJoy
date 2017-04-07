/**
* \file bootloader.c
* \author tom.m
*
* \see bootloader.h.
*/

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

#include "project.h"
#include "bootloader.h"
#include "spi_flash.h"
#include "srec.h"

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

#ifndef BOOTLOADER_ForDstToZero
#define BOOTLOADER_ForDstToZero                                       (1)
#endif

#ifndef BOOTLOADER_Verify
#define BOOTLOADER_Verify                                             (1)
#endif

#ifndef BOOTLOADER_ShowSrec
#define BOOTLOADER_ShowSrec                                           (0)
#endif

#ifndef BOOTLOADER_Leds
#define BOOTLOADER_Leds                                               (1)
#endif

#ifndef BOOTLOADER_IntFlashWrite_Enable
#define BOOTLOADER_IntFlashWrite_Enable                               (1)
#endif

#if (DIAGNOSTICS)
#define USART_SERIAL                                           (&USARTF0)
#define USART_SERIAL_BAUDRATE                                    (921600)
#define USART_SERIAL_CHAR_LENGTH                   (USART_CHSIZE_8BIT_gc)
#define USART_SERIAL_PARITY                     (USART_PMODE_DISABLED_gc)
#define USART_SERIAL_STOP_BIT                                     (false)
#endif

#if (128 == TARGET)
#define BOOTLOADER_BootSrecLength                              (0x2000UL)
#endif

#if (256 == TARGET)
#define BOOTLOADER_BootSrecLength                              (0x4000UL)
#endif

/************************************************************************/
/* Types.                                                               */
/************************************************************************/

/**
* Specifies the valid BootStatus values.
*/
typedef enum
{
    BOOTLOADER_BootStatus_Execute   = 0x00,
    BOOTLOADER_BootStatus_Load      = 0xaa,
    BOOTLOADER_BootStatus_Corrupt   = 0x55,
    BOOTLOADER_BootStatus_New       = 0xff
} BOOTLOADER_BootStatus_t;


/**
* Specifies the valid device types.
*/
typedef enum
{
    BOOTLOADER_DeviceType_Null = 0x00,
    BOOTLOADER_DeviceType_Gatway,
    BOOTLOADER_DeviceType_Sensor,
    BOOTLOADER_DeviceType_Tag,
    BOOTLOADER_DeviceType_Gatway_2,
    BOOTLOADER_DeviceType_Tag_2,
    BOOTLOADER_DeviceType_Count,
} BOOTLOADER_DeviceType_t;

typedef enum
{
    BOOTLOADER_ConfigParameters_Status          = 107,
    BOOTLOADER_ConfigParameters_AppAddress      = 108,
    BOOTLOADER_ConfigParameters_SrecAddress_G   = 112,
    BOOTLOADER_ConfigParameters_SrecAddress_S   = 116,
    BOOTLOADER_ConfigParameters_SrecAddress_T   = 120,
    BOOTLOADER_ConfigParameters_SrecRevert      = 221,
    BOOTLOADER_ConfigParameters_DeviceType      = 124
} BOOTLOADER_ConfigParameters_t;

typedef enum
{
    BOOTLOADER_S0 = 0,
    BOOTLOADER_S1 = 1,
    BOOTLOADER_S2 = 2,
    BOOTLOADER_S3 = 3,
    BOOTLOADER_S5 = 5,
    BOOTLOADER_S7 = 7,
    BOOTLOADER_S8 = 8,
    BOOTLOADER_S9 = 9
} SrecType_t;

typedef enum
{
    BOOTLOADER_NONE,
    BOOTLOADER_MINIMAL_INIT,
    BOOTLOADER_RESET,
    BOOTLOADER_INIT,
    BOOTLOADER_START,
} BootloaderState_t;


typedef struct
{
    uint8_t type;
    uint8_t count;
    uint32_t address;
    uint8_t data[SREC_BootSrecBlockSize];
    uint8_t checksum;
}SrecFormat_t;

typedef enum
{
    BOOTLOADER_ERROR_None,
    BOOTLOADER_ERROR_BadChecksum,
    BOOTLOADER_ERROR_BadDeviceType,
    BOOTLOADER_ERROR_BadCharacter,
    BOOTLOADER_ERROR_BadLength,
    BOOTLOADER_ERROR_BadType,
    BOOTLOADER_ERROR_BadCount,
    BOOTLOADER_ERROR_BadJ,
    BOOTLOADER_ERROR_BadI
} BOOTLOADER_Error_t;

/************************************************************************/
/* Private prototypes.                                                  */
/************************************************************************/

void BOOTLOAD_Run (void);
static void BOOTLOADER_LoadSrec(void);
#if (DIAGNOSTICS)
static int usart_tx (char data, FILE *stream);
static int usart_rx (FILE *stream);
#endif
static void BOOTLOADER_Exit (void);
static void BOOTLOADER_ResolveDeviceType (void);

/************************************************************************/
/* Constants.                                                           */
/************************************************************************/

/************************************************************************/
/* Private variables.                                                   */
/************************************************************************/

static BOOTLOADER_BootStatus_t _status = 0x00;
static BootloaderState_t _state = BOOTLOADER_MINIMAL_INIT;
static SrecFormat_t _srec_format;
static uint8_t _srec_buf[SREC_BootSrecBlockSize];
static BOOTLOADER_DeviceType_t _dev_type = BOOTLOADER_DeviceType_Null;
static uint32_t _src_address = 0x0;
static uint32_t _dst_address = 0x0;
static BOOTLOADER_Error_t _error;
#if (BOOTLOADER_Leds)
static ioport_pin_t _led_green;
static ioport_pin_t _led_red;
#endif
#if (BOOTLOADER_Verify)
static uint16_t _line_reader;
#endif
#if (DIAGNOSTICS)
static FILE usart_stdio =
FDEV_SETUP_STREAM (usart_tx, usart_rx, _FDEV_SETUP_RW);
#endif
static char _type_strings[6][BOOTLOADER_DeviceType_Count] =
    {
        "??",
        "g1",
        "s1",
        "t1",
        "g2",
        "j2"
    };
static uint8_t
    _reset_reason;

/************************************************************************/
/* Macros.                                                              */
/************************************************************************/

#define LED_GREEN_ON                                                    \
    ioport_configure_pin(_led_green, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH)

#define LED_GREEN_OFF                                                   \
    ioport_configure_pin(_led_green, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW)

#define LED_RED_ON                                                      \
    ioport_configure_pin(_led_red, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH)

#define LED_RED_OFF                                                     \
    ioport_configure_pin(_led_red, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW)

/************************************************************************/
/* Implementations.                                                     */
/************************************************************************/

/*----------------------------------------------------------------------*/
void BOOTLOADER_Run(void)
{
    switch(_state)
    {
        case BOOTLOADER_MINIMAL_INIT:
            sysclk_init();
            nvm_eeprom_read_buffer
            (
                BOOTLOADER_ConfigParameters_Status,
                &_status,
                1
            );
            BOOTLOADER_ResolveDeviceType();
            #if (DIAGNOSTICS)
            stdout = &usart_stdio;
            stdin = &usart_stdio;
            static usart_rs232_options_t usart_options =
            {
                .baudrate = USART_SERIAL_BAUDRATE,
                .charlength = USART_SERIAL_CHAR_LENGTH,
                .paritytype = USART_SERIAL_PARITY,
                .stopbits = USART_SERIAL_STOP_BIT
            };
            usart_serial_init(USART_SERIAL, &usart_options);
            printf
            (
                "#boot:v2.0 (%s) [%u]\r\n",
                _type_strings[_dev_type],
                _status
            );
            #endif
            /*
            The bootloader resets itself with WDT, and any WDT reset during
            application execution is certainly not one from which running
            the bootloader was intended. Therefore, just jump to the app
            if WDT reset is flagged.
            */
            _reset_reason = reset_cause_get_causes();
            if (_reset_reason & RESET_CAUSE_WDT)
            {
                /* Jump to application. */
                void(* target)(void) __attribute__ ((noreturn)) =
                (void(*)(void))    ((uint32_t)(0));
                EIND = 0;
                cpu_irq_disable();
                target();
            }
            nvm_eeprom_erase_and_write_buffer
            (
                _reset_reason_address,
                &_reset_reason,
                1
            );
            _state = BOOTLOADER_INIT;
            break;
        
        case BOOTLOADER_INIT:
            /* System setup. */
            SPI_FLASH_Init();
            pmic_init();
            irq_initialize_vectors();
            cpu_irq_enable();
            _error = BOOTLOADER_ERROR_None;
            memset(_srec_buf, 0, sizeof(_srec_buf));
            #if (BOOTLOADER_ForDstToZero)
            _dst_address = 0;
            #else
            nvm_eeprom_read_buffer
            (
                BOOTLOADER_ConfigParameters_AppAddress,
                &_dst_address,
                sizeof(uint32_t)
            );
            #endif
            _state = BOOTLOADER_START;
            break;
        
        case BOOTLOADER_START:
        {
            switch(_status)
            {
                case BOOTLOADER_BootStatus_Load:
                {
                    #if (BOOTLOADER_Verify)
                    for
                    (
                        _line_reader = 0;
                        _line_reader < BOOTLOADER_BootSrecLength;
                        _line_reader++
                    )
                    {
                        #if (BOOTLOADER_Leds)
                        if(_line_reader & 0x40)
                        {
                            LED_GREEN_ON;
                            LED_RED_OFF;
                        }
                        else
                        {
                            LED_GREEN_OFF;
                            LED_RED_ON;
                        }
                        #endif
                        memset(_srec_buf, 0, SREC_BootSrecBlockSize);
                        SPI_FLASH_Read
                        (
                            (
                                _src_address
                                +
                                (_line_reader*SREC_BootSrecBlockSize)
                            ),
                            _srec_buf,
                            SREC_BootSrecBlockSize
                        );
                        // Perform crc check...
                        if
                        (
                            (_error = SREC_ParseLine(_srec_buf))
                            !=
                            BOOTLOADER_ERROR_None
                        )
                        {
                            #if (DIAGNOSTICS)
                            printf
                            (
                                "#err:%u (%u) [%s]\r\n",
                                _error,
                                _line_reader,
                                _srec_buf
                            );
                            printf
                            (
                                "#srec:addr=%lu,"
                                "cnt=0x%02x,"
                                "type=0x%02x,"
                                "chk=0x%02x\r\n",
                                _srec_format.address,
                                _srec_format.count,
                                _srec_format.type,
                                _srec_format.checksum
                            );
                            #endif
                            _status = BOOTLOADER_BootStatus_Corrupt;
                            nvm_eeprom_erase_and_write_buffer
                            (
                                BOOTLOADER_ConfigParameters_Status,
                                &_status,
                                1
                            );
                            BOOTLOADER_Exit();
                        }
                        if
                        (
                            (_srec_format.type >= BOOTLOADER_S7)
                            &&
                            (_srec_format.type <= BOOTLOADER_S9)
                        )
                        {
                            break;
                        }
                        #if (BOOTLOADER_ShowSrec)
                        #if (DIAGNOSTICS)
                        printf("#ok: (%lu) [%s]\r\n", _line_reader, _srec_buf);
                        #endif
                        #endif
                    }
                    #endif
                    BOOTLOADER_LoadSrec();
                    break;
                }
                case BOOTLOADER_BootStatus_New:
                case BOOTLOADER_BootStatus_Execute:
                {
                    BOOTLOADER_Exit();
                    break;
                }
                default:
                {
                    while(1)
                    {
                    }
                    break;
                }
            }
            break;
        }

        default:
        {
            #if (BOOTLOADER_Leds)
            LED_RED_ON;
            #endif
            while(1)
            {
            }
            break;
        }
    }
}

/*----------------------------------------------------------------------*/
static void BOOTLOADER_ResolveDeviceType (void)
{
    nvm_eeprom_read_buffer
    (
        BOOTLOADER_ConfigParameters_DeviceType,
        &_dev_type,
        1
    );
    switch(_dev_type)
    {
        case BOOTLOADER_DeviceType_Gatway:
            nvm_eeprom_read_buffer
            (
                BOOTLOADER_ConfigParameters_SrecAddress_G,
                &_src_address,
                sizeof(uint32_t)
            );
            ioport_configure_pin
            (
                G1_LED_RED, IOPORT_DIR_OUTPUT
                |
                IOPORT_INIT_LOW
            );
            ioport_configure_pin(G1_LED_GREEN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
            ioport_configure_pin(G1_XMEGA_SPIX_SS_ETH, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
            _led_red = G1_LED_RED;
            _led_green = G1_LED_GREEN;
            #if (DIAGNOSTICS)
            ioport_set_pin_dir(G1_USART_RX, IOPORT_DIR_INPUT);
            ioport_set_pin_dir(G1_USART_TX, IOPORT_DIR_OUTPUT);
            #endif
            break;
            
        case BOOTLOADER_DeviceType_Gatway_2:
            nvm_eeprom_read_buffer
            (
                BOOTLOADER_ConfigParameters_SrecAddress_G,
                &_src_address,
                sizeof(uint32_t)
            );
            ioport_configure_pin(G2_XMEGA_SPIX_SS_ETH, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
            _led_red = G2_LED_RED;
            _led_green = G2_LED_GREEN;
            #if (DIAGNOSTICS)
            ioport_set_pin_dir(G2_USART_RX, IOPORT_DIR_INPUT);
            ioport_set_pin_dir(G2_USART_TX, IOPORT_DIR_OUTPUT);
            #endif
            break;
        
        case BOOTLOADER_DeviceType_Sensor:
            nvm_eeprom_read_buffer
            (
                BOOTLOADER_ConfigParameters_SrecAddress_S,
                &_src_address,
                sizeof(uint32_t)
            );
            ioport_configure_pin(S1_XMEGA_SPIX_SS_ETH, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
            _led_red = S1_LED_RED;
            _led_green = S1_LED_GREEN;
            #if (DIAGNOSTICS)
            ioport_set_pin_dir(S1_USART_RX, IOPORT_DIR_INPUT);
            ioport_set_pin_dir(S1_USART_TX, IOPORT_DIR_OUTPUT);
            #endif
            break;
        
        case BOOTLOADER_DeviceType_Tag:
            nvm_eeprom_read_buffer
            (
                BOOTLOADER_ConfigParameters_SrecAddress_T,
                &_src_address,
                sizeof(uint32_t)
            );
            _led_red = J1_LED_RED;
            _led_green = J1_LED_GREEN;
            ioport_configure_pin(J1_SPIX_SS_RTC, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
            ioport_configure_pin(J1_SPIX_SS_26K80, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
            ioport_configure_pin(J2_XMEGA_PIC_RESET, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
            #if (DIAGNOSTICS)
            ioport_set_pin_dir(S1_USART_RX, IOPORT_DIR_INPUT);
            ioport_set_pin_dir(S1_USART_TX, IOPORT_DIR_OUTPUT);
            #endif
            break;
            
        case BOOTLOADER_DeviceType_Tag_2:
            nvm_eeprom_read_buffer
            (
                BOOTLOADER_ConfigParameters_SrecAddress_T,
                &_src_address,
                sizeof(uint32_t)
            );
            _led_red = J2_LED_RED;
            _led_green = J2_LED_GREEN;
            ioport_configure_pin(J2_SPIX_SS_26K80, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
            ioport_configure_pin(J2_XMEGA_PIC_RESET, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
            #if (DIAGNOSTICS)
            ioport_set_pin_dir(J2_USART_RX, IOPORT_DIR_INPUT);
            ioport_set_pin_dir(J2_USART_TX, IOPORT_DIR_OUTPUT);
            #endif
            break;
        
        default:
            _dev_type = BOOTLOADER_DeviceType_Null;
            _error = BOOTLOADER_ERROR_BadDeviceType;
            #if (DIAGNOSTICS)
            ioport_set_pin_dir(J1_USART_RX, IOPORT_DIR_INPUT);
            ioport_set_pin_dir(J1_USART_TX, IOPORT_DIR_OUTPUT);
            #endif
            break;
    }
    LED_GREEN_ON;
    LED_RED_OFF;
}

/*----------------------------------------------------------------------*/
static void BOOTLOADER_LoadSrec(void)
{
    uint32_t i;
    for(i = 0; i < BOOTLOADER_BootSrecLength; i++)
    {
        #if (BOOTLOADER_Leds)
        if(i & 0x40)
        {
            LED_RED_ON;
        }
        else
        {
            LED_RED_OFF;
        }
        #endif
        
        SPI_FLASH_Read(_src_address+(i*SREC_BootSrecBlockSize), _srec_buf, SREC_BootSrecBlockSize);
        
        #if (DIAGNOSTICS > 1)
        printf("#load: (%lu) [%s]\r\n", i, _srec_buf);
        #endif
        
        if((_error = SREC_ParseLine(_srec_buf)) != BOOTLOADER_ERROR_None)
        {
            #if (DIAGNOSTICS)
            printf("#err:%u(%lu)[%s]\r\n", _error, i, _srec_buf);
            printf("#srec:addr=%lu,cnt=0x%02x,type=0x%02x,chk=0x%02x\r\n", _srec_format.address, _srec_format.count, _srec_format.type, _srec_format.checksum);
            #endif
            while(1)
            {
            }
        }
        else
        {
            switch(_srec_format.type)
            {
                case BOOTLOADER_S0: // header record
                case BOOTLOADER_S5: // record count
                {
                    break;
                }
                case BOOTLOADER_S1: // data record
                case BOOTLOADER_S2:
                case BOOTLOADER_S3:
                {
                    nvm_flash_erase_and_write_buffer(_srec_format.address, _srec_format.data, _srec_format.count - (SREC_AddressLen(_srec_format.type)+1), true);
                    break;
                }
                case BOOTLOADER_S7: // termination record
                case BOOTLOADER_S8:
                case BOOTLOADER_S9:
                {
                    _dst_address = _srec_format.address;
                    BOOTLOADER_Exit();
                }
                default:
                {
                    break;
                }
            }
        }
    }
}

/*----------------------------------------------------------------------*/
static void BOOTLOADER_Exit (void)
{
    BOOT_Reason_t reboot_reason = BOOT_Reason_Boot;
    nvm_eeprom_erase_and_write_buffer
    (
        _reboot_reason_address,
        &reboot_reason,
        1
    );
    printf("#exe\r\n");
    wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_8CLK);
    wdt_enable();
    while(1)
    {
    }
}

/*----------------------------------------------------------------------*/
#if (DIAGNOSTICS)
static int usart_tx (char data, FILE *stream)
{
    usart_serial_putchar(USART_SERIAL, data);
    return (0);
}
#endif

/*----------------------------------------------------------------------*/
#if (DIAGNOSTICS)
static int usart_rx (FILE *stream)
{
    uint8_t data;
    usart_serial_getchar(USART_SERIAL, &data);
    return (data);
}
#endif
