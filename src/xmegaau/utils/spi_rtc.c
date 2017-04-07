/**
 * \file spi_rtc.c
 * \author tom.m
 *
 * \see spi_rtc.h.
 */

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

#include <asf.h>
#include "spi_rtc.h"

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/
#ifndef SPI_RTC_INT_INSTANCE
#define SPI_RTC_INT_INSTANCE    (&PORTD)
#endif

#ifndef SPI_RTC_SPI_INSTANCE
#define SPI_RTC_SPI_INSTANCE    &SPIE
#endif

#ifndef SPI_RTC_SPI_BAUDRATE
#define SPI_RTC_SPI_BAUDRATE    250000
#endif

#ifndef SPI_RTC_SPI_MODE
#define SPI_RTC_SPI_MODE SPI_MODE_0
#endif

/************************************************************************/
/* Types.                                                               */
/************************************************************************/

/************************************************************************/
/* Private prototypes.                                                  */
/************************************************************************/
uint8_t SPI_RTC_Bcd2Bin (uint8_t val);
uint8_t SPI_RTC_Bin2Bcd (uint8_t val);
/************************************************************************/
/* Constants.                                                           */
/************************************************************************/

/************************************************************************/
/* Private variables.                                                   */
/************************************************************************/

struct spi_device SPI_RTC_SPI_SLAVE = {
    .id = SPIX_SS_RTC
};

/************************************************************************/
/* Macros.                                                              */
/************************************************************************/
/*
**  STATUS Register
*/
#define CB_MASK            0x80
#define BAT_MASK        0x40
#define WDT_MASK        0x20
#define BL_MASK            0x10
#define TIM_MASK        0x08
#define ALM_MASK        0x04
#define EX2_MASK        0x02
#define EX1_MASK        0x01

/*
**  CONTROL1 Register
*/
#define STOP_MASK        0x80
#define HOURS_MASK        0x40
#define OUTB_MASK        0x20
#define OUT_MASK        0x10
#define RSP_MASK        0x08
#define ARST_MASK        0x04
#define PWR2_MASK        0x02
#define WRTC_MASK        0x01

/*
**  CONTROL2 Register
*/
#define OUTPP_MASK        0x80
#define RS1E_MASK        0x20
#define OUT2S_MASK        0x1C
#define OUT1S_MASK        0x03

/*
**  INTERRUPT MASK Register
*/
#define CEB_MASK        0x80
#define IM_MASK            0x60
#define BLIE_MASK        0x10
#define TIE_MASK        0x08
#define AIE_MASK        0x04
#define EX2E_MASK        0x02
#define EX1E_MASK        0x01

/*
**  COUNTDOWN CONTROL Register
*/
#define TE_MASK            0x80
#define TM_MASK            0x40
#define TRPT_MASK        0x20
#define RPT_MASK        0x1C
#define TFS_MASK        0x03

/*
**  COUNTDOWN CONTROL Register
*/
#define TE_MASK            0x80
#define TM_MASK            0x40
#define TRPT_MASK        0x20
#define RPT_MASK        0x1C
#define TFS_MASK        0x03

/*
**  WATCHDOG CONTROL Register
*/
#define WDS_MASK            0x80
#define BMB_MASK            0x7C
#define WRB_MASK            0x03

/************************************************************************/
/* Implementations.                                                     */
/************************************************************************/

void SPI_RTC_Init(void)
{
  uint8_t data;
    
  /* Initialize configuration for spi and rtc ports */
  ioport_configure_pin(RTC_WDT_KICK, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);    
  ioport_configure_pin(SPIX_SS_RTC, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
  ioport_configure_pin(SPIE_MOSI,IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
  ioport_configure_pin(SPIE_MISO, IOPORT_DIR_INPUT);
  ioport_configure_pin(SPIE_SCK,IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
    
  PORTD.DIRCLR = (PIN1_bm | PIN2_bm | PIN3_bm);
  PORTD.PIN1CTRL = (PORT_OPC_PULLDOWN_gc | PORT_ISC_RISING_gc);
  PORTD.PIN2CTRL = (PORT_OPC_PULLDOWN_gc | PORT_ISC_RISING_gc);
  PORTD.INTCTRL = (PORTD.INTCTRL | (PORT_INT0LVL_MED_gc | PORT_INT1LVL_MED_gc));
  PORTD.INT0MASK = PIN1_bm;
  PORTD.INT1MASK = PIN2_bm;
      
  spi_master_init(SPI_RTC_SPI_INSTANCE);
  spi_master_setup_device(SPI_RTC_SPI_INSTANCE, &SPI_RTC_SPI_SLAVE, SPI_RTC_SPI_MODE, SPI_RTC_SPI_BAUDRATE, 0);
  spi_enable(SPI_RTC_SPI_INSTANCE);
  
  data = 0x84;
  SPI_RTC_Write(SPI_RTC_REG_ADDR_Control1, &data, sizeof(data));
  
  data = 0x00;
  SPI_RTC_Write(SPI_RTC_REG_ADDR_Control2, &data, sizeof(data));
  
  data = 0x80;
  SPI_RTC_Write(SPI_RTC_REG_ADDR_IntMask, &data, sizeof(data));  
  
  data = 0x04;
  SPI_RTC_Write(SPI_RTC_REG_ADDR_Control1, &data, sizeof(data));
}

int SPI_RTC_Read(const uint8_t address, void *buffer, const uint16_t data_len)
{
  uint8_t *data;
  data = (uint8_t *)buffer;
  uint8_t reg_offset;
  
  reg_offset = address & 0x7f;
  
  spi_select_device(SPI_RTC_SPI_INSTANCE, &SPI_RTC_SPI_SLAVE);
  
  spi_write_packet(SPI_RTC_SPI_INSTANCE, &reg_offset, 1);
  
  spi_read_packet(SPI_RTC_SPI_INSTANCE, data, data_len);
  
  spi_deselect_device(SPI_RTC_SPI_INSTANCE, &SPI_RTC_SPI_SLAVE);
  
  return 0;  
}

int SPI_RTC_Write(const uint8_t address, const void *buffer, const uint16_t data_len)
{
  uint8_t *data;
  data = (uint8_t *)buffer;
  uint8_t reg_offset;

  reg_offset = address | 0x80;

  spi_select_device(SPI_RTC_SPI_INSTANCE, &SPI_RTC_SPI_SLAVE);

  spi_write_packet(SPI_RTC_SPI_INSTANCE, &reg_offset, 1);

  spi_read_packet(SPI_RTC_SPI_INSTANCE, data, data_len);

  spi_deselect_device(SPI_RTC_SPI_INSTANCE, &SPI_RTC_SPI_SLAVE);

  return 0;    
}

int SPI_RTC_Ioctl(const uint8_t req, const uint32_t arg, void * argp)
{
  uint8_t data;
  
  switch((SPI_RTC_Req_t)req)
  {
    case SPI_RTC_REQ_SetDateTime:
    {    
      SPI_RTC_Read(SPI_RTC_REG_ADDR_Control1, &data, sizeof(data));
      data = data | WRTC_MASK;
      SPI_RTC_Write(SPI_RTC_REG_ADDR_Control1, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->msec);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_HundredthsDate, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->sec);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_SecondDate, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->min);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_MinuteDate, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->hour);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_Hour24Date, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->mday);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_DayDate, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->mon);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_MonthDate, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->year);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_YearDate, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->wday);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_WeekdayDate, &data, sizeof(data));
      SPI_RTC_Read(SPI_RTC_REG_ADDR_Control1, &data, sizeof(data));
      data = data & ~WRTC_MASK;
      SPI_RTC_Write(SPI_RTC_REG_ADDR_Control1, &data, sizeof(data));
      break;
    }
    case SPI_RTC_REQ_GetDateTime:
    {
      SPI_RTC_Read(SPI_RTC_REG_ADDR_HundredthsDate, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->msec = SPI_RTC_Bcd2Bin(data);
      SPI_RTC_Read(SPI_RTC_REG_ADDR_SecondDate, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->sec = SPI_RTC_Bcd2Bin(data);
      SPI_RTC_Read(SPI_RTC_REG_ADDR_MinuteDate, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->min = SPI_RTC_Bcd2Bin(data);
      SPI_RTC_Read(SPI_RTC_REG_ADDR_Hour24Date, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->hour = SPI_RTC_Bcd2Bin(data);
      SPI_RTC_Read(SPI_RTC_REG_ADDR_DayDate, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->mday = SPI_RTC_Bcd2Bin(data);
      SPI_RTC_Read(SPI_RTC_REG_ADDR_MonthDate, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->mon = SPI_RTC_Bcd2Bin(data);
      SPI_RTC_Read(SPI_RTC_REG_ADDR_YearDate, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->year = SPI_RTC_Bcd2Bin(data);
      SPI_RTC_Read(SPI_RTC_REG_ADDR_WeekdayDate, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->wday = SPI_RTC_Bcd2Bin(data);
      break;
    }
    case SPI_RTC_REQ_SetAlarmTime:
    {
      SPI_RTC_Read(SPI_RTC_REG_ADDR_Control1, &data, sizeof(data));
      data = data | WRTC_MASK;
      SPI_RTC_Write(SPI_RTC_REG_ADDR_Control1, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->msec);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_HundredthsAlarm, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->sec);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_SecondAlarm, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->min);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_MinuteAlarm, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->hour);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_Hour24Alarm, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->mday);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_DateAlarm, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->mon);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_MonthAlarm, &data, sizeof(data));
      data = SPI_RTC_Bin2Bcd(((SPI_RTC_Time_t *)(argp))->wday);
      SPI_RTC_Write(SPI_RTC_REG_ADDR_WeekdayAlarm, &data, sizeof(data));
      SPI_RTC_Read(SPI_RTC_REG_ADDR_Control1, &data, sizeof(data));
      data = data & ~WRTC_MASK;
      SPI_RTC_Write(SPI_RTC_REG_ADDR_Control1, &data, sizeof(data));
    break;
    }
    case SPI_RTC_REQ_GetAlarmTime:
    {
      SPI_RTC_Read(SPI_RTC_REG_ADDR_HundredthsAlarm, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->msec = SPI_RTC_Bcd2Bin(data);
      SPI_RTC_Read(SPI_RTC_REG_ADDR_SecondAlarm, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->sec = SPI_RTC_Bcd2Bin(data);
      SPI_RTC_Read(SPI_RTC_REG_ADDR_MinuteAlarm, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->min = SPI_RTC_Bcd2Bin(data);
      SPI_RTC_Read(SPI_RTC_REG_ADDR_Hour24Alarm, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->hour = SPI_RTC_Bcd2Bin(data);
      SPI_RTC_Read(SPI_RTC_REG_ADDR_DateAlarm, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->mday = SPI_RTC_Bcd2Bin(data);
      SPI_RTC_Read(SPI_RTC_REG_ADDR_MonthAlarm, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->mon = SPI_RTC_Bcd2Bin(data);
      SPI_RTC_Read(SPI_RTC_REG_ADDR_WeekdayAlarm, &data, sizeof(data));
      ((SPI_RTC_Time_t *)(argp))->wday = SPI_RTC_Bcd2Bin(data);
      break;
    }
    case SPI_RTC_REQ_SetAlarmIrq:
    {
      switch((SPI_RTC_IrqState_t)arg)
      {
        case SPI_RTC_IRQ_STATE_Off:
        {
          break;
        }
        case SPI_RTC_IRQ_STATE_On:
        {
          break;
        }
        default:
        {
          break;
        }
      }
      break;
    }
    case SPI_RTC_REQ_SetPeriodicIrq:
    {
      switch((SPI_RTC_IrqState_t)arg)
      {
        case SPI_RTC_IRQ_STATE_Off:
        {
          break;
        }
        case SPI_RTC_IRQ_STATE_On:
        {
          break;
        }
        default:
        {
          break;
        }
      }
      break;
    }
    case SPI_RTC_REQ_SetWatchdogIrq:
    {
      switch((SPI_RTC_IrqState_t)arg)
      {
        case SPI_RTC_IRQ_STATE_Off:
        {
          break;    
        }
        case SPI_RTC_IRQ_STATE_On:
        {
          break;
        }
        default:
        {
          break;
        }
      }
      break;
    }
    case SPI_RTC_REQ_SetBatteryIrq:
    {
      switch((SPI_RTC_IrqState_t)arg)
      {
        case SPI_RTC_IRQ_STATE_Off:
        {
          break;
        }
        case SPI_RTC_IRQ_STATE_On:
        {
          break;
        }
        default:
        {
          break;
        }
      }
      break;
    }
    default:
    {
      break;
    }
  }
  return 0;
}

uint8_t SPI_RTC_Bcd2Bin (uint8_t val)
{
  return (val & 0x0f) + (val >> 4) * 10;
}
uint8_t SPI_RTC_Bin2Bcd (uint8_t val)
{
  return ((val / 10) << 4) + val % 10;
}

ISR(PORTD_INT0_vect, ISR_NOBLOCK)
{
  irqflags_t flags;
    
  flags = cpu_irq_save();
    
  cpu_irq_restore(flags);
}

ISR(PORTD_INT1_vect, ISR_ALIASOF(PORTD_INT0_vect));