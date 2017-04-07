/**
 * \file spi_rtc.h
 * \author tom.w
 *
 * Provides functionality for interoperating with the ab18 family of
 * rtc chips. All calls are blocking.
 */

#ifndef _SPI_RTC_H
#define _SPI_RTC_H

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

#include "spi_master.h"

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
  SPI_RTC_REQ_SetDateTime, 
  SPI_RTC_REQ_GetDateTime,
  SPI_RTC_REQ_SetAlarmTime, 
  SPI_RTC_REQ_GetAlarmTime, 
  SPI_RTC_REQ_SetAlarmIrq, 
  SPI_RTC_REQ_SetPeriodicIrq,
  SPI_RTC_REQ_SetWatchdogIrq,
  SPI_RTC_REQ_SetBatteryIrq,
}SPI_RTC_Req_t;

typedef struct {
    int msec;    //Millisecond [0,999]
    int sec;    // Seconds [0,60]
    int min;    // Seconds [0,60]
    int hour;    // Hour [0,23]
    int mday;    // Day of month [1,31]
    int mon;    // Month of year [0,11]
    int year;    // Years since 1970
    int wday;
}SPI_RTC_Time_t;

/**
 * Enumerates irq state.
 */
typedef enum {
  SPI_RTC_IRQ_STATE_Off, 
  SPI_RTC_IRQ_STATE_On,
}SPI_RTC_IrqState_t;

/**
 * Enumerates valid register offset.
 */
typedef enum {
    SPI_RTC_REG_ADDR_HundredthsDate = 0x0,
    SPI_RTC_REG_ADDR_SecondDate,
    SPI_RTC_REG_ADDR_MinuteDate,
    SPI_RTC_REG_ADDR_Hour24Date,
    SPI_RTC_REG_ADDR_Hour12Date,
    SPI_RTC_REG_ADDR_DayDate,
    SPI_RTC_REG_ADDR_MonthDate,
    SPI_RTC_REG_ADDR_YearDate,
    SPI_RTC_REG_ADDR_WeekdayDate,
    SPI_RTC_REG_ADDR_HundredthsAlarm,
    SPI_RTC_REG_ADDR_SecondAlarm,
    SPI_RTC_REG_ADDR_MinuteAlarm,
    SPI_RTC_REG_ADDR_Hour24Alarm,
    SPI_RTC_REG_ADDR_Hour12Alarm,
    SPI_RTC_REG_ADDR_DateAlarm,
    SPI_RTC_REG_ADDR_MonthAlarm,
    SPI_RTC_REG_ADDR_WeekdayAlarm,
    SPI_RTC_REG_ADDR_Status,
    SPI_RTC_REG_ADDR_Control1,
    SPI_RTC_REG_ADDR_Control2,
    SPI_RTC_REG_ADDR_IntMask,
    SPI_RTC_REG_ADDR_Sqw,
    SPI_RTC_REG_ADDR_CalXT,
    SPI_RTC_REG_ADDR_CalRCHi,
    SPI_RTC_REG_ADDR_CalRCLow,
    SPI_RTC_REG_ADDR_SleepControl,
    SPI_RTC_REG_ADDR_TimerControl,
    SPI_RTC_REG_ADDR_Timer,
    SPI_RTC_REG_ADDR_TimerInitial,
    SPI_RTC_REG_ADDR_WatchdogTimer,
    SPI_RTC_REG_ADDR_OscControl,
    SPI_RTC_REG_ADDR_OscStatus,
    SPI_RTC_REG_ADDR_Reserved0,
    SPI_RTC_REG_ADDR_ConfigurationKey,    
    SPI_RTC_REG_ADDR_Trickle,
    SPI_RTC_REG_ADDR_BrefControl,
    SPI_RTC_REG_ADDR_Reserved1,
    SPI_RTC_REG_ADDR_Reserved2,
    SPI_RTC_REG_ADDR_Reserved3,
    SPI_RTC_REG_ADDR_Reserved4,
    SPI_RTC_REG_ADDR_Reserved5,
    SPI_RTC_REG_ADDR_Reserved6,
    SPI_RTC_REG_ADDR_Id0,
    SPI_RTC_REG_ADDR_Id1,
    SPI_RTC_REG_ADDR_Id2,
    SPI_RTC_REG_ADDR_Id3,
    SPI_RTC_REG_ADDR_Id4,
    SPI_RTC_REG_ADDR_Id5,
    SPI_RTC_REG_ADDR_Id6,
    SPI_RTC_REG_ADDR_AStat,
    SPI_RTC_REG_ADDR_OCtrl,
    SPI_RTC_REG_ADDR_ExtAddress,
    SPI_RTC_REG_ADDR_Ram,
}SPI_RTC_RegAddr_t;

/************************************************************************/
/* Public prototypes.                                                   */
/************************************************************************/

/**
 * Date Time Related Functions
 */

void SPI_RTC_Init(void);
int SPI_RTC_Read(const uint8_t address, void *buffer, const uint16_t data_len);
int SPI_RTC_Write(const uint8_t address, const void *buffer, const uint16_t data_len);
int SPI_RTC_Ioctl(const uint8_t req, const uint32_t arg, void * argp);

/************************************************************************/
/* Public variables.                                                    */
/************************************************************************/

/************************************************************************/
/* Public Macros.                                                       */
/************************************************************************/

#endif  /* _SPI_RTC_H */
