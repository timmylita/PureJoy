/**
 * \file i2c_accel.h
 * \author tom.m
 *
 * Provides functionality for interoperating with the MMA8653 3-axis, 
 * 10-bit digital accelerometer.
 */

#ifndef _I2C_ACCEL
#define _I2C_ACCEL

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

#include "twim.h"
#include "stats.h"

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

/************************************************************************/
/* Types.                                                               */
/************************************************************************/

enum
{
    I2C_ACCEL_REG_ADDR_Status = 0,    // 0x00 
    I2C_ACCEL_REG_ADDR_OutXMSB,     // 0x01
    I2C_ACCEL_REG_ADDR_OutXLSB,     // 0x02
    I2C_ACCEL_REG_ADDR_OutYMSB,     // 0x03
    I2C_ACCEL_REG_ADDR_OutYLSB,     // 0x04
    I2C_ACCEL_REG_ADDR_OutZMSB,     // 0x05
    I2C_ACCEL_REG_ADDR_OutZLSB,     // 0x06
    I2C_ACCEL_REG_ADDR_Rsvd0,       // 0x07
    I2C_ACCEL_REG_ADDR_Rsvd1,       // 0x08
    I2C_ACCEL_REG_ADDR_Rsvd2,       // 0x09
    I2C_ACCEL_REG_ADDR_Rsvd3,       // 0x0a
    I2C_ACCEL_REG_ADDR_SysMod,      // 0x0B
    I2C_ACCEL_REG_ADDR_IntSource,    // 0x0C
    I2C_ACCEL_REG_ADDR_WhoAmI,      // 0x0D
    I2C_ACCEL_REG_ADDR_XYZDataCfg,  // 0x0E
    I2C_ACCEL_REG_ADDR_Rsvd4,       // 0x0F
    I2C_ACCEL_REG_ADDR_PlStatus,    // 0x10
    I2C_ACCEL_REG_ADDR_PlCfg,       // 0x11
    I2C_ACCEL_REG_ADDR_PlCount,     // 0x12
    I2C_ACCEL_REG_ADDR_PlBfZComp,   // 0x13
    I2C_ACCEL_REG_ADDR_PlThsReg,    // 0x14
    I2C_ACCEL_REG_ADDR_FfMtCfg,     // 0x15
    I2C_ACCEL_REG_ADDR_FfMtSrc,     // 0x16
    I2C_ACCEL_REG_ADDR_FfMtThs,     // 0x17
    I2C_ACCEL_REG_ADDR_FfMtCount,   // 0x18
    I2C_ACCEL_REG_ADDR_Rsvd5,       // 0x19
    I2C_ACCEL_REG_ADDR_Rsvd6,       // 0x1A
    I2C_ACCEL_REG_ADDR_Rsvd7,       // 0x1B
    I2C_ACCEL_REG_ADDR_Rsvd8,       // 0x1C
    I2C_ACCEL_REG_ADDR_Rsvd9,       // 0x1D 
    I2C_ACCEL_REG_ADDR_Rsvd10,      // 0x1E
    I2C_ACCEL_REG_ADDR_Rsvd11,      // 0x1F
    I2C_ACCEL_REG_ADDR_Rsvd12,      // 0x20
    I2C_ACCEL_REG_ADDR_Rsvd13,      // 0x21
    I2C_ACCEL_REG_ADDR_Rsvd14,      // 0x22
    I2C_ACCEL_REG_ADDR_Rsvd15,      // 0x23
    I2C_ACCEL_REG_ADDR_Rsvd16,      // 0x24
    I2C_ACCEL_REG_ADDR_Rsvd17,      // 0x25
    I2C_ACCEL_REG_ADDR_Rsvd18,      // 0x26
    I2C_ACCEL_REG_ADDR_Rsvd19,      // 0x27
    I2C_ACCEL_REG_ADDR_Rsvd20,      // 0x28
    I2C_ACCEL_REG_ADDR_AslpCount,   // 0x29
    I2C_ACCEL_REG_ADDR_CtrlReg1,    // 0x2A
    I2C_ACCEL_REG_ADDR_CtrlReg2,    // 0x2B
    I2C_ACCEL_REG_ADDR_CtrlReg3,    // 0x2C
    I2C_ACCEL_REG_ADDR_CtrlReg4,    // 0x2D
    I2C_ACCEL_REG_ADDR_CtrlReg5,    // 0x2E
    I2C_ACCEL_REG_ADDR_OffX,        // 0x2F
    I2C_ACCEL_REG_ADDR_OffY,        // 0x30
    I2C_ACCEL_REG_ADDR_OffZ         // 0x31
}I2C_ACCEL_RegAddr_t;

/**
 * Enumerates valid request types.
 */
typedef enum {
  I2C_ACCEL_REQ_DynamicRange,
  I2C_ACCEL_REQ_AsleepDataRate,
  I2C_ACCEL_REQ_AwakeDataRate,
  I2C_ACCEL_REQ_Oversampling,
  I2C_ACCEL_REQ_IrqEnable,
  I2C_ACCEL_REQ_DetectionSettings,
  I2C_ACCEL_REQ_AutoZeroCalibration,
  I2C_ACCEL_REQ_Reset,
  I2C_ACCEL_REQ_Status,
  I2C_ACCEL_REQ_Data
}I2C_ACCEL_Req_t;

/**
 * Enumerates Dynamic Range types.
 */
typedef enum {
  I2C_ACCEL_DYNAMIC_RANGE_2GActiveMode,
  I2C_ACCEL_DYNAMIC_RANGE_4GActiveMode,
  I2C_ACCEL_DYNAMIC_RANGE_8GActiveMode
}I2C_ACCEL_DynamicRange_t;

/**
 * Enumerates Asleep Data Rate types.
 */
typedef enum {
  I2C_ACCEL_ASLEEP_DATA_RATE_50Hz,
  I2C_ACCEL_ASLEEP_DATA_RATE_12_5Hz,
  I2C_ACCEL_ASLEEP_DATA_RATE_6_25Hz,
  I2C_ACCEL_ASLEEP_DATA_RATE_1_56Hz
}I2C_ACCEL_AsleepDataRate_t;

/**
 * Enumerates Awake Data Rate types.
 */
typedef enum {
  I2C_ACCEL_AWAKE_DATA_RATE_800Hz,
  I2C_ACCEL_AWAKE_DATA_RATE_400Hz,
  I2C_ACCEL_AWAKE_DATA_RATE_200Hz,
  I2C_ACCEL_AWAKE_DATA_RATE_100Hz,
  I2C_ACCEL_AWAKE_DATA_RATE_50Hz,
  I2C_ACCEL_AWAKE_DATA_RATE_12_5Hz,
  I2C_ACCEL_AWAKE_DATA_RATE_6_25Hz,
  I2C_ACCEL_AWAKE_DATA_RATE_1_56Hz
}I2C_ACCEL_AwakeDataRate_t;

/**
 * Enumerates Oversampling Mode types.
 */
typedef enum {
  I2C_ACCEL_OVERSAMPLING_MODE_Normal,
  I2C_ACCEL_OVERSAMPLING_MODE_LowNoiseLowPower,
  I2C_ACCEL_OVERSAMPLING_MODE_HighResolution,
  I2C_ACCEL_OVERSAMPLING_MODE_LowPower
}I2C_ACCEL_OversamplingMode_t;

/**
 * Enumerates irq source flags.
 */
typedef enum {
  I2C_ACCEL_IRQ_SOURCE_DtRdy   = 0x01,
  I2C_ACCEL_IRQ_SOURCE_FfMt    = 0x04,
  I2C_ACCEL_IRQ_SOURCE_LndPrt  = 0x10,
  I2C_ACCEL_IRQ_SOURCE_AslpAwk = 0x80,
  I2C_ACCEL_IRQ_SOURCE_None = 0x00
}I2C_ACCEL_IrqSource_t;
/**
 * Enumerates irq channel.
 */
typedef enum {
  I2C_ACCEL_IRQ_CHANNEL_Int2,
  I2C_ACCEL_IRQ_CHANNEL_Int1,
}I2C_ACCEL_IrqChannel_t;

/**
 * Irq data available notifier
 */
typedef void (*I2C_ACCEL_IrqCallback_t)(void);

typedef struct
{
  uint8_t irq_sources;  
  I2C_ACCEL_IrqCallback_t irq_callback;  
}I2C_ACCEL_Irq_t;

/**
 * Enumerates event detection configurations.
 */
typedef enum {    
  I2C_ACCEL_DETECTION_SETTING_FfMt,
  I2C_ACCEL_DETECTION_SETTING_LndPrt
}I2C_ACCEL_DetectionSetting_t;

/**
 * Enumerates FfMt XYZ-axis enabling.
 */
typedef enum {
  I2C_ACCEL_FFMT_ENABLE_AXIS_X = 0x08,
  I2C_ACCEL_FFMT_ENABLE_AXIS_Y = 0x10,
  I2C_ACCEL_FFMT_ENABLE_AXIS_Z = 0x20
}I2C_ACCEL_FfMtEnableAxis_t;

/**
 * Enumerates FfMt detection flag.
 */
typedef enum {
  I2C_ACCEL_FFMT_DETECT_FLAG_Or,
  I2C_ACCEL_FFMT_DETECT_FLAG_And
}I2C_ACCEL_FfMtDetectFlag_t;

/**
 * Enumerates FfMt latching.
 */
typedef enum {
    I2C_ACCEL_FFMT_LATCH_Disable,
    I2C_ACCEL_FFMT_LATCH_Enable
}I2C_ACCEL_FfMtLatch_t;

/**
 * Enumerates FfMt debounce counter mode selection.
 */
typedef enum {
  I2C_ACCEL_DB_CNT_M_Decrement,
  I2C_ACCEL_DB_CNT_M_Clear
}I2C_ACCEL_FfMtDbCntM_t;

typedef struct
{
  I2C_ACCEL_FfMtLatch_t latch;
  I2C_ACCEL_FfMtDetectFlag_t detect_flag;
  uint8_t enable_axis;
  float threshold; 
  I2C_ACCEL_FfMtDbCntM_t debounce_counter_mode; 
  float debounce_count; 
}I2C_ACCEL_FfMt_t;

/**
 * Enumerates Status type
 */
typedef enum
{
  I2C_ACCEL_STATUS_Poll,
  I2C_ACCEL_STATUS_Irq
}I2C_ACCEL_Status_t;

/**
 * Enumerates Status type
 */
typedef enum
{
  I2C_ACCEL_STATUS_POLL_Busy,
  I2C_ACCEL_STATUS_POLL_Ready
}I2C_ACCEL_PollStatus_t;

/**
 * Enumerates top view orientation.
 */
typedef enum {
  I2C_ACCEL_TOP_VIEW_PrtUp,
  I2C_ACCEL_TOP_VIEW_PrtDown,
  I2C_ACCEL_TOP_VIEW_LndRight,
  I2C_ACCEL_TOP_VIEW_LndLeft
}I2C_ACCEL_TopView_t;

/**
 * Enumerates side view orientation.
 */
typedef enum {
  I2C_ACCEL_SIDE_VIEW_Back,
  I2C_ACCEL_SIDE_VIEW_Front
}I2C_ACCEL_SideView_t;

/**
 * Enumerates data type.
 */
typedef enum {
  I2C_ACCEL_DATA_Acceleration,
  I2C_ACCEL_DATA_Motion,
  I2C_ACCEL_DATA_Orientation
}I2C_ACCEL_Data_t;

/**
 * Enumerates motion detection flag.
 */
typedef enum {
  I2C_ACCEL_MOTION_FLAG_NotDetected,
  I2C_ACCEL_MOTION_FLAG_Deteted,
}I2C_ACCEL_MotionFlag_t;

/**
 * Enumerates motion detection polarity flag.
 */
typedef enum {
  I2C_ACCEL_MOTION_POLARITY_FLAG_Positive,
  I2C_ACCEL_MOTION_POLARITY_FLAG_Negative,
}I2C_ACCEL_MotionPolarityFlag_t;

typedef struct
{
  I2C_ACCEL_MotionFlag_t x_motion_flag;
  I2C_ACCEL_MotionPolarityFlag_t x_motion_polarity_flag;
  I2C_ACCEL_MotionFlag_t y_motion_flag;
  I2C_ACCEL_MotionPolarityFlag_t y_motion_polarity_flag;
  I2C_ACCEL_MotionFlag_t z_motion_flag;
  I2C_ACCEL_MotionPolarityFlag_t z_motion_polarity_flag;
}I2C_ACCEL_Motion_t;


typedef struct
{
  I2C_ACCEL_TopView_t top_view;
  I2C_ACCEL_SideView_t side_view;
}I2C_ACCEL_Orientation_t;


/************************************************************************/
/* Public prototypes.                                                   */
/************************************************************************/

/**
 * Initialized the i2c accelerometer interface.
 */
void I2C_ACCEL_Init
(
    void
);

/**
 * Reads data from the accelerometer chip.
 */
int I2C_ACCEL_Read
(
    const uint8_t addr,
    void *buf,
    const uint8_t buf_len
);

/**
 * Write data to the accelerometer chip.
 */
int I2C_ACCEL_Write
(
    const uint8_t addr,
    const void *buf,
    const uint8_t buf_len
);

/**
 * Performs chip-specific io requests.
 */
int I2C_ACCEL_Ioctl
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

#endif  /* _I2C_ACCEL */
