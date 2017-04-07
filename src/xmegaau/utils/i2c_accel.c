/**
 * \file i2c_accel.c
 * \author tom.m
 *
 * \see i2c_accel.h.
 */

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

#include "project.h"
#include "i2c_accel.h"

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

/*
**  STATUS Register
*/
#define ZYXOW_MASK            0x80
#define ZOW_MASK              0x40
#define YOW_MASK              0x20
#define XOW_MASK              0x10
#define ZYXDR_MASK            0x08
#define ZDR_MASK              0x04
#define YDR_MASK              0x02
#define XDR_MASK              0x01

/*
**  XYZ Data Registers
*/
#define OUT_X_MSB_REG         0x01
#define OUT_X_LSB_REG         0x02
#define OUT_Y_MSB_REG         0x03
#define OUT_Y_LSB_REG         0x04
#define OUT_Z_MSB_REG         0x05
#define OUT_Z_LSB_REG         0x06

/*
**  SYSMOD System Mode Register
*/
#define SYSMOD0_MASK          0x01
#define SYSMOD1_MASK          0x02
#define SYSMOD_MASK           0x03
//
#define SYSMOD_STANDBY        0x00
#define SYSMOD_WAKE           (SYSMOD0_MASK)
#define SYSMOD_SLEEP          (SYSMOD1_MASK)

/*
**  INT_SOURCE System Interrupt Status Register
*/
#define SRC_ASLP_MASK         0x80
#define SRC_LNDPRT_MASK       0x10
#define SRC_FF_MT_MASK        0x04
#define SRC_DRDY_MASK         0x01

/*
**  WHO_AM_I Device ID Register
*/
#define MMA8652Q              0x4A
#define MMA8653Q              0x5A

/*
**  XYZ_DATA_CFG Sensor Data Configuration Register
*/
#define FS0_MASK              0x01
#define FS1_MASK              0x02
#define FS_MASK               0x03

#define FULL_SCALE_2G         0x00
#define FULL_SCALE_4G         (FS0_MASK)
#define FULL_SCALE_8G         (FS1_MASK)

/*
**  PL_STATUS Portrait/Landscape Status Register
*/
#define NEWLP_MASK            0x80
#define LO_MASK               0x40
#define LAPO1_MASK            0x04
#define LAPO0_MASK            0x02
#define BAFRO_MASK            0x01
#define LAPO_MASK             0x06

/*
**  PL_CFG Portrait/Landscape Configuration Register
*/
#define DBCNTM_MASK           0x80
#define PL_EN_MASK            0x40

/*
**  PL_BF_ZCOMP Back/Front and Z Compensation Register
*/
#define BKFR1_MASK            0x80
#define BKFR0_MASK            0x40
#define ZLOCK2_MASK           0x04
#define ZLOCK1_MASK           0x02
#define ZLOCK0_MASK           0x01
#define BKFR_MASK             0xC0
#define ZLOCK_MASK            0x07

/*
**  PL_P_L_THS Portrait to Landscape Threshold Register
*/
#define P_L_THS4_MASK         0x80
#define P_L_THS3_MASK         0x40
#define P_L_THS2_MASK         0x20
#define P_L_THS1_MASK         0x10
#define P_L_THS0_MASK         0x08
#define HYS2_MASK             0x04
#define HYS1_MASK             0x02
#define HYS0_MASK             0x01
#define P_L_THS_MASK          0xF8
#define HYS_MASK              0x07

/*
**  FF_MT_CFG Freefall and Motion Configuration Register
*/
#define ELE_MASK              0x80
#define OAE_MASK              0x40
#define ZEFE_MASK             0x20
#define YEFE_MASK             0x10
#define XEFE_MASK             0x08

/*
**  FF_MT_SRC Freefall and Motion Source Registers
*/
#define EA_MASK               0x80
#define ZHE_MASK              0x20
#define ZHP_MASK              0x10
#define YHE_MASK              0x08
#define YHP_MASK              0x04
#define XHE_MASK              0x02
#define XHP_MASK              0x01

/*
**  FF_MT_THS Freefall and Motion Threshold Registers
**  TRANSIENT_THS Transient Threshold Register
*/
#define DBCNTM_MASK           0x80
#define THS6_MASK             0x40
#define THS5_MASK             0x20
#define THS4_MASK             0x10
#define THS3_MASK             0x08
#define THS2_MASK             0x04
#define THS1_MASK             0x02
#define THS0_MASK             0x01
#define THS_MASK              0x7F

/*
**  CTRL_REG1 System Control 1 Register
*/
#define ASLP_RATE1_MASK       0x80
#define ASLP_RATE0_MASK       0x40
#define DR2_MASK              0x20
#define DR1_MASK              0x10
#define DR0_MASK              0x08
#define LNOISE_MASK           0x04
#define FREAD_MASK            0x02
#define ACTIVE_MASK           0x01
#define ASLP_RATE_MASK        0xC0
#define DR_MASK               0x38

#define ASLP_RATE_20MS        0x00
#define ASLP_RATE_80MS        (ASLP_RATE0_MASK)
#define ASLP_RATE_160MS       (ASLP_RATE1_MASK)
#define ASLP_RATE_640MS       (ASLP_RATE1_MASK+ASLP_RATE0_MASK)

#define ASLP_RATE_50HZ        (ASLP_RATE_20MS)
#define ASLP_RATE_12_5HZ      (ASLP_RATE_80MS)
#define ASLP_RATE_6_25HZ      (ASLP_RATE_160MS)
#define ASLP_RATE_1_56HZ      (ASLP_RATE_640MS)

#define DATA_RATE_1250US      0x00
#define DATA_RATE_2500US      (DR0_MASK)
#define DATA_RATE_5MS         (DR1_MASK)
#define DATA_RATE_10MS        (DR1_MASK+DR0_MASK)
#define DATA_RATE_20MS        (DR2_MASK)
#define DATA_RATE_80MS        (DR2_MASK+DR0_MASK)
#define DATA_RATE_160MS       (DR2_MASK+DR1_MASK)
#define DATA_RATE_640MS       (DR2_MASK+DR1_MASK+DR0_MASK)

#define DATA_RATE_800HZ       (DATA_RATE_1250US)
#define DATA_RATE_400HZ       (DATA_RATE_2500US)
#define DATA_RATE_200HZ       (DATA_RATE_5MS)
#define DATA_RATE_100HZ       (DATA_RATE_10MS)
#define DATA_RATE_50HZ        (DATA_RATE_20MS)
#define DATA_RATE_12_5HZ      (DATA_RATE_80MS)
#define DATA_RATE_6_25HZ      (DATA_RATE_160MS)
#define DATA_RATE_1_56HZ      (DATA_RATE_640MS)

#define ACTIVE                (ACTIVE_MASK)
#define STANDBY               0x00

/*
**  CTRL_REG2 System Control 2 Register
*/
#define ST_MASK               0x80
#define RST_MASK              0x40
#define SMODS1_MASK           0x10
#define SMODS0_MASK           0x08
#define SLPE_MASK             0x04
#define MODS1_MASK            0x02
#define MODS0_MASK            0x01
#define SMODS_MASK            0x18
#define MODS_MASK             0x03

#define SMOD_NORMAL           0x00
#define SMOD_LOW_NOISE        (SMODS0_MASK)
#define SMOD_HIGH_RES         (SMODS1_MASK)
#define SMOD_LOW_POWER        (SMODS1_MASK+SMODS0_MASK)

#define MOD_NORMAL            0x00
#define MOD_LOW_NOISE         (MODS0_MASK)
#define MOD_HIGH_RES          (MODS1_MASK)
#define MOD_LOW_POWER         (MODS1_MASK+MODS0_MASK)

/*
**  CTRL_REG3 Interrupt Control Register
*/
#define WAKE_LNDPRT_MASK      0x20
#define WAKE_FF_MT_MASK       0x08
#define IPOL_MASK             0x02
#define PP_OD_MASK            0x01

/*
**  CTRL_REG4 Interrupt Enable Register
*/
#define INT_EN_ASLP_MASK      0x80
#define INT_EN_LNDPRT_MASK    0x10
#define INT_EN_FF_MT_MASK     0x04
#define INT_EN_DRDY_MASK      0x01

/*
**  CTRL_REG5 Interrupt Configuration Register
*/
#define INT_CFG_ASLP_MASK     0x80
#define INT_CFG_LNDPRT_MASK   0x10
#define INT_CFG_FF_MT_MASK    0x04
#define INT_CFG_DRDY_MASK     0x01

/*
**  values for calculation
*/
#define SHIFT_2G        8
#define INCR_2G            128
#define SHIFT_4G        7
#define INCR_4G            4
#define SHIFT_8G        6
#define INCR_8G            32
#define DYN_RANGE_2G    2
#define DYN_RANGE_4G    4
#define DYN_RANGE_8G    8


#define AWAKE_DATA_RATE_COUNT 8

#define OVERSAMPLING_MODE_COUNT 4

#define I2C_ACCEL_DYNAMIC_RANGE_COUNT 3


/************************************************************************/
/* Types.                                                               */
/************************************************************************/

typedef struct
{
  float min_range;
  float max_range;
  float step;
}I2C_ACCEL_Resolution_t;

/************************************************************************/
/* Private prototypes.                                                  */
/************************************************************************/

/************************************************************************/
/* Constants.                                                           */
/************************************************************************/

/**
 * Freefall/Motion mininum and maximum time range (ms) and time step (ms) for debouncing counter in relation to ODR
 */
static const I2C_ACCEL_Resolution_t I2C_ACCEL_FfMtCount[AWAKE_DATA_RATE_COUNT][OVERSAMPLING_MODE_COUNT] = 
{
  {{0.0, 319.0, 1.25}, {0.0, 319.0, 1.25}, {0.0, 319.0, 1.25}, {0.0, 319.0, 1.25}},
  {{0.0, 638.0, 2.5}, {0.0, 638.0, 2.5}, {0.0, 638.0, 2.5}, {0.0, 638.0, 2.5}},
  {{0.0, 1280.0, 5.0}, {0.0, 1280.0, 5.0}, {0.0, 638.0, 2.5}, {0.0, 1280.0, 5.0}},
  {{0.0, 2550.0, 10.0}, {0.0, 2550.0, 10.0}, {0.0, 638.0, 2.5}, {0.0, 2550.0, 10.0}},
  {{0.0, 5100.0, 20.0}, {0.0, 5100.0, 20.0}, {0.0, 638.0, 2.5}, {0.0, 5100.0, 20.0}},
  {{0.0, 5100.0, 20.0}, {0.0, 20400.0, 80.0}, {0.0, 638.0, 2.5}, {0.0, 20400.0, 80.0}},
  {{0.0, 5100.0, 20.0}, {0.0, 20400.0, 80.0}, {0.0, 638.0, 2.5}, {0.0, 40800.0, 160.0}},
  {{0.0, 5100.0, 20.0}, {0.0, 20400.0, 80.0}, {0.0, 638.0, 2.5}, {0.0, 40800.0, 160.0}}
};

/**
 * The threshold resolution is 0.063 g/LSB and the threshold register has a range of 0 to 127 counts. 
 * The maximum range is to ±8 g.
 */
static const I2C_ACCEL_Resolution_t I2C_ACCEL_FfMtThs = {-8.0, 8.0, 0.063};

/**
 * Accelerometer 10-bit output data minimum and maximum g range (mg) and g steping (mg) in relation to dynmaic range
 */
static const I2C_ACCEL_Resolution_t I2C_ACCEL_OutputData[I2C_ACCEL_DYNAMIC_RANGE_COUNT] = 
{
  {-2000.0, 1996.0, 3.9},
  {-4000.0, 3992.0, 7.8},
  {-8000.0, 7984.0, 15.6}
};


/************************************************************************/
/* Private variables.                                                   */
/************************************************************************/

static I2C_ACCEL_IrqCallback_t
    _irq_handler[2] =
    {0, 0};
static STATS_VectorCoordinate_t
    _accel_data =
    {0.0, 0.0, 0.0};
static I2C_ACCEL_Motion_t
    _mt_data = 
    {
        .x_motion_flag = I2C_ACCEL_MOTION_FLAG_NotDetected, 
        .x_motion_polarity_flag = I2C_ACCEL_MOTION_POLARITY_FLAG_Positive, 
        .y_motion_flag = I2C_ACCEL_MOTION_FLAG_NotDetected, 
        .y_motion_polarity_flag = I2C_ACCEL_MOTION_POLARITY_FLAG_Positive, 
        .z_motion_flag = I2C_ACCEL_MOTION_FLAG_NotDetected, 
        .z_motion_polarity_flag = I2C_ACCEL_MOTION_POLARITY_FLAG_Positive
    };
static twi_package_t
    _packet = 
    {
      .chip = 0x1D,
      .no_wait = false
    };
static twi_master_options_t
    _opt =
    {
      .speed = 400000,
      .chip  = 0x1D
    };
static I2C_ACCEL_AwakeDataRate_t
    _data_rate;
static I2C_ACCEL_OversamplingMode_t
    _oversampling_mode;
static uint8_t
    _data[6];
static I2C_ACCEL_DynamicRange_t
    _dynamic_range;

/************************************************************************/
/* Macros.                                                              */
/************************************************************************/

/************************************************************************/
/* Implementations.                                                     */
/************************************************************************/

/*----------------------------------------------------------------------*/
void I2C_ACCEL_Init(void)
{
    uint8_t data;
    uint32_t count = 0;

    ioport_configure_pin
    (
        ACCEL_SDA,
        IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH
    );
    ioport_configure_pin
    (
        ACCEL_SCL,
        IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH
    );
    ioport_configure_pin
    (
        ACCEL_INT_1,
        IOPORT_DIR_INPUT
    );
    ioport_configure_pin
    (
        ACCEL_INT_2,
        IOPORT_DIR_INPUT
    );
  
    PORTE.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
    PORTE.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc;
    PORTE.DIRCLR = (PORTE.DIRCLR | PIN2_bm | PIN3_bm);
    PORTE.PIN2CTRL = (PORT_OPC_PULLDOWN_gc | PORT_ISC_RISING_gc);
    PORTE.PIN3CTRL = (PORT_OPC_PULLDOWN_gc | PORT_ISC_RISING_gc);
    PORTE.INTCTRL = (PORTE.INTCTRL | PORT_INT0LVL_MED_gc | PORT_INT1LVL_MED_gc); 
    PORTE.INT0MASK = (PORTE.INT0MASK | PIN2_bm);
    PORTE.INT1MASK = (PORTE.INT1MASK | PIN3_bm);
  
    twi_master_setup(&TWIE, &_opt);
    twi_master_enable(&TWIE);
    data = RST_MASK;
    I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg2, &data, 1);

    do
    {
    I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg2, &data, 1);
    } while((data & RST_MASK) && (++count <= 0x7FFFFFFF));
    
    I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg1, &data, 1);
    data = (data & ~FREAD_MASK);
    I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg1, &data, 1);
    data = PP_OD_MASK | IPOL_MASK | WAKE_FF_MT_MASK | WAKE_LNDPRT_MASK;
    I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg3, &data, 1);
}

/*----------------------------------------------------------------------*/
int I2C_ACCEL_Read(const uint8_t addr, void *buf, const uint8_t buf_len)
{ 
  _packet.addr[0] = addr;
  _packet.addr_length = 1;
  _packet.buffer = buf;
  _packet.length = buf_len;
  return (twi_master_read(&TWIE, &_packet) == TWI_SUCCESS)? 0:-1;
}

/*----------------------------------------------------------------------*/
int I2C_ACCEL_Write
(
    const uint8_t addr,
    const void *buf,
    const uint8_t buf_len
)
{
  uint8_t data = 0;
  
  _packet.addr[0] = I2C_ACCEL_REG_ADDR_CtrlReg1;
  _packet.addr_length = 1;
  _packet.buffer = &data;
  _packet.length = 1;
  if(twi_master_read(&TWIE, &_packet) != TWI_SUCCESS)
  {
    return -1;
  }
  
  data = data & ~ACTIVE_MASK;
  _packet.buffer = &data;
  if(twi_master_write(&TWIE, &_packet) != TWI_SUCCESS)
  {
    return -1;
  }    

  _packet.addr[0] = addr;
  _packet.addr_length = 1;
  _packet.buffer = (void *)buf;
  _packet.length = buf_len; 
  
  if(twi_master_write(&TWIE, &_packet) != TWI_SUCCESS)
  {
    return -1;
  }
  
  data = 0;
  _packet.addr[0] = I2C_ACCEL_REG_ADDR_CtrlReg1;
  _packet.addr_length = 1;
  _packet.buffer = &data;
  _packet.length = 1;
  if(twi_master_read(&TWIE, &_packet) != TWI_SUCCESS)
  {
    return -1;
  }
  
  data = data | ACTIVE_MASK;
  _packet.buffer = &data;
  if(twi_master_write(&TWIE, &_packet) != TWI_SUCCESS)
  {
    return -1;
  }
  
  
  return 0;
}

/*----------------------------------------------------------------------*/
int I2C_ACCEL_Ioctl
(
    const uint8_t req,
    const uint32_t arg,
    void * argp
)
{
  uint8_t reg = 0;

  switch((I2C_ACCEL_Req_t)req)
  {
    case I2C_ACCEL_REQ_DynamicRange:
    {
      switch ((I2C_ACCEL_DynamicRange_t)arg)
      {
        case I2C_ACCEL_DYNAMIC_RANGE_2GActiveMode:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_XYZDataCfg, &reg, 1);
          reg = (reg & ~FS_MASK) |  FULL_SCALE_2G;
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_XYZDataCfg, &reg, 1);
          break;
        }
        case I2C_ACCEL_DYNAMIC_RANGE_4GActiveMode:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_XYZDataCfg, &reg, 1);
          reg = (reg & ~FS_MASK) |  FULL_SCALE_4G;
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_XYZDataCfg, &reg, 1);
          break;
        }
        case I2C_ACCEL_DYNAMIC_RANGE_8GActiveMode:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_XYZDataCfg, &reg, 1);
          reg = (reg & ~FS_MASK) |  FULL_SCALE_8G;
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_XYZDataCfg, &reg, 1);
          break;
        }
        default:
        {
          break;
        }
      }    
      break;    
    }
    case I2C_ACCEL_REQ_AsleepDataRate:
    {
      switch ((I2C_ACCEL_AsleepDataRate_t)arg)
      {
        case I2C_ACCEL_ASLEEP_DATA_RATE_50Hz:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          reg = (reg & ~ASLP_RATE_MASK) |  ASLP_RATE_50HZ;
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          break;
        }
        case I2C_ACCEL_ASLEEP_DATA_RATE_12_5Hz:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          reg = (reg & ~ASLP_RATE_MASK) |  ASLP_RATE_12_5HZ;
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          break;
        }
        case I2C_ACCEL_ASLEEP_DATA_RATE_6_25Hz:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          reg = (reg & ~ASLP_RATE_MASK) |  ASLP_RATE_6_25HZ;
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          break;
        }
        case I2C_ACCEL_ASLEEP_DATA_RATE_1_56Hz:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          reg = (reg & ~ASLP_RATE_MASK) |  ASLP_RATE_1_56HZ;
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          break;
        }
        default:
        {
          break;
        }
      }
      break;    
    }
    case I2C_ACCEL_REQ_AwakeDataRate:
    {
      switch ((I2C_ACCEL_AwakeDataRate_t)arg)
      {
        case I2C_ACCEL_AWAKE_DATA_RATE_800Hz:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          reg = (reg & ~DR_MASK) |  DATA_RATE_800HZ;
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          break;
        }
        case I2C_ACCEL_AWAKE_DATA_RATE_400Hz:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          reg = (reg & ~DR_MASK) |  DATA_RATE_400HZ;
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          break;
        }
        case I2C_ACCEL_AWAKE_DATA_RATE_200Hz:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          reg = (reg & ~DR_MASK) |  DATA_RATE_200HZ;
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          break;
        }
        case I2C_ACCEL_AWAKE_DATA_RATE_100Hz:
        {    
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          reg = (reg & ~DR_MASK) |  DATA_RATE_100HZ;
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);        
          break;
        }
        case I2C_ACCEL_AWAKE_DATA_RATE_50Hz:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          reg = (reg & ~DR_MASK) |  DATA_RATE_50HZ;    
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);    
          break;
        }
        case I2C_ACCEL_AWAKE_DATA_RATE_12_5Hz:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          reg = (reg & ~DR_MASK) |  DATA_RATE_12_5HZ;    
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          break;
        }
        case I2C_ACCEL_AWAKE_DATA_RATE_6_25Hz:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          reg = (reg & ~DR_MASK) |  DATA_RATE_6_25HZ;    
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          break;
        }
        case I2C_ACCEL_AWAKE_DATA_RATE_1_56Hz:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          reg = (reg & ~DR_MASK) |  DATA_RATE_1_56HZ;    
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);    
          break;
        }
        default:
        {
          break;
        }
      }
      break;    
    }
    case I2C_ACCEL_REQ_Oversampling:
    {
      switch ((I2C_ACCEL_OversamplingMode_t)arg)
      {
        case I2C_ACCEL_OVERSAMPLING_MODE_Normal:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg2, &reg, 1);
          reg = (reg & ~MODS_MASK);    
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg2, &reg, 1);
          break;
        }
        case I2C_ACCEL_OVERSAMPLING_MODE_LowNoiseLowPower:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg2, &reg, 1);
          reg = (reg & ~MODS_MASK) |  MOD_LOW_NOISE;
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg2, &reg, 1);
          break;
        }
        case I2C_ACCEL_OVERSAMPLING_MODE_HighResolution:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg2, &reg, 1);
          reg = (reg & ~MODS_MASK) |  MOD_HIGH_RES;
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg2, &reg, 1);
          break;
        }
        case I2C_ACCEL_OVERSAMPLING_MODE_LowPower:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg2, &reg, 1);
          reg = (reg & ~MODS_MASK) |  MOD_LOW_POWER;
          I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg2, &reg, 1);
          break;
        }
        default:
        {
          break;
        }
      }
      break;
    }
    case I2C_ACCEL_REQ_IrqEnable:
    {
      _irq_handler[arg & 0x1] = ((I2C_ACCEL_Irq_t *)(argp))->irq_callback;
      reg = (uint8_t)(((I2C_ACCEL_Irq_t *)(argp))->irq_sources & 0xff);
      I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg4, &reg, 1);
      I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_CtrlReg5, &reg, 1);
      break;
    }
    case I2C_ACCEL_REQ_DetectionSettings:
    {
      switch ((I2C_ACCEL_DetectionSetting_t)arg)
      {
        case I2C_ACCEL_DETECTION_SETTING_FfMt:
        {
          
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg1, &reg, 1);
          _data_rate = (I2C_ACCEL_AwakeDataRate_t)((reg & ASLP_RATE_MASK) >> 6);
          
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_CtrlReg2, &reg, 1);
          _oversampling_mode = (I2C_ACCEL_OversamplingMode_t)(reg & MODS_MASK);
          
          if(((I2C_ACCEL_FfMt_t *)(argp))->debounce_count >= I2C_ACCEL_FfMtCount[_data_rate][_oversampling_mode].min_range && 
             ((I2C_ACCEL_FfMt_t *)(argp))->debounce_count <= I2C_ACCEL_FfMtCount[_data_rate][_oversampling_mode].max_range &&
             ((I2C_ACCEL_FfMt_t *)(argp))->threshold >= I2C_ACCEL_FfMtThs.min_range &&
             ((I2C_ACCEL_FfMt_t *)(argp))->threshold <= I2C_ACCEL_FfMtThs.max_range)
          {
              I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_FfMtCfg, &reg, 1);
              reg = (reg & ~ELE_MASK) |  (((I2C_ACCEL_FfMt_t *)(argp))->latch & ELE_MASK);
              reg = (reg & ~OAE_MASK) |  (((I2C_ACCEL_FfMt_t *)(argp))->detect_flag & OAE_MASK);
              reg = (reg & ~(XEFE_MASK | YEFE_MASK | ZEFE_MASK)) |  (((I2C_ACCEL_FfMt_t *)(argp))->enable_axis & (XEFE_MASK | YEFE_MASK | ZEFE_MASK));
              I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_FfMtCfg, &reg, 1);
              
              I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_FfMtThs, &reg, 1);
              reg = (reg & ~DBCNTM_MASK) |  (((I2C_ACCEL_FfMt_t *)(argp))->debounce_counter_mode & DBCNTM_MASK);
              reg = (reg & ~THS_MASK) |  ((uint8_t)(((I2C_ACCEL_FfMt_t *)(argp))->threshold/I2C_ACCEL_FfMtThs.step) & THS_MASK);
              I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_FfMtThs, &reg, 1);
              
              reg = ((uint8_t)(((I2C_ACCEL_FfMt_t *)(argp))->debounce_count/I2C_ACCEL_FfMtCount[_data_rate][_oversampling_mode].step) & DBCNTM_MASK);
              I2C_ACCEL_Write(I2C_ACCEL_REG_ADDR_FfMtCount, &reg, 1);
          }
          else
          {
            return -1;
          }
          break;
        }
        case I2C_ACCEL_DETECTION_SETTING_LndPrt:
        {
          /** 
           * @todo fix me next
           */
          break;
        }
        default:
        {
          break;
        }
      }
      break;
    }
    case I2C_ACCEL_REQ_AutoZeroCalibration:
    {
      /** 
       * @todo fix me next
       */
      break;
    }
    case I2C_ACCEL_REQ_Status:
    {
      switch((I2C_ACCEL_Status_t)arg)
      {
        case I2C_ACCEL_STATUS_Poll:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_Status, &reg, 1);
          
          *((I2C_ACCEL_PollStatus_t *)(argp)) = (reg & ZYXDR_MASK)?I2C_ACCEL_STATUS_POLL_Ready:I2C_ACCEL_STATUS_POLL_Busy;
          
          if(reg & ZYXDR_MASK)
          {
            uint16_t x, y, z;
            I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_XYZDataCfg, &reg, 1);
            _dynamic_range = (I2C_ACCEL_DynamicRange_t)(reg & FS_MASK);
            
            I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_OutXMSB, _data, 6);
            x = ((((uint16_t)_data[0] << 8) & 0xff00) | ((uint16_t)_data[1] & 0x00c0)) >> 6;
            y = ((((uint16_t)_data[2] << 8) & 0xff00) | ((uint16_t)_data[3] & 0x00c0)) >> 6;
            z = ((((uint16_t)_data[4] << 8) & 0xff00) | ((uint16_t)_data[5] & 0x00c0)) >> 6;
            
            _accel_data.x = (float)((x&0x1ff) * I2C_ACCEL_OutputData[_dynamic_range].step);
            _accel_data.y = (float)((y&0x1ff) * I2C_ACCEL_OutputData[_dynamic_range].step);
            _accel_data.z = (float)((z&0x1ff) * I2C_ACCEL_OutputData[_dynamic_range].step);
            
            _accel_data.x += (0x0200&x)?I2C_ACCEL_OutputData[_dynamic_range].min_range:0.0;
            _accel_data.y += (0x0200&y)?I2C_ACCEL_OutputData[_dynamic_range].min_range:0.0;
            _accel_data.z += (0x0200&z)?I2C_ACCEL_OutputData[_dynamic_range].min_range:0.0;
            
          }        
          break;
        }
        case I2C_ACCEL_STATUS_Irq:
        {
          I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_IntSource, &reg, 1);
          *((I2C_ACCEL_IrqSource_t *)(argp)) = (I2C_ACCEL_IrqSource_t) (reg & (SRC_ASLP_MASK | SRC_LNDPRT_MASK | SRC_FF_MT_MASK | SRC_DRDY_MASK));

          if((reg & SRC_DRDY_MASK) || (reg & SRC_FF_MT_MASK))
          {
            uint16_t x, y, z;
            I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_XYZDataCfg, &reg, 1);
            _dynamic_range = (I2C_ACCEL_DynamicRange_t)(reg & FS_MASK);
            
            I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_OutXMSB, _data, 6);
            x = ((((uint16_t)_data[0] << 8) & 0xff00) | ((uint16_t)_data[1] & 0x00c0)) >> 6;
            y = ((((uint16_t)_data[2] << 8) & 0xff00) | ((uint16_t)_data[3] & 0x00c0)) >> 6;
            z = ((((uint16_t)_data[4] << 8) & 0xff00) | ((uint16_t)_data[5] & 0x00c0)) >> 6;
            
            _accel_data.x = (float)((x&0x1ff) * I2C_ACCEL_OutputData[_dynamic_range].step);
            _accel_data.y = (float)((y&0x1ff) * I2C_ACCEL_OutputData[_dynamic_range].step);
            _accel_data.z = (float)((z&0x1ff) * I2C_ACCEL_OutputData[_dynamic_range].step);
            
            _accel_data.x += (0x0200&x)?I2C_ACCEL_OutputData[_dynamic_range].min_range:0.0;
            _accel_data.y += (0x0200&y)?I2C_ACCEL_OutputData[_dynamic_range].min_range:0.0;
            _accel_data.z += (0x0200&z)?I2C_ACCEL_OutputData[_dynamic_range].min_range:0.0;
                  
            if(reg & SRC_FF_MT_MASK)
            {
          
              I2C_ACCEL_Read(I2C_ACCEL_REG_ADDR_XYZDataCfg, &reg, 1);
              if(reg & EA_MASK)
              {
                _mt_data.x_motion_flag = (reg & XHE_MASK)?I2C_ACCEL_MOTION_FLAG_Deteted:I2C_ACCEL_MOTION_FLAG_NotDetected;
                _mt_data.x_motion_polarity_flag = (reg & XHP_MASK)?I2C_ACCEL_MOTION_POLARITY_FLAG_Negative:I2C_ACCEL_MOTION_POLARITY_FLAG_Positive;
                _mt_data.y_motion_flag = (reg & YHE_MASK)?I2C_ACCEL_MOTION_FLAG_Deteted:I2C_ACCEL_MOTION_FLAG_NotDetected;
                _mt_data.y_motion_polarity_flag = (reg & YHP_MASK)?I2C_ACCEL_MOTION_POLARITY_FLAG_Negative:I2C_ACCEL_MOTION_POLARITY_FLAG_Positive;
                _mt_data.z_motion_flag = (reg & ZHE_MASK)?I2C_ACCEL_MOTION_FLAG_Deteted:I2C_ACCEL_MOTION_FLAG_NotDetected;
                _mt_data.z_motion_polarity_flag = (reg & ZHP_MASK)?I2C_ACCEL_MOTION_POLARITY_FLAG_Negative:I2C_ACCEL_MOTION_POLARITY_FLAG_Positive;
              }
            }
          }
          break;
        }
        default:
        {
          break;
        }
      }
      break;
    }
    case I2C_ACCEL_REQ_Data:
    {
      switch((I2C_ACCEL_Data_t)arg)
      {
        case I2C_ACCEL_DATA_Acceleration:
        {
          ((STATS_VectorCoordinate_t *)(argp))->x = _accel_data.x;
          ((STATS_VectorCoordinate_t *)(argp))->y = _accel_data.y;
          ((STATS_VectorCoordinate_t *)(argp))->z = _accel_data.z;
          break;
        }
        case I2C_ACCEL_DATA_Motion:
        {
          ((I2C_ACCEL_Motion_t *)(argp))->x_motion_flag = _mt_data.x_motion_flag;
          ((I2C_ACCEL_Motion_t *)(argp))->x_motion_polarity_flag = _mt_data.x_motion_polarity_flag;
          ((I2C_ACCEL_Motion_t *)(argp))->y_motion_flag = _mt_data.y_motion_flag;
          ((I2C_ACCEL_Motion_t *)(argp))->y_motion_polarity_flag = _mt_data.y_motion_polarity_flag;
          ((I2C_ACCEL_Motion_t *)(argp))->z_motion_flag = _mt_data.z_motion_flag;
          ((I2C_ACCEL_Motion_t *)(argp))->z_motion_polarity_flag = _mt_data.z_motion_polarity_flag;
          break;
        }
        case I2C_ACCEL_DATA_Orientation:
        {
          /** 
           * @todo fix me next
           */
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

/*----------------------------------------------------------------------*/
ISR(PORTE_INT0_vect) 
{ 
  irqflags_t flags;
  
  flags = cpu_irq_save();
  
  if(_irq_handler[I2C_ACCEL_IRQ_CHANNEL_Int2])
  {
    _irq_handler[I2C_ACCEL_IRQ_CHANNEL_Int2]();
  }
  
  cpu_irq_restore(flags);
}

/*----------------------------------------------------------------------*/
ISR(PORTE_INT1_vect)
{
  irqflags_t flags;
  
  flags = cpu_irq_save();
  
  if(_irq_handler[I2C_ACCEL_IRQ_CHANNEL_Int1])
  {
    _irq_handler[I2C_ACCEL_IRQ_CHANNEL_Int1]();
  }
  
  cpu_irq_restore(flags);
}