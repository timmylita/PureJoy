/**
 * \file ublox.h
 * \author tom.m
 *
 * Ublox functionality, codex, types and parsers.
 */

#ifndef _UBLOX_H
#define _UBLOX_H

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

#include <math.h>

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

#ifndef UBLOX_SPI_BAUDRATE
#define UBLOX_SPI_BAUDRATE                                     (1000000UL)
#endif

#define UBLOX_NMEA_MAX_LENGTH                                       (300)

#define LEAP_YEAR(_year)                                 (0 == (_year%4))

#define UBLOX_UBX_Reserved                                            (0)

#define UBLOX_UBX_HEADER_LENGTH                                       (8)

#define PACKING

/************************************************************************/
/* Types.                                                               */
/************************************************************************/

typedef struct 
{
    float latitude;
    float longitude;
    float altitude;
    float accuracy;
    float altitude_accuracy;
    float heading;
    float speed;
}GNSS_Coordinate_t;

typedef struct
{
    uint64_t
        utc;
	uint16_t
        year;
	uint8_t
        month,
        day,
        hour,
        minute,
        second;
}GNSS_Timestamp_t;

/**
 * Defines a data indication message.
 */
typedef struct
{
    GNSS_Coordinate_t coords;
    GNSS_Timestamp_t timestamp;
} GNSS_Position_t;

typedef enum
{
    UBLOX_SCALE_Millimeter,
    UBLOX_SCALE_Meter,
    UBLOX_SCALE_Km,
    UBLOX_SCALE_Count
} UBLOX_SCALE_t;

/* UBX Protocol. ********************************************************/

typedef enum
{
    UBLOX_GNSS_ID_GPS,
    UBLOX_GNSS_ID_SBAS,
    UBLOX_GNSS_ID_Galileo,
    UBLOX_GNSS_ID_BeiDou,
    UBLOX_GNSS_ID_IMES,
    UBLOX_GNSS_ID_QZSS,
    UBLOX_GNSS_ID_GLONASS,
    UBLOX_GNSS_ID_Count
} UBLOX_GNSS_ID_t;

typedef enum
{
    UBLOX_UBX_MessageType_In,
    UBLOX_UBX_MessageType_Out,
    UBLOX_UBX_MessageType_InOut,
    UBLOX_UBX_MessageType_PollRequest,
    UBLOX_UBX_MessageType_Command,
    UBLOX_UBX_MessageType_Set,
    UBLOX_UBX_MessageType_Get,
    UBLOX_UBX_MessageType_SetGet,
    UBLOX_UBX_MessageType_PeriodicPolled,
    UBLOX_UBX_MessageType_PollResponse,
    UBLOX_UBX_MessageType_Aperiodic,
    UBLOX_UBX_MessageType_Notification,
    UBLOX_UBX_MessageType_Singleton
} UBLOX_UBX_MessageType_t;

typedef enum
{
    UBLOX_UBX_CLASS_NUL             = 0x00,
    UBLOX_UBX_CLASS_NAV             = 0x01,
    UBLOX_UBX_CLASS_RXM             = 0x02,
    UBLOX_UBX_CLASS_INF             = 0x04,
    UBLOX_UBX_CLASS_ACK             = 0x05,
    UBLOX_UBX_CLASS_CFG             = 0x06,
    UBLOX_UBX_CLASS_UPD             = 0x09,
    UBLOX_UBX_CLASS_MON             = 0x0a,
    UBLOX_UBX_CLASS_AID             = 0x0b,
    UBLOX_UBX_CLASS_TIM             = 0x0d,
    UBLOX_UBX_CLASS_MGA             = 0x13,
    UBLOX_UBX_CLASS_LOG             = 0x21
} UBLOX_UBX_CLASS_t;

typedef enum
{
    UBLOX_UBX_NAV_AOPSTATUS         = 0x60,
    UBLOX_UBX_NAV_ATT               = 0x05,
    UBLOX_UBX_NAV_CLOCK             = 0x22,
    UBLOX_UBX_NAV_DGPS              = 0x31,
    UBLOX_UBX_NAV_DOP               = 0x04,
    UBLOX_UBX_NAV_EOE               = 0x61,
    UBLOX_UBX_NAV_GEOFENCE          = 0x39,
    UBLOX_UBX_NAV_ODO               = 0x09,
    UBLOX_UBX_NAV_ORB               = 0x34,
    UBLOX_UBX_NAV_POSECEF           = 0x01,
    UBLOX_UBX_NAV_POSLLH            = 0x02,
    UBLOX_UBX_NAV_PVT               = 0x07,
    UBLOX_UBX_NAV_RESETODO          = 0x10,
    UBLOX_UBX_NAV_SAT               = 0x35,
    UBLOX_UBX_NAV_SBAS              = 0x32,
    UBLOX_UBX_NAV_SOL               = 0x06,
    UBLOX_UBX_NAV_STATUS            = 0x03,
    UBLOX_UBX_NAV_SVINFO            = 0x30,
    UBLOX_UBX_NAV_TIMEBDS           = 0x24,
    UBLOX_UBX_NAV_TIMEGAL           = 0x25,
    UBLOX_UBX_NAV_TIMEGLO           = 0x23,
    UBLOX_UBX_NAV_TIMEGPS           = 0x20,
    UBLOX_UBX_NAV_TIMELS            = 0x26,
    UBLOX_UBX_NAV_TIMEUTC           = 0x21,
    UBLOX_UBX_NAV_VELECEF           = 0x11,
    UBLOX_UBX_NAV_VELNED            = 0x12
} UBLOX_UBX_CLASS_NAV_t;

typedef enum
{
    UBLOX_UBX_ID_RXM_IMES           = 0x61,
    UBLOX_UBX_ID_RXM_MEASX          = 0x14,
    UBLOX_UBX_ID_RXM_PMREQ          = 0x41,
    UBLOX_UBX_ID_RXM_RAWX           = 0x15,
    UBLOX_UBX_ID_RXM_RLM            = 0x59,
    UBLOX_UBX_ID_RXM_SFRBX          = 0x13,
    UBLOX_UBX_ID_RXM_SVSI           = 0x20
} UBLOX_UBX_CLASS_RXM_t;

typedef enum
{
    UBLOX_UBX_INF_DEBUG             = 0x04,
    UBLOX_UBX_INF_ERROR             = 0x00,
    UBLOX_UBX_INF_NOTICE            = 0x02,
    UBLOX_UBX_INF_TEST              = 0x03,
    UBLOX_UBX_INF_WARNING           = 0x01
} UBLOX_UBX_CLASS_INF_t;

typedef enum
{
    UBLOX_UBX_ACK_NAK               = 0x00,
    UBLOX_UBX_ACK_ACK               = 0x01
} UBLOX_UBX_CLASS_ACK_t;

typedef enum
{
    UBLOX_UBX_CFG_ANT               = 0x13,
    UBLOX_UBX_CFG_CFG               = 0x09,
    UBLOX_UBX_CFG_DAT               = 0x06,
    UBLOX_UBX_CFG_DOSC              = 0x61,
    UBLOX_UBX_CFG_DYNSEED           = 0x85,
    UBLOX_UBX_CFG_ESRC              = 0x60,
    UBLOX_UBX_CFG_FIXSEED           = 0x84,
    UBLOX_UBX_CFG_GEOFENCE          = 0x69,
    UBLOX_UBX_CFG_GNSS              = 0x3e,
    UBLOX_UBX_CFG_HNR               = 0x5c,
    UBLOX_UBX_CFG_INF               = 0x02,
    UBLOX_UBX_CFG_ITFM              = 0x39,
    UBLOX_UBX_CFG_LOGFILTER         = 0x47,
    UBLOX_UBX_CFG_MSG               = 0x01,
    UBLOX_UBX_CFG_NAV5              = 0x24,
    UBLOX_UBX_CFG_NAVX5             = 0x23,
    UBLOX_UBX_CFG_NMEA              = 0x17,
    UBLOX_UBX_CFG_ODO               = 0x1e,
    UBLOX_UBX_CFG_PM2               = 0x3b,
    UBLOX_UBX_CFG_PMS               = 0x86,
    UBLOX_UBX_CFG_PRT               = 0x00,
    UBLOX_UBX_CFG_PWR               = 0x57,
    UBLOX_UBX_CFG_RATE              = 0x08,
    UBLOX_UBX_CFG_RINV              = 0x34,
    UBLOX_UBX_CFG_RST               = 0x04,
    UBLOX_UBX_CFG_RXM               = 0x11,
    UBLOX_UBX_CFG_SBAS              = 0x16,
    UBLOX_UBX_CFG_SMGR              = 0x62,
    UBLOX_UBX_CFG_TMODE2            = 0x3d,
    UBLOX_UBX_CFG_TP5               = 0x31,
    UBLOX_UBX_CFG_TXSLOT            = 0x53,
    UBLOX_UBX_CFG_USB               = 0x1b
} UBLOX_UBX_CLASS_CFG_t;

typedef enum
{
    UBLOX_UBX_UPD_SOS               = 0x14
} UBLOX_UBX_CLASS_UPD_t;

typedef enum
{
    UBLOX_UBX_MON_GNSS              = 0x28,
    UBLOX_UBX_MON_HW2               = 0x0b,
    UBLOX_UBX_MON_HW                = 0x09,
    UBLOX_UBX_MON_IO                = 0x02,
    UBLOX_UBX_MON_MSGPP             = 0x06,
    UBLOX_UBX_MON_PATCH             = 0x27,
    UBLOX_UBX_MON_RXBUF             = 0x07,
    UBLOX_UBX_MON_RXR               = 0x21,
    UBLOX_UBX_MON_SMGR              = 0x2e,
    UBLOX_UBX_MON_TXBUF             = 0x08,
    UBLOX_UBX_MON_VER               = 0x04
} UBLOX_UBX_CLASS_MON_t;

typedef enum
{
    UBLOX_UBX_AID_INI               = 0x01,
    UBLOX_UBX_AID_HUI               = 0x02,
    UBLOX_UBX_AID_ALM               = 0x30,
    UBLOX_UBX_AID_EPH               = 0x31,
    UBLOX_UBX_AID_AOP               = 0x33
} UBLOX_UBX_CLASS_AID_t;

typedef enum
{
    UBLOX_UBX_HNR_PVT               = 0x00
} UBLOX_UBX_CLASS_HNR_t;

typedef enum
{
    UBLOX_UBX_TIM_DOSC              = 0x11,
    UBLOX_UBX_TIM_FCHG              = 0x16,
    UBLOX_UBX_TIM_HOC               = 0x17,
    UBLOX_UBX_TIM_SMEAS             = 0x13,
    UBLOX_UBX_TIM_SVIN              = 0x04,
    UBLOX_UBX_TIM_TM2               = 0x03,
    UBLOX_UBX_TIM_TOS               = 0x12,
    UBLOX_UBX_TIM_TP                = 0x01,
    UBLOX_UBX_TIM_VCOCAL            = 0x15,
    UBLOX_UBX_TIM_VRFY              = 0x06
} UBLOX_UBX_CLASS_TIM_t;

typedef enum
{
    UBLOX_UBX_ID_MGA_ACK_DATA0      = 0x60,
    UBLOX_UBX_ID_MGA_ANO            = 0x20,
    UBLOX_UBX_ID_MGA_DBS            = 0x03,    
    UBLOX_UBX_ID_MGA_DBD            = 0x80,
    UBLOX_UBX_ID_MGA_FLASH          = 0x21,
    UBLOX_UBX_ID_MGA_GAL            = 0x02,
    UBLOX_UBX_ID_MGA_GLO            = 0x06,
    UBLOX_UBX_ID_MGA_GPS            = 0x00,
    UBLOX_UBX_ID_MGA_INI            = 0x40,
    UBLOX_UBX_ID_MGA_QZSS           = 0x05
} UBLOX_UBX_CLASS_MGA_t;

typedef enum
{
    UBLOX_UBX_LOG_CREATE            = 0x07,
    UBLOX_UBX_LOG_ERASE             = 0x03,
    UBLOX_UBX_LOG_FINDTIME          = 0x0e,
    UBLOX_UBX_LOG_INFO              = 0x08,
    UBLOX_UBX_LOG_RETRIEVEPOSEXTRA  = 0x0f,
    UBLOX_UBX_LOG_RETRIEVEPOS       = 0x0b,
    UBLOX_UBX_LOG_RETRIEVESTR       = 0x0d,
    UBLOX_UBX_LOG_RETRIEVE          = 0x09,
    UBLOX_UBX_LOG_STRING            = 0x04
} UBLOX_UBX_CLASS_LOG_t;

/* UBLOX Configuration. *************************************************/

typedef struct
{
	uint8_t class;
	uint8_t id;
} UBLOX_UBX_MsgType_t;

typedef struct
{
	uint8_t *payload_addr;
	uint16_t payload_len;
	uint8_t class;
	uint8_t id;
	uint8_t checksum[2];
} UBLOX_UBX_Msg_t;

// CFG-CFG
typedef union
{
	uint32_t value;
	uint8_t bytes[4];
	struct
	{
		uint32_t reserved_1:19;
		uint8_t fts_conf:1;
		uint8_t log_conf:1;
		uint8_t ant_conf:1;
		uint8_t rinv_conf:1;
		uint8_t sen_conf:1;
		uint8_t reserved_3:3;
		uint8_t rxm_conf:1;
		uint8_t nav_conf:1;
		uint8_t inf_conf:1;
		uint8_t msg_conf:1;
		uint8_t io_port:1;
	} fields;
} UBLOX_UBX_CFG_CFG_Mask_t;

typedef struct
{
	UBLOX_UBX_CFG_CFG_Mask_t clear_mask;
	UBLOX_UBX_CFG_CFG_Mask_t load_mask;
	UBLOX_UBX_CFG_CFG_Mask_t save_mask;
} PACKING UBLOX_UBX_CFG_Cfg_t;

// CFG-PM2
typedef union
{
	uint32_t value;
	uint8_t bytes[4];
	struct
	{
		uint8_t reserved_1:4;
		uint8_t ext_int_sel:1; // EVA-M8M uses EXTINT0.
		uint8_t ext_int_wake:1;
		uint8_t ext_int_backup:1;
		uint8_t reserved_2:1;
		uint8_t limit_peak_curr:2;
		uint8_t wait_time_fix:1;
		uint8_t update_rtc:1;
		uint8_t update_eph:1;
		uint8_t reserved_3:3;
		uint8_t do_not_enter_off:1;
		uint8_t mode:2;
		uint8_t reserved_4:5;
		uint8_t reserved_5:8;
	} fields;
} UBLOX_UBX_CFG_PM2_Flags_t;

typedef struct
{
	uint8_t version;
	uint8_t reserved_1;
	uint8_t max_startup_state_dur;
	uint8_t reserved_2;
	UBLOX_UBX_CFG_PM2_Flags_t flags;
	uint32_t update_period;
	uint32_t search_period;
	uint32_t grid_offset;
	uint16_t on_time;
	uint16_t min_acq_time;
	uint8_t reserved_3[20];
} PACKING UBLOX_UBX_CFG_Pm2_t;

// CFG-NAV5
typedef enum
{
	UBLOX_UBX_CFG_NAV5_MODEL_Portable   = 0x00,
	UBLOX_UBX_CFG_NAV5_MODEL_Stationary = 0x02,
	UBLOX_UBX_CFG_NAV5_MODEL_Pedestrian = 0x03,
	UBLOX_UBX_CFG_NAV5_MODEL_Automotive = 0x04,
	UBLOX_UBX_CFG_NAV5_MODEL_Sea        = 0x05,
	UBLOX_UBX_CFG_NAV5_MODEL_Airborn_1G = 0x06,
	UBLOX_UBX_CFG_NAV5_MODEL_Airborn_2G = 0x07,
	UBLOX_UBX_CFG_NAV5_MODEL_Airborn_4G = 0x08,
	UBLOX_UBX_CFG_NAV5_MODEL_Wristwatch = 0x09
} UBLOX_UBX_CFG_NAV5_Model_t;

typedef enum
{
	UBLOX_UBX_CFG_NAV5_FIX_MODE_Unknown = 0x00,
	UBLOX_UBX_CFG_NAV5_FIX_MODE_2D_Only = 0x01,
	UBLOX_UBX_CFG_NAV5_FIX_MODE_3D_Only = 0x02,
	UBLOX_UBX_CFG_NAV5_FIX_MODE_Auto    = 0x03
} UBLOX_UBX_CFG_NAV5_FixMode_t;

typedef enum
{
	UBLOX_UBX_CFG_NAV5_MASK_model               = 1 << 0,
	UBLOX_UBX_CFG_NAV5_MASK_min_el              = 1 << 1,
	UBLOX_UBX_CFG_NAV5_MASK_pos_fix_mode        = 1 << 2,
	UBLOX_UBX_CFG_NAV5_MASK_dr_lim              = 1 << 3,
	UBLOX_UBX_CFG_NAV5_MASK_pos_mask            = 1 << 4,
	UBLOX_UBX_CFG_NAV5_MASK_time_mask           = 1 << 5,
	UBLOX_UBX_CFG_NAV5_MASK_static_hold_mask    = 1 << 6,
	UBLOX_UBX_CFG_NAV5_MASK_dgps_mask           = 1 << 7,
	UBLOX_UBX_CFG_NAV5_MASK_cno_threshold       = 1 << 8,
	UBLOX_UBX_CFG_NAV5_MASK_utc                 = 1 << 10
} UBLOX_UBX_CFG_NAV5_BitMask_t;
typedef union
{
	uint16_t value;
	uint8_t bytes[2];
	struct
	{
		int dyn:1;
		int min_el:1;
		int pos_mix_mode:1;
		int dr_lim:1;
		int pos_mask:1;
		int time_mask:1;
		int static_hold_mask:1;
		int dgps_mask:1;
		int cno_threshold:1;
		int reserved_1:1;
		int utc:1;
	} fields;
} UBLOX_UBX_CFG_NAV5_Mask_t;

typedef struct
{
	UBLOX_UBX_CFG_NAV5_Mask_t mask;
	uint8_t dyn_model;
	uint8_t fix_mode;
	int32_t fixed_alt;
	uint32_t fixed_alt_var;
	int8_t min_elev;
	uint8_t dr_limit;
	uint16_t p_dop;
	uint16_t t_dop;
	uint16_t p_acc;
	uint16_t t_acc;
	uint8_t static_hold_thresh;
	uint8_t dgnss_timeout;
	uint8_t cno_thresh_num_svs;
	uint8_t cno_thresh;
	uint8_t reserved_1[2];
	uint16_t static_hold_max_dist;
	uint8_t utc_standard;
	uint8_t reserved_2[5];
} PACKING UBLOX_UBX_CFG_Nav5_t;

// CFG-MSG
typedef struct
{
	UBLOX_UBX_MsgType_t type;
	uint8_t rate;
} PACKING UBLOX_UBX_CFG_Msg_t;

// CFG-MONVER
typedef struct
{
	uint8_t sw_version[30];
	uint8_t hw_version[10];
} PACKING UBLOX_UBX_CFG_MonVer_t;

// CFG-PRT
typedef enum
{
	UBLOX_UBX_CFG_PRT_PORT_ID_Ddc       = 0x00,
	UBLOX_UBX_CFG_PRT_PORT_ID_Uart1     = 0x01,
	UBLOX_UBX_CFG_PRT_PORT_ID_Uart2     = 0x02,
	UBLOX_UBX_CFG_PRT_PORT_ID_Usb       = 0x03,
	UBLOX_UBX_CFG_PRT_PORT_ID_Spi       = 0x04
} UBLOX_UBX_CFG_PRT_PortId_t;

typedef enum
{
	UBLOX_UBX_CFG_PRT_ProtoMask_none        = 0x00,
	UBLOX_UBX_CFG_PRT_ProtoMask_ubx         = 1 << 0,
	UBLOX_UBX_CFG_PRT_ProtoMask_nmea        = 1 << 1,
	UBLOX_UBX_CFG_PRT_ProtoMask_rtcm        = 1 << 2,
	UBLOX_UBX_CFG_PRT_ProtoMask_rtcm3       = 1 << 5
} UBLOX_UBX_CFG_PRT_ProtoMask_t;

typedef union
{
	uint16_t value;
	uint8_t bytes[2];
	struct
	{
		uint8_t thresh_1:8;
		uint8_t thresh_0:1;
		uint8_t pin:5;
		uint8_t pol:1;
		uint8_t en:1;
	} fields;
} UBLOX_UBX_CFG_PRT_TxReady_t;

typedef union
{
	uint32_t value;
	uint8_t bytes[4];
	struct
	{
		    
		uint32_t reserved_1:18;
		uint8_t n_stop_bits:2;
		uint8_t parity:3;
		uint8_t reserved_2:1;
		uint8_t char_len:2;
		uint8_t reserved_3:6;
	} fields;
} UBLOX_UBX_CFG_PRT_Mode_t;

typedef union
{
	uint16_t value;
	uint8_t bytes[2];
	struct
	{
		uint16_t reserved:13;
		uint8_t in_rtc_m:1;
		uint8_t in_nmea:1;
		uint8_t in_ubx:1;
	} fields;
} UBLOX_UBX_CFG_PRT_InProtoMask_t;

typedef union
{
	uint16_t value;
	uint8_t bytes[2];
	struct
	{
		uint16_t reserved:14;
		uint8_t out_nmea:1;
		uint8_t out_ubx:1;
	} fields;
} UBLOX_UBX_CFG_PRT_OutProtoMask_t;

typedef union
{
	uint16_t value;
	uint8_t bytes[2];
	struct
	{
		uint16_t reserved_1:14;
		uint8_t ext_tx_timeout:1;
		uint8_t reserved_2:1;
	} fields;
} UBLOX_UBX_CFG_PRT_Flags_t;

typedef struct
{
	uint8_t port_id;
	uint8_t reserved_1;
	UBLOX_UBX_CFG_PRT_TxReady_t tx_ready;
	UBLOX_UBX_CFG_PRT_Mode_t mode;
	uint8_t reserved_2[4];
	uint32_t baudrate;
	UBLOX_UBX_CFG_PRT_InProtoMask_t in_proto_mask;
	UBLOX_UBX_CFG_PRT_OutProtoMask_t out_proto_mask;
	UBLOX_UBX_CFG_PRT_Flags_t flags;
	uint8_t reserved_3[2];
} PACKING UBLOX_UBX_CFG_Prt_t;

// CFG-RATE
typedef struct
{
	uint16_t meas_rate;
	uint16_t nav_rate;
	uint16_t time_ref;
} PACKING UBLOX_UBX_CFG_Rate_t;

// CFG-RST
typedef enum
{
	UBLOX_UBX_CFG_RST_NavBbrMask_SpecialSet_HotStart    = 0x0000,
	UBLOX_UBX_CFG_RST_NavBbrMask_SpecialSet_WarmStart   = 0x0001,
	UBLOX_UBX_CFG_RST_NavBbrMask_SpecialSet_ColdStart   = 0xffff
} UBLOX_UBX_CFG_RST_NavBbrMask_SpecialSet_t;

typedef enum
{
	UBLOX_UBX_CFG_RST_TYPE_Hardware_WtdImmediate        = 0x00,
	UBLOX_UBX_CFG_RST_TYPE_SoftControlled               = 0x01,
	UBLOX_UBX_CFG_RST_TYPE_SoftController_GnssOnly      = 0x02,
	UBLOX_UBX_CFG_RST_TYPE_Hardware_WtdAfterShutdown    = 0x04,
	UBLOX_UBX_CFG_RST_TYPE_ControlledGnssStop           = 0x08,
	UBLOX_UBX_CFG_RST_TYPE_ControlledGnssStart          = 0x09
} UBLOX_UBX_CFG_RST_Type_t;

typedef union
{
	uint16_t value;
	uint8_t bytes[2];
	struct
	{
		uint8_t aop:1;
		uint8_t reserved:6;
		uint8_t rtc:1;
		uint8_t utc:1;
		uint8_t osc:1;
		uint8_t clkd:1;
		uint8_t pos:1;
		uint8_t klob:1;
		uint8_t health:1;
		uint8_t alm:1;
		uint8_t eph:1;
	} fields;
} UBLOX_UBX_CFG_RST_NavBbrMask_t;

typedef struct
{
	UBLOX_UBX_CFG_RST_NavBbrMask_t nav_bbr_mask;
	uint8_t rst_type;
	uint8_t reserved;
} PACKING UBLOX_UBX_CFG_Rst_t;

// CFG-RXM
typedef enum
{
	UBLOX_UBX_CFG_RXM_GPS_SENSITIVITY_MODE_Normal,
	UBLOX_UBX_CFG_RXM_GPS_SENSITIVITY_MODE_FastAcquisition,
	UBLOX_UBX_CFG_RXM_GPS_SENSITIVITY_MODE_HighSensitivity,
	UBLOX_UBX_CFG_RXM_GPS_SENSITIVITY_MODE_Auto
} UBLOX_UBX_CFG_RXM_GpsSensitivityMode_t;

typedef enum
{
	UBLOX_UBX_CFG_RXM_LOW_POWER_MODE_ContinuousTracking   = 0x00,
	UBLOX_UBX_CFG_RXM_LOW_POWER_MODE_PowerSaveMode        = 0x01,
} UBLOX_UBX_CFG_RXM_LowPowerMode_t;
	
typedef struct
{
	uint8_t reserved_0;
	uint8_t lp_mode;
} PACKING UBLOX_UBX_CFG_Rxm_t;

/*
SBAS
*/

#if (UBLOX_UBX_CFG_SBAS_Include)
typedef enum
{
	UBLOX_UBX_CFG_SBAS_MASK_MODE_enable         = 1 << 0,
	UBLOX_UBX_CFG_SBAS_MASK_MODE_test           = 1 << 1
} UBLOX_UBX_CFG_SBAS_MASK_Mode_t;
#endif

#if (UBLOX_UBX_CFG_SBAS_Include)
typedef enum
{
	UBLOX_UBX_CFG_SBAS_MASK_USAGE_range         = 1 << 0,
	UBLOX_UBX_CFG_SBAS_MASK_USAGE_diff_corr     = 1 << 1,
	UBLOX_UBX_CFG_SBAS_MASK_USAGE_integrity     = 1 << 2
} UBLOX_UBX_CFG_SBAS_MASK_Usage_t;
#endif

#if (UBLOX_UBX_CFG_SBAS_Include)
typedef enum
{
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE2_PRN152    = 1 << 0,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE2_PRN153    = 1 << 1,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE2_PRN154    = 1 << 2,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE2_PRN155    = 1 << 3,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE2_PRN156    = 1 << 4,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE2_PRN157    = 1 << 5,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE2_PRN158    = 1 << 6
} CFG_SBAS_MASK_ScanMode2_t;
#endif

#if (UBLOX_UBX_CFG_SBAS_Include)
typedef enum
{
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN120    = 1 << 0,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN121    = 1 << 1,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN122    = 1 << 2,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN123    = 1 << 3,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN124    = 1 << 4,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN125    = 1 << 5,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN126    = 1 << 6,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN127    = 1 << 7,
	    
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN128    = 1 << 8,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN129    = 1 << 9,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN130    = 1 << 10,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN131    = 1 << 11,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN132    = 1 << 12,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN133    = 1 << 13,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN134    = 1 << 14,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN135    = 1 << 15,
	    
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN136    = 1 << 16,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN137    = 1 << 17,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN138    = 1 << 18,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN139    = 1 << 19,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN140    = 1 << 20,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN141    = 1 << 21,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN142    = 1 << 22,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN143    = 1 << 23,
	    
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN144    = 1 << 24,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN145    = 1 << 25,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN146    = 1 << 26,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN147    = 1 << 27,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN148    = 1 << 28,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN149    = 1 << 29,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN150    = 1 << 30,
	UBLOX_UBX_CFG_SBAS_MASK_SCANMODE1_PRN151    = 1 << 31,
} UBLOX_UBX_CFG_SBAS_MASK_ScanMode1_t;
#endif

#if (UBLOX_UBX_CFG_SBAS_Include)
typedef union
{
	uint8_t value;
	uint8_t bytes[1];
	struct
	{
		uint8_t reserved:6;
		uint8_t test:1;
		uint8_t enabled:1;
	} fields;
} UBLOX_UBX_CFG_SBAS_Mode_t;
#endif

#if (UBLOX_UBX_CFG_SBAS_Include)
typedef union
{
	uint8_t value;
	uint8_t bytes[1];
	struct
	{
		uint8_t reserved:5;
		uint8_t integrity:1;
		uint8_t diff_corr:1;
		uint8_t range:1;
	} fields;
} UBLOX_UBX_CFG_SBAS_Usage_t;
#endif

#if (UBLOX_UBX_CFG_SBAS_Include)
typedef union
{
	uint8_t value;
	uint8_t bytes[1];
	struct
	{
		uint8_t reserved:1;
		uint8_t PRN:7;
	} fields;
} UBLOX_UBX_CFG_SBAS_ScanMode2_t;
#endif

#if (UBLOX_UBX_CFG_SBAS_Include)
typedef union
{
	uint32_t value;
	uint8_t bytes[4];
	struct
	{
		uint32_t PRN:32;
	} fields;
} UBLOX_UBX_CFG_SBAS_ScanMode1_t;
#endif

#if (UBLOX_UBX_CFG_SBAS_Include)
typedef struct
{
	UBLOX_UBX_CFG_SBAS_Mode_t mode;
	UBLOX_UBX_CFG_SBAS_Usage_t usage;
	uint8_t max_sbas;
	UBLOX_UBX_CFG_SBAS_ScanMode2_t scan_mode_2;
	UBLOX_UBX_CFG_SBAS_ScanMode1_t scan_mode_1;
} PACKING UBLOX_UBX_CFG_Sbas_t;
#endif

/*
TP5
*/

#if (UBLOX_UBX_CFG_TP5_Include)
typedef enum
{
	UBLOX_UBX_CFG_TP5_TP_IDX_TIMETPULSE  = 0x00,
	UBLOX_UBX_CFG_TP5_TP_IDX_TIMETPULSE2 = 0x01
} UBLOX_UBX_CFG_TP5_TpIdx_t;
#endif
	
#if (UBLOX_UBX_CFG_TP5_Include)
typedef union
{
	uint32_t value;
	uint8_t bytes[4];
	struct
	{
		uint32_t reserved:24;
		uint8_t active:1;
		uint8_t grid_utc_gps:1;
		uint8_t polarity:1;
		uint8_t align_to_tow:1;
		uint8_t is_length:1;
		uint8_t is_freq:1;
		uint8_t locked_other_set:1;
		uint8_t lock_gps_freq:1;
	} fields;
} UBLOX_UBX_CFG_TP5_Flags_t;
#endif

#if (UBLOX_UBX_CFG_TP5_Include)
typedef struct
{
	uint8_t tp_idx;
	uint8_t version;
	uint8_t reserved_1[2];
	uint16_t ant_cable_delay;
	uint16_t rf_group_delay;
	uint32_t freq_period;
	uint32_t freq_period_lock;
	uint32_t pulse_len_ratio;
	uint32_t pulse_len_ratio_lock;
	int32_t user_config_delay;
	UBLOX_UBX_CFG_TP5_Flags_t flags;
}  PACKING UBLOX_UBX_CFG_Tp5_t;
#endif

typedef enum
{
	UBLOX_UBX_CFG_GNSS_ID_Gps		= 0x00,
	UBLOX_UBX_CFG_GNSS_ID_Sbas		= 0x01,
	UBLOX_UBX_CFG_GNSS_ID_Galileo	= 0x02,
	UBLOX_UBX_CFG_GNSS_ID_Beidou	= 0x03,
	UBLOX_UBX_CFG_GNSS_ID_Imes		= 0x04,
	UBLOX_UBX_CFG_GNSS_ID_Qzss		= 0x05,
	UBLOX_UBX_CFG_GNSS_ID_Glonass	= 0x06
} UBLOX_UBX_CFG_GNSS_Id_t;

typedef union
{
	uint32_t value;
	uint8_t bytes[4];
	struct
	{
		uint8_t reserved_1:8;
		uint8_t sig_cfg_mask:8;
		uint16_t reserved_2:15;
		uint8_t enable:1;
	} fields;
} UBLOX_UBX_CFG_GNSS_Flags_t;

typedef struct
{
	uint8_t gnss_id;
	uint8_t res_trk_ch;
	uint8_t max_trk_ch;
	uint8_t resereved_1;
	UBLOX_UBX_CFG_GNSS_Flags_t flags;
}PACKING UBLOX_UBX_CFG_GNSS_ConfigBlock_t;

typedef struct
{
	uint8_t msg_ver;
	uint8_t num_trk_ch_hw;  
	uint8_t num_trk_ch_use;
	uint8_t num_config_blocks;
	UBLOX_UBX_CFG_GNSS_ConfigBlock_t config_blocks[8];
} PACKING UBLOX_UBX_CFG_Gnss_t;

/* UBLOX Polling. *************************************************/

typedef union
{
	uint8_t value;
	uint8_t bytes[1];
	struct
	{
		uint8_t reserved_1:5;
		uint8_t fully_resolved:1;
		uint8_t valid_time:1;
		uint8_t valid_date:1;
	} fields;
} UBLOX_UBX_NAV_PVT_Valid_t;

typedef union
{
	uint8_t value;
	uint8_t bytes[1];
	struct
	{
		uint8_t carr_soln:1;
		uint8_t reserved_1:1;
		uint8_t head_veh_valid:1;
		uint8_t psm_state:1;
		uint8_t reserved_2:2;
		uint8_t diff_soln:1;
		uint8_t gnss_fix_ok:1;
	} fields;
} UBLOX_NAV_PVT_Flags_t;

typedef union
{
	uint8_t value;
	uint8_t bytes[1];
	struct
	{
		uint8_t confirmed_time:1;
		uint8_t confirmed_date:1;
		uint8_t confirmed_avai:1;
		uint8_t reserved_1:5;
	} fields;
} UBLOX_NAV_PVT_Flags2_t;

typedef struct
{
	uint32_t i_tow;
	uint16_t year;
	uint8_t month, day, hour, minute, second;
	UBLOX_UBX_NAV_PVT_Valid_t valid;
	uint32_t t_acc;
	int32_t nano;
	uint8_t fix_type;
	UBLOX_NAV_PVT_Flags_t flags;
	UBLOX_NAV_PVT_Flags2_t flags2;
	uint8_t num_sv;
	int32_t lon, lat;
	int32_t height, h_msl;
	uint32_t h_acc, v_acc;
	int32_t velN, velE, velD, gspeed;
	int32_t head_mot;
	uint32_t s_acc;
	uint32_t head_acc;
	uint16_t p_dop;
	uint8_t reserved_1[6];
	int32_t head_veh;
	uint8_t reserved_2[4];
} PACKING UBLOX_UBX_NAV_Pvt_t;


/* NMEA_ Functions. *****************************************************/

typedef enum
{
  UBLOX_NMEA_SENTENCE_ID_Invalid    = -1,
  UBLOX_NMEA_SENTENCE_ID_Unknown    = 0,
  UBLOX_NMEA_SENTENCE_ID_Rmc,
  UBLOX_NMEA_SENTENCE_ID_Gga,
  UBLOX_NMEA_SENTENCE_ID_Pubx
} UBLOX_NMEA_SentenceId_t;

typedef struct
{
  int_least32_t value;
  int_least32_t scale;
} UBLOX_NMEA_Float_t;

typedef struct
{
  int day;
  int month;
  int year;
} UBLOX_NMEA_Date_t;

typedef struct 
{
  int hours;
  int minutes;
  int seconds;
  int microseconds;
} UBLOX_NMEA_Time_t;

typedef struct
{
    UBLOX_NMEA_Time_t       time;
    bool                    valid;
    UBLOX_NMEA_Float_t      latitude;
    UBLOX_NMEA_Float_t      longitude;
    UBLOX_NMEA_Float_t      speed;
    UBLOX_NMEA_Float_t      course;
    UBLOX_NMEA_Date_t       date;
    UBLOX_NMEA_Float_t      variation;
} UBLOX_NMEA_SentenceRmc_t;

typedef struct 
{
    UBLOX_NMEA_Time_t       time;
    UBLOX_NMEA_Float_t      latitude;
    UBLOX_NMEA_Float_t      longitude;
    int                     fix_quality;
    int                     satellites_tracked;
    UBLOX_NMEA_Float_t      hdop;
    UBLOX_NMEA_Float_t      altitude;
    char                    altitude_units;
    UBLOX_NMEA_Float_t      height;
    char                    height_units;
    int                     dgps_age;
} UBLOX_NMEA_SentenceGga_t;

typedef struct
{
	UBLOX_NMEA_Date_t date;	
	UBLOX_NMEA_Time_t time;
    UBLOX_NMEA_Float_t latitude;
    UBLOX_NMEA_Float_t longitude;
	UBLOX_NMEA_Float_t altitude;
	bool status;
	UBLOX_NMEA_Float_t hacc;
	UBLOX_NMEA_Float_t vacc;
	UBLOX_NMEA_Float_t sog;
	UBLOX_NMEA_Float_t cog;
} UBLOX_NMEA_SentencePubx_t;


/* SPI protocol. ********************************************************/

typedef enum
{
    UBLOX_SPI_REQ_Config,
	UBLOX_SPI_REQ_Poll,
    UBLOX_SPI_REQ_Data,
    UBLOX_SPI_REQ_Sleep,
    UBLOX_SPI_REQ_Wake
} UBLOX_SPI_Req_t;

typedef enum
{
	UBLOX_SPI_POLL_MSG_NavPvt
} UBLOX_SPI_POLL_Msg_t;

typedef enum
{
	UBLOX_SPI_DATA_FORMAT_Nmea,
	UBLOX_SPI_DATA_FORMAT_Ubx
} UBLOX_SPI_DATA_Format_t;

typedef struct 
{
    UBLOX_NMEA_SentenceRmc_t  rmc;
    UBLOX_NMEA_SentenceGga_t  gga;
} UBLOX_SPI_DATA_Nmea_t;

/************************************************************************/
/* Public prototypes.                                                   */
/************************************************************************/

/*----------------------------------------------------------------------*/
void 
UBLOX_SPI_Init
(
    void
);

/*----------------------------------------------------------------------*/
int 
UBLOX_SPI_Ioctl
(
    const uint8_t req,
    const uint32_t arg,
    const void * argp
);

/*----------------------------------------------------------------------*/
void UBLOX_CycleScale (void);

/*----------------------------------------------------------------------*/
UBLOX_SCALE_t UBLOX_GetScale (void);

/*----------------------------------------------------------------------*/
float UBLOX_ResolveScale (void);

/************************************************************************/
/* Public variables.                                                    */
/************************************************************************/

/************************************************************************/
/* Public Macros.                                                       */
/************************************************************************/

/************************************************************************/
/* Implementations.                                                     */
/************************************************************************/

/************************************************************************
 * Rescale a fixed-point value to a different scale. Rounds towards zero.
 */
static inline
int_least32_t
UBLOX_NMEA_Rescale
(
    UBLOX_NMEA_Float_t *f,
    int_least32_t new_scale
)
{
    if (f->scale == 0)
        return 0;
    if (f->scale == new_scale)
        return f->value;
    if (f->scale > new_scale)
        return
        (
            (
                f->value
                + 
                (
                    (f->value > 0)
                    - (f->value < 0)
                )
                * f->scale/new_scale/2
            )
            / (f->scale/new_scale)
        );
    else
        return f->value * (new_scale/f->scale);
}

/************************************************************************
 * Convert a fixed-point value to a floating-point value.
 * Returns NaN for "unknown" values.
 */
static inline float
UBLOX_NMEA_ToFloat
(
    UBLOX_NMEA_Float_t *f
)
{
    if (f->scale == 0)
        return NAN;
    return (float) f->value / (float) f->scale;
}

/************************************************************************
 * Convert a raw coordinate to a floating point DD.DDD... value.
 * Returns NaN for "unknown" values.
 */
static inline float
UBLOX_NMEA_ToCoord
(
    UBLOX_NMEA_Float_t *f
)
{
    if (f->scale == 0)
        return NAN;
    int_least32_t degrees = f->value / (f->scale * 100);
    int_least32_t minutes = f->value % (f->scale * 100);
    return (float) degrees + (float) minutes / (60 * f->scale);
}

/************************************************************************
 * Converts date and time to ms count.
 */
static inline uint64_t
UBLOX_NMEA_ToMillis
(
    UBLOX_NMEA_Date_t *d,
    UBLOX_NMEA_Time_t *t
)
{
    const uint8_t monthDays[]
        ={31,28,31,30,31,30,31,31,30,31,30,31};
    uint16_t i;
    uint64_t ms;

    ms = (d->year-1970)*(60UL*60UL*24UL*365UL*1000UL);

    // add extra days for leap years
    for (i=1970; i<d->year; i++) 
    {
        if (LEAP_YEAR(i)) 
        {
            ms += 60UL*60UL*24UL*1000UL;
        }
    }
    // add days for this year
    for (i=0; i<d->month; i++) 
    {    
        ms
            += 60UL*60UL*24UL*1000UL
            *
            (i==1 && LEAP_YEAR(d->year))
            ?
            monthDays[i]
            :
            1;    
    }

    ms += (d->day-1)*3600*24*1000UL;
    ms += t->hours*3600UL*1000UL;
    ms += t->minutes*60*1000UL;
    ms += t->seconds*1000UL;
    ms += t->microseconds/1000UL;
    return ms;    
}

#endif // _UBLOX_H
