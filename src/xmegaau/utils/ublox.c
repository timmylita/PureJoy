/**
* \file ublox.c
* \author tom.m
*
* \see ublox.h.
*/

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

#include "project.h"
#include <ctype.h>
#include "ublox.h"
#include "maths.h"

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

#ifndef UBLOX_SPI_INSTANCE
#define UBLOX_SPI_INSTANCE                                        (&SPIE)
#endif

#ifndef UBLOX_SPI_MODE
#define UBLOX_SPI_MODE                                       (SPI_MODE_0)
#endif

#ifndef UBLOX_RawDebug
#define UBLOX_RawDebug                                                (0)
#endif

/************************************************************************/
/* Types.                                                               */
/************************************************************************/

#if (UBLOX_UBX_CFG_NMEA_Include)
typedef struct 
{
	bool    flagRead;  
	bool    flagDataReady; 
	char    words[256]; 
	int     wordIdx;
} UBLOX_NMEA_Encoder_t;
#endif

#if (UBLOX_UBX_CFG_NMEA_Include)
typedef struct
{
	UBLOX_NMEA_SentenceRmc_t rmc;
	UBLOX_NMEA_SentenceGga_t gga;
} UBLOX_NMEA_Parser_t;
#endif

/************************************************************************/
/* Private prototypes.                                                  */
/************************************************************************/

/*----------------------------------------------------------------------*/
static int
UBLOX_UBX_Read
(
    uint8_t *buf,
    int len
);

/*----------------------------------------------------------------------*/
static int
UBLOX_UBX_Write
(
    uint8_t *buf,
    int len
);

/* SPI Function prototypes. *********************************************/

/*----------------------------------------------------------------------*/
static int 
UBLOX_SPI_Read
(
    void *buf,
    const uint8_t buf_len
);

/*----------------------------------------------------------------------*/
static int 
UBLOX_SPI_Write
(
    const void *buf,
    const uint8_t buf_len
);

/* UBX Protocol Function prototypes. ************************************/

/*----------------------------------------------------------------------*/
static void
UBLOX_UBX_Checksum
(
    uint8_t *packet,
    int size
);

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_IssueCmd
(
    uint8_t *packet,
    int len
);

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_ReadNextMsg
(
    UBLOX_UBX_Msg_t *msg,
    const UBLOX_UBX_MsgType_t *expected_type
);

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_SendRequest
(
    uint8_t *buf,
    int size
);

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_Sleep
(
    void
);

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_Wake
(
    void
);

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_ResetGps
(
    char *type
);

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_ReadAck
(
    const UBLOX_UBX_MsgType_t *expected_type
);

/*----------------------------------------------------------------------*/
static bool 
UBLOX_UBX_ConfigMsg
(	
	UBLOX_UBX_CFG_Msg_t msg, 
	bool readack
);

/*----------------------------------------------------------------------*/
static bool 
ConfigMsgNmeaUbx
(
	uint8_t send_rate, 
	bool all, 
	bool readack
);

/*----------------------------------------------------------------------*/
static bool 
ConfigMsgNmeaStd
(
	uint8_t send_rate, 
	bool all, 
	bool readack
);

/*----------------------------------------------------------------------*/
static bool 
UBLOX_UBX_ConfigCfg
(
	UBLOX_UBX_CFG_Cfg_t cfg, 
	bool readack
);

/*----------------------------------------------------------------------*/
static bool 
ConfigPrt
(
	UBLOX_UBX_CFG_PRT_PortId_t port_id, 
	uint8_t in_protocol, 
	uint8_t out_protocol, 
	uint32_t baudrate, bool readack
);

/*----------------------------------------------------------------------*/
static bool 
UBLOX_UBX_ConfigPrt
(
	UBLOX_UBX_CFG_Prt_t prt, 
	bool readack
);

/*----------------------------------------------------------------------*/
static bool 
UBLOX_UBX_PollPrt
(
	UBLOX_UBX_CFG_Prt_t * prt, 
	UBLOX_UBX_CFG_PRT_PortId_t prt_id, 
	bool readack
);
/*----------------------------------------------------------------------*/
static bool 
ConfigPm2
(
	bool readack
);

/*----------------------------------------------------------------------*/
static bool 
UBLOX_UBX_ConfigPm2
(
	UBLOX_UBX_CFG_Pm2_t pm2, 
	bool readack
);

/*----------------------------------------------------------------------*/
static bool 
UBLOX_UBX_PollPm2
(
	UBLOX_UBX_CFG_Pm2_t * pm2, 
	bool readack
);

/*----------------------------------------------------------------------*/
static bool 
ConfigRate
(
	uint16_t update_rate, 
	bool readack
);

/*----------------------------------------------------------------------*/
static bool 
UBLOX_UBX_ConfigRate
(
	UBLOX_UBX_CFG_Rate_t rate, 
	bool readack
);

/*----------------------------------------------------------------------*/
static bool 
UBLOX_UBX_PollRate
(
	UBLOX_UBX_CFG_Rate_t * rate, 
	bool readack
);

/*----------------------------------------------------------------------*/
static bool 
ConfigRxm
(
	UBLOX_UBX_CFG_RXM_GpsSensitivityMode_t gps_mode, 
	UBLOX_UBX_CFG_RXM_LowPowerMode_t lp_mode, 
	bool readack
);

/*----------------------------------------------------------------------*/
static bool 
UBLOX_UBX_ConfigRxm
(
	UBLOX_UBX_CFG_Rxm_t rxm, 
	bool readack
);

/*----------------------------------------------------------------------*/
static bool 
UBLOX_UBX_PollRxm
(
	UBLOX_UBX_CFG_Rxm_t * rxm, 
	bool readack
);

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_SBAS_Include)
static bool 
ConfigSbas
(
	UBLOX_UBX_CFG_SBAS_MASK_Mode_t mode, 
	UBLOX_UBX_CFG_SBAS_MASK_Usage_t usage, 
	bool readack
);
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_SBAS_Include)
static bool 
UBLOX_UBX_ConfigSbas
(
	UBLOX_UBX_CFG_Sbas_t sbas, 
	bool readack
);
#endif

/*----------------------------------------------------------------------*/
static bool
ConfigGnss
(
	UBLOX_UBX_CFG_GNSS_ConfigBlock_t * blocks,
	uint8_t count,
	bool readacks
);

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_SBAS_Include)
static bool 
UBLOX_UBX_PollSbas
(
	UBLOX_UBX_CFG_Sbas_t * sbas, bool readack
);
#endif

/*----------------------------------------------------------------------*/
static bool 
ConfigRst
(
	UBLOX_UBX_CFG_RST_NavBbrMask_SpecialSet_t nav_bbr_mask, 
	UBLOX_UBX_CFG_RST_Type_t type, 
	bool readack
);

/*----------------------------------------------------------------------*/
static bool 
UBLOX_UBX_ConfigRst
(
	UBLOX_UBX_CFG_Rst_t rst, 
	bool readack
);

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NAV5_Include)
static bool 
ConfigNav5
(
	UBLOX_UBX_CFG_NAV5_Model_t model, 
	UBLOX_UBX_CFG_NAV5_FixMode_t fix_mode, 
	uint8_t maxsv, 
	bool readack
);
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NAV5_Include)
static bool 
UBLOX_UBX_ConfigNav5
(
	UBLOX_UBX_CFG_Nav5_t nav5, 
	bool readack
);
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NAV5_Include)
static bool 
UBLOX_UBX_PollNav5
(
	UBLOX_UBX_CFG_Nav5_t * nav5, 
	bool readack
);
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_TP5_Include)
static bool 
UBLOX_UBX_ConfigTp5
(
	UBLOX_UBX_CFG_Tp5_t tp5, 
	bool readack
);
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_TP5_Include)
static bool 
UBLOX_UBX_PollTp5
(
	UBLOX_UBX_CFG_Tp5_t * tp5, 
	UBLOX_UBX_CFG_TP5_TpIdx_t tp_idx, 
	bool readack
);
#endif

/*----------------------------------------------------------------------*/
static bool 
UBLOX_UBX_PollMonVer
(
	UBLOX_UBX_CFG_MonVer_t * mon_ver, 
	bool readack
);

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_PollNavPvt
(
	void
);

/*----------------------------------------------------------------------*/
static bool 
UBLOX_UBX_GetNavPvt
(
	UBLOX_UBX_NAV_Pvt_t * pvt, 
	bool readack
);

/* NMEA Function prototypes. ********************************************/

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static UBLOX_NMEA_SentenceId_t
UBLOX_NMEA_SentenceId
(
    const char *sentence,
    bool strict
);
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static bool
UBLOX_NMEA_ParseRmc
(
    UBLOX_NMEA_SentenceRmc_t * frame,
    const char *sentence
);
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static bool
UBLOX_NMEA_ParseGga
(
    UBLOX_NMEA_SentenceGga_t * frame,
    const char *sentence
);
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static bool
UBLOX_NMEA_ParsePubx
(
	UBLOX_NMEA_SentencePubx_t *frame,
	const char *sentence
);
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static char *
UBLOX_NMEA_Encode
(
    char c
);
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static void
UBLOX_NMEA_Read
(
    void
);
#endif

/************************************************************************/
/* Constants.                                                           */
/************************************************************************/
		
static const UBLOX_UBX_MsgType_t
	type_cfg_cfg = {UBLOX_UBX_CLASS_CFG, UBLOX_UBX_CFG_CFG};

static const UBLOX_UBX_MsgType_t
	type_cfg_prt = {UBLOX_UBX_CLASS_CFG, UBLOX_UBX_CFG_PRT};

static const UBLOX_UBX_MsgType_t
	type_cfg_pm2 = {UBLOX_UBX_CLASS_CFG, UBLOX_UBX_CFG_PM2};

static const UBLOX_UBX_MsgType_t
	type_cfg_tp5 = {UBLOX_UBX_CLASS_CFG, UBLOX_UBX_CFG_TP5};
		
static const UBLOX_UBX_MsgType_t
	type_cfg_msg = {UBLOX_UBX_CLASS_CFG, UBLOX_UBX_CFG_MSG};
		
static const UBLOX_UBX_MsgType_t
	type_cfg_rate = {UBLOX_UBX_CLASS_CFG, UBLOX_UBX_CFG_RATE};
		
static const UBLOX_UBX_MsgType_t
	type_cfg_sbas = {UBLOX_UBX_CLASS_CFG, UBLOX_UBX_CFG_SBAS};
		
static const UBLOX_UBX_MsgType_t
	type_cfg_rst = {UBLOX_UBX_CLASS_CFG, UBLOX_UBX_CFG_RST};
		
static const UBLOX_UBX_MsgType_t
	type_cfg_rxm = {UBLOX_UBX_CLASS_CFG, UBLOX_UBX_CFG_RXM};

static const UBLOX_UBX_MsgType_t
	type_cfg_mon_ver = {UBLOX_UBX_CLASS_MON, UBLOX_UBX_MON_VER};
		
static const UBLOX_UBX_MsgType_t
	type_cfg_nav5 = {UBLOX_UBX_CLASS_CFG, UBLOX_UBX_CFG_NAV5};
		
static const UBLOX_UBX_MsgType_t
	type_cfg_gnss = {UBLOX_UBX_CLASS_CFG, UBLOX_UBX_CFG_GNSS};

static const UBLOX_UBX_MsgType_t
	type_nav_pvt = {UBLOX_UBX_CLASS_NAV, UBLOX_UBX_NAV_PVT};

static const UBLOX_UBX_CFG_GNSS_ConfigBlock_t 
	gnss_cfg_table [4] = 
	{
		{
			.gnss_id  = UBLOX_UBX_CFG_GNSS_ID_Gps,
			.res_trk_ch = 0,
			.max_trk_ch = 24,
			.resereved_1 = 0,
			.flags = {0x01,0x00,0x00,0x01}			
		},
		{
			.gnss_id  = UBLOX_UBX_CFG_GNSS_ID_Sbas,
			.res_trk_ch = 0,
			.max_trk_ch = 4,
			.resereved_1 = 0,
			.flags = {0x01,0x00,0x00,0x01}	
		},
		{
			.gnss_id  = UBLOX_UBX_CFG_GNSS_ID_Galileo,
			.res_trk_ch = 0,
			.max_trk_ch = 22,
			.resereved_1 = 0,
			.flags = {0x01,0x00,0x00,0x01}	
		},
		{
			.gnss_id  = UBLOX_UBX_CFG_GNSS_ID_Glonass,
			.res_trk_ch = 0,
			.max_trk_ch = 22,
			.resereved_1 = 0,
			.flags = {0x01,0x00,0x00,0x01}	
		}
	};


/************************************************************************/
/* Private variables.                                                   */
/************************************************************************/

static struct
    spi_device UBLOX_SPI_SLAVE =
    {
        .id = SPIX_SS_UBLOX
    };
    
#if (UBLOX_UBX_CFG_NMEA_Include)
static UBLOX_NMEA_Encoder_t
    _encoder;
#endif

static UBLOX_SCALE_t
    _scale;
	
UBLOX_NMEA_SentenceRmc_t _rmc;
UBLOX_NMEA_SentenceGga_t _gga;
UBLOX_NMEA_SentencePubx_t _pubx;

/************************************************************************/
/* Macros.                                                              */
/************************************************************************/

/*----------------------------------------------------------------------*/
#define UBLOX_READ_Float(p)                                             \
    (                                                                   \
        (float)                                                         \
        (                                                               \
            ((uint32_t)(*(p+3)) << 24)                                  \
            |                                                           \
            ((uint32_t)(*(p+2)) << 16)                                  \
            |                                                           \
            ((uint32_t)(*(p+1)) << 8)                                   \
            |                                                           \
            ((uint32_t)(*(p+0)))                                        \
        )                                                               \
    )

/*----------------------------------------------------------------------*/
#define UBLOX_READ_Double(p)                                            \
    (                                                                   \
        (double)                                                        \
        (                                                               \
            ((u8)(*(p+7))<<56)                                          \
            |                                                           \
            ((u8)(*(p+6))<<48)                                          \
            |                                                           \
            ((u8)(*(p+5))<<40)                                          \
            |                                                           \
            ((u8)(*(p+4))<<32)                                          \
            |                                                           \
            ((u8)(*(p+3))<<24)                                          \
            |                                                           \
            ((u8)(*(p+2))<<16)                                          \
            |                                                           \
            ((u8)(*(p+1))<<8)                                           \
            |                                                           \
            ((u8)(*(p+0)))                                              \
        )                                                               \
    )

/*----------------------------------------------------------------------*/
#define UBLOX_SPI_Delay()                                               \
do                                                                      \
{                                                                       \
    for(volatile uint8_t i = 0; i < 0xff; i++);                         \
} while (0)

/*----------------------------------------------------------------------*/
#define READ_uint8_t(p)                                            (*(p))

/*----------------------------------------------------------------------*/
#define READ_uint16_t(p)                             ((*(p+1)<<8)|(*(p)))

/*----------------------------------------------------------------------*/
#define READ_uint32_t(p)   ((*(p+3)<<24)|(*(p+2)<<16)|(*(p+1)<<8)|(*(p)))

/*----------------------------------------------------------------------*/
#define READ_int8_t(p)                     ((signed char)READ_uint8_t(p))

/*----------------------------------------------------------------------*/
#define READ_int16_t(p)                  ((signed short)READ_uint16_t(p))

/*----------------------------------------------------------------------*/
#define READ_int32_t(p)                    ((signed int)READ_uint32_t(p))

/************************************************************************/
/* Implementations.                                                     */
/************************************************************************/

/*----------------------------------------------------------------------*/
void UBLOX_CycleScale (void)
{
    switch(_scale)
    {
        case UBLOX_SCALE_Millimeter:
            _scale = UBLOX_SCALE_Meter;
            break;
            
        case UBLOX_SCALE_Meter:
            _scale = UBLOX_SCALE_Km;
            break;
            
        case UBLOX_SCALE_Km:
            _scale = UBLOX_SCALE_Millimeter;
            break;
            
        default:
        err((PSTR("{\"ublox\":{\"scale\":%u}}"), _scale));
            break;
    }
}    

/*----------------------------------------------------------------------*/
float UBLOX_ResolveScale (void)
{
    switch(_scale)
    {
        case UBLOX_SCALE_Millimeter:
            return (1.0f);
            break;
        
        case UBLOX_SCALE_Meter:
            return (1000.0f);
            break;
        
        case UBLOX_SCALE_Km:
            return (1000000.0f);
            break;
        
        default:
            err((PSTR("{\"ublox\":{\"scale\":%u}}"), _scale));
            return (1.0f);
            break;
    }
}

/*----------------------------------------------------------------------*/
UBLOX_SCALE_t UBLOX_GetScale (void)
{
    return _scale;
}    

/* UBLOX SPI Function. **************************************************/

/*----------------------------------------------------------------------*/
void UBLOX_SPI_Init
(
    void
)
{
    ioport_configure_pin
        (SPIX_SS_UBLOX, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
    ioport_configure_pin
        (SPIE_MOSI, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
    ioport_configure_pin
        (SPIE_MISO, IOPORT_DIR_INPUT);
    ioport_configure_pin
        (SPIE_SCK, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
    ioport_configure_pin
        (GPS_RESET_B, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
    ioport_configure_pin
        (GPS_EXT_INT, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
    ioport_configure_pin
        (GPS_TIMEPULSE, IOPORT_DIR_INPUT);
    ioport_configure_pin
        (GPS_SBB, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
    
    spi_master_init
    (
        UBLOX_SPI_INSTANCE
    );
    spi_master_setup_device
    (
        UBLOX_SPI_INSTANCE,
        &UBLOX_SPI_SLAVE,
        UBLOX_SPI_MODE,
        UBLOX_SPI_BAUDRATE,
        0
    );
    spi_enable(UBLOX_SPI_INSTANCE);
}

/*----------------------------------------------------------------------*/
static int 
UBLOX_SPI_Read
(
    void *buf,
    const uint8_t buf_len
)
{
    int ret;
    spi_select_device(UBLOX_SPI_INSTANCE, &UBLOX_SPI_SLAVE);
    UBLOX_SPI_Delay();
    ret = spi_read_packet(UBLOX_SPI_INSTANCE, buf, buf_len);
    UBLOX_SPI_Delay();
    spi_deselect_device(UBLOX_SPI_INSTANCE, &UBLOX_SPI_SLAVE);
    return ret;
}

/*----------------------------------------------------------------------*/
static int 
UBLOX_SPI_Write
(
    const void *buf,
    const uint8_t buf_len
)
{
    int ret;
    spi_select_device(UBLOX_SPI_INSTANCE, &UBLOX_SPI_SLAVE);
    UBLOX_SPI_Delay();
    ret = spi_write_packet(UBLOX_SPI_INSTANCE, buf, buf_len);
    UBLOX_SPI_Delay();
    spi_deselect_device(UBLOX_SPI_INSTANCE, &UBLOX_SPI_SLAVE);
    return ret;
}

/*----------------------------------------------------------------------*/
int 
UBLOX_SPI_Ioctl
(
    const uint8_t req,
    const uint32_t arg,
    const void * argp
)
{
    switch(req)
    {
        case UBLOX_SPI_REQ_Config:
		{
			UBLOX_UBX_CFG_Msg_t cfg_nav_pvt = {type_nav_pvt, 1};
			/*if
			(
				ConfigRst
				(
					UBLOX_UBX_CFG_RST_NavBbrMask_SpecialSet_HotStart,
					UBLOX_UBX_CFG_RST_TYPE_SoftControlled,
					false
				)
			)
			{
				vbg((PSTR("{\"ublox\":{\"config\":\"RST\"}}")));
			}
			else
			{
				exc((PSTR("{\"ublox\":{\"config\":\"RST\"}}")));
			}*/

			if(ConfigPm2(false))
			{
				vbg((PSTR("{\"ublox\":{\"config\":\"PM2\"}}")));
			}
			else
			{
				exc((PSTR("{\"ublox\":{\"config\":\"PM2\"}}")));
			}

			if
			(
				ConfigPrt
				(
					UBLOX_UBX_CFG_PRT_PORT_ID_Spi,
					UBLOX_UBX_CFG_PRT_ProtoMask_ubx
					,
					UBLOX_UBX_CFG_PRT_ProtoMask_ubx
					|
					UBLOX_UBX_CFG_PRT_ProtoMask_nmea
					,
					1000,
					false
				)
			)
			{
				vbg((PSTR("{\"ublox\":{\"config\":\"PRT\"}}")));
			}
			else
			{
				exc((PSTR("{\"ublox\":{\"config\":\"PRT\"}}")));
			}
	
			if
			(
				ConfigRxm
				(
					UBLOX_UBX_CFG_RXM_GPS_SENSITIVITY_MODE_Auto,
					UBLOX_UBX_CFG_RXM_LOW_POWER_MODE_ContinuousTracking,
					false
				)
			)
			{
				vbg((PSTR("{\"ublox\":{\"config\":\"RXM\"}}")));
			}
			else
			{
				exc((PSTR("{\"ublox\":{\"config\":\"RXM\"}}")));
			}
	
			if(ConfigRate(1000, false))
			{
				vbg((PSTR("{\"ublox\":{\"config\":\"RATE\"}}")));
			}
			else
			{
				exc((PSTR("{\"ublox\":{\"config\":\"RATE\"}}")));
			}
			#if (0)
			if
			(
				ConfigNav5
				(
					UBLOX_UBX_CFG_NAV5_MODEL_Stationary,
					UBLOX_UBX_CFG_NAV5_FIX_MODE_Auto,
					31,
					false
				)
			)
			{
				vbg((PSTR("{\"ublox\":{\"config\":\"NAV5\"}}")));
			}
			else
			{
				exc((PSTR("{\"ublox\":{\"config\":\"NAV5\"}}")));
			}
			#endif			
			switch((UBLOX_SPI_DATA_Format_t)arg)
			{
				case UBLOX_SPI_DATA_FORMAT_Nmea:
				{

					if(ConfigMsgNmeaStd(1, false, false))
					{
						vbg((PSTR("{\"ublox\":{\"config\":\"NMEA_STD_MSG\"}}")));
					}
					else
					{
						exc((PSTR("{\"ublox\":{\"config\":\"NMEA_STD_MSG\"}}")));
					}
		
					if(ConfigMsgNmeaUbx(1, false, false))
					{
						vbg((PSTR("{\"ublox\":{\"config\":\"NMEA_UBX_MSG\"}}")));
					}
					else
					{
						exc((PSTR("{\"ublox\":{\"config\":\"NMEA_UBX_MSG\"}}")));
					}
					break;
				}
				case UBLOX_SPI_DATA_FORMAT_Ubx:
				{
					if(ConfigMsgNmeaStd(0, false, false))
					{
						vbg((PSTR("{\"ublox\":{\"config\":\"NMEA_STD_MSG\"}}")));
					}
					else
					{
						exc((PSTR("{\"ublox\":{\"config\":\"NMEA_STD_MSG\"}}")));
					}
					if(ConfigMsgNmeaUbx(0, false, false))
					{
						vbg((PSTR("{\"ublox\":{\"config\":\"NMEA_UBX_MSG\"}}")));
					}
					else
					{
						exc((PSTR("{\"ublox\":{\"config\":\"NMEA_UBX_MSG\"}}")));
					}
					if(ConfigGnss(gnss_cfg_table, 4, false))
					{
						vbg((PSTR("{\"ublox\":{\"config\":\"GNSS\"}}")));
					}
					else
					{
						exc((PSTR("{\"ublox\":{\"config\":\"GNSS\"}}")));
					}
					if(UBLOX_UBX_ConfigMsg(cfg_nav_pvt, false))
					{
						vbg((PSTR("{\"ublox\":{\"config\":\"MSG\"}}")));
					}
					else
					{
						exc((PSTR("{\"ublox\":{\"config\":\"MSG\"}}")));
					}
					break;
				}
				default:
					break;
			}
			break;
		}
        case UBLOX_SPI_REQ_Poll:
        {
			switch((UBLOX_SPI_POLL_Msg_t)arg)
			{
				case UBLOX_SPI_POLL_MSG_NavPvt:
				{
					UBLOX_UBX_PollNavPvt();
					break;
				}
				default:
					break;
			}
	        break;
        }
		case UBLOX_SPI_REQ_Data:
		{
			switch((UBLOX_SPI_DATA_Format_t)arg)
			{
                
                #if (UBLOX_UBX_CFG_NMEA_Include)
				case UBLOX_SPI_DATA_FORMAT_Nmea:
				{
					GNSS_Position_t * pos = (GNSS_Position_t *)(argp);
					UBLOX_NMEA_Read();
					
					if(_rmc.valid && _gga.fix_quality)
					{
						pos->coords.latitude = UBLOX_NMEA_ToCoord(&_rmc.latitude);
						pos->coords.longitude = UBLOX_NMEA_ToCoord(&_rmc.longitude);
						pos->coords.altitude = UBLOX_NMEA_ToFloat(&_gga.altitude);
						pos->coords.accuracy = UBLOX_NMEA_ToFloat(&_pubx.hacc);
						pos->coords.altitude_accuracy = UBLOX_NMEA_ToFloat(&_pubx.vacc);
						pos->coords.heading = UBLOX_NMEA_ToFloat(&_rmc.course);
						pos->coords.speed = UBLOX_NMEA_ToFloat(&_rmc.speed);
						pos->timestamp.utc = UBLOX_NMEA_ToMillis(&_rmc.date, &_rmc.time);	
					}
					
					break;
				}
                #endif
                
				case UBLOX_SPI_DATA_FORMAT_Ubx:
				{
					UBLOX_UBX_NAV_Pvt_t pvt;
					GNSS_Position_t * pos = (GNSS_Position_t *)(argp);
					
					if
					(
						UBLOX_UBX_GetNavPvt
						(
							&pvt, 
							false
						)
					)
					{
						pos->coords.latitude = (float)(pvt.lat/10000000.0f);
						pos->coords.longitude = (float)(pvt.lon/10000000.0f);
						pos->coords.altitude = (float)(pvt.h_msl/1000.0f);
						pos->coords.accuracy = (float)(pvt.h_acc/UBLOX_ResolveScale());
                        pos->coords.altitude_accuracy = (float)(pvt.v_acc/UBLOX_ResolveScale());
						pos->coords.heading = (float)(pvt.head_veh/100000.0f);
						pos->coords.speed = (float)(pvt.gspeed/1000.0f);
						pos->timestamp.utc = 0;
                        pos->timestamp.year = pvt.year;
                        pos->timestamp.month = pvt.month;
                        pos->timestamp.day = pvt.day;
                        pos->timestamp.hour = pvt.hour;
                        pos->timestamp.minute = pvt.minute;
                        pos->timestamp.second = pvt.second;
					}
					break;
				}
				default:
					break;
			}
			break;
		}
        
    case UBLOX_SPI_REQ_Sleep:
        UBLOX_UBX_Sleep();
        break;
        
    case UBLOX_SPI_REQ_Wake:
        UBLOX_UBX_Wake();
        break;
          
    default:
        break;
    }
    return 0;
}

/*----------------------------------------------------------------------*/
static int
UBLOX_UBX_Read
(
    uint8_t *buf,
    int len
)
{
    return (UBLOX_SPI_Read(buf, len) == 0)?len:0;
}

/*----------------------------------------------------------------------*/
static int
UBLOX_UBX_Write
(
    uint8_t *buf,
    int len
)
{
    return (UBLOX_SPI_Write(buf, len) == 0)?len:0;
}

/* NMEA_ Functions. *****************************************************/

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static void
UBLOX_NMEA_Read
(
	void
)
{
    uint8_t val = 0x0;
	bool rmc_flag = false, gga_flag = false;
    const char * buf;
    
    UBLOX_SPI_Read(&val, 1);
    
    // must be an ascii character or throw it away
    while(val < 0x80)
    {
        mute((PSTR("%c"), (char)val));
        buf = UBLOX_NMEA_Encode(val); 
        if(buf)
        {
            UBLOX_NMEA_SentenceId_t id
                = UBLOX_NMEA_SentenceId(buf, true);
            switch(id)
            {
                case UBLOX_NMEA_SENTENCE_ID_Rmc:
				{
					UBLOX_NMEA_SentenceRmc_t tmp;
                    mute((PSTR("#\t%s\r\n"), buf));
					if(UBLOX_NMEA_ParseRmc(&tmp, buf) && tmp.valid)
					{
						rmc_flag = true;
						memcpy(&_rmc, &tmp, sizeof(UBLOX_NMEA_SentenceRmc_t));
					}
                    break;     
				}
                case UBLOX_NMEA_SENTENCE_ID_Gga:
				{
					UBLOX_NMEA_SentenceGga_t tmp;
                    mute((PSTR("#\t%s\r\n"), buf));
					if(UBLOX_NMEA_ParseGga(&tmp, buf) && tmp.fix_quality)
					{
						gga_flag = true;
						memcpy(&_gga, &tmp, sizeof(UBLOX_NMEA_SentenceGga_t));
					}
					break;
				}
                case UBLOX_NMEA_SENTENCE_ID_Pubx:
				{
					UBLOX_NMEA_SentencePubx_t tmp;
                    out((PSTR("#\t%s\r\n"), buf));
                    UBLOX_NMEA_ParsePubx(&tmp, buf);
                    break;
				}
				
                default:
                    break;
            }
        }
		
		if(rmc_flag && gga_flag)
		{
			break;
		}

		UBLOX_SPI_Read(&val, 1);  
    }
}
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static int
UBLOX_NMEA_Hex2Int
(
    char c
)
{
	if (c >= '0' && c <= '9')
	{
		return c - '0';
	}
	if (c >= 'A' && c <= 'F')
	{
		return c - 'A' + 10;
	}
	if (c >= 'a' && c <= 'f')
	{
		return c - 'a' + 10;
	}
	return -1;
}

#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static bool
UBLOX_NMEA_Checksum
(
    const char *sentence,
    bool strict
)
{
	uint8_t checksum = 0x00;

	// Sequence length is limited.
	if (strlen(sentence) > UBLOX_NMEA_MAX_LENGTH + 3)
	{
		return false;
	}

	// A valid sentence starts with "$".
	if (*sentence++ != '$')
	{
		return false;
	}

	// The optional checksum is an XOR of all bytes between "$" and "*".
	while (*sentence && *sentence != '*' && isprint((unsigned char) *sentence))
	{
		checksum ^= *sentence++;
	}

	// If checksum is present...
	if (*sentence == '*') 
	{
		// Extract checksum.
		sentence++;
		int upper = UBLOX_NMEA_Hex2Int(*sentence++);
		if (upper == -1)
		{
			return false;
		}
		int lower = UBLOX_NMEA_Hex2Int(*sentence++);
		if (lower == -1)
		{
			return false;
		}
		int expected = upper << 4 | lower;

		// Check for checksum mismatch.
		if (checksum != expected)
		{
				return false;
		}
	} 
	else if (strict) 
	{
		// Discard non-checksummed frames in strict mode.
		return false;
	}

	// The only stuff allowed at this point is a newline.
	if (*sentence && strcmp(sentence, "\n") && strcmp(sentence, "\r\n"))
	{
		return false;
	}

	return true;
}
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static inline
bool UBLOX_NMEA_IsField
(
    char c
) 
{
	return isprint((unsigned char) c) && c != ',' && c != '*';
}
#endif

/**
 * Scanf-like processor for NMEA sentences. Supports the following formats:
 * c - single character (char *)
 * d - direction, returned as 1/-1, default 0 (int *)
 * f - fractional, returned as value + scale (int *, int *)
 * i - decimal, default zero (int *)
 * s - string (char *)
 * t - talker identifier and type (char *)
 * T - date/time stamp (int *, int *, int *)
 * Returns true on success. See library source code for details.
 */
#if (UBLOX_UBX_CFG_NMEA_Include)
static bool
UBLOX_NMEA_Scan
(
    const char *sentence,
    const char *format,
    ...
)
{
  bool result = false;
  bool optional = false;
  va_list ap;
  va_start(ap, format);

	const char *field = sentence;
#define next_field() \
	do \
	{ \
		/* Progress to the next field. */ \
		while (UBLOX_NMEA_IsField(*sentence)) \
		{ \
			sentence++; \
		} \
		/* Make sure there is a field there. */ \
		if (*sentence == ',') \
		{ \
			sentence++; \
			field = sentence; \
		} \
		else \
		{ \
			field = NULL; \
		} \
	} while (0)

  while (*format) 
	{
    char type = *format++;

    if (type == ';') 
		{
      // All further fields are optional.
      optional = true;
      continue;
    }

    if (!field && !optional) 
		{
      // Field requested but we ran out if input. Bail out.
      goto parse_error;
    }

    switch (type) 
		{
      case 'c': 
			{ 
				// Single character field (char).
        char value = '\0';

        if (field && UBLOX_NMEA_IsField(*field))
				{
					value = *field;
				}

        *va_arg(ap, char *) = value;
				break;
      } 

      case 'd': 
			{ 
				// Single character direction field (int).
        int value = 0;

        if (field && UBLOX_NMEA_IsField(*field)) 
				{
	        switch (*field) 
					{
		        case 'N':
		        case 'E':
		        value = 1;
		        break;
		        case 'S':
		        case 'W':
		        value = -1;
		        break;
		        default:
		        goto parse_error;
	        }
        }

        *va_arg(ap, int *) = value;
        break;
      } 

      case 'f': 
			{ 
				// Fractional value with scale (NMEA_Float_t).
        int sign = 0;
        int_least32_t value = -1;
        int_least32_t scale = 0;

        if (field) 
				{
          while (UBLOX_NMEA_IsField(*field)) 
					{
            if (*field == '+' && !sign && value == -1) 
						{
							sign = 1;
            } 
						else if (*field == '-' && !sign && value == -1) 
						{
							sign = -1;
            } 
						else if (isdigit((unsigned char) *field)) 
						{
              int digit = *field - '0';
              if (value == -1)
							{
								value = 0;
							}
              if (value > (INT_LEAST32_MAX-digit) / 10) 
							{
                /* we ran out of bits, what do we do? */
                if (scale) 
								{
                  /* truncate extra precision */
                  break;
                } 
								else 
								{
                  /* integer overflow. bail out. */
                  goto parse_error;
                }
              }
              value = (10 * value) + digit;
              if (scale)
							{
								scale *= 10;
							}
            } 
						else if (*field == '.' && scale == 0) 
						{
							scale = 1;
            } 
						else if (*field == ' ') 
						{
              /* Allow spaces at the start of the field. Not NMEA
                * conformant, but some modules do this. */
              if (sign != 0 || value != -1 || scale != 0)
							{
								goto parse_error;
							}
            } 
						else 
						{
							goto parse_error;
            }
            field++;
          }
        }

        if ((sign || scale) && value == -1)
				{
					goto parse_error;
				}
					
        if (value == -1) 
				{
          /* No digits were scanned. */
          value = 0;
          scale = 0;
        } 
				else if (scale == 0) 
				{
          /* No decimal point. */
          scale = 1;
        }
        if (sign)
				{
					value *= sign;
				}

        *va_arg(ap, UBLOX_NMEA_Float_t *) = (UBLOX_NMEA_Float_t) {value, scale};
				break;
      }

      case 'i': 
			{ 
				// Integer value, default 0 (int).
        int value = 0;

        if (field) 
				{
          char *endptr;
          value = strtol(field, &endptr, 10);
          if (UBLOX_NMEA_IsField(*endptr))
					{
						goto parse_error;
					}
        }

        *va_arg(ap, int *) = value;				
				break;
      } 

      case 's': 
			{ 
				// String value (char *).
        char *buf = va_arg(ap, char *);

        if (field) 
				{
          while (UBLOX_NMEA_IsField(*field))
					{
						*buf++ = *field++;
					}
        }

        *buf = '\0';
				break;
      } 

      case 't': 
			{ 
				// NMEA talker+sentence identifier (char *).
        // This field is always mandatory.
        if (!field)
				{
					goto parse_error;
				}

        if (field[0] != '$')
				{
					goto parse_error;
				}
        for (int f=0; f<5; f++)
				{
          if (!UBLOX_NMEA_IsField(field[1+f]))
					{
						goto parse_error;
					}
				}

        char *buf = va_arg(ap, char *);
        memcpy(buf, field+1, 5);
        buf[5] = '\0';
				break;
      } 

      case 'D': 
			{ 
				// Date (int, int, int), -1 if empty.
        UBLOX_NMEA_Date_t*date = va_arg(ap, UBLOX_NMEA_Date_t*);

        int d = -1, m = -1, y = -1;

        if (field && UBLOX_NMEA_IsField(*field)) 
				{
          // Always six digits.
          for (int f=0; f<6; f++)
					{
            if (!isdigit((unsigned char) field[f]))
						{
							goto parse_error;
						}
					}

          char dArr[] = {field[0], field[1], '\0'};
          char mArr[] = {field[2], field[3], '\0'};
          char yArr[] = {field[4], field[5], '\0'};
          d = strtol(dArr, NULL, 10);
          m = strtol(mArr, NULL, 10);
          y = strtol(yArr, NULL, 10);
        }

        date->day = d;
        date->month = m;
        date->year = y;
				break;
      } 

      case 'T': 
			{ 
				// Time (int, int, int, int), -1 if empty.
        UBLOX_NMEA_Time_t *time_ = va_arg(ap, UBLOX_NMEA_Time_t *);

        int h = -1, i = -1, s = -1, u = -1;

        if (field && UBLOX_NMEA_IsField(*field)) 
				{
          // Minimum required: integer time.
          for (int f=0; f<6; f++)
					{
            if (!isdigit((unsigned char) field[f]))
						{
							goto parse_error;
						}
					}

          char hArr[] = {field[0], field[1], '\0'};
          char iArr[] = {field[2], field[3], '\0'};
          char sArr[] = {field[4], field[5], '\0'};
          h = strtol(hArr, NULL, 10);
          i = strtol(iArr, NULL, 10);
          s = strtol(sArr, NULL, 10);
          field += 6;

          // Extra: fractional time. Saved as microseconds.
          if (*field++ == '.') 
					{
            int32_t value = 0;
            int32_t scale = 1000000;
            while (isdigit((unsigned char) *field) && scale > 1) 
						{
              value = (value * 10) + (*field++ - '0');
              scale /= 10;
            }
            u = value * scale;
          } 
					else 
					{
						u = 0;
          }
        }

        time_->hours = h;
        time_->minutes = i;
        time_->seconds = s;
        time_->microseconds = u;
				break;
      } 

      case '_': 
			{ 
				// Ignore the field.
				break;
      } 

      default: 
			{ 
				// Unknown.
        goto parse_error;
				break;
      } 
    }

    next_field();
	}

	result = true;

parse_error:
  va_end(ap);
  return result;
}
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static UBLOX_NMEA_SentenceId_t
UBLOX_NMEA_SentenceId
(
    const char *sentence,
    bool strict
)
{
	if (!UBLOX_NMEA_Checksum(sentence, strict))
	{
		return UBLOX_NMEA_SENTENCE_ID_Invalid;
	}

	char type[6];
	
	if (!UBLOX_NMEA_Scan(sentence, "t", type))
	{
		return UBLOX_NMEA_SENTENCE_ID_Invalid;
	}

	if (!strcmp(type+2, "RMC"))
	{
		return UBLOX_NMEA_SENTENCE_ID_Rmc;
	}
	if (!strcmp(type+2, "GGA"))
	{
		return UBLOX_NMEA_SENTENCE_ID_Gga;
	}
	if (!strcmp(type, "PUBX"))
	{
		return UBLOX_NMEA_SENTENCE_ID_Pubx;
	}

	return UBLOX_NMEA_SENTENCE_ID_Unknown;
}
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static bool
UBLOX_NMEA_ParseRmc
(
    UBLOX_NMEA_SentenceRmc_t * frame,
    const char *sentence
)
{
	// $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
	bool ret = false;
	char type[6];
	char validity;
	int latitude_direction;
	int longitude_direction;
	int variation_direction;
	if (UBLOX_NMEA_Scan(sentence, "tTcfdfdffDfd",
						type,
						&frame->time,
						&validity,
						&frame->latitude, &latitude_direction,
						&frame->longitude, &longitude_direction,
						&frame->speed,
						&frame->course,
						&frame->date,
						&frame->variation, &variation_direction))
	{
		if (!strcmp(type+2, "RMC"))
		{
			frame->valid = (validity == 'A');
			frame->latitude.value *= latitude_direction;
			frame->longitude.value *= longitude_direction;
			frame->variation.value *= variation_direction;
			ret = true;
		}
	}	
	
	return ret;
}
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static bool
UBLOX_NMEA_ParseGga
(
    UBLOX_NMEA_SentenceGga_t * frame,
    const char *sentence
)
{
	// $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
	bool ret = false;
	char type[6];
	int latitude_direction;
	int longitude_direction;
	if (UBLOX_NMEA_Scan(sentence, "tTfdfdiiffcfci_",
						type,
						&frame->time,
						&frame->latitude, &latitude_direction,
						&frame->longitude, &longitude_direction,
						&frame->fix_quality,
						&frame->satellites_tracked,
						&frame->hdop,
						&frame->altitude, &frame->altitude_units,
						&frame->height, &frame->height_units,
						&frame->dgps_age))
	{
		if (!strcmp(type+2, "GGA"))
		{
			frame->latitude.value *= latitude_direction;
			frame->longitude.value *= longitude_direction;
			ret = true;
		}
	}	
	
	return ret;
}
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static bool
UBLOX_NMEA_ParsePubx
(
	UBLOX_NMEA_SentencePubx_t * frame,
	const char *sentence
)
{
	bool ret = false;
	char type[6];
	int msg_id;

	if (UBLOX_NMEA_Scan(sentence, "ti",
						type,
						&msg_id))
	{
		if (!strcmp(type, "PUBX"))
		{
			switch(msg_id)
			{
				case 0:
				{
					// $PUBX,00,081350.00,4717.113210,N,00833.915187,E,546.589,G3,2.1,2.0,0.007,77.52,0.007,,0.92,1.19,0.77,9,0,0*5F
					int latitude_direction;
					int longitude_direction;
					char nav_status[2];
					
					if (UBLOX_NMEA_Scan(sentence, "__Tfdfdfsffff_",
										&frame->time,
										&frame->latitude, &latitude_direction,
										&frame->longitude, &longitude_direction,
										&frame->altitude,
										nav_status,
										&frame->hacc,
										&frame->vacc,
										&frame->sog,
										&frame->cog										
										))
					{
						frame->latitude.value *= latitude_direction;
						frame->longitude.value *= longitude_direction;
						frame->status = (!strcmp(nav_status, "G3") 
										|| 
										!strcmp(nav_status, "G2"))?
										true:false;
						ret = true;
					}
					break;
				}
				case 4:
				{
					// $PUBX,04,073731.00,091202,113851.00,1196,15D,1930035,-2660.664,43,*3C
					if (UBLOX_NMEA_Scan(sentence, "__TT_",
										&frame->time,
										&frame->date
					))
					{
						ret = true;
					}
					break;
				}
				default:
				{
					break;
				}
			}
		}
	}

	return ret;
}
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NMEA_Include)
static char *
UBLOX_NMEA_Encode
(
    char c
)
{
    
	if (c == '$') 
	{
		_encoder.flagRead = true;
		_encoder.wordIdx = 0;
	}

	if (_encoder.flagRead) 
	{
		// check ending
		if (c == '\r' || c== '\n') 
		{
			// catch last ending item too
			_encoder.words[_encoder.wordIdx] = 0;
			
			// sentence complete, read done
			_encoder.flagRead = false;
			
			return _encoder.words;
		} 
		else 
		{
			_encoder.words[_encoder.wordIdx] = c;			
			_encoder.wordIdx++;
		}
	}
	return 0;
}
#endif
    
/* UBX protocol functions. **********************************************/
    
/************************************************************************
 * <buf>: checksum must be set already.
 */
static bool
UBLOX_UBX_SendRequest
(
    uint8_t *buf,
    int size
)
{
    #if (UBLOX_RawDebug)
    print((PSTR("# tx -> ")));
    for (uint16_t i = 0; i < size; i++)
    {
        switch(i)
        {
            case 2:
            case 4:
                print((PSTR("| ")));
                break;
            case 6:
                if (size > UBLOX_UBX_HEADER_LENGTH)
                {
                    print((PSTR("| ")));
                }
                break;
        }
        if (i == (size - 2))
        {
            print((PSTR("_ ")));
        }
        print((PSTR("0x%02x "), buf[i]));
    }
    print((PSTR("\r\n")));
    #endif
    
    int written = 0;
    while (true) 
    {
        if
        (
            (written = UBLOX_UBX_Write(&buf[written], size))
            <=
            0
        )
        {
            return false;
        }
        size -= written;
        if (size == 0)
        {
            break;
        }
        buf += written;
    }
    return true;
}

/************************************************************************
 * NOTE: the packet must end with 2 chars of checksum at count size and
 * size+1.
 */
static void
UBLOX_UBX_Checksum
(
    uint8_t *packet,
    int size
)
{
    uint32_t a = 0x00;
    uint32_t b = 0x00;
    int i = 0;
    while(i<size) 
    {
        a += packet[i++];
        b += a;
    }
    packet[size] = a & 0xFF;
    packet[size+1] = b & 0xFF;
}

/************************************************************************
 * If the read size < expected length, retry even if EOF.
 */
static bool
read_fixed_len
(
    uint8_t *buf,
    int expected_len
)
{
    int len = 0, count;

    while (true) {
        count = UBLOX_UBX_Read(&buf[len], expected_len);
        if (count == expected_len) {
            return true;
        } else if (count <= 0) {
            return false;
        } else {
            len += count;
            expected_len -= count;
            if (expected_len == 0)
                return true;
        }
    }
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_read_header
(
    uint8_t *buf,
    int next_len
)
{
    uint8_t c;
    int status = 0;
    int ret;

    while (1) 
    {
        if ((ret = UBLOX_UBX_Read(&c, 1)) != 1) 
        {
            return false;
        }

        if (c == 0xB5) 
        {
            status = 1;
        } 
        else if (status == 1) 
        {
            if (c == 0x62)
                break;
            else 
            {
                continue;
            }
        }
    }

    /* class, id, length: 1 + 1 + 2, or full message -- ack */
    if (! read_fixed_len(buf, next_len)) 
    {
        return false;
    }

    return true;
}

/************************************************************************
 * When messages from the Class CFG are sent to the receiver, the
 * receiver will send an Acknowledge (ACK-ACK (0x05 0x01)) or a Not
 * Acknowledge (ACK-NAK (0x05 0x00)) message back to the sender,
 * depending on whether or not the message was processed correctly.
 *
 * There is no ACK/NAK mechanism for message poll_suspending requests
 * outside Class CFG.
 */
static bool
UBLOX_UBX_ReadAck
(
    const UBLOX_UBX_MsgType_t *expected_type
)
{
    static uint8_t ack_buf[8];

    if (! UBLOX_UBX_read_header(ack_buf, 8))
    {
        return false;
    }
    
    #if (UBLOX_RawDebug)
    print((PSTR("# <- rx ")));
    for (uint8_t i = 0; i < 8; i++)        
    {
        if(0 == (i%2) && (i > 0))
        {
            print((PSTR("| 0x%02x "), ack_buf[i]));
        }
        else
        {
            print((PSTR("0x%02x "), ack_buf[i]));
        }        
    }
    if
    (
        UBLOX_UBX_CLASS_ACK == ack_buf[0]
        &&
        0x02 == ack_buf[2]
        &&
        0x00 == ack_buf[3]
        &&
        expected_type->class == ack_buf[4]
        &&
        expected_type->id == ack_buf[5]
    )
    {
        if (UBLOX_UBX_ACK_ACK == ack_buf[1])
        {
            print((PSTR(" --- ack --- \r\n")));
        }
        else if (UBLOX_UBX_ACK_NAK == ack_buf[1])
        {
            print((PSTR(" --- nak ---\r\n")));
        }
        else
        {
            print((PSTR(" --- err ---\r\n")));
        }
    }
    #endif

    if
    (
        UBLOX_UBX_CLASS_ACK == ack_buf[0]
        &&
        0x02 == ack_buf[2]
        &&
        0x00 == ack_buf[3]
        &&
        expected_type->class == ack_buf[4]
        &&
        expected_type->id == ack_buf[5]
    ) 
    {
        if (UBLOX_UBX_ACK_ACK == ack_buf[1])
        {
            return (true);
        }
    }
    return (false);
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_ReadNextMsg
(
    UBLOX_UBX_Msg_t *msg,
    const UBLOX_UBX_MsgType_t *expected_type
)
{
    static uint8_t buf[1024];

    if (! UBLOX_UBX_read_header(buf, 4))
    {
        return false;
    }

    msg->class = buf[0];
    msg->id = buf[1];

    /* LITTLE ENDIAN ALWAYS */
    msg->payload_len = buf[2] | (buf[3] << 8);

    /* payload length + checksum(2) */
    if (! read_fixed_len(buf, msg->payload_len + 2)) 
    {
        return false;
    }

    msg->payload_addr = buf;
    msg->checksum[0] = buf[msg->payload_len];
    msg->checksum[1] = buf[msg->payload_len + 1];

    /* Previous operations may leave garbage in the out buffer. */
    if
    (
        expected_type != NULL
        &&
        (
            msg->class != expected_type->class
            ||
            msg->id != expected_type->id
        )
    ) 
    {
        return false;
    }
    
    #if (UBLOX_RawDebug)
    print
    ((
        PSTR ("# <- rx 0xb5 0x62 | 0x%02x 0x%02x | 0x%02x 0x%02x |"),
        msg->class,
        msg->id,
        msg->payload_len & 0xff,
        (msg->payload_len >> 8) & 0xff
    ));
    for (uint16_t i = 0; i < msg->payload_len; i++)
    {
        print((PSTR(" 0x%02x"), msg->payload_addr[i]));
    }
    print
    ((
        PSTR(" _ 0x%02x 0x%02x\r\n"),
        msg->checksum[0],
        msg->checksum[1]
    ));
    #endif
    return true;
}


/************************************************************************
 * Send a command.
 * The response content is ignored
 */
static bool
UBLOX_UBX_IssueCmd
(
    uint8_t *packet,
    int len
)
{
    UBLOX_UBX_Checksum(&packet[2], len - 4);
    if (! UBLOX_UBX_SendRequest(packet, len)) 
    {
        return false;
    }
    return true;
}


/*----------------------------------------------------------------------*/
static bool UBLOX_UBX_Sleep (void)
{
    #if (UBLOX_UBX_SleepByCommand)
    uint8_t packet[] =
    {
        0xB5, 0x62, // Header.
        0x02, // Class.
        0x41, // ID.
        0x08, 0x00,  // Length.
        0x00, 0x00, 0x00, 0x00, // Payload: duration.
        0x00, 0x00, 0x00, 0x02, // Payload: flags.
        0x00, // Check A.
        0x00 // Check B.
    };
    return UBLOX_UBX_IssueCmd(packet, sizeof(packet));
    #else
    ioport_set_pin_level(GPS_EXT_INT, false);
    return true;
    #endif
}

/*----------------------------------------------------------------------*/
static bool UBLOX_UBX_Wake (void)
{
    ioport_set_pin_level(GPS_EXT_INT, true);
    return true;
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_ResetGps
(
    char *type
)
{
    uint16_t bbr;
    if (strcmp(type, "hot") == 0)
    {
        bbr = 0x0000;
    }
    else if (strcmp(type, "warm") == 0)
    {
        bbr = 0x0001;
    }
    else if (strcmp(type, "cold") == 0)
    {
        bbr = 0xFFFF;
    }
    else
    {
        err((PSTR("{\"ublox\":{\"reset\":\"%s\"}}"), type));
        return false;
    }

    return ConfigRst(bbr, 0x02, false);
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_ConfigMsg
(
	UBLOX_UBX_CFG_Msg_t msg,
	bool readack
)
{
    uint8_t packet[16] = {
	    0xB5, 0x62,
	    type_cfg_msg.class, type_cfg_msg.id,
	    0x08, 0x00 /* length */
    };
	
	packet[6] = msg.type.class; 
	packet[7] = msg.type.id;
	packet[8] = 0;				/*target 0 - I2C*/
	packet[9] = 0;				/*target 1 - UART1*/
	packet[10] = 0;				/*target 2 - UART2*/
	packet[11] = 0;				/*target 3 - USB*/
	packet[12] = msg.rate;		/*target 4 - SPI*/
	packet[13] = 0;

    return 
	UBLOX_UBX_IssueCmd
	(
		packet, 
		sizeof(packet)) 
		&& 
		(! readack || UBLOX_UBX_ReadAck(&type_cfg_msg)
	);
}

/*----------------------------------------------------------------------*/
static bool
ConfigMsgNmeaUbx
(
	uint8_t send_rate,
	bool all,
	bool readack
)
{
    /* class=0xf1, id={0x00, 0x01, 0x03, 0x04} */
    int i, off;
    uint8_t val;
    uint8_t ids[] = {0x00, 0x01, 0x03, 0x04};
    bool flag = true, enable[] = {true, false, false, true};

    for (i=0; i<5; i++) {
	    uint8_t packet[16] = {
		    0xB5, 0x62,
		    type_cfg_msg.class, type_cfg_msg.id,
		    0x08, 0x00
	    };
	    off = 6;
	    val = all? send_rate : (enable[i]? send_rate : 0x00);
	    packet[off] = 0xF1;		/* NMEA ubx msg class*/
	    packet[off+1] = ids[i];
	    packet[off+2] = 0;		/*target 0 - I2C*/
	    packet[off+3] = 0;		/*target 1 - UART1*/
	    packet[off+4] = 0;		/*target 2 - UART2*/
	    packet[off+5] = 0;		/*target 3 - USB*/
	    packet[off+6] = val;	/*target 4 - SPI*/
	    packet[off+7] = 0;	
	    flag = UBLOX_UBX_IssueCmd
				(
					packet, 
					sizeof(packet)) 
					&& 
					(! readack || UBLOX_UBX_ReadAck(&type_cfg_msg)
				);
    }

    return flag;
}

/*----------------------------------------------------------------------*/
static bool
ConfigMsgNmeaStd
(
	uint8_t send_rate,
	bool all,
	bool readack
)
{
    /* class=0xf0, id={0x00 - 0x0a} */
    int i, off;
    uint8_t val;
    uint8_t ids[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a};
    bool flag = true, enable[] = {true, false, false, false, true, false, false, false, false, false, false};

    for (i=0; i<11; i++) {
	    uint8_t packet[16] = {
		    0xB5, 0x62,
		    type_cfg_msg.class, type_cfg_msg.id,
		    0x08, 0x00
	    };
	    off = 6;
	    val = all? send_rate : (enable[i]? send_rate : 0x00);
	    packet[off] = 0xF0;		/* NMEA std msg class*/
	    packet[off+1] = ids[i];
	    packet[off+2] = 0;		/*target 0 - I2C*/
	    packet[off+3] = 0;		/*target 1 - UART1*/
	    packet[off+4] = 0;		/*target 2 - UART2*/
	    packet[off+5] = 0;		/*target 3 - USB*/
		packet[off+6] = val;	/*target 4 - SPI*/
		packet[off+7] = 0;  
	    flag = UBLOX_UBX_IssueCmd
				(
					packet, 
					sizeof(packet)) 
					&& 
					(! readack || UBLOX_UBX_ReadAck(&type_cfg_msg)
				);
    }

    return flag;
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_ConfigCfg
(
	UBLOX_UBX_CFG_Cfg_t cfg,
	bool readack
)
{
    uint8_t packet[6+sizeof(UBLOX_UBX_CFG_Cfg_t)+2] = {
	    0xB5, 0x62,
	    type_cfg_cfg.class, type_cfg_cfg.id,
	    sizeof(UBLOX_UBX_CFG_Cfg_t)%255, sizeof(UBLOX_UBX_CFG_Cfg_t)/255 /* length */
    };

    /* Adjust endianess byte arrangement*/
    cfg.clear_mask.value = (uint32_t)READ_uint32_t(&cfg.clear_mask.value);
    cfg.load_mask.value = (uint32_t)READ_uint32_t(&cfg.load_mask.value);
    cfg.save_mask.value = (uint32_t)READ_uint32_t(&cfg.save_mask.value);

    memcpy(&packet[6], &cfg, sizeof(UBLOX_UBX_CFG_Cfg_t));

    return 
	UBLOX_UBX_IssueCmd
	(
		packet, 
		sizeof(packet)) 
		&& 
		(! readack || UBLOX_UBX_ReadAck(&type_cfg_msg)
	);	
}

/*----------------------------------------------------------------------*/
static bool
ConfigPrt
(
	UBLOX_UBX_CFG_PRT_PortId_t port_id,
	uint8_t in_protocol,
	uint8_t out_protocol,
	uint32_t baudrate, bool readack
)
{
    bool flag = false;
    uint8_t packet[28] = {
	    0xB5, 0x62,
	    type_cfg_prt.class, type_cfg_prt.id,
	    0x14, 0x00, /* length */
	    port_id & 0xFF, /* port ID */
	    0x00, 0x00, 0x00, /* reserved */
	    0x00, 0x08, 0x00, 0x00, /* mode */
	    0x00, 0x00, 0x00, 0x00, /* baud rate */
	    in_protocol & 0xFF, 0x00, /* in protocol */
	    out_protocol & 0xFF, 0x00, /* out protocol */
	    0x02, 0x00, /* flags */
	    0x00, 0x00, /* pad */
	    0x00, 0x00 /* checksum */
    };
	
    if
	(
		UBLOX_UBX_IssueCmd
		(
			packet, 
			sizeof(packet)
		) 
		&& 
		(! readack || UBLOX_UBX_ReadAck(&type_cfg_prt))
	)
    {
	    flag = true;
    }

    return flag;	
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_ConfigPrt
(
	UBLOX_UBX_CFG_Prt_t prt,
	bool readack
)
{
    bool flag = false;
    uint8_t packet[6+sizeof(UBLOX_UBX_CFG_Prt_t)+2] = {
	    0xB5, 0x62,
	    type_cfg_prt.class, type_cfg_prt.id,
	    sizeof(UBLOX_UBX_CFG_Prt_t)%255, sizeof(UBLOX_UBX_CFG_Prt_t)/255 /* length */
    };

    /* Adjust endianess byte arrangement*/
    prt.port_id = (uint8_t)READ_uint8_t(&prt.port_id);
    prt.tx_ready.value = (uint16_t)READ_uint16_t(&prt.tx_ready.value);
    prt.baudrate = (uint32_t)READ_uint32_t(&prt.baudrate);
    prt.in_proto_mask.value = (uint16_t)READ_uint16_t(&prt.in_proto_mask.value);
    prt.out_proto_mask.value = (uint16_t)READ_uint16_t(&prt.out_proto_mask.value);
    prt.flags.value = (uint16_t)READ_uint16_t(&prt.flags.value);

    memcpy(&packet[6], &prt, sizeof(UBLOX_UBX_CFG_Prt_t));

    if
	(
		UBLOX_UBX_IssueCmd
		(
			packet, 
			sizeof(packet)
		) 
		&& 
		(! readack || UBLOX_UBX_ReadAck(&type_cfg_prt))
	)
    {
	    flag = true;
    }

    return flag;
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_PollPrt
(
	UBLOX_UBX_CFG_Prt_t * prt,
	UBLOX_UBX_CFG_PRT_PortId_t prt_id,
	bool readack
)
{
    bool flag = false;
    uint8_t packet[9] =
    {
	    0xB5, 0x62,
	    type_cfg_prt.class, type_cfg_prt.id,
	    0x01, 0x00,
	    prt_id & 0xff,
	    0x00, 0x00
    };

    UBLOX_UBX_Msg_t msg;

    memset(&msg, 0x00, sizeof(msg));
    
    if
	(
		UBLOX_UBX_IssueCmd
		(
			packet, 
			sizeof(packet)
		) 
		&& 
		UBLOX_UBX_ReadNextMsg
		(
			&msg, 
			&type_cfg_prt
		) 
		&& 
		(! readack || UBLOX_UBX_ReadAck(&type_cfg_prt))
	)
    {
	    memcpy(prt, msg.payload_addr, sizeof(UBLOX_UBX_CFG_Prt_t));

	    /* Adjust endianess byte arrangement*/
	    prt->port_id = (uint8_t)READ_uint8_t(&prt->port_id);
	    prt->tx_ready.value = (uint16_t)READ_uint16_t(&prt->tx_ready.value);
	    prt->baudrate = (uint32_t)READ_uint32_t(&prt->baudrate);
	    prt->in_proto_mask.value = (uint16_t)READ_uint16_t(&prt->in_proto_mask.value);
	    prt->out_proto_mask.value = (uint16_t)READ_uint16_t(&prt->out_proto_mask.value);
	    prt->flags.value = (uint16_t)READ_uint16_t(&prt->flags.value);

	    flag = true;
    }
    return flag;	
}

/*----------------------------------------------------------------------*/
static bool
ConfigPm2
(
	bool readack
)
{
	uint8_t packet[52] = {
		0xB5, 0x62,
		type_cfg_pm2.class, type_cfg_pm2.id,
		0x2C, 0x00, /* len */
		0x01, /* version */
		0x00, /* reserved1 */
		0x00, /* max time to spend in acq state disable */
		0x00, /* reserved2 */
		0x00, 0x00, 0x00, 0x00, /* PSM configuration flags */
		0x00, 0x00, 0x00, 0x00, /* position update period(ms) */
		0x00, 0x00, 0x00, 0x00, /* acq retry period(ms) if previously failed */
		0x00, 0x00, 0x00, 0x00, /* grid offset relative to GPS start of week */
		0x00, 0x00, /* time to stay in tracking state */
		0x00, 0x00, /* minimal search time */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00  /* checksum */
	};

	return 
	UBLOX_UBX_IssueCmd
	(
		packet, 
		sizeof(packet)
	) 
	&& 
	(!readack || UBLOX_UBX_ReadAck(&type_cfg_pm2));
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_ConfigPm2
(
	UBLOX_UBX_CFG_Pm2_t pm2,
	bool readack
)
{
    uint8_t packet[6+sizeof(UBLOX_UBX_CFG_Pm2_t)+2] = {
	    0xB5, 0x62,
	    type_cfg_pm2.class, type_cfg_pm2.id,
	    sizeof(UBLOX_UBX_CFG_Pm2_t)%255, sizeof(UBLOX_UBX_CFG_Pm2_t)/255 /* length */
    };

    /* Adjust endianess byte arrangement*/
    pm2.version = (uint8_t)READ_uint8_t(&pm2.version);
    pm2.max_startup_state_dur = (uint8_t)READ_uint8_t(&pm2.max_startup_state_dur);
    pm2.flags.value = (uint32_t)READ_uint32_t(&pm2.flags.value);
    pm2.update_period = (uint32_t)READ_uint32_t(&pm2.update_period);
    pm2.search_period = (uint32_t)READ_uint32_t(&pm2.search_period);
    pm2.grid_offset = (uint32_t)READ_uint32_t(&pm2.grid_offset);
    pm2.on_time = (uint16_t)READ_uint16_t(&pm2.on_time);
    pm2.min_acq_time = (uint16_t)READ_uint16_t(&pm2.min_acq_time);

    memcpy(&packet[6], &pm2, sizeof(UBLOX_UBX_CFG_Pm2_t));

    return 
	UBLOX_UBX_IssueCmd
	(
		packet, 
		sizeof(packet)
	) 
	&& 
	(! readack || UBLOX_UBX_ReadAck(&type_cfg_pm2));	
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_PollPm2
(
	UBLOX_UBX_CFG_Pm2_t * pm2,
	bool readack
)
{
    bool flag = false;
    uint8_t packet[8] =
    {
	    0xB5, 0x62,
	    type_cfg_pm2.class, type_cfg_pm2.id,
	    0x00, 0x00,
	    0x00, 0x00
    };
    
    UBLOX_UBX_Msg_t msg;

    memset(&msg, 0x00, sizeof(msg));

    if
	(
		UBLOX_UBX_IssueCmd
		(
			packet, 
			sizeof(packet)
		) 
		&& 
		UBLOX_UBX_ReadNextMsg
		(
			&msg, 
			&type_cfg_pm2
		) 
		&& 
		(! readack || UBLOX_UBX_ReadAck(&type_cfg_pm2))
	)
    {
	    memcpy(pm2, msg.payload_addr, sizeof(UBLOX_UBX_CFG_Pm2_t));

	    /* Adjust endianess byte arrangement*/
	    pm2->version = (uint8_t)READ_uint8_t(&pm2->version);
	    pm2->max_startup_state_dur = (uint8_t)READ_uint8_t(&pm2->max_startup_state_dur);
	    pm2->flags.value = (uint32_t)READ_uint32_t(&pm2->flags.value);
	    pm2->update_period = (uint32_t)READ_uint32_t(&pm2->update_period);
	    pm2->search_period = (uint32_t)READ_uint32_t(&pm2->search_period);
	    pm2->grid_offset = (uint32_t)READ_uint32_t(&pm2->grid_offset);
	    pm2->on_time = (uint16_t)READ_uint16_t(&pm2->on_time);
	    pm2->min_acq_time = (uint16_t)READ_uint16_t(&pm2->min_acq_time);

	    flag = true;
    }

    return flag;
}

/*----------------------------------------------------------------------*/
static bool
ConfigRate
(
	uint16_t update_rate,
	bool readack
)
{
    uint8_t packet[8+6] = {
	    0xB5, 0x62,
	    type_cfg_rate.class, type_cfg_rate.id,
	    0x06, 0x00, /* len */
	    update_rate & 0xFF, (update_rate >> 8) & 0xFF, /* meas */
	    0x01, 0x00, /* nav */
	    0x01, 0x00, /* time: aligned to GPS time */
	    0x00, 0x00  /* checksum */
    };

    return 
	UBLOX_UBX_IssueCmd
	(
		packet, 
		sizeof(packet)
	)
	&& 
	(!readack || UBLOX_UBX_ReadAck(&type_cfg_rate));
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_ConfigRate
(
	UBLOX_UBX_CFG_Rate_t rate,
	bool readack
)
{
    uint8_t packet[6+sizeof(UBLOX_UBX_CFG_Rate_t)+2] = {
	    0xB5, 0x62,
	    type_cfg_rate.class, type_cfg_rate.id,
	    sizeof(UBLOX_UBX_CFG_Rate_t)%255, sizeof(UBLOX_UBX_CFG_Rate_t)/255 /* length */
    };

    /* Adjust endianess byte arrangement*/
    rate.meas_rate = (uint16_t)READ_uint16_t(&rate.meas_rate);
    rate.nav_rate = (uint16_t)READ_uint16_t(&rate.nav_rate);
    rate.time_ref = (uint16_t)READ_uint16_t(&rate.time_ref);

    memcpy(&packet[6], &rate, sizeof(UBLOX_UBX_CFG_Rate_t));

    return
    UBLOX_UBX_IssueCmd
    (
		packet,
		sizeof(packet)
    )
    &&
    (! readack || UBLOX_UBX_ReadAck(&type_cfg_rate));
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_PollRate
(
	UBLOX_UBX_CFG_Rate_t * rate,
	bool readack
)
{
    bool flag = false;
    uint8_t packet[8] =
    {
	    0xB5, 0x62,
	    type_cfg_rate.class, type_cfg_rate.id,
	    0x00, 0x00,
	    0x00, 0x00
    };
    
    UBLOX_UBX_Msg_t msg;

    memset(&msg, 0x00, sizeof(msg));

    if
	(
		UBLOX_UBX_IssueCmd
		(
			packet, 
			sizeof(packet)
		) 
		&& 
		UBLOX_UBX_ReadNextMsg
		(
			&msg, 
			&type_cfg_rate
		) 
		&& 
		(! readack || UBLOX_UBX_ReadAck(&type_cfg_rate))
	)
    {
	    memcpy(rate, msg.payload_addr, sizeof(UBLOX_UBX_CFG_Rate_t));

	    /* Adjust endianess byte arrangement*/
	    rate->meas_rate = (uint16_t)READ_uint16_t(&rate->meas_rate);
	    rate->nav_rate = (uint16_t)READ_uint16_t(&rate->nav_rate);
	    rate->time_ref = (uint16_t)READ_uint16_t(&rate->time_ref);

	    flag = true;
    }

    return flag;
}

/*----------------------------------------------------------------------*/
static bool
ConfigRxm
(
	UBLOX_UBX_CFG_RXM_GpsSensitivityMode_t gps_mode,
	UBLOX_UBX_CFG_RXM_LowPowerMode_t lp_mode,
	bool readack
)
{
    uint8_t packet[10] = {
	    0xB5, 0x62,
	    type_cfg_rxm.class, type_cfg_rxm.id,
	    0x02, 0x00,
	    gps_mode & 0xFF, lp_mode & 0xFF,
	    0x00, 0x00
    };

    return 
	UBLOX_UBX_IssueCmd
	(
		packet, 
		sizeof(packet)
	) 
	&& 
	(! readack || UBLOX_UBX_ReadAck(&type_cfg_rxm));
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_ConfigRxm
(
	UBLOX_UBX_CFG_Rxm_t rxm,
	bool readack
)
{
    uint8_t packet[6+sizeof(UBLOX_UBX_CFG_Rxm_t)+2] = {
	    0xB5, 0x62,
	    type_cfg_rxm.class, type_cfg_rxm.id,
	    sizeof(UBLOX_UBX_CFG_Rxm_t)%255, sizeof(UBLOX_UBX_CFG_Rxm_t)/255 /* length */
    };

    /* Adjust endianess byte arrangement*/
    rxm.lp_mode = (uint8_t)READ_uint8_t(&rxm.lp_mode);

    memcpy(&packet[6], &rxm, sizeof(UBLOX_UBX_CFG_Rxm_t));

    return 
	UBLOX_UBX_IssueCmd
	(
		packet, 
		sizeof(packet)
	) 
	&& 
	(! readack || UBLOX_UBX_ReadAck(&type_cfg_rxm));	
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_PollRxm
(
	UBLOX_UBX_CFG_Rxm_t * rxm,
	bool readack
)
{
    bool flag = false;
    uint8_t packet[8] =
    {
	    0xB5, 0x62,
	    type_cfg_rxm.class, type_cfg_rxm.id,
	    0x00, 0x00,
	    0x00, 0x00
    };
    
    UBLOX_UBX_Msg_t msg;

    memset(&msg, 0x00, sizeof(msg));

    if
	(
		UBLOX_UBX_IssueCmd
		(
			packet, 
			sizeof(packet)
		) 
		&& 
		UBLOX_UBX_ReadNextMsg
		(
			&msg, 
			&type_cfg_rxm
		) 
		&& 
		(! readack || UBLOX_UBX_ReadAck(&type_cfg_rxm))
	)
    {
	    memcpy(rxm, msg.payload_addr, sizeof(UBLOX_UBX_CFG_Rxm_t));

	    /* Adjust endianess byte arrangement*/
	    rxm->lp_mode = (uint8_t)READ_uint8_t(&rxm->lp_mode);

	    flag = true;
    }

    return flag;	
}

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_SBAS_Include)
static bool
ConfigSbas
(
	UBLOX_UBX_CFG_SBAS_MASK_Mode_t mode,
	UBLOX_UBX_CFG_SBAS_MASK_Usage_t usage,
	bool readack
)
{
    uint8_t max_channels_searched = 0x03;

    uint8_t packet[16] = {
	    0xB5, 0x62,
	    type_cfg_sbas.class, type_cfg_sbas.id,
	    0x08, 0x00, /* length */
	    mode & 0xff, /* mode */
	    usage & 0xff, /* usage */
	    max_channels_searched, /* maxsbas: 0-3 */
	    0x00, /* reserved */
	    0x00, 0x00, 0x00, 0x00, /* scanmode: all zeros->auto */
	    0X00, 0X00 /* checksum */
    };

    return 
	UBLOX_UBX_IssueCmd
	(
		packet, 
		sizeof(packet)
	)
	&& 
	(! readack || UBLOX_UBX_ReadAck(&type_cfg_sbas));
}
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_SBAS_Include)
static bool
UBLOX_UBX_ConfigSbas
(
	UBLOX_UBX_CFG_Sbas_t sbas,
	bool readack
)
{
    uint8_t packet[6+sizeof(UBLOX_UBX_CFG_Sbas_t)+2] = {
	    0xB5, 0x62,
	    type_cfg_sbas.class, type_cfg_sbas.id,
	    sizeof(UBLOX_UBX_CFG_Sbas_t)%255, sizeof(UBLOX_UBX_CFG_Sbas_t)/255 /* length */
    };

    /* Adjust endianess byte arrangement*/
    sbas.mode.value = (uint8_t)READ_uint8_t(&sbas.mode.value);
    sbas.usage.value = (uint8_t)READ_uint8_t(&sbas.usage.value);
    sbas.max_sbas = (uint8_t)READ_uint8_t(&sbas.max_sbas);
    sbas.scan_mode_2.value = (uint8_t)READ_uint8_t(&sbas.scan_mode_2.value);
    sbas.scan_mode_1.value = (uint32_t)READ_uint32_t(&sbas.scan_mode_1.value);

    memcpy(&packet[6], &sbas, sizeof(UBLOX_UBX_CFG_Sbas_t));

    return 
	UBLOX_UBX_IssueCmd
	(
		packet, 
		sizeof(packet)
	) 
	&& 
	(! readack || UBLOX_UBX_ReadAck(&type_cfg_sbas));	
}
#endif

/*----------------------------------------------------------------------*/
static bool
ConfigGnss
(
	UBLOX_UBX_CFG_GNSS_ConfigBlock_t * blocks,
	uint8_t count,
	bool readack
)
{
	uint8_t gnss_pkt_hdr[10] = {
		0xB5, 0x62,
		type_cfg_gnss.class, type_cfg_gnss.id,
		(4+sizeof(UBLOX_UBX_CFG_GNSS_ConfigBlock_t)*count)%255, (4+sizeof(UBLOX_UBX_CFG_GNSS_ConfigBlock_t)*count)/255, /* length */
		0x00,
		0x00,
		0xFF,
		count		
	};
	uint8_t packet[sizeof(gnss_pkt_hdr)+sizeof(UBLOX_UBX_CFG_GNSS_ConfigBlock_t)*count+2];
	
	memcpy(packet, gnss_pkt_hdr, sizeof(gnss_pkt_hdr));
	memcpy(packet + sizeof(gnss_pkt_hdr), blocks, count*sizeof(UBLOX_UBX_CFG_GNSS_ConfigBlock_t));

	return
	UBLOX_UBX_IssueCmd
	(
		packet,
		sizeof(packet)
	)
	&&
	(! readack || UBLOX_UBX_ReadAck(&type_cfg_gnss));
}

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_SBAS_Include)
static bool
UBLOX_UBX_PollSbas
(
	UBLOX_UBX_CFG_Sbas_t * sbas, bool readack
)
{
    bool flag = false;
    uint8_t packet[8] =
    {
	    0xB5, 0x62,
	    type_cfg_sbas.class, type_cfg_sbas.id,
	    0x00, 0x00,
	    0x00, 0x00
    };
    
    UBLOX_UBX_Msg_t msg;

    memset(&msg, 0x00, sizeof(msg));

    if
	(
		UBLOX_UBX_IssueCmd
		(
			packet, 
			sizeof(packet)
		) 
		&& 
		UBLOX_UBX_ReadNextMsg
		(
			&msg, 
			&type_cfg_sbas
		) 
		&& 
		(! readack || UBLOX_UBX_ReadAck(&type_cfg_sbas))
	)
    {
	    memcpy(sbas, msg.payload_addr, sizeof(UBLOX_UBX_CFG_Sbas_t));

	    /* Adjust endianess byte arrangement*/
	    sbas->mode.value = (uint8_t)READ_uint8_t(&sbas->mode.value);
	    sbas->usage.value = (uint8_t)READ_uint8_t(&sbas->usage.value);
	    sbas->max_sbas = (uint8_t)READ_uint8_t(&sbas->max_sbas);
	    sbas->scan_mode_2.value = (uint8_t)READ_uint8_t(&sbas->scan_mode_2.value);
	    sbas->scan_mode_1.value = (uint32_t)READ_uint32_t(&sbas->scan_mode_1.value);

	    flag = true;
    }

    return flag;	
}
#endif

/*----------------------------------------------------------------------*/
static bool
ConfigRst
(
	UBLOX_UBX_CFG_RST_NavBbrMask_SpecialSet_t nav_bbr_mask,
	UBLOX_UBX_CFG_RST_Type_t type, 
	bool readack
)
{
    uint8_t packet[12] = {
	    0xB5, 0x62,
	    type_cfg_rst.class, type_cfg_rst.id,
	    0x04, 0x00,
	    nav_bbr_mask & 0xFF, (nav_bbr_mask >> 8) & 0xFF,
	    type & 0xFF,
	    0x00,
	    0x00, 0x00
    };
    return 
	UBLOX_UBX_IssueCmd
	(
		packet, 
		sizeof(packet)
	) 
	&& 
	(! readack || UBLOX_UBX_ReadAck(&type_cfg_rst));
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_ConfigRst
(
	UBLOX_UBX_CFG_Rst_t rst,
	bool readack
)
{
    uint8_t packet[6+sizeof(UBLOX_UBX_CFG_Rst_t)+2] = {
	    0xB5, 0x62,
	    type_cfg_rst.class, type_cfg_rst.id,
	    sizeof(UBLOX_UBX_CFG_Rst_t)%255, sizeof(UBLOX_UBX_CFG_Rst_t)/255 /* length */
    };

    /* Adjust endianess byte arrangement*/
    rst.nav_bbr_mask.value = (uint16_t)READ_uint16_t(&rst.nav_bbr_mask.value);
    rst.rst_type = (uint8_t)READ_uint8_t(&rst.rst_type);

    memcpy(&packet[6], &rst, sizeof(UBLOX_UBX_CFG_Rst_t));

    return 
	UBLOX_UBX_IssueCmd
	(
		packet, 
		sizeof(packet)
	) 
	&& 
	(! readack || UBLOX_UBX_ReadAck(&type_cfg_rst));	
}

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NAV5_Include)
static bool
ConfigNav5
(
	UBLOX_UBX_CFG_NAV5_Model_t model,
	UBLOX_UBX_CFG_NAV5_FixMode_t fix_mode,
	uint8_t maxsv,
	bool readack
)
{
    uint8_t min_ELE = 0x05;
    uint8_t allow_alma_nav = 0X0;

    uint8_t packet[48] = {
	    0xB5, 0x62,
	    type_cfg_nav5.class, type_cfg_nav5.id,
	    40, 0x00,
	    model,
	    0x00,
	    0x00, 0x00,
	    0x03,
	    0x03,
	    maxsv & 0x1F,
	    fix_mode,
	    0x50, 0xC3, 0x00, 0x00,
	    0x0F,
	    0x0A,
	    min_ELE,
	    0x3C,
	    0x0F, /* default 0 */
	    allow_alma_nav,
	    0x00, 0x00,
	    0xFA, 0x00,
	    0xFA, 0x00,
	    0x64, 0x00,
	    0x2C, 0x01,
	    0x00,
	    0x00,
	    0x00, 0x00,
	    0x00, 0x00, 0x00, 0x00,
	    0x00, 0x00, 0x00, 0x00,
	    0x00, 0x00
    };

    return 
	UBLOX_UBX_IssueCmd
	(
		packet, 
		sizeof(packet)
	) 
	&& 
	(! readack || UBLOX_UBX_ReadAck(&type_cfg_nav5));	
}
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NAV5_Include)
static bool
UBLOX_UBX_ConfigNav5
(
	UBLOX_UBX_CFG_Nav5_t nav5,
	bool readack
)
{
    uint8_t packet[6+sizeof(UBLOX_UBX_CFG_Nav5_t)+2] = {
	    0xB5, 0x62,
	    type_cfg_nav5.class, type_cfg_nav5.id,
	    sizeof(UBLOX_UBX_CFG_Nav5_t)%255, sizeof(UBLOX_UBX_CFG_Nav5_t)/255 /* length */
    };

    /* Adjust endianess byte arrangement*/
    nav5.mask.value = (uint16_t)READ_uint16_t(&nav5.mask.value);
    nav5.dyn_model = (uint8_t)READ_uint8_t(&nav5.dyn_model);
    nav5.fix_mode = (uint8_t)READ_uint8_t(&nav5.fix_mode);
    nav5.fixed_alt = (int32_t)READ_int32_t(&nav5.fixed_alt);
    nav5.fixed_alt_var = (uint32_t)READ_uint32_t(&nav5.fixed_alt_var);
    nav5.min_elev = (int8_t)READ_int8_t(&nav5.min_elev);
    nav5.dr_limit = (uint8_t)READ_uint8_t(&nav5.dr_limit);
    nav5.p_dop = (uint16_t)READ_uint16_t(&nav5.p_dop);
    nav5.t_dop = (uint16_t)READ_uint16_t(&nav5.t_dop);
    nav5.p_acc = (uint16_t)READ_uint16_t(&nav5.p_acc);
    nav5.t_acc = (uint16_t)READ_uint16_t(&nav5.t_acc);
    nav5.static_hold_thresh = (uint8_t)READ_uint8_t(&nav5.static_hold_thresh);
    nav5.dgnss_timeout = (uint8_t)READ_uint8_t(&nav5.dgnss_timeout);
    nav5.cno_thresh_num_svs = (uint8_t)READ_uint8_t(&nav5.cno_thresh_num_svs);
    nav5.cno_thresh = (uint8_t)READ_uint8_t(&nav5.cno_thresh);
    nav5.static_hold_max_dist = (uint16_t)READ_uint16_t(&nav5.static_hold_max_dist);
    nav5.utc_standard = (uint8_t)READ_uint8_t(&nav5.utc_standard);

    memcpy(&packet[6], &nav5, sizeof(UBLOX_UBX_CFG_Nav5_t));

    return 
	UBLOX_UBX_IssueCmd
	(
		packet, 
		sizeof(packet)
	) 
	&& 
	(! readack || UBLOX_UBX_ReadAck(&type_cfg_nav5));	
}
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_NAV5_Include)
static bool
UBLOX_UBX_PollNav5
(
	UBLOX_UBX_CFG_Nav5_t * nav5,
	bool readack
)
{
    bool flag = false;
    uint8_t packet[8] =
    {
	    0xB5, 0x62,
	    type_cfg_nav5.class, type_cfg_nav5.id,
	    0x00, 0x00,
	    0x00, 0x00
    };

    UBLOX_UBX_Msg_t msg;
    
    if
	(
		UBLOX_UBX_IssueCmd
		(
			packet, 
			sizeof(packet)
		)
		&& 
		UBLOX_UBX_ReadNextMsg
		(
			&msg, 
			&type_cfg_nav5
		) 
		&& 
		(! readack || UBLOX_UBX_ReadAck(&type_cfg_nav5))
	)
    {
	    memcpy(nav5, msg.payload_addr, sizeof(UBLOX_UBX_CFG_Nav5_t));

	    /* Adjust endianess byte arrangement*/
	    nav5->mask.value = (uint16_t)READ_uint16_t(&nav5->mask.value);
	    nav5->dyn_model = (uint8_t)READ_uint8_t(&nav5->dyn_model);
	    nav5->fix_mode = (uint8_t)READ_uint8_t(&nav5->fix_mode);
	    nav5->fixed_alt = (int32_t)READ_int32_t(&nav5->fixed_alt);
	    nav5->fixed_alt_var = (uint32_t)READ_uint32_t(&nav5->fixed_alt_var);
	    nav5->min_elev = (int8_t)READ_int8_t(&nav5->min_elev);
	    nav5->dr_limit = (uint8_t)READ_uint8_t(&nav5->dr_limit);
	    nav5->p_dop = (uint16_t)READ_uint16_t(&nav5->p_dop);
	    nav5->t_dop = (uint16_t)READ_uint16_t(&nav5->t_dop);
	    nav5->p_acc = (uint16_t)READ_uint16_t(&nav5->p_acc);
	    nav5->t_acc = (uint16_t)READ_uint16_t(&nav5->t_acc);
	    nav5->static_hold_thresh = (uint8_t)READ_uint8_t(&nav5->static_hold_thresh);
	    nav5->dgnss_timeout = (uint8_t)READ_uint8_t(&nav5->dgnss_timeout);
	    nav5->cno_thresh_num_svs = (uint8_t)READ_uint8_t(&nav5->cno_thresh_num_svs);
	    nav5->cno_thresh = (uint8_t)READ_uint8_t(&nav5->cno_thresh);
	    nav5->static_hold_max_dist = (uint16_t)READ_uint16_t(&nav5->static_hold_max_dist);
	    nav5->utc_standard = (uint8_t)READ_uint8_t(&nav5->utc_standard);

	    flag = true;
    }
    return flag;	
}
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_TP5_Include)
static bool
UBLOX_UBX_ConfigTp5
(
	UBLOX_UBX_CFG_Tp5_t tp5,
	bool readack
)
{
    uint8_t packet[6+sizeof(UBLOX_UBX_CFG_Tp5_t)+2] = {
	    0xB5, 0x62,
	    type_cfg_tp5.class, type_cfg_tp5.id,
	    sizeof(UBLOX_UBX_CFG_Tp5_t)%255, sizeof(UBLOX_UBX_CFG_Tp5_t)/255 /* length */
    };

    tp5.tp_idx = (uint8_t)READ_uint8_t(&tp5.tp_idx);
    tp5.version = (uint8_t)READ_uint8_t(&tp5.version);
    tp5.ant_cable_delay = (uint16_t)READ_uint16_t(&tp5.ant_cable_delay);
    tp5.rf_group_delay = (uint16_t)READ_uint16_t(&tp5.rf_group_delay);
    tp5.freq_period = (uint32_t)READ_uint32_t(&tp5.freq_period);
    tp5.freq_period_lock = (uint32_t)READ_uint32_t(&tp5.freq_period_lock);
    tp5.pulse_len_ratio = (uint32_t)READ_uint32_t(&tp5.pulse_len_ratio);
    tp5.pulse_len_ratio_lock = (uint32_t)READ_uint32_t(&tp5.pulse_len_ratio_lock);
    tp5.user_config_delay = (int32_t)READ_int32_t(&tp5.user_config_delay);
    tp5.flags.value = (uint32_t)READ_uint32_t(&tp5.flags.value);

    memcpy(&packet[6], &tp5, sizeof(UBLOX_UBX_CFG_Tp5_t));

    return 
	UBLOX_UBX_IssueCmd
	(
		packet, 
		sizeof(packet)
	) 
	&& 
	(! readack || UBLOX_UBX_ReadAck(&type_cfg_tp5));	
}
#endif

/*----------------------------------------------------------------------*/
#if (UBLOX_UBX_CFG_TP5_Include)
static bool
UBLOX_UBX_PollTp5
(
	UBLOX_UBX_CFG_Tp5_t * tp5,
	UBLOX_UBX_CFG_TP5_TpIdx_t tp_idx,
	bool readack
)
{
    bool flag = false;
    uint8_t packet[9] =
    {
	    0xB5, 0x62,
	    type_cfg_tp5.class, type_cfg_tp5.id,
	    0x01, 0x00,
	    tp_idx & 0xff,
	    0x00, 0x00
    };

    UBLOX_UBX_Msg_t msg;

    memset(&msg, 0x00, sizeof(msg));
    
    if
	(
		UBLOX_UBX_IssueCmd
		(
			packet, 
			sizeof(packet)
		) 
		&& 
		UBLOX_UBX_ReadNextMsg
		(
			&msg, 
			&type_cfg_tp5
		) 
		&& 
		(! readack || UBLOX_UBX_ReadAck(&type_cfg_tp5))
	)
    {
	    memcpy(tp5, msg.payload_addr, sizeof(UBLOX_UBX_CFG_Tp5_t));

	    tp5->tp_idx = (uint8_t)READ_uint8_t(&tp5->tp_idx);
	    tp5->version = (uint8_t)READ_uint8_t(&tp5->version);
	    tp5->ant_cable_delay = (uint16_t)READ_uint16_t(&tp5->ant_cable_delay);
	    tp5->rf_group_delay = (uint16_t)READ_uint16_t(&tp5->rf_group_delay);
	    tp5->freq_period = (uint32_t)READ_uint32_t(&tp5->freq_period);
	    tp5->freq_period_lock = (uint32_t)READ_uint32_t(&tp5->freq_period_lock);
	    tp5->pulse_len_ratio = (uint32_t)READ_uint32_t(&tp5->pulse_len_ratio);
	    tp5->pulse_len_ratio_lock = (uint32_t)READ_uint32_t(&tp5->pulse_len_ratio_lock);
	    tp5->user_config_delay = (int32_t)READ_int32_t(&tp5->user_config_delay);
	    tp5->flags.value = (uint32_t)READ_uint32_t(&tp5->flags.value);

	    flag = true;
    }
    return flag;	
}
#endif

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_PollMonVer
(
	UBLOX_UBX_CFG_MonVer_t * mon_ver,
	bool readack
)
{
    bool flag = false;
    uint8_t packet[8] =
    {
	    0xB5, 0x62,
	    type_cfg_mon_ver.class, type_cfg_mon_ver.id,
	    0x00, 0x00,
	    0x00, 0x00
    };

    UBLOX_UBX_Msg_t msg;
    
    if
	(
		UBLOX_UBX_IssueCmd
		(
			packet, 
			sizeof(packet)
		) 
		&& 
		UBLOX_UBX_ReadNextMsg
		(
			&msg, 
			&type_cfg_mon_ver
		) 
		&& 
		(! readack || UBLOX_UBX_ReadAck(&type_cfg_mon_ver))
	)
    {
	    memcpy(mon_ver, msg.payload_addr, sizeof(UBLOX_UBX_CFG_MonVer_t));
	    flag = true;
    }
    return flag;	
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_GetNavPvt
(
	UBLOX_UBX_NAV_Pvt_t * pvt,
	bool readack
)
{
    bool flag = false;

    UBLOX_UBX_Msg_t msg;
    
    if
	(
		UBLOX_UBX_ReadNextMsg
		(
			&msg, 
			&type_nav_pvt
		) 
		&& 
		(! readack || UBLOX_UBX_ReadAck(&type_nav_pvt))
	)
    {
	    memcpy(pvt, msg.payload_addr, sizeof(UBLOX_UBX_NAV_Pvt_t));

	    flag = true;
    }
    return flag;
}

/*----------------------------------------------------------------------*/
static bool
UBLOX_UBX_PollNavPvt
(
	void
)
{

	uint8_t packet[8] =
	{
		0xB5, 0x62,
		type_nav_pvt.class, type_nav_pvt.id,
		0x00, 0x00,
		0x00, 0x00
	};

	return
	UBLOX_UBX_IssueCmd
	(
		packet,
		sizeof(packet)
	);
}
