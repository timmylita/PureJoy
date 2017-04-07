/**
 * \file gl865.c
 * \author tom.m
 *
 * \see gl865.h.
 */

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

#include "project.h"
#include "timer.h"
#include "gl865.h"
#include "halCell.h"

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

/************************************************************************/
/* Types.                                                               */
/************************************************************************/

/************************************************************************/
/* Private prototypes.                                                  */
/************************************************************************/\
//Handle Telit Data
static const char* const catchTelitData(uint32_t acqTimeout, bool quickCheck, uint16_t dataSize);
static const char* const parseData(const char* const str, const char* begStr, const char* endStr);
static const char* const parseSplit(const char* const str, const char* delimiters, uint16_t field);
static bool parseFind(const char* const str, const char* subStr);

//Talking to Telit
static void sendATCommand(const char* str, bool terminate);

static const char* const sendRecQuickATCommand(const char* str);
static const char* const sendRecATCommand(const char* str);
static const char* const sendRecATCommandParse(const char* str, const char* begStr, const char* endStr);
static const char* const sendRecATCommandSplit(const char* str, const char* delimiterStr, uint16_t field);

//Hardware
static bool turnOn(void);
static bool turnOff(void);
static bool init(uint16_t band);

/************************************************************************/
/* Constants.                                                           */
/************************************************************************/

/************************************************************************/
/* Private variables.                                                   */
/************************************************************************/
char *  fullData;	// full telit response data, filled by catchTelitData
char *  parsedData; // parsed response
/************************************************************************/
/* Macros.                                                              */
/************************************************************************/

/************************************************************************/
/* Implementations.                                                     */
/************************************************************************/

static const char* const getFullData(void)
{
	return fullData;
}
static const char* const getParsedData(void)
{
	return parsedData;
}

static const char* const catchTelitData(uint32_t acqTimeout, bool quickCheck, uint16_t dataSize)
{
	char* storeData;
	int32_t dataPos = 0;
	
	free(fullData);
	
	fullData=NULL;
	
	if (quickCheck)
	{
		dataSize = 20; //If it is just a quick check max size is "/n/nERROR/n/n/0"
	}
	
	storeData = (char*) malloc(sizeof(char) * (dataSize));
	
	if (storeData == NULL)
	{
		return 0;
	}
	
	dataPos = HAL_CELL_Read(storeData, dataSize, acqTimeout);
	 
	if(dataPos < 0)
	{
		return 0;
	}
	
	free(fullData);
	fullData = NULL;
	
	fullData = (char*) malloc(sizeof(char) * (dataPos+1));
	
	if (fullData == NULL) 
	{
		return 0;
	}
	memcpy(fullData, storeData, dataPos+1);
	
	free(storeData);
	storeData = NULL;
	
	storeData[dataPos] = '\0';

	if(quickCheck)
	{
		if (parseFind(fullData, "\r\nOK\r\n"))
		{
			return fullData; 
		}
		else
		{
			return 0;
		}
	}
	return fullData;
}


static const char* const parseData(const char* const str, const char* begStr, const char* endStr)
{
	size_t begSize = strlen(begStr);
	char* begPtr = strstr(str,begStr);
	char* endPtr;
	uint16_t dataPos=0;
	
	if (!str)
	{
		return 0;
	}
	
	if(!begPtr)
	{
		return 0;
	}
	
	begPtr += begSize;
	endPtr = strstr ((begPtr), endStr);
	if(!endPtr)
	{
		return 0;
	}
	
	free(parsedData);
	parsedData = NULL;
	parsedData = (char*) malloc((size_t)(sizeof(char)*(endPtr-begPtr)+1));
	if (parsedData == NULL)
	{
		return 0;
	}
	
	while (begPtr != endPtr)
	{
		parsedData[dataPos++]= *begPtr++;
	}
	parsedData[dataPos]= '\0';
	return parsedData;
}

static const char* const parseSplit(const char* const str, const char* delimiters, uint16_t field)
{
	char * temp;
	char deadStr[strlen(str)+1];
	
	if (!str)
	{
		return 0;
	}

	strcpy(deadStr, str);
	temp = strtok(deadStr, delimiters);
	for(uint16_t i=0; i<field; i++)
	{
		temp = strtok (NULL,delimiters);
	}

	free(parsedData);
	parsedData=NULL;
	if(!temp)
	{
		return 0;
	}
	parsedData =  (char*) malloc(sizeof(char)* (strlen(temp)+1));
	if (parsedData == NULL)
	{
		return 0;
	}
	strcpy(parsedData,temp);

	return parsedData;
}

static bool parseFind(const char* const str, const char* subStr)
{
	return (str && strstr(str,subStr))?true:false;
}


static void sendATCommand(const char* str, bool terminate)
{
	HAL_CELL_Write(str, strlen(str), 200);
	if(terminate)
	{
		HAL_CELL_Write("\r", 1, 200);
	}
}

static const char* const sendRecQuickATCommand(const char* str)
{
	sendATCommand(str, true);
	return	catchTelitData(2000, true, 300);
}

static const char* const sendRecATCommand(const char* str)
{
	sendATCommand(str, true);
	return	catchTelitData(180000, false, 300);
}

static const char* const sendRecATCommandParse(const char* str, const char* begStr, const char* endStr)
{
	sendATCommand(str, true);
	return parseData(catchTelitData(180000, false, 300), begStr, endStr);
}

static const char* const sendRecATCommandSplit(const char* str, const char* delimiters, uint16_t field)
{
	sendATCommand(str, true);
	return	parseSplit(catchTelitData(180000, false, 300), delimiters, field);
}

static bool init(uint16_t band)
{;
	sendATCommand("ATE0", true);
	catchTelitData(2000,true,20);	//needs a long baud delay to catch ok after echo
	if
	(
		!sendRecQuickATCommand("ATV1") ||
		!sendRecQuickATCommand("AT&K0") ||
		!sendRecQuickATCommand("AT+IPR=0") ||
		!sendRecQuickATCommand("AT+CMEE=2")
	)
	{
		return 0;
	}
	switch(band)
	{
		case 0:
		{
			if(!sendRecQuickATCommand("AT#BND=0")) //0 - GSM 900MHz + DCS 1800MHz
			{
				return 0;
			}
			break;
		}
		case 1:
		{
			if(!sendRecQuickATCommand("AT#BND=1")) //1 - GSM 900MHz + PCS 1900MHz
			{
				return 0;
			}
			break;
		}
		case 2:
		{
			if(!sendRecQuickATCommand("AT#BND=2")) //2 - GMS 850MHz + DCS 1800MHz
			{
				return 0;
			}
			break;
		}
		case 3:
		{
			if(!sendRecQuickATCommand("AT#BND=3")) //3 - GMS 850MHz + PCS 1900MHz
			{
				return 0;
			}
			break;
		}
		default:
		{
			return 0;
		}
	}

	return 1;
}

/*----------------------------------------------------------------------*/
bool 
TELIT_gsmInit
(
	void
)
{
	HAL_CELL_Init();
	
	if(!init(3))
	{
		return false;
	}
	
	if(!sendRecQuickATCommand("AT+CMGF=1")) 
	{
		return false; 
	}
		
	if(!sendRecQuickATCommand("AT+CNMI=0,0,0,0,0")) 
	{
		return false;
	}

	sendRecQuickATCommand("AT#SMSMODE=0");	
				
	return true;
}

/*----------------------------------------------------------------------*/
void
TELIT_gsmReset
(
	void
)
{
	HAL_CELL_Ioctl(HAL_CELL_REQ_Reset, 0, NULL);
}

/*----------------------------------------------------------------------*/
bool
TELIT_gsmTurnOn
(
	void
)
{
	sendATCommand("AT", true);
	
	if(catchTelitData(2000,true,20))
	{
		return true;
	}
	
	HAL_CELL_Ioctl(HAL_CELL_REQ_PowerEnable, 0, NULL);
	
	sendATCommand("AT", true);
	
	return catchTelitData(2000,true,20)?true:false;		
}

/*----------------------------------------------------------------------*/
bool
TELIT_gsmTurnOff
(
	void
)
{
	TIMER_t tmr;	
	
	sendRecQuickATCommand("AT#SHDN");
	
	TIMER_Configure(&tmr, 15, 0);
	
	while(!TIMER_Check(&tmr));
	
	return !sendRecQuickATCommand("AT")?true:false;
}

/*----------------------------------------------------------------------*/
const char * const 
TELIT_getTemperatureTEMPMON
(
	void
)
{
	return  sendRecATCommandSplit("AT+TEMPMON=1",",",1);
}

/*----------------------------------------------------------------------*/
bool 
TELIT_checkCREG
(
	void
)
{
	return (sendRecATCommandSplit("AT+CREG?",":,",2)[0] == '1')?true:false;
}


/*----------------------------------------------------------------------*/
const char * const 
TELIT_checkCOPS
(
	void
)
{
	return sendRecATCommandParse("AT+COPS=?","1,\"","\"");
}

/*----------------------------------------------------------------------*/
const char * const 
TELIT_checkGSN
(
	void
)
{
	return sendRecATCommandParse("AT+GSN","\r\n","\r\n");
}

/*----------------------------------------------------------------------*/
uint32_t 
TELIT_checkCSQ
(
	void
)
{
	return atoi(sendRecATCommandSplit("AT+CSQ",":,",1));
}

/*----------------------------------------------------------------------*/
const char * const 
TELIT_checkMONI
(
	void
)
{
	return sendRecATCommandParse("AT#MONI","\r\n","\r\n");
}

/*----------------------------------------------------------------------*/
const char * const 
TELIT_getMyNumCNUM
(
	void
)
{
	return  sendRecATCommandSplit("AT+CNUM",",",1);
}


/*----------------------------------------------------------------------*/
bool 
TELIT_sendNoSaveCMGS
(
	const char * theNumber,
	const char * sendString
)
{
	char charByte[2] = {0x1A, 0x1B};
		
	sendATCommand("AT+CMGS=\"", false);
	sendATCommand(theNumber, false);
	sendATCommand("\"", true);
	if( parseFind(catchTelitData(180000, false, 300), ">"))
	{
		sendATCommand(sendString, false);
		HAL_CELL_Write(&charByte[0], 1, 200);
		return true;					
	}
	else
	{
		HAL_CELL_Write(&charByte[1], 1, 200);
	} 
	
	return false;						
}

/*----------------------------------------------------------------------*/
const char * const  
TELIT_saveMessageCMGW
(
	const char * theNumber,
	const char * sendString
)
{
	char charByte[2] = {0x1A, 0x1B};
	
	sendATCommand("AT+CMGW=\"", false);
	sendATCommand(theNumber, false);
	sendATCommand("\"", true);
	if( parseFind(catchTelitData(180000, false, 300), ">"))
	{
		sendATCommand(sendString, false);
		HAL_CELL_Write(&charByte[0], 1, 200);
	}
	else
	{
		HAL_CELL_Write(&charByte[1], 1, 200);
	}
	
	return parseSplit(catchTelitData(180000, false, 300), ":", 1);
}

/*----------------------------------------------------------------------*/
bool 
TELIT_sendSavedMessageCMSS
(
	const char * const theMesNum
)
{
	sendATCommand("AT+CMSS=", false);
	sendATCommand(theMesNum, true);

	return	catchTelitData(10000, true, 300)?true:false;
}

/*----------------------------------------------------------------------*/
const char * const 
TELIT_getNumMesInMemCPMS 
(
	uint16_t whichMemSpace
)
{
	return sendRecATCommandSplit(" AT+CPMS=\"SM\" ", ":,", whichMemSpace);
}

/*----------------------------------------------------------------------*/
const char * const  
TELIT_checkCMGDList
(
	void
)
{
	return sendRecATCommandParse("AT+CMGD=?", "(",")");
}

/*----------------------------------------------------------------------*/
const char * const 
TELIT_readMessageCMGR
(
	const char * const whichMessage 
)
{

	sendATCommand("AT+CMGR=", false);
	sendATCommand(whichMessage, true);

	return	parseData(catchTelitData(180000, false, 300), "\r\n", "\r\n");
}


/*----------------------------------------------------------------------*/
const char * const 
TELIT_readAllCMGL
(
	const char * const messageType, 
	uint16_t dataSize
)
{
	
	sendATCommand("AT+CMGL=\"", false);
	sendATCommand(messageType, false);
	sendATCommand("\"", true);
	return	catchTelitData(10000,false,dataSize);
}

/*----------------------------------------------------------------------*/
bool 
TELIT_deletMessagesCMGD
(
	const char * const whichMessages
)
{
	sendATCommand("AT+CMGD=", false);
	sendATCommand(whichMessages, true);

	return	catchTelitData(180000, false, 300)?true:false;
}



/*----------------------------------------------------------------------*/
bool 
TELIT_setApnCGDCONT
(
	const char * const userSetContextID, 
	const char * const PDPtype,
	const char * const APN,
	const char * const requestedStaticIP, 
	const char * const dataCompression,
	const char * const headerCompression
)
{	
	sendATCommand("AT+CGDCONT=", false);
	sendATCommand(userSetContextID, false);
	sendATCommand(",\"", false);
	sendATCommand(PDPtype, false);
	sendATCommand("\",\"", false);
	sendATCommand(APN, false);
	sendATCommand("\",\"", false);
	sendATCommand(requestedStaticIP, false);
	sendATCommand("\",", false);
	sendATCommand(dataCompression, false);
	sendATCommand(",", false);	
	sendATCommand(headerCompression, true);
	
	return	catchTelitData(2000, true, 300)?true:false;
}

/*----------------------------------------------------------------------*/
bool 
TELIT_setTcpIpStackSCFG
(
	const char * const userSetConnectionID,
	const char * const userSetContextID,
	const char * const minPacketSize,
	const char * const globalTimeout, 
	const char * const connectionTimeout,
	const char * const txTimeout
)
{
	sendATCommand("AT#SCFG=", false);
	sendATCommand(userSetConnectionID, false);
	sendATCommand(",", false);
	sendATCommand(userSetContextID, false);
	sendATCommand(",", false);
	sendATCommand(minPacketSize, false);
	sendATCommand(",", false);
	sendATCommand(globalTimeout, false);
	sendATCommand(",", false);
	sendATCommand(connectionTimeout, false);
	sendATCommand(",", false);
	sendATCommand(txTimeout, true);
	
	return	catchTelitData(2000, true, 300)?true:false;
}

/*----------------------------------------------------------------------*/
const char * const 
TELIT_setContextSGACT
(
	const char * const userSetConnectionID,
	const char * const statusOfConnection,
	const char * const username,
	const char * const password
)
{
	
	sendATCommand("AT#SGACT=", false);
	sendATCommand(userSetConnectionID, false);
	sendATCommand(",", false);
	
	if(username != NULL || password != NULL)
	{
		sendATCommand(statusOfConnection, false);
		if(username != NULL)
		{
			sendATCommand(",", false);
			sendATCommand(username, true);
		}
		if(password != NULL)
		{
			sendATCommand(",", false);
			sendATCommand(password, true);
		}
	}
	else
	{
		sendATCommand(statusOfConnection, true);
	}
	
	return catchTelitData(180000, false, 300);
}

/*----------------------------------------------------------------------*/
bool 
TELIT_setQualityCGQMIN
(
	const char * const userSetConnectionID ,
	const char * const precedence,
	const char * const delay,
	const char * const reliability, 
	const char * const peak,
	const char * const mean
)
{
	sendATCommand("AT+CGQMIN=", false);
	sendATCommand(userSetConnectionID, false);
	sendATCommand(",", false);
	sendATCommand(precedence, false);
	sendATCommand(",", false);
	sendATCommand(delay, false);
	sendATCommand(",", false);
	sendATCommand(reliability, false);
	sendATCommand(",", false);
	sendATCommand(peak, false);
	sendATCommand(",", false);
	sendATCommand(mean, true);
	
	return	catchTelitData(2000, true, 300)?true:false;
}

/*----------------------------------------------------------------------*/
bool 
TELIT_requestQualityCGQREQ
(
	const char * const userSetConnectionID,
	const char * const precedence,
	const char * const delay,
	const char * const reliability, 
	const char * const peak,
	const char * const mean
)
{
	sendATCommand("AT+CGQREQ=", false);
	sendATCommand(userSetConnectionID, false);
	sendATCommand(",", false);
	sendATCommand(precedence, false);
	sendATCommand(",", false);
	sendATCommand(delay, false);
	sendATCommand(",", false);
	sendATCommand(reliability, false);
	sendATCommand(",", false);
	sendATCommand(peak, false);
	sendATCommand(",", false);
	sendATCommand(mean, true);
	
	return	catchTelitData(2000, true, 300)?true:false;
}


/*----------------------------------------------------------------------*/
bool 
TELIT_setSecuritySGACTAUTH
(
	const char * const securitySetting
)
{
	sendATCommand("AT+SGACTAUTH=", false);
	sendATCommand(securitySetting, true);

	return	catchTelitData(2000, true, 300)?true:false;
}

/*----------------------------------------------------------------------*/
bool 
TELIT_socketDialSD
(
	const char * const userSetConnectionID ,
	const char * const protocol,
	const char * const remotePort,
	const char * const ipAddress
)
{
	sendATCommand("AT+CGQREQ=", false);
	sendATCommand(userSetConnectionID, false);
	sendATCommand(",", false);
	sendATCommand(protocol, false);
	sendATCommand(",", false);
	sendATCommand(remotePort, false);
	sendATCommand(",", false);
	sendATCommand(ipAddress, true);
	
	return parseFind(catchTelitData(180000, false, 300),"CONNECT")?true:false;
}


/*----------------------------------------------------------------------*/
bool 
TELIT_suspendSocket
(
	void
)
{
	sendATCommand("+++", false);
	return catchTelitData(4000, true, 300)?true:false;
}

/*----------------------------------------------------------------------*/
bool 
TELIT_resumeSocketSO
(
	const char * const whichSocket
)
{ 
	sendATCommand("AT#SO=", false);
	sendATCommand(whichSocket, true);
	
	return parseFind(catchTelitData(180000, false, 300),"CONNECT")?true:false;
}

/*----------------------------------------------------------------------*/
bool
TELIT_closeSocketSH
(
	const char * const whichSocket
)
{
	sendATCommand("AT#SH=", false);
	sendATCommand(whichSocket, true);
	
	return catchTelitData(2000, true, 300)?true:false;
}

/*----------------------------------------------------------------------*/
const char * const 
TELIT_socketStatusSS
(
	void
)
{
	sendATCommand("AT#SS", true);
	return catchTelitData(180000, false, 300);
}

/*----------------------------------------------------------------------*/
const char * const 
TELIT_socketInfoSI
(
	const char * const connectionID
)
{
	sendATCommand("AT#SI=", false);
	sendATCommand(connectionID, true);
	return catchTelitData(180000, false, 300);
}


/*----------------------------------------------------------------------*/
bool 
TELIT_socketListenSL
(
	const char * const connectionID, 
	const char * const listenState,
	const char * const listenPort
)
{	
	sendATCommand("AT#SL= ", false);
	sendATCommand(connectionID, false);
	sendATCommand(",", false);
	sendATCommand(listenState, false);
	sendATCommand(",", false);
	sendATCommand(listenPort, true);
	
	return	catchTelitData(2000, true, 300)?true:false;
}

/*----------------------------------------------------------------------*/
bool 
TELIT_socketAcceptSA
(
	const char * const connectionID
)
{	
	sendATCommand("AT#SA= ", false);
	sendATCommand(connectionID, true);
	
	return parseFind(catchTelitData(180000, false, 300),"CONNECT")?true:false;	
}

/*----------------------------------------------------------------------*/
const char * const 
TELIT_getHTTP
(
	uint16_t dataSize, 
	const char * const host, 
	const char * const resource, 
	const char * const httpVersion,
	bool keepAlive
)
{
	sendATCommand("GET ", false);
	sendATCommand(resource, false);
	sendATCommand(" HTTP/", false);
	sendATCommand(httpVersion, true);
	sendATCommand("\n", false);
	sendATCommand("HOST: ", false);
	sendATCommand(host, true);
	if(keepAlive)
	{
		sendATCommand("\n", false);
		sendATCommand("Connection: keep-alive", true);
		sendATCommand("\n", true);
	}
	else
	{
		sendATCommand("\n", true);
	}
	sendATCommand("\n", true);
	sendATCommand("\n", false);
	
	return  catchTelitData(180000, false, dataSize); 
}

/*----------------------------------------------------------------------*/
const char * const
TELIT_postHTTP
(
	uint16_t dataSize,
	const char * const host, 
	const char * const resource,
	const char * const secretAgent, 
	const char * const httpVersion,
	bool keepAlive, 
	const char * const reqStr
)
{
	size_t length = strlen(reqStr);	
	char asciiLength[7+1];
	
	itoa(length, asciiLength, 10);

	sendATCommand("POST ", false);
	sendATCommand(resource, false);
	sendATCommand(" HTTP/", false);
	sendATCommand(httpVersion, true);
	sendATCommand("\n", false);
	sendATCommand("HOST: ", false);
	sendATCommand(host, true);
	sendATCommand("\n", false);
	sendATCommand("USER-AGENT: ", false);
	sendATCommand(secretAgent, true);
	sendATCommand("\n", false);
	sendATCommand("Content-Type: application/x-www-form-urlencoded", true); // media type
	sendATCommand("\n", false);
	if(keepAlive)
	{
		sendATCommand("Connection: keep-alive", true);
		sendATCommand("\n", false);
	}
	sendATCommand("Content-Length: ", false);
	sendATCommand(asciiLength, true);
	sendATCommand("\n", true);
	sendATCommand("\n", false);
	sendATCommand(reqStr, true);
	sendATCommand("\n", true);
	sendATCommand("\n", false);

	return catchTelitData(180000, false, dataSize);
}

/*----------------------------------------------------------------------*/
bool 
TELIT_ftpTimeOutFTPO
(
	const char * const timeOut
)
{ 

	sendATCommand("AT#FTPO=", false);
	sendATCommand(timeOut, true);

	return catchTelitData(2000, true, 300)?true:false;
}


/*----------------------------------------------------------------------*/
bool 
TELIT_FTPOPEN
(
	const char * const serverPort, 
	const char * const username, 
	const char * password,
	const char * const mode
)
{
	sendATCommand("AT#FTPOPEN=\"", false);
	sendATCommand(serverPort, false);
	sendATCommand("\",\"", false);
	sendATCommand(username, false);
	sendATCommand("\",\"", false);
	sendATCommand(password, false);
	sendATCommand("\",", false);
	sendATCommand(mode, true);

	return catchTelitData(2000, true, 300)?true:false;
}

/*----------------------------------------------------------------------*/
bool 
TELIT_ftpDataTypeFTPTYPE
(
	const char * const binaryAscii
)
{
	sendATCommand("AT#FTPTYPE=", false);
	sendATCommand(binaryAscii, true);

	return catchTelitData(2000, true, 300)?true:false;
}

/*----------------------------------------------------------------------*/
bool
TELIT_FTPCLOSE
(
	void
)
{
	sendATCommand("AT#FTPCLOSE", true);

	return catchTelitData(180000, false, 300)?true:false;
}

/*----------------------------------------------------------------------*/
bool 
TELIT_FTPPUT
(
	const char * const fileWriteName, 
	const char * const data
)
{
	sendATCommand("AT#FTPPUT=\"", false);
	sendATCommand(fileWriteName, false);
	sendATCommand("\"", true);					
  	if(!parseFind(catchTelitData(180000, false, 300),"CONNECTED"))
	{
		return false;
	}
  	sendATCommand(data, false);
	return TELIT_suspendSocket();
}

/*----------------------------------------------------------------------*/
const char * const 
TELIT_FTPGET
(
	const char * const fileName,
	uint16_t dataSize
)
{
	sendATCommand("AT#FTPGET=\"", false);
	sendATCommand(fileName, false);
	sendATCommand("\"", true);	
			
	if(!parseFind(catchTelitData(180000, false, 300), "CONNECTED"))
	{
		return false;
	}

 	const char* const getData = catchTelitData(180000, false, dataSize);
	TELIT_suspendSocket(); 
	
	return getData;
}	

/*----------------------------------------------------------------------*/
bool 
TELIT_changeDirFTPCWD
(
	const char * const directory
)
{
	sendATCommand("AT#FTPCWD=\"", false);
	sendATCommand(directory, false);
	sendATCommand("\"", true);
	return catchTelitData(2000, true, 300);
}
