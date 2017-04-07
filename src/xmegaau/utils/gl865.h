/**
 * \file gl865.h
 * \author tom.M
 *
 * Provides functionality to send, receive and manage SMS text messages
 * as well as to open, close and manage GPRS connections
 */

#ifndef _GL865
#define _GL865

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

/************************************************************************/
/* Types.                                                               */
/************************************************************************/

/**
 * Initialize gl865 module for receiving/sending SMS
 */
bool 
TELIT_gsmInit
(
	void
);

/**
 * Reset gl865 module
 */
void 
TELIT_gsmReset
(
	void
);

/**
 * Turn gl865 module ON
 */
bool 
TELIT_gsmTurnOn
(
	void
);

/**
 * Turn gl865 module OFF
 */
bool 
TELIT_gsmTurnOff
(
	void
);

/**
 * Get temp of gl865 module in Celsius
 */
const char * const 
TELIT_getTemperatureTEMPMON
(
	void
);

/**
 * Gets registration status
 * true - REGISTERED
 * false - not REGISTERED
 */
bool 
TELIT_checkCREG
(
	void
);

/**
 * Gets network availability and current registration
 */
const char * const 
TELIT_checkCOPS
(
	void
);

/**
 * Gets serial number
 */
const char * const 

TELIT_checkGSN
(
	void
);

/**
 * Gets signal quality
 */
uint32_t 
TELIT_checkCSQ
(
	void
);

/**
 * Gets info for all surrounding available cell towers
 */
const char * const 
TELIT_checkMONI
(
	void
);

/**
 * Get the phone # number of the device, if stored on SIM 
 */
const char * const 
TELIT_getMyNumCNUM
(
	void
);

/**
 * Sends SMS without storing
 */
bool 
TELIT_sendNoSaveCMGS
(
	const char * theNumber,
	const char * sendString
);

/**
 * Sends message to storage
 */
const char * const  
TELIT_saveMessageCMGW
(
	const char * theNumber,
	const char * sendString
);

/**
 * Sends stored message
 */
bool 
TELIT_sendSavedMessageCMSS
(
	const char * const theMesNum
);

/**
 * Gets number of messages in specified mem
 */
const char * const 
TELIT_getNumMesInMemCPMS 
(
	uint16_t whichMemSpace
);

/**
 * Gets a numerical list of all messages separated by comma
 */
const char * const  
TELIT_checkCMGDList
(
	void
);

/**
 * Reads the message 1
 */
const char * const 
TELIT_readMessageCMGR
(
	const char * const whichMessage 
);

/**
 *  Returns all messages with specific listing, gives full messages 
 *  "REC UNREAD" - new message
 *  "REC READ" - read message
 *  "STO UNSENT" - stored message not yet sent
 *  "STO SENT" - stored message already sent
 *  "ALL" - all messages.
 */
const char * const 
TELIT_readAllCMGL
(
	const char * const messageType, 
	uint16_t dataSize
);

/**
 *  searches and deletes a specific message or all messages of a type
 *  If passed 1 arg EG: "AT+CMGD=1" it erases that message.
 *  If passed 2 args EG: "AT+CMGD=1,1" the second arg chooses all messages of that kind to erase.
 *  1 - delete all read messages from <memr> storage, leaving unread
 *      messages and stored mobile originated messages (whether sent or not) untouched
 *  2 - delete all read messages from <memr> storage and sent mobile
 *      originated messages, leaving unread messages and unsent mobile originated messages untouched
 *  3 - delete all read messages from <memr> storage, sent and unsent mobile originated messages, 
 *      leaving unread messages untouched
 *  4 - delete all messages from <memr> storage.
 */
bool 
TELIT_deletMessagesCMGD
(
	const char * const whichMessages
);

/**
 *  Sets up ISP information for the context 
 */
bool 
TELIT_setApnCGDCONT
(
	const char * const userSetContextID, 
	const char * const PDPtype,
	const char * const APN,
	const char * const requestedStaticIP, 
	const char * const dataCompression,
	const char * const headerCompression
);

/**
 *  Set the socket connection settings
 */
bool 
TELIT_setTcpIpStackSCFG
(
	const char * const userSetConnectionID,
	const char * const userSetContextID,
	const char * const minPacketSize,
	const char * const globalTimeout, 
	const char * const connectionTimeout,
	const char * const txTimeout
);

/**
 *  Activates/closes (statusOfConnection=1/0) context, gets IP from gateway
 */
const char * const 
TELIT_setContextSGACT
(
	const char * const userSetConnectionID,
	const char * const statusOfConnection,
	const char * const username,
	const char * const password
);

/**
 *  Defines a min quality of service for the telit to send
 */
bool 
TELIT_setQualityCGQMIN
(
	const char * const userSetConnectionID ,
	const char * const precedence,
	const char * const delay,
	const char * const reliability, 
	const char * const peak,
	const char * const mean
);

/**
 *  Requests a specific quality of service from network	
 */
bool 
TELIT_requestQualityCGQREQ
(
	const char * const userSetConnectionID,
	const char * const precedence,
	const char * const delay,
	const char * const reliability, 
	const char * const peak,
	const char * const mean
);

/**
 *  Sets security protocal used with network
 *  0 - no authentication
 *  1 - PAP authentication (factory default)
 *  2 - CHAP authentication   
 */
bool 
TELIT_setSecuritySGACTAUTH
(
	const char * const securitySetting
);

/**
 *  Socket dial opens a socket to remote server.	
 */
bool 
TELIT_socketDialSD
(
	const char * const userSetConnectionID ,
	const char * const protocol,
	const char * const remotePort,
	const char * const ipAddress
);

/**
 *  Suspends listing to socket,socket can still receive data till a SH command is issued to shut the socket
 */
bool 
TELIT_suspendSocket
(
	void
);

/**
 *  Reopens a suspended connection
 */
bool 
TELIT_resumeSocketSO
(
	const char * const whichSocket
);

/**
 *  Closes the socket connection, no data in or out
 */
bool
TELIT_closeSocketSH
(
	const char * const whichSocket
);

/**
 *  View the status of a socket
 *  0 - Socket Closed.
 *  1 - Socket with an active data transfer connection.
 *  2 - Socket suspended.
 *  3 - Socket suspended with pending data.
 *  4 - Socket listening.
 *  5 - Socket with an incoming connection. Waiting for the user accept or shutdown command.
 */
const char * const 
TELIT_socketStatusSS
(
	void
);

/**
 *  Execution command is used to get information about socket data traffic.
 */
const char * const 
TELIT_socketInfoSI
(
	const char * const connectionID
);

/**
 *  Starts listening on a socket
 */
bool 
TELIT_socketListenSL
(
	const char * const connectionID, 
	const char * const listenState,
	const char * const listenPort
);

/**
 *  Accept the connection
 */
bool 
TELIT_socketAcceptSA
(
	const char * const connectionID
);

/**
 *  Constructs and send a GET request on opened socket
 */
const char * const 
TELIT_getHTTP
(
	uint16_t dataSize, 
	const char * const host, 
	const char * const resource, 
	const char * const httpVersion,
	bool keepAlive
);

/**
 *  Constructs and send a POST request on opened socket
 */
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
);

/**
 *  Sets FTP timeouts
 */
bool 
TELIT_ftpTimeOutFTPO
(
	const char* const timeOut
);

/**
 *  Opens a ftp connection
 */
bool 
TELIT_FTPOPEN
(
	const char * const serverPort, 
	const char * const username, 
	const char * password,
	const char * const mode
);

/**
 *  Sets ftp transfer type
 *  1 ASCII
 *  0 BINARY
 */
bool 
TELIT_ftpDataTypeFTPTYPE
(
	const char * const binaryAscii
);

/**
 *  Closes FTP connection
 */
bool
TELIT_FTPCLOSE
(
	void
);

/**
 *  Uploads file into selected file name
 */
bool 
TELIT_FTPPUT
(
	const char * const fileWriteName, 
	const char * const data
);

/**
 *  Gets a specific file
 */
const char * const 
TELIT_FTPGET
(
	const char * const fileName,
	uint16_t dataSize
);

/**
 *  Changes directory
 */
bool 
TELIT_changeDirFTPCWD
(
	const char * const directory
);

/************************************************************************/
/* Public prototypes.                                                   */
/************************************************************************/

/************************************************************************/
/* Public variables.                                                    */
/************************************************************************/

/************************************************************************/
/* Public Macros.                                                       */
/************************************************************************/

#endif  /* _GL_865 */