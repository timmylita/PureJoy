/**
 * \file bootloader.h
 * \author tom.m
 *
 * Replace all occurences with case sensitivity for BOOTLOADER, Bootloader
 * and bootloader.  Note, an entry must be added to the Task_Id_t enumerator
 * and Tasks[Task_Id_Bootloader] = BOOTLOADER_Init(Task_Id_Bootloader); called
 * in main().  Then replace this paragraph with some useful information
 * about the purpose and implementation of the task.
 */
 
#ifndef _BOOTLOADER_H
#define _BOOTLOADER_H

/************************************************************************/
/* Includes.                                                            */
/************************************************************************/

/************************************************************************/
/* Definitions.                                                         */
/************************************************************************/

#define _reboot_reason_address          0x00fa
#define _reset_reason_address           0x00fb

/************************************************************************/
/* Types.                                                               */
/************************************************************************/

typedef enum
{
    BOOT_Reason_Unintentional     = 0x00,
    BOOT_Reason_Assert            = 0x01,
    BOOT_Reason_Request           = 0x02,
    BOOT_Reason_Otap              = 0x03,
    BOOT_Reason_Error             = 0x04,
    BOOT_Reason_Boot              = 0x05,
    BOOT_Reason_New               = 0xff
} BOOT_Reason_t;

/************************************************************************/
/* Public prototypes.                                                   */
/************************************************************************/

void BOOTLOADER_Run (void);

/************************************************************************/
/* Public variables.                                                    */
/************************************************************************/

/************************************************************************/
/* Public Macros.                                                       */
/************************************************************************/

#endif // _BOOTLOADER_H