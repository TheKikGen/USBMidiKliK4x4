/*
__ __| |           |  /_) |     ___|             |           |
  |   __ \   _ \  ' /  | |  / |      _ \ __ \   |      _` | __ \   __|
  |   | | |  __/  . \  |   <  |   |  __/ |   |  |     (   | |   |\__ \
 _|  _| |_|\___| _|\_\_|_|\_\\____|\___|_|  _| _____|\__,_|_.__/ ____/
  -----------------------------------------------------------------------------
  USBMIDIKLIK 4X4 - USB Midi advanced firmware for STM32F1 platform.
  Copyright (C) 2019 by The KikGen labs.
  LICENCE CREATIVE COMMONS - Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)

  This file is part of the USBMIDIKLIK-4x4 distribution
  https://github.com/TheKikGen/USBMidiKliK4x4
  Copyright (c) 2019 TheKikGen Labs team.
  -----------------------------------------------------------------------------
  Disclaimer.

  This work is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License.
  To view a copy of this license, visit http://creativecommons.org/licenses/by-nc/4.0/
  or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

  NON COMMERCIAL - PERSONAL USE ONLY : You may not use the material for pure
  commercial closed code solution without the licensor permission.

  You are free to copy and redistribute the material in any medium or format,
  adapt, transform, and build upon the material.

  You must give appropriate credit, a link to the github site
  https://github.com/TheKikGen/USBMidiKliK4x4 , provide a link to the license,
  and indicate if changes were made. You may do so in any reasonable manner,
  but not in any way that suggests the licensor endorses you or your use.

  You may not apply legal terms or technological measures that legally restrict
  others from doing anything the license permits.

  You do not have to comply with the license for elements of the material
  in the public domain or where your use is permitted by an applicable exception
  or limitation.

  No warranties are given. The license may not give you all of the permissions
  necessary for your intended use.  This program is distributed in the hope that
  it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*/
///////////////////////////////////////////////////////////////////////////////
//  EEPROM UTILITIES
//-----------------------------------------------------------------------------
// Due to the unusual make process of Arduino platform, this file is directly
// included in the main ino one.
// EEPROM library is not used anymore due to low frequenry rate of update of
// setting.  To avoid conflicts with that library, some core ST flash FUNCTIONS
// are included here directly in the source code.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include "libmaple/util.h"
#include "libmaple/flash.h"

typedef enum
	{
	FLASH_BUSY = 1,
	FLASH_ERROR_PG,
	FLASH_ERROR_WRP,
	FLASH_ERROR_OPT,
	FLASH_COMPLETE,
	FLASH_TIMEOUT,
	FLASH_BAD_ADDRESS
	} FLASH_Status;

#define IS_FLASH_ADDRESS(ADDRESS) (((ADDRESS) >= 0x08000000) && ((ADDRESS) < 0x0807FFFF))

#define FLASH_KEY1			((uint32)0x45670123)
#define FLASH_KEY2			((uint32)0xCDEF89AB)

/* Delay definition */
#define ERASE_TIMEOUT		((uint32)0x00000FFF)
#define PROGRAM_TIMEOUT	((uint32)0x0000001F)

///////////////////////////////////////////////////////////////////////////////
//  FUNCTIONS PROTOTYPES
///////////////////////////////////////////////////////////////////////////////
FLASH_Status FLASH_WaitForLastOperation(uint32 Timeout) __attribute__((optimize("-Os")));
FLASH_Status FLASH_ErasePage(uint32 Page_Address) __attribute__((optimize("-Os")));
FLASH_Status FLASH_ProgramHalfWord(uint32 Address, uint16 Data) __attribute__((optimize("-Os")));

void FLASH_Unlock(void) __attribute__((optimize("-Os")));
void FLASH_Lock(void) __attribute__((optimize("-Os")));

void EE_PrmInit(bool factorySettings=false) __attribute__((optimize("-Os")));
void EEPROM_Update(uint8_t* ,uint16_t ) __attribute__((optimize("-Os")));
void EEPROM_Get(uint8_t* ,uint16_t, uint16_t) __attribute__((optimize("-Os")));
void EE_PrmLoad() __attribute__((optimize("-Os")));
void EE_PrmSave() __attribute__((optimize("-Os")));
void EEPROM_Format() __attribute__((optimize("-Os")));
void EEPROM_FlashMemoryDump(uint8_t , uint8_t ) __attribute__((optimize("-Os")));
boolean EEPROM_DiffPage(uint8_t , uint8_t) __attribute__((optimize("-Os")));

// External
boolean TransPacketPipeline_ClearSlot(uint8_t pipelineSlot);

///////////////////////////////////////////////////////////////////////////////
// Original STM32 core flash FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

/**
  * @brief  Inserts a time delay.
  * @param  None
  * @retval None
  */
static void delay(void)
{
	__IO uint32 i = 0;
	for(i = 0xFF; i != 0; i--) { }
}

/**
  * @brief  Returns the FLASH Status.
  * @param  None
  * @retval FLASH Status: The returned value can be:
  *   FLASH_BUSY,
  *   FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP
  *   FLASH_OPT
  *   FLASH_COMPLETE
  */
FLASH_Status FLASH_GetStatus(void)
{
	if ((FLASH_BASE->SR & FLASH_SR_BSY) == FLASH_SR_BSY)
		return FLASH_BUSY;

	if ((FLASH_BASE->SR & FLASH_SR_PGERR) != 0)
		return FLASH_ERROR_PG;

	if ((FLASH_BASE->SR & FLASH_SR_WRPRTERR) != 0 )
		return FLASH_ERROR_WRP;

	if ((FLASH_BASE->SR & FLASH_OBR_OPTERR) != 0 )
		return FLASH_ERROR_OPT;

	return FLASH_COMPLETE;
}

/**
  * @brief  Waits for a Flash operation to complete or a TIMEOUT to occur.
  * @param  Timeout: FLASH progamming Timeout
  * @retval FLASH Status: The returned value can be :
  *   any from FLASH_GetStatus(),
  *   or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_WaitForLastOperation(uint32 Timeout)
{
	FLASH_Status status;

	/* Check for the Flash Status */
	status = FLASH_GetStatus();
	/* Wait for a Flash operation to complete or a TIMEOUT to occur */
	while ((status == FLASH_BUSY) && (Timeout != 0x00))
	{
		delay();
		status = FLASH_GetStatus();
		Timeout--;
	}
	if (Timeout == 0)
		status = FLASH_TIMEOUT;
	/* Return the operation status */
	return status;
}

/**
  * @brief  Erases a specified FLASH page.
  * @param  Page_Address: The page address to be erased.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_ErasePage(uint32 Page_Address)
{
	FLASH_Status status = FLASH_COMPLETE;
	/* Check the parameters */
	ASSERT(IS_FLASH_ADDRESS(Page_Address));
	/* Wait for last operation to be completed */
	status = FLASH_WaitForLastOperation(ERASE_TIMEOUT);

	if(status == FLASH_COMPLETE)
	{
		/* if the previous operation is completed, proceed to erase the page */
		FLASH_BASE->CR |= FLASH_CR_PER;
		FLASH_BASE->AR = Page_Address;
		FLASH_BASE->CR |= FLASH_CR_STRT;

		/* Wait for last operation to be completed */
		status = FLASH_WaitForLastOperation(ERASE_TIMEOUT);
		if(status != FLASH_TIMEOUT)
		{
			/* if the erase operation is completed, disable the PER Bit */
			FLASH_BASE->CR &= ~FLASH_CR_PER;
		}
		FLASH_BASE->SR = (FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR);
	}
	/* Return the Erase Status */
	return status;
}

/**
  * @brief  Programs a half word at a specified address.
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_ProgramHalfWord(uint32 Address, uint16 Data)
{
	FLASH_Status status = FLASH_BAD_ADDRESS;

	if (IS_FLASH_ADDRESS(Address))
	{
		/* Wait for last operation to be completed */
		status = FLASH_WaitForLastOperation(PROGRAM_TIMEOUT);
		if(status == FLASH_COMPLETE)
		{
			/* if the previous operation is completed, proceed to program the new data */
			FLASH_BASE->CR |= FLASH_CR_PG;
			*(__IO uint16*)Address = Data;
			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(PROGRAM_TIMEOUT);
			if(status != FLASH_TIMEOUT)
			{
				/* if the program operation is completed, disable the PG Bit */
				FLASH_BASE->CR &= ~FLASH_CR_PG;
			}
			FLASH_BASE->SR = (FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR);
		}
	}
	return status;
}

/**
  * @brief  Unlocks the FLASH Program Erase Controller.
  * @param  None
  * @retval None
  */
void FLASH_Unlock(void)
{

	FLASH_BASE->KEYR = FLASH_KEY1;
	FLASH_BASE->KEYR = FLASH_KEY2;
  FLASH_WaitForLastOperation(ERASE_TIMEOUT);
}

/**
  * @brief  Locks the FLASH Program Erase Controller.
  * @param  None
  * @retval None
  */
void FLASH_Lock(void)
{
	/* Set the Lock Bit to lock the FPEC and the FCR */
	FLASH_BASE->CR |= FLASH_CR_LOCK;
}

///////////////////////////////////////////////////////////////////////////////
// Intialize parameters stores in EEPROM
//----------------------------------------------------------------------------
// Retrieve global parameters from EEPROM, or Initalize it
// If factorySetting is true, all settings will be forced to factory default
//////////////////////////////////////////////////////////////////////////////
void EE_PrmInit(bool factorySettings)
{

  // Read the EEPROM parameters structure
  EE_PrmLoad();

  // If the signature is not found, of not the same version of parameters structure,
  // or new version, or new size then initialize (factory settings)

	// New fimware  uploaded
	if ( memcmp(EE_Prm.TimestampedVersion,TimestampedVersion,sizeof(EE_Prm.TimestampedVersion)) )
	{
		// Update timestamp and activate

		if  ( memcmp( EE_Prm.signature,EE_SIGNATURE,sizeof(EE_Prm.signature) )
						|| EE_Prm.prmVersion != EE_PRMVER
						|| ( EE_Prm.majorVersion != VERSION_MAJOR && EE_Prm.minorVersion != VERSION_MINOR )
						|| EE_Prm.size != sizeof(EEPROM_Prm_t)
				) factorySettings = true;
		else
		// New build only. We keep existing settings but reboot in config mode
 		{
			memcpy( EE_Prm.TimestampedVersion,TimestampedVersion,sizeof(EE_Prm.TimestampedVersion) );

			// Default boot mode when new firmware uploaded
	    EE_Prm.nextBootMode = bootModeConfigMenu;
			return;

		}

	}

	// Force factory setting
  if (  factorySettings )
	{
    memset( &EE_Prm,0,sizeof(EEPROM_Prm_t) );
    memcpy( EE_Prm.signature,EE_SIGNATURE,sizeof(EE_Prm.signature) );

		EE_Prm.majorVersion = VERSION_MAJOR;
		EE_Prm.minorVersion = VERSION_MINOR;

		EE_Prm.prmVersion = EE_PRMVER;
		EE_Prm.size = sizeof(EEPROM_Prm_t);

    memcpy( EE_Prm.TimestampedVersion,TimestampedVersion,sizeof(EE_Prm.TimestampedVersion) );

    EE_Prm.debugMode = false;

		// Default I2C Device ID and bus mode
		EE_Prm.I2C_DeviceId = B_MASTERID;
		EE_Prm.I2C_BusModeState = B_DISABLED;

		ResetMidiRoutingRules(ROUTING_RESET_ALL);

	  EE_Prm.vendorID  = USB_MIDI_VENDORID;
    EE_Prm.productID = USB_MIDI_PRODUCTID;

    memcpy(EE_Prm.productString,USB_MIDI_PRODUCT_STRING,sizeof(USB_MIDI_PRODUCT_STRING));

		EE_Prm.nextBootMode = bootModeMidi;

		//Write the whole param struct
    EE_PrmSave();

		// Default boot mode when new firmware uploaded (not saved as one shot mode)
		EE_Prm.nextBootMode = bootModeConfigMenu;

  }

}

///////////////////////////////////////////////////////////////////////////////
// Check if differences exist between an EEPROM page and a buffer
//////////////////////////////////////////////////////////////////////////////
boolean EEPROM_DiffPage(uint8_t page, uint8_t *bloc,uint16_t sz) {
  uint32_t addressRead = EE_FLASH_MEMORY_BASE + page * EE_PAGE_SIZE ;
  for (uint16_t n = 0 ; n < sz && n < EE_PAGE_SIZE ; n ++ ) {
      if ( (*(__IO uint8*)addressRead++) != *bloc++ ) return true;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////
// Update a bytes buffer to FLASH EMULATED EEPROM avoiding writes if possible.
// Use it with significant size buffer and low frequency updates rate.
//////////////////////////////////////////////////////////////////////////////
void EEPROM_Update(uint8_t* bloc,uint16_t sz)
{
  // Nothing to do if size above Capacity
  if ( sz > EE_CAPACITY )  return;

  uint32_t addressWrite = EE_BASE;

  union  {
      uint16_t u16;
      uint8_t  u8[2];
  } val ;

  uint8_t* pp = bloc;
  uint8_t page = 0;

  // Unlock the flash controller
  FLASH_Unlock();

  val.u16 = 0;

  // Write 2 uint8_t in an uint16t
  for (uint16_t offset = 0; offset < sz; offset+=2) {

    // Check if that page needs to be updated
    if ( offset % EE_PAGE_SIZE == 0 ) {
        // Compare current buffer with page EEPROM content
        if ( EEPROM_DiffPage(EE_PAGE_BASE + page, pp, sz-offset ) ) {
          page++;
          FLASH_ErasePage(addressWrite);
        } else {
          // Skip that page
          page++;
          addressWrite += EE_PAGE_SIZE;
          offset += EE_PAGE_SIZE;
          pp += EE_PAGE_SIZE;
          continue;
        }
    }

    val.u16 = 0;
    val.u8[0] = *pp++;

    // last byte, odd size?
    if ( offset+1 < sz ) val.u8[1] = *pp++;

    // Write the 16 bits value in the page
    if ( (*(__IO uint8*)addressWrite) != val.u16 )
        FLASH_ProgramHalfWord(addressWrite, val.u16);

    addressWrite += 2;
  }
  FLASH_Lock();
}

///////////////////////////////////////////////////////////////////////////////
// Get byte buffer from EEPROM
//////////////////////////////////////////////////////////////////////////////
void EEPROM_Get(uint8_t* bloc,uint16_t sz, uint16_t offset)
{
  // Nothing to do if size above Capacity
  if ( sz > EE_CAPACITY )  return;

  uint32_t addressRead = EE_BASE + offset;

	for (uint16_t idx = 0 ; idx < sz; idx++) {
      *bloc++ = (*(__IO uint8*)addressRead++);
  }

}

///////////////////////////////////////////////////////////////////////////////
// HIGH LEVEL LOAD PARAMETERS FROM EEPROM
//----------------------------------------------------------------------------
// High level abstraction parameters read function
//////////////////////////////////////////////////////////////////////////////
void EE_PrmLoad()
{
	EEPROM_Get((uint8*)&EE_Prm,sizeof(EEPROM_Prm_t),0);
}

///////////////////////////////////////////////////////////////////////////////
// HIGH LEVEL SAVE PARAMETERS TO EEPROM
//----------------------------------------------------------------------------
// High level abstraction parameters read function
//////////////////////////////////////////////////////////////////////////////
void EE_PrmSave()
{
	EEPROM_Update((uint8*)&EE_Prm,sizeof(EEPROM_Prm_t)) ;
}

///////////////////////////////////////////////////////////////////////////////
// Format EEPROM
//////////////////////////////////////////////////////////////////////////////
void EEPROM_Format() {

  uint32_t addressWrite = EE_BASE;

  // Unlock the flash controller
  FLASH_Unlock();

	for (uint16_t idx = 0; idx < EE_CAPACITY; idx +=2 ) {

    // Erase the page if we reach a page size or base
    if ( idx % EE_PAGE_SIZE == 0 ) FLASH_ErasePage(addressWrite);

    // Write the 16 bits value in the page
    FLASH_ProgramHalfWord(addressWrite, (uint16_t) 0xFFFF);
    addressWrite += 2;
	}

  FLASH_Lock();
}

///////////////////////////////////////////////////////////////////////////////
// DUMP FLASH MEMORY BY PAGE (First page is 0)
// To DUMP EEPROM, use EE_PAGE_BASE,EE_NBPAGE.  16 pages max at a time..
//////////////////////////////////////////////////////////////////////////////
void EEPROM_FlashMemoryDump(uint8_t startPage, uint8_t nbPage) {

  if (nbPage > 16 ) return;

  uint32_t addressRead = EE_FLASH_MEMORY_BASE + startPage * EE_PAGE_SIZE ;
  uint8_t b;
  char asciiBuff[17];
  uint8_t c=0;

  for (uint16_t idx = 0 ; idx < (EE_PAGE_SIZE * nbPage); idx++) {
      if ( idx % EE_PAGE_SIZE == 0 ) {
          Serial.print("Page ");Serial.print(startPage++);
          Serial.print(" - EE_PAGE_SIZE = 0x");Serial.println(EE_PAGE_SIZE,HEX);
      }
      if (c == 0 ) {
          Serial.print(addressRead,HEX);Serial.print(" : ");
      }
      b = (*(__IO uint8*)addressRead++);
      if (b <= 0x0F ) Serial.print("0");
      Serial.print(b,HEX);Serial.print(" ");
      asciiBuff[c++] = ( b >= 0x20 && b< 127? b : '.' ) ;
      if ( c == 16 ) {
        asciiBuff[c] = 0;
        c = 0;
        Serial.print(" | ");Serial.println(&asciiBuff[0]);
      }
  }
}
