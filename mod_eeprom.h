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
//-----------------------------------------------------------------------------
// The FPEC block handles the program and erase operations of the Flash memory.
// The FPEC consists of seven 32-bit registers.
// . FPEC key register (FLASH_KEYR)
// . Option byte key register (FLASH_OPTKEYR)
// . Flash control register (FLASH_CR)
// . Flash status register (FLASH_SR)
// . Flash address register (FLASH_AR)
// . Option byte register (FLASH_OBR)
// . Write protection register (FLASH_WRPR)
//
// After reset, the FPEC block and so the FLASH_CR register are locked.
// To unlock the FPEC block, where two key values (KEY1 and KEY2) must be
// written to the FLASH_KEYR.  Any wrong sequence locks up the FPEC block
// and FLASH_CR register until the next reset.
//
// FLASH_SR register :
// Bits 31:6 Reserved, must be kept cleared.
// Bit 5EOP: End of operation
// Set by hardware when a Flash operation (programming / erase) is completed. Reset by
// writing a 1
// Note: EOP is asserted at the end of each successful program or erase operation
// Bit 4WRPRTERR: Write protection error
// Set by hardware when programming a write-protected address of the Flash memory.
// Reset by writing 1.
// Bit 3 Reserved, must be kept cleared.
// Bit 2PGERR: Programming error
// Set by hardware when an address to be programmed contains a value different from
// '0xFFFF' before programming.
// Reset by writing 1.
// Note: The STRT bit in the FLASH_CR register should be reset before starting a programming
// operation.
// Bit 1 Reserved, must be kept cleared
// Bit 0BSY: Busy
// This indicates that a Flash operation is in progress. This is set on the beginning of a Flash
// operation and reset when the operation finishes or when an error occurs
///////////////////////////////////////////////////////////////////////////////

#pragma once

typedef enum
	{
	FLASH_COMPLETE = 0,
	FLASH_BUSY,
	FLASH_ERROR_WRITE,
	FLASH_ERROR_PG,
	FLASH_ERROR_WRP,
	FLASH_ERROR_OPT,
	FLASH_ERROR_BAD_PAGE_SIZE,
	FLASH_BAD_ADDRESS
	} FLASH_Status;

#define IS_FLASH_ADDRESS(ADDRESS) (((ADDRESS) >= 0x08000000) && ((ADDRESS) < 0x0807FFFF))
#define FLASH_KEY1  0x45670123
#define FLASH_KEY2  0xCDEF89AB


///////////////////////////////////////////////////////////////////////////////
//  FUNCTIONS PROTOTYPES
///////////////////////////////////////////////////////////////////////////////
// Low level functions
void  __O_SMALL  FLASH_Unlock();
void  __O_SMALL  FLASH_Lock();
void  __O_SMALL FLASH_WaitEndOfOperation();
FLASH_Status __O_SMALL FLASH_ErasePage(uint32);
FLASH_Status __O_SMALL FLASH_ProgramHalfWord(uint32, uint16);
FLASH_Status __O_SMALL FLASH_WritePage(uint8_t,uint8_t *,uint16_t);
boolean __O_SMALL FLASH_DiffPage(uint8_t,uint8_t *,uint16_t);

// High level functions
void __O_SMALL EE_PrmLoad();
void __O_SMALL EE_PrmSave();
void __O_SMALL EE_PrmInit(bool factorySettings=false);

// EEPROM emulation functions
void __O_SMALL EEPROM_Update(uint8_t* ,uint16_t );
void __O_SMALL EEPROM_Get(uint8_t* ,uint16_t, uint16_t);
void __O_SMALL EEPROM_Format();
void __O_SMALL EEPROM_FlashMemoryDump(uint8_t , uint8_t );

///////////////////////////////////////////////////////////////////////////////
// STM32F103 flash memory FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Unlock FPEC block
///////////////////////////////////////////////////////////////////////////////
void FLASH_Unlock()
{
	// If already unlocked, do nothing
	if ( FLASH_BASE->CR & FLASH_CR_LOCK ) {
		FLASH_BASE->KEYR = FLASH_KEY1;
		FLASH_BASE->KEYR = FLASH_KEY2;
	}
}

///////////////////////////////////////////////////////////////////////////////
// lock FPEC block
///////////////////////////////////////////////////////////////////////////////
void FLASH_Lock()
{
	FLASH_BASE->CR |= FLASH_CR_LOCK;
}

///////////////////////////////////////////////////////////////////////////////
// Wait end of current flash operaiton. return false if timeout reached.
///////////////////////////////////////////////////////////////////////////////
void FLASH_WaitEndOfOperation() {
	while ( (FLASH_BASE->SR & FLASH_SR_BSY) ) ;
}

///////////////////////////////////////////////////////////////////////////////
// Erase a flash memory page
//-----------------------------------------------------------------------------
// . Check the BSY bit in the FLASH_SR register
// . Set the PER bit in the FLASH_CR register
// . Program the FLASH_AR register to select a page to erase
// . Set the STRT bit in the FLASH_CR register
// . Wait for the BSY bit to be reset
// . Read the erased page and verify (eventually)
///////////////////////////////////////////////////////////////////////////////
FLASH_Status FLASH_ErasePage(uint32 pageAddress)
{
	if ( ! IS_FLASH_ADDRESS(pageAddress) ) return FLASH_BAD_ADDRESS;

	FLASH_WaitEndOfOperation();

	FLASH_Status status = FLASH_COMPLETE;

	FLASH_Unlock(); FLASH_WaitEndOfOperation();

	// Erase the page
	FLASH_BASE->CR |= FLASH_CR_PER;
	FLASH_BASE->AR =  pageAddress;
	FLASH_BASE->CR = FLASH_CR_STRT | FLASH_CR_PER;;
	FLASH_WaitEndOfOperation();

	// PER bit in CR register must be cleared to allows other operations on flash to be run after that
	FLASH_BASE->CR &= ~FLASH_CR_PER;
	FLASH_BASE->SR = (FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR);

	FLASH_Lock();	FLASH_WaitEndOfOperation();

	return status;
}

///////////////////////////////////////////////////////////////////////////////
// Program a half word 16 bits value
//-----------------------------------------------------------------------------
// . Check the BSY bit in the FLASH_SR register.
// . Set the PG bit in the FLASH_CR register.
// . Perform the data write (half-word) at the desired address.
// . Wait for the BSY bit to be reset.
// . Read the programmed value and verify.
///////////////////////////////////////////////////////////////////////////////
FLASH_Status FLASH_ProgramHalfWord(uint32 writeAddress, uint16 value )
{

	if ( ! IS_FLASH_ADDRESS(writeAddress) ) return FLASH_BAD_ADDRESS;

	// If value not changed, nothing to do...
	if ( value == *(uint16*)writeAddress ) return FLASH_COMPLETE;

	FLASH_WaitEndOfOperation();

	FLASH_Status status = FLASH_COMPLETE;
	FLASH_Unlock(); FLASH_WaitEndOfOperation() ;

	FLASH_BASE->CR |= FLASH_CR_PG;
	*(uint16*)writeAddress = value;

	FLASH_WaitEndOfOperation();
	if (FLASH_BASE->SR & FLASH_SR_PGERR) status = FLASH_ERROR_PG;
	else if ( FLASH_BASE->SR & FLASH_SR_WRPRTERR )status = FLASH_ERROR_WRP;
	//Verify the value written
	else if ( value != *(volatile uint16_t *)writeAddress ) status = FLASH_ERROR_WRITE;

	FLASH_BASE->CR &= ~FLASH_CR_PG;
	FLASH_BASE->SR = (FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR);

	FLASH_Lock();	FLASH_WaitEndOfOperation();

	return status;
}

///////////////////////////////////////////////////////////////////////////////
// Write bytes to a page of the flash memory
//////////////////////////////////////////////////////////////////////////////
FLASH_Status FLASH_WritePage(uint8_t page, uint8_t *bloc,uint16_t sz)
{
	// The page must be prepared before the call.
	if ( sz < 2 || sz > EE_PAGE_SIZE ) return FLASH_ERROR_BAD_PAGE_SIZE;

	// No necessary to write when no differences
	if ( ! FLASH_DiffPage(page, bloc,sz) ) return FLASH_COMPLETE;

	FLASH_WaitEndOfOperation();

	uint32_t volatile addressWrite = (EE_FLASH_MEMORY_BASE + page * EE_PAGE_SIZE) ;

	FLASH_Status status = FLASH_ErasePage(addressWrite);
	if ( status != FLASH_COMPLETE ) return status;

// A byte is 8 bits, but flash write operation is mandatory 16 bit.
	uint16_t *pAddress = (uint16_t *) addressWrite;
	uint16_t *pValue   = (uint16_t *) bloc ;

	uint16_t size = sz / 2 + ( sz % 2 ? 1:0 );
	boolean  cleanLast =  ( (size * 2) > sz );

  while(size) {
		// Write only if differences to preserve the Flash memory
		if ( *pAddress != *pValue ) {
			uint16_t flashValue = 0XFF00;  // 0xFF is the "format" value
			// Write our last value without out of bound garbage byte
			if (size == 1 && cleanLast ) flashValue += *( (uint8_t*)pValue ) ;
			else flashValue = *pValue;
			status = FLASH_ProgramHalfWord((uint32_t)pAddress, flashValue);
			if (  status != FLASH_COMPLETE ) return status ;
		}
		pAddress++; pValue++; size--;
	}
	return status;
}

///////////////////////////////////////////////////////////////////////////////
// Check if differences exist between a page and a buffer
//////////////////////////////////////////////////////////////////////////////
boolean FLASH_DiffPage(uint8_t page, uint8_t *bloc,uint16_t sz)
{
  uint32_t addressRead = EE_FLASH_MEMORY_BASE + page * EE_PAGE_SIZE ;
	return ( memcmp((void *)addressRead,(void *)bloc,sz) != 0 ) ;
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

	// New firmware  uploaded
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

			// Default boot mode when new firmware uploaded only for this session.
			bootMagicWord = BOOT_CONFIG_MAGIC;
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

		// Default I2C Device ID and bus mode
		EE_Prm.I2C_DeviceId = B_MASTERID;
		EE_Prm.I2C_BusModeState = B_DISABLED;

		ResetMidiRoutingRules(ROUTING_RESET_ALL);

		// Set BPM and disable all clocks
		// MTC was initialized to false above with the first memset
		SetMidiBpmClock(0x7F,DEFAULT_BPM);
		SetMidiEnableClock(0x7F,false);


	  EE_Prm.vendorID  = USB_MIDI_VENDORID;
    EE_Prm.productID = USB_MIDI_PRODUCTID;

    memcpy(EE_Prm.productString,USB_MIDI_PRODUCT_STRING,sizeof(USB_MIDI_PRODUCT_STRING));

		//Write the whole param struct
    EE_PrmSave();

		// Default boot mode when new firmware uploaded (not saved as one shot mode)
		bootMagicWord = BOOT_CONFIG_MAGIC;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Update a bytes buffer to FLASH EMULATED EEPROM avoiding writes if possible.
// Use it with significant size buffer and low frequency updates rate.
//////////////////////////////////////////////////////////////////////////////
void EEPROM_Update(uint8_t* bloc,uint16_t sz)
{
  // Nothing to do if size above Capacity
  if ( sz <2 || sz > EE_CAPACITY )  return;

	uint16_t nbPageWrite = ( sz / EE_PAGE_SIZE ) + ( sz % EE_PAGE_SIZE ? 1:0);

	for ( uint16_t page = 0 ; page != nbPageWrite ; page++ ) {
			uint16_t NbBytesWrite = (sz > EE_PAGE_SIZE ? EE_PAGE_SIZE : sz);
			FLASH_WritePage(page + EE_PAGE_BASE, bloc, NbBytesWrite );
			bloc += NbBytesWrite;
			sz -= NbBytesWrite;
	}
}

///////////////////////////////////////////////////////////////////////////////
// Get byte buffer from EEPROM
//////////////////////////////////////////////////////////////////////////////
void EEPROM_Get(uint8_t* bloc,uint16_t sz, uint16_t offset)
{
  // Check if size above Capacity
  if ( sz > EE_CAPACITY ) return ;

  uint32_t addressRead = EE_BASE + offset;

	for (uint16_t idx = 0 ; idx < sz; idx++) {
      *bloc++ = (*(__IO uint8*)addressRead++);
  }

}

///////////////////////////////////////////////////////////////////////////////
// Format EEPROM
//////////////////////////////////////////////////////////////////////////////
void EEPROM_Format() {

  uint32_t addressWrite = EE_BASE;

	for (uint8_t i = 0; i != EE_NBPAGE ; i ++ ) {
			FLASH_ErasePage(addressWrite);
    	addressWrite += EE_PAGE_SIZE;
	}
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
      if ( idx % EE_PAGE_SIZE == 0 )
				SerialPrintf("Page %4d - EE_PAGE_SIZE = 0x%04x.%n", startPage++,EE_PAGE_SIZE);
      if (c == 0 ) SerialPrintf("%08x : ",addressRead);
      b = (*(__IO uint8*)addressRead++);
			SerialPrintf("%02x ",b);
      asciiBuff[c++] = ( b >= 0x20 && b< 127? b : '.' ) ;
      if ( c == 16 ) {
        asciiBuff[c] = 0;
        c = 0;
        SerialPrintf(" | %s%n", &asciiBuff[0]);
      }
  }
}
