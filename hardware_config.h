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

#ifndef _HARDWARE_CONFIG_H_
 #define _HARDWARE_CONFIG_H_
 #pragma once

// Macros to expand preprocessor variables
 #define __VALUE_TO_STRING(x) #x
 #define __VALUE(x) __VALUE_TO_STRING(x)
 #define __VAR_NAME_VALUE(var) #var " = "  __VALUE(var)

// About STM32F103xx microcontrollers :
// Low-density devices have a flash memory between 16 and 32 Kbytes.
// Medium-density devices have a flash memory between 32 and 128 Kbytes.
// High-density devices have a a flash memory between 256 and 512 Kbytes.
// Low and medium-density devices use a 1K bytes page size.
// High-density density devices use a 2K bytes page size.

// Flash memory base address (cf STM32F103xx datasheet page 34)
 #define EE_FLASH_MEMORY_BASE 0x08000000

 #ifdef  MCU_STM32F103RC

  // Set EEPROM parameters for the STMF103RC (high density)
  #define EE_PAGE_SIZE  0x800
  #define EE_FLASH_SIZEK 256
  #define EE_NBPAGE 1
  #define EE_CAPACITY   EE_NBPAGE*EE_PAGE_SIZE

  // Comment the line below for a generic STM32F103RC
  // This drives the DISC pin for USB with the Miditech 4x4
  // and the connect LED pin #.
  // Activated by default.

  #define HAS_MIDITECH_HARDWARE

  #define SERIAL_INTERFACE_MAX  4
  #define SERIALS_PLIST &Serial1,&Serial2,&Serial3,&Serial4
  #ifdef HAS_MIDITECH_HARDWARE
     #define HARDWARE_TYPE "MIDITECH4x4 STM32F103RC"
     #define LED_CONNECT PC9
  #else
     #define HARDWARE_TYPE "STM32F103RC 256K FLASH"
     #define LED_CONNECT PC13
  #endif

 #else
  #if defined(MCU_STM32F103C8) || defined(MCU_STM32F103CB)

    // Set EEPROM parameters for the STMF103Cx
    #define EE_PAGE_SIZE  0x400
    #define EE_NBPAGE 1
    #define EE_CAPACITY   EE_NBPAGE*EE_PAGE_SIZE

    #if defined(MCU_STM32F103C8)
      #define HARDWARE_TYPE "BLUEPILL STMF103C8x 64K FLASH"
      #define EE_FLASH_SIZEK 64
    #else
      #define HARDWARE_TYPE "BLUEPILL STMF103CBx 128K FLASH"
      #define EE_FLASH_SIZEK 128
    #endif

    #define SERIAL_INTERFACE_MAX  3
    #define SERIALS_PLIST &Serial1,&Serial2,&Serial3
    #define LED_CONNECT PC13
  #else
   #error "PLEASE CHOOSE STM32F103RC (4 serial ports) or STM32F103C8/CB (3 serial ports) variants to compile ."
  #endif
 #endif

 #pragma message(__VAR_NAME_VALUE(HARDWARE_TYPE))

 // Reserve the last pages for the EEPROM emulation
 #define EE_BASE EE_FLASH_MEMORY_BASE + EE_FLASH_SIZEK * 1024 - EE_CAPACITY
 #define EE_PAGE_BASE ( EE_BASE - EE_FLASH_MEMORY_BASE ) / EE_PAGE_SIZE

 #define USBCABLE_INTERFACE_MAX USB_MIDI_IO_PORT_NUM

  // USBDM (USB -) PIN
  #define PIN_USBDM PA11

#endif
