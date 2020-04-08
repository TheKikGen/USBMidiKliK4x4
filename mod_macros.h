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
#pragma once
///////////////////////////////////////////////////////////////////////////////
//  MACRO UTILITIES
///////////////////////////////////////////////////////////////////////////////

// Macro to flash LEDS IN
#ifdef LED_MIDI_SIZE
  #define FLASH_LED_IN(thisLed) LED_Flash(&LED_MidiInTick[thisLed])
  #define FLASH_LED_OUT(thisLed) LED_Flash(&LED_MidiOutTick[thisLed])
#else
  #define FLASH_LED_IN(thisLed) LED_Flash(&LED_ConnectTick)
  #define FLASH_LED_OUT(thisLed) LED_Flash(&LED_ConnectTick)
#endif

// Add/Sub constraint used for uint (do not add ; at the end )
#define CONSTRAINT_ADD(a,b,c) if ( (a + b) < c ) a += b; else a = c
#define CONSTRAINT_SUB(a,b,c) if ( (a - b) > c ) a -= b; else a = c

// Macro to compute the max serial port in bus mode or not.
#define SERIAL_INTERFACE_COUNT (EE_Prm.I2C_BusModeState == B_ENABLED ? B_SERIAL_INTERFACE_MAX:SERIAL_INTERFACE_MAX)

// Macro to compute if Master/Slave On Bus
#define B_IS_MASTER (EE_Prm.I2C_BusModeState == B_ENABLED && EE_Prm.I2C_DeviceId == B_MASTERID)
#define B_IS_SLAVE (EE_Prm.I2C_BusModeState == B_ENABLED && EE_Prm.I2C_DeviceId != B_MASTERID)

// Macro to compute a device Id from a serial ports
#define GET_DEVICEID_FROM_SERIALNO(s) ((s) / SERIAL_INTERFACE_MAX + B_MASTERID)

// Macro to compute a "virtual bus serial port" from a local device and serial port
#define GET_BUS_SERIALNO_FROM_LOCALDEV(d,s) ((s) + (d-B_MASTERID) * SERIAL_INTERFACE_MAX)
