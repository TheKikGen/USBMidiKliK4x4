/*
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
#ifdef LEDS_MIDI
  #define FLASH_LED_IN(thisLed) flashLED_IN[thisLed]->start()
  #define FLASH_LED_OUT(thisLed) flashLED_OUT[thisLed]->start()
#else
  #define FLASH_LED_IN(thisLed) flashLED_CONNECT->start()
  #define FLASH_LED_OUT(thisLed) flashLED_CONNECT->start()
#endif

// Macro to compute the max serial port in bus mode or not.
#define SERIAL_INTERFACE_COUNT (EEPROM_Params.I2C_BusModeState == B_ENABLED ? B_SERIAL_INTERFACE_MAX:SERIAL_INTERFACE_MAX)

// Macro to compute if Master/Slave On Bus
#define B_IS_MASTER (EEPROM_Params.I2C_BusModeState == B_ENABLED && EEPROM_Params.I2C_DeviceId == B_MASTERID)
#define B_IS_SLAVE (EEPROM_Params.I2C_BusModeState == B_ENABLED && EEPROM_Params.I2C_DeviceId != B_MASTERID)

// Macro to compute a device Id from a serial ports
#define GET_DEVICEID_FROM_SERIALNO(s) ((s) / SERIAL_INTERFACE_MAX + B_MASTERID)

// Macro to compute a "virtual bus serial port" from a local device and serial port
#define GET_BUS_SERIALNO_FROM_LOCALDEV(d,s) ((s) + (d-B_MASTERID) * SERIAL_INTERFACE_MAX)

// Macro for debugging purpose when MIDI active
#define DEBUG_SERIAL Serial3
#ifdef DEBUG_MODE
  // Timer used to display debug msg and avoid com overflow
  PulseOut I2C_DebugTimer(0xFF,1000);

  #define DEBUG_PRINT(txt,val) if (midiUSBLaunched) { DEBUG_SERIAL.print((txt));DEBUG_SERIAL.print((val));} else {Serial.print((txt));Serial.print((val));}
  #define DEBUG_PRINTLN(txt,val) if (midiUSBLaunched) { DEBUG_SERIAL.print((txt));DEBUG_SERIAL.println((val));} else {Serial.print((txt));Serial.println((val));}
  #define DEBUG_PRINT_BIN(txt,val) if (midiUSBLaunched) { DEBUG_SERIAL.print((txt));DEBUG_SERIAL.print((val),BIN);} else {Serial.print((txt));Serial.print((val),BIN);}
  #define DEBUG_PRINT_HEX(txt,val) if (midiUSBLaunched) { DEBUG_SERIAL.print((txt));DEBUG_SERIAL.print((val),HEX);} else {Serial.print((txt));Serial.print((val),HEX);}
  #define DEBUG_DUMP(buff,sz) if (midiUSBLaunched) { ShowBufferHexDumpDebugSerial(buff,sz);} else { ShowBufferHexDump(buff,sz);}
  #define DEBUG_ASSERT(cond,txt,val) if ( (cond) ) { DEBUG_PRINTLN(txt,val) }
  #define DEBUG_BEGIN if ( EEPROM_Params.debugMode) {
  #define DEBUG_BEGIN_TIMER if ( EEPROM_Params.debugMode && I2C_DebugTimer.start() ) {
  #define DEBUG_END }
#else
  #define DEBUG_PRINT(txt,val)
  #define DEBUG_PRINTLN(txt,val)
  #define DEBUG_PRINT_BIN(txt,val)
  #define DEBUG_PRINT_HEX(txt,val)
  #define DEBUG_DUMP(buff,sz)
  #define DEBUG_ASSERT(cond,txt,val)
  #define DEBUG_BEGIN
  #define DEBUG_BEGIN_TIMER
  #define DEBUG_END
#endif
