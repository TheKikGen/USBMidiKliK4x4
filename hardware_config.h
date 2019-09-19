/*
  USB MidiKliK 4X4 - USB MIDI 4 IN X 4 OUT firmware
  Based on the MIDITECH / MIDIPLUS 4X4 harware.
  Copyright (C) 2017/2018 by The KikGen labs.

  SOME HARDWARE CONFIGURATION

  ------------------------   CAUTION  ----------------------------------
  THIS NOT A COPY OR A HACK OF ANY EXISTING MIDITECH/MIDIPLUS FIRMWARE.
  THAT FIRMWARE WAS ENTIRELY CREATED FROM A WHITE PAGE, WITHOUT
  DISASSEMBLING ANY SOFTWARE FROM MIDITECH/MIDIPLUS.

  UPLOADING THIS FIRMWARE TO YOUR MIDIPLUS/MIDITECH 4X4 USB MIDI
  INTERFACE  WILL PROBABLY CANCEL YOUR WARRANTY.

  IT WILL NOT BE POSSIBLE ANYMORE TO UPGRADE THE MODIFIED INTERFACE
  WITH THE MIDITECH/MIDIPLUS TOOLS AND PROCEDURES. NO ROLLBACK.

  THE AUTHOR DISCLAIM ANY DAMAGES RESULTING OF MODIFYING YOUR INTERFACE.
  YOU DO IT AT YOUR OWN RISKS.
  ---------------------------------------------------------------------

  This file is part of the USBMIDIKLIK-4x4 distribution
  https://github.com/TheKikGen/USBMidiKliK4x4
  Copyright (c) 2018 TheKikGen Labs team.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, version 3.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.

*/
#ifndef _HARDWARE_CONFIG_H_
#define _HARDWARE_CONFIG_H_
#pragma once

#ifdef MCU_STM32F103RC
  #warning "MIDITECH OR MCU_STM32F103RC HARDWARE DETECTED"
  
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
     #define HARDWARE_TYPE "STM32F103RC"
     #define LED_CONNECT PC13 
  #endif

#else 
  #if defined(MCU_STM32F103C8) || defined(MCU_STM32F103CB)
    #warning "BLUEPILL HARDWARE DETECTED"
    #define HARDWARE_TYPE "BLUEPILL STMF103C8x"
    #define SERIAL_INTERFACE_MAX  3
    #define SERIALS_PLIST &Serial1,&Serial2,&Serial3    
    #define LED_CONNECT PC13
  #else
   #error "PLEASE CHOOSE STM32F103RC (4 serial ports) or STM32F103RC (3 serial ports) variants to compile ."
  #endif
#endif

#endif
