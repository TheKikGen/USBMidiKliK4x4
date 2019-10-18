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
     #warning "MIDITECH4X4 STM32F103RC HARDWARE DETECTED"
     #define HARDWARE_TYPE "MIDITECH4x4 STM32F103RC"
     #define LED_CONNECT PC9
  #else
     #warning "STM32F103RC HARDWARE DETECTED"
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

  #define USBCABLE_INTERFACE_MAX USB_MIDI_IO_PORT_NUM

  // USBDM (USB -) PIN
  #define PIN_USBDM PA11

#endif
