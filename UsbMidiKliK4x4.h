/*
  USB MidiKliK 4X4 - USB MIDI 4 IN X 4 OUT firmware
  Based on the MIDITECH / MIDIPLUS 4X4 harware.
  Copyright (C) 2017/2018 by The KikGen labs.

  MAIN INCLUDE

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
#pragma once

#ifdef HAS_MIDITECH_HARDWARE
  #warning "MIDITECH HARDWARE DETECTED"
  #define HARDWARE_TYPE "MIDITECH HARDWARE"
  // Miditech 4X4 has 4 cables and 4 serials
  #define SERIAL_INTERFACE_MAX  4
  #define SERIALS_PLIST &Serial1,&Serial2,&Serial3,&Serial4

  #define LED_CONNECT PC9

#else
  // Assume a "Blue Pill" like configuration
  #warning "BLUE PILL HARDWARE ASSUMED"
  #define HARDWARE_TYPE "BLUE PILL"
  #define SERIAL_INTERFACE_MAX  3
  #define SERIALS_PLIST &Serial1,&Serial2,&Serial3

  // Blue Pill has only one LED...not optimal here...
  #define LED_CONNECT PC13

#endif

// USBDM (USB -) PIN
#define PIN_USBDM PA11

// Timer for attachCompare1Interrupt
#define TIMER2_RATE_MICROS 1000

// Sysex used to set some parameters of the interface.
#define SYSEX_INTERNAL_HEADER 0xF0,0x77,0x77,0x78
#define SYSEX_INTERNAL_BUFF_SIZE 32

// LED light duration in milliseconds
#define LED_PULSE_MILLIS  5

// When MIDI SERIAL is inactive beyond the timeout..
#define MIDI_SERIAL_TIMEOUT_MILLIS  30000

// MIDI Routing
#define FROM_SERIAL 0
#define FROM_USB    1

// Routing from an USB cable OUT
#define DEFAULT_MIDI_CABLE_ROUTING_TARGET  0B00000001,0B00000010,0B00000100,0B00001000

// Routing from a serial MIDI IN
#define DEFAULT_MIDI_SERIAL_ROUTING_TARGET 0B00010000,0B00100000,0B01000000,0B10000000

// Intelligent Serial default MIDI Thru
// No IN actives - 4 midi out - channel messages
#define DEFAULT_INTELLIGENT_MIDI_THRU_OUT 0B1111
#define DEFAULT_INTELLIGENT_MIDI_THRU_IN  0B00000001

// Default number of 15 secs periods to start after USB midi inactivity
// Can be changed by SYSEX
#define DEFAULT_INTELLIGENT_MIDI_THRU_DELAY_PERIOD 2
