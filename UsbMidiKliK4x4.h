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
#ifndef _USBMIDIKLIK4X4_H_
#define _USBMIDIKLIK4X4_H_
#pragma once

#ifdef HAS_MIDITECH_HARDWARE
  #warning "MIDITECH HARDWARE DETECTED"
  #define HARDWARE_TYPE "MIDITECH HARDWARE"
  // Miditech 4X4 has 4 cables and 4 serials
  // NOT BEYOND 4 !!!
  #define SERIAL_INTERFACE_MAX  4
  #define SERIALS_PLIST &Serial1,&Serial2,&Serial3,&Serial4

  #define LED_CONNECT PC9

#else
  // Assume a "Blue Pill" like configuration
  #warning "BLUEPILL HARDWARE ASSUMED"
  #define HARDWARE_TYPE "BLUEPILL"
  #define SERIAL_INTERFACE_MAX  3
  #define SERIALS_PLIST &Serial1,&Serial2,&Serial3

  // Blue Pill has only one LED...not optimal here...
  #define LED_CONNECT PC13

#endif

// Routing targets can be greater than serial interfaces.
// 4 Max however. e.g. 3 serials, 4 cables.
#define MIDI_ROUTING_TARGET_MAX 4

// USBDM (USB -) PIN
#define PIN_USBDM PA11

// Timer for attachCompare1Interrupt
#define TIMER2_RATE_MICROS 1000

// Sysex used to set some parameters of the interface.
#define SYSEX_INTERNAL_HEADER 0xF0,0x77,0x77,0x78,
#define SYSEX_INTERNAL_ACK 0x7F
#define SYSEX_INTERNAL_BUFF_SIZE 32

// LED light duration in milliseconds
#define LED_PULSE_MILLIS  5

// MIDI Routing
#define FROM_SERIAL 0
#define FROM_USB    1

// Routing from an USB cable OUT
#define DEFAULT_MIDI_CABLE_ROUTING_TARGET  0B00000001,0B00000010,0B00000100,0B00001000

// Routing from a serial jack MIDI IN
#define DEFAULT_MIDI_SERIAL_ROUTING_TARGET 0B00010000,0B00100000,0B01000000,0B10000000

// All midi messages. Used for cable and serial routing.
#define DEFAULT_MIDI_MSG_ROUTING_FILTER 0B11111111,0B11111111,0B11111111,0B11111111

// Intelligent Serial default MIDI Thru
// No IN actives - Alls msg -
// IN1->OUT1 - IN2->OUT1,2 IN3->OUT->1,2,3 IN4->OUT1,2,3,4
#define DEFAULT_INTELLIGENT_MIDI_THRU_OUT   0B11110001,0B11110011,0B11110111,0B11111111
#define DEFAULT_INTELLIGENT_MIDI_THRU_IN    0B0000

// Default number of 15 secs periods to start after USB midi inactivity
// Can be changed by SYSEX
#define DEFAULT_INTELLIGENT_MIDI_THRU_DELAY_PERIOD 2

// Use this structure to send and receive packet to/from USB
typedef union  {
    uint32_t i;
    uint8_t  packet[4];
} midiPacket_t;

// Functions prototypes
void Timer2Handler(void);
static void SendMidiMsgToSerial(uint8_t const *, uint8_t);
static void SerialWritePacket(const midiPacket_t *, uint8_t);
static void RouteStdMidiMsg( uint8_t, midiXparser* ) ;
static void RouteSysExMidiMsg( uint8_t , midiXparser*  ) ;
static void ParseSysExInternal(const midiPacket_t *) ;
static void RoutePacketToTarget(uint8_t, const midiPacket_t *) ;
static void ProcessSysExInternal() ;
void CheckEEPROM(bool);
int EEPROM_writeBlock(uint16 , const uint8 *, uint16  );
int EEPROM_readBlock(uint16 , uint8 *, uint16  );
static uint8_t GetInt8FromHexChar(char);
static uint16_t GetInt16FromHex4Char(char *);
static uint16_t GetInt16FromHex4Bin(char * );
static char USBSerialGetDigit();
static char USBSerialGetChar();
static uint8_t USBSerialScanHexChar(char *, uint8_t ,char,char);
static void ShowCurrentSettings();
void ConfigRootMenu();

#endif
