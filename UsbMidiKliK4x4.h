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



// Timer for attachCompare1Interrupt
#define TIMER2_RATE_MICROS 1000

// Sysex used to set some parameters of the interface.
#define SYSEX_INTERNAL_HEADER 0xF0,0x77,0x77,0x78,
#define SYSEX_INTERNAL_ACK 0x7F
#define SYSEX_INTERNAL_BUFF_SIZE 32

// LED light duration in milliseconds
#define LED_PULSE_MILLIS  5

// Macro to flash LEDS IN
#ifdef LEDS_MIDI
  #define FLASH_LED_IN(thisLed) flashLED_IN[thisLed]->start()
#else
  #define FLASH_LED_IN(thisLed) flashLED_CONNECT->start()
#endif

// MIDI Routing rules

#define FROM_SERIAL 1
#define FROM_USB    2

#define SERIAL_RULE 3
#define USBCABLE_RULE 4
#define INTELLITHRU_RULE 5

#define ROUTING_RESET_ALL 0
#define ROUTING_RESET_MIDIUSB 1
#define ROUTING_RESET_INTELLITHRU 2
#define ROUTING_INTELLITHRU_OFF 3

// Filter all midi messages by default.
#define DEFAULT_MIDI_MSG_ROUTING_FILTER 0B11111111

typedef struct {
      uint8_t  filterMsk;
      uint16_t cableInTargetsMsk;
      uint16_t jackOutTargetsMsk;
} midiRoutingRule_t;

typedef struct {
      uint8_t  filterMsk;
      uint16_t jackOutTargetsMsk;
} midiRoutingRuleJack_t;

// Filter all midi messages by default.
#define DEFAULT_MIDI_MSG_ROUTING_FILTER 0B11111111,0B11111111,0B11111111,0B11111111

// Routing from an USB cable OUT
#define DEFAULT_MIDI_CABLE_ROUTING_TARGET  0B00000001,0B00000010,0B00000100,0B00001000

// Routing from a serial jack MIDI IN
#define DEFAULT_MIDI_SERIAL_ROUTING_TARGET 0B00010000,0B00100000,0B01000000,0B10000000


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
void EEPROM_Check(bool);
int EEPROM_writeBlock(uint16_t , const uint8_t *, uint16_t  );
int EEPROM_readBlock(uint16_t , uint8_t *, uint16_t  );
static uint8_t GetInt8FromHexChar(char);
static uint16_t GetInt16FromHex4Char(char *);
static uint16_t GetInt16FromHex4Bin(char * );
static uint16_t AsknNumber(uint8_t) ;
static char AskDigit();
static char AskChar();
static uint8_t AsknHexChar(char *, uint8_t ,char,char);
static void ShowGlobalSettings();
void ShowConfigMenu();

#endif
