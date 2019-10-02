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


typedef struct {
      uint8_t  filterMsk;
      uint16_t cableInTargetsMsk;
      uint16_t jackOutTargetsMsk;
} __packed midiRoutingRule_t;

typedef struct {
      uint8_t  filterMsk;
      uint16_t jackOutTargetsMsk;
} __packed midiRoutingRuleJack_t;

// Use this structure to send and receive packet to/from USB
typedef union  {
    uint32_t i;
    uint8_t  packet[4];
} midiPacket_t;

// Specific midi packet for master .
// packed clause is mandatory to reflect the real size!!!
typedef union {
  struct {
        uint8_t dest;
        midiPacket_t pk;
  } __packed mpk;
  uint8_t packet[5];
} __packed masterMidiPacket_t;

const midiPacket_t NULL_MIDI_PACKET = { .i = 0 };

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
  #define FLASH_LED_OUT(thisLed) flashLED_OUT[thisLed]->start()
#else
  #define FLASH_LED_IN(thisLed) flashLED_CONNECT->start()
  #define FLASH_LED_OUT(thisLed) flashLED_CONNECT->start()
#endif


// MIDI Routing rules

enum MidiRouteSourceDest {
  FROM_SERIAL,
  FROM_USB,
  TO_SERIAL,
  TO_USB,
} ;

#define SERIAL_RULE 3
#define USBCABLE_RULE 4
#define INTELLITHRU_RULE 5

#define ROUTING_RESET_ALL 0
#define ROUTING_RESET_MIDIUSB 1
#define ROUTING_RESET_INTELLITHRU 2
#define ROUTING_INTELLITHRU_OFF 3

// BUS MODE (I2C)

#define B_RING_BUFFER_PACKET_SIZE  8*sizeof(midiPacket_t)
#define B_RING_BUFFER_MPACKET_SIZE 8*sizeof(masterMidiPacket_t)

// 16 cables/jacks is the maximum value allowed by the midi usb standard
#define B_MAX_NB_DEVICE 16/SERIAL_INTERFACE_MAX
#define B_SERIAL_INTERFACE_MAX B_MAX_NB_DEVICE * SERIAL_INTERFACE_MAX
#define B_MASTERID 4
#define B_SLAVE_DEVICE_BASE_ADDR B_MASTERID + 1
#define B_SLAVE_DEVICE_LAST_ADDR B_SLAVE_DEVICE_BASE_ADDR + B_MAX_NB_DEVICE -2
#define B_DISABLED 0
#define B_ENABLED 1
#define B_FREQ 100000
#define B_SLAVE_REBOOT_TIMEOUT  5000

// Bus commands
enum BusCommand {
  B_CMD_NONE=0,
  B_CMD_ISPACKET_AVAIL,
  B_CMD_GET_MPACKET,
  B_CMD_ALL_SLAVE_RESET,
  B_CMD_ALL_SLAVE_SYNC_ROUTING,
  B_CMD_ENABLE_INTELLITHRU,
  B_CMD_DISABLE_INTELLITHRU,
  B_CMD_USB_NO_CX,
  B_CMD_IS_SLAVE_READY
} ;

#define   B_STATE_READY 1
#define   B_STATE_BUSY  0

// Corresponding "requestFrom" answer bytes size without command
uint8_t static const BusCommandRequestSize[]= {
  0,                        // B_CMD_NONE
  1,                        // B_CMD_ISPACKET_AVAIL
  sizeof(midiPacket_t),     // B_CMD_GET_PACKET
  0,                        // B_CMD_ALL_SLAVE_RESET
  0,                        // B_CMD_ALL_SLAVE_SYNC_ROUTING
  0,                        // B_CMD_ENABLE_INTELLITHRU
  0,                        // B_CMD_DISABLE_INTELLITHRU
  0,                        // B_CMD_USB_NO_CX
  1,                        // B_CMD_IS_SLAVE_READY
};

// Macro to compute the max serial port in bus mode or not.
#define SERIAL_INTERFACE_COUNT (EEPROM_Params.I2C_BusModeState == B_ENABLED ? B_SERIAL_INTERFACE_MAX:SERIAL_INTERFACE_MAX)

// Macro to compute if Master/Slave On Bus
#define B_IS_MASTER (EEPROM_Params.I2C_BusModeState == B_ENABLED && EEPROM_Params.I2C_DeviceId == B_MASTERID)
#define B_IS_SLAVE (EEPROM_Params.I2C_BusModeState == B_ENABLED && EEPROM_Params.I2C_DeviceId != B_MASTERID)

// Macro to compute a device Id from a serial ports
#define GET_DEVICEID_FROM_SERIALNO(s) ((s) / SERIAL_INTERFACE_MAX + B_MASTERID)

// Macro to compute a "virtual bus serial port" from a local device and serial port
#define GET_BUS_SERIALNO_FROM_LOCALDEV(d,s) ((s) + (d-B_MASTERID) * SERIAL_INTERFACE_MAX)

// Macro for debugging purpose

#define DEBUG_PRINT(txt,val) Serial.print((txt));Serial.println((val))
#define DEBUG_PRINT_BIN(txt,val) Serial.print((txt));Serial.println((val),BIN)
#define DEBUG_PRINT_HEX(txt,val) Serial.print((txt));Serial.println((val),HEX)



// Default number of 15 secs periods to start after USB midi inactivity
// Can be changed by SYSEX
#define DEFAULT_INTELLIGENT_MIDI_THRU_DELAY_PERIOD 2


// Functions prototypes
void Timer2Handler(void);
static void SerialSendMidiMsg(uint8_t const *, uint8_t);
static void SerialSendMidiPacket(const midiPacket_t *, uint8_t);
static void RouteMidiMsg( uint8_t, midiXparser* ) ;
static void RouteSysExMidiMsg( uint8_t , midiXparser*  ) ;
static void ParseSysExInternal(const midiPacket_t *) ;
static void RoutePacketToTarget(uint8_t, const midiPacket_t *) ;
static void ProcessSysExInternal() ;
void EEPROM_Check(bool);
void EEPROM_ParamsLoad();
void EEPROM_ParamsSave();

static uint8_t GetInt8FromHexChar(char);
static uint16_t GetInt16FromHex4Char(char *);
static uint16_t GetInt16FromHex4Bin(char * );
static uint16_t AsknNumber(uint8_t) ;
static char AskDigit();
static char AskChar();
static uint8_t AsknHexChar(char *, uint8_t ,char,char);
static void ShowGlobalSettings();
void ShowConfigMenu();
void PrintCleanHEX(uint8_t);

#endif
