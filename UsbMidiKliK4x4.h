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

#ifndef _USBMIDIKLIK4X4_H_
#define _USBMIDIKLIK4X4_H_
#pragma once


// Routing rules structures
typedef struct {
      uint8_t  filterMsk;
      uint16_t cableInTargetsMsk;
      uint16_t jackOutTargetsMsk;
} __packed midiRoutingRule_t;

typedef struct {
      uint8_t  filterMsk;
      uint16_t jackOutTargetsMsk;
} __packed midiRoutingRuleJack_t;

// Use this structure to send and receive packet to/from USB /serial/BUS
typedef union  {
    uint32_t i;
    uint8_t  packet[4];
} __packed midiPacket_t;

// Specific midi packet for master on BUS.
// packed clause is mandatory to reflect the real size!!!
typedef union {
  struct {
        uint8_t dest;
        midiPacket_t pk;
  } __packed mpk;
  uint8_t packet[5];
} __packed masterMidiPacket_t;

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

#define B_RING_BUFFER_PACKET_SIZE  16*sizeof(midiPacket_t)
#define B_RING_BUFFER_MPACKET_SIZE 16*sizeof(masterMidiPacket_t)

// 16 cables/jacks is the maximum value allowed by the midi usb standard
#define B_MAX_NB_DEVICE 16/SERIAL_INTERFACE_MAX
#define B_SERIAL_INTERFACE_MAX B_MAX_NB_DEVICE * SERIAL_INTERFACE_MAX
#define B_MASTERID 4
#define B_SLAVE_DEVICE_BASE_ADDR B_MASTERID + 1
#define B_SLAVE_DEVICE_LAST_ADDR B_SLAVE_DEVICE_BASE_ADDR + B_MAX_NB_DEVICE -2
#define B_DISABLED 0
#define B_ENABLED 1
#define B_FREQ 400000
#define B_MASTER_READY_TIMEOUT 5000

// Bus commands
enum BusCommand {
  B_CMD_NONE,
  B_CMD_ISPACKET_AVAIL,
  B_CMD_GET_MPACKET,
  B_CMD_IS_SLAVE_READY,
  B_CMD_USBCX_AVAILABLE,
  B_CMD_USBCX_UNAVAILABLE,
  B_CMD_USBCX_SLEEP,
  B_CMD_USBCX_AWAKE,
  B_CMD_INTELLITHRU_ENABLED,
  B_CMD_INTELLITHRU_DISABLED,
  B_CMD_HARDWARE_RESET,
  B_CMD_START_SYNC,
  B_CMD_END_SYNC,
  B_CMD_DEBUG_MODE_ENABLED,
  B_CMD_DEBUG_MODE_DISABLED,
} ;

// Bus data types for transfers
enum BusDataType {
  B_DTYPE_MIDI_ROUTING_RULES_CABLE,
  B_DTYPE_MIDI_ROUTING_RULES_SERIAL,
  B_DTYPE_MIDI_ROUTING_RULES_INTELLITHRU,
  B_DTYPE_MIDI_ROUTING_INTELLITHRU_JACKIN_MSK,
  B_DTYPE_MIDI_ROUTING_INTELLITHRU_DELAY_PERIOD,
};

#define   B_STATE_READY 1
#define   B_STATE_BUSY  0
#define   B_STATE_KO 1
#define   B_STATE_OK 0

// Corresponding "requestFrom" answer bytes size without command
uint8_t static const BusCommandRequestSize[]= {
  0,                         // B_CMD_NONE
  1,                         // B_CMD_ISPACKET_AVAIL
  sizeof(masterMidiPacket_t),// B_CMD_GET_MPACKET
  1,                         // B_CMD_IS_SLAVE_READY
  0,                         // B_CMD_USBCX_AVAILABLE
  0,                         // B_CMD_USBCX_UNAVAILABLE
  0,                         // B_CMD_USBCX_SLEEP
  0,                         // B_CMD_USBCX_AWAKE
  0,                         // B_CMD_INTELLITHRU_ENABLED
  0,                         // B_CMD_INTELLITHRU_DISABLED
  0,                         // B_CMD_B_CMD_HARDWARE_RESET
  0,                         // B_CMD_START_SYNC,
  0,                         // B_CMD_END_SYNC,
  0,                         // B_CMD_DEBUG_MODE_ENABLED
  0,                         // B_CMD_DEBUG_MODE_DISABLED
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

// Macro for debugging purpose when MIDI active

// Comment this to remove all debug instructions from the compilation.
//#define DEBUG_MODE

#define DEBUG_SERIAL Serial3
#ifdef DEBUG_MODE
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

// Default number of 15 secs periods to start after USB midi inactivity
// Can be changed by SYSEX
#define DEFAULT_INTELLIGENT_MIDI_THRU_DELAY_PERIOD 2


// Functions prototypes
void Timer2Handler(void);
void FlashAllLeds(uint8_t);
static void SerialSendMidiMsg(uint8_t const *, uint8_t);
static void SerialSendMidiPacket(const midiPacket_t *, uint8_t);
static void I2C_BusSerialSendMidiPacket(const midiPacket_t *, uint8_t );
int8_t I2C_ParseDataSync(uint8_t ,uint8_t ,uint8_t );
void I2C_ParseImmediateCmd();
void I2C_SlaveReceiveEvent(int);
void I2C_SlaveRequestEvent ();
void I2C_BusChecks();
void I2C_BusStartWire();
int16_t I2C_SendCommand(uint8_t,BusCommand);
void I2C_ShowActiveDevice();
static void SerialMidi_RouteMsg( uint8_t, midiXparser*  );
static void SerialMidi_RouteSysEx( uint8_t , midiXparser* );
static void ParseSysExInternal(const midiPacket_t *);
static void RoutePacketToTarget(uint8_t , const midiPacket_t *);
void ResetMidiRoutingRules(uint8_t);
static void ProcessSysExInternal();
void EEPROM_ParamsInit(bool );
void PrintCleanHEX(uint8_t);
void ShowBufferHexDump(uint8_t* , uint16_t );
void ShowBufferHexDumpDebugSerial(uint8_t* , uint16_t );
void EEPROM_Put(uint8_t* ,uint16_t );
void EEPROM_Get(uint8_t* ,uint16_t );
void EEPROM_ParamsLoad();
void EEPROM_ParamsSave();
static uint8_t GetInt8FromHexChar(char);
static uint16_t GetInt16FromHex4Char(char *);
static uint16_t GetInt16FromHex4Bin(char * );
static uint16_t AsknNumber(uint8_t) ;
static char AskDigit();
static char AskChar();
static uint8_t AsknHexChar(char *, uint8_t ,char,char);
char AskChoice(const char * , char * );
void ShowMidiRoutingLine(uint8_t ,uint8_t , void *);
void ShowMidiRouting(uint8_t);
static void ShowMidiKliKHeader();
static void ShowGlobalSettings();
uint16_t AskMidiRoutingTargets(uint8_t,uint8_t , uint8_t );
void AskMidiRouting(uint8_t);
uint8_t AskMidiFilter(uint8_t, uint8_t );
void AskProductString();
void AskVIDPID();
void ShowConfigMenu();
void CheckBootMode();
void USBMidi_Init();
void USBMidi_Process();
void SerialMidi_Process();
int16_t I2C_getPacket(uint8_t , masterMidiPacket_t *);
boolean I2C_isDeviceActive(uint8_t );
int8_t I2C_SendData(const uint8_t, uint8_t, uint8_t, uint8_t * , uint16_t );
void I2C_SlavesRoutingSyncFromMaster();
void I2C_ProcessMaster ();
void I2C_ProcessSlave ();

#endif
