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

#ifndef _USBMIDIKLIK4X4_H_
#define _USBMIDIKLIK4X4_H_
#pragma once

#include "usb_midi_device.h"
#include "hardware_config.h"

// Comment this to remove all debug instructions from the compilation.

//#define DEBUG_MODE

// Timer for attachCompare1Interrupt
#define TIMER2_RATE_MICROS 1000

// Sysex used to set some parameters of the interface.
// Be aware that the 0x77 manufacturer id is reserved in the MIDI standard (but not used)
// The second byte is usually an id number or a func code + the midi channel (forced to 0x77 here)
// The Third is the product id
#define SYSEX_MANUFACTURER_ID 0x77
// UsbMidiKlik multi-port interface STM32F103 family
#define SYSEX_PRODUCT_FAMILY 0x0,0x01
#define SYSEX_MODEL_NUMBER 0x00,0x78
#define SYSEX_INTERNAL_HEADER 0xF0,SYSEX_MANUFACTURER_ID,0x77,0x78,
#define SYSEX_INTERNAL_ACK 0x7F
#define SYSEX_INTERNAL_IDENTITY_RQ_REPLY 0xF0,0x7E,0x7F,0x06,0x02,\
        SYSEX_MANUFACTURER_ID,SYSEX_PRODUCT_FAMILY,SYSEX_MODEL_NUMBER,VERSION_MAJOR,VERSION_MINOR,0x00,0X00,0xF7
#define SYSEX_INTERNAL_BUFF_SIZE 64

// LED light duration in milliseconds
#define LED_PULSE_MILLIS  5

// Boot modes
enum nextBootMode {
    bootModeMidi   = 0,
    bootModeConfigMenu = 2,
};

///////////////////////////////////////////////////////////////////////////////
// ROUTING RULES & TRANSFORMATION PIPELINES
///////////////////////////////////////////////////////////////////////////////

// Default number of 15 secs periods to start after USB midi inactivity
// Can be changed by SYSEX
#define DEFAULT_INTELLIGENT_MIDI_THRU_DELAY_PERIOD 2

// Size of a pipeline (number of pipes)
#define MIDI_TRANS_PIPELINE_SIZE 8

// Number of pipelines slots
#define MIDI_TRANS_PIPELINE_SLOT_SIZE 8

enum MidiRoutingDirection {
  FROM_SERIAL,
  FROM_USB,
  TO_SERIAL,
  TO_USB,
} ;

enum MidiRoutingRuleType {
  USBCABLE_RULE=0,
  SERIAL_RULE=1,
  INTELLITHRU_RULE=2
};

enum MidiRoutingReset {
  ROUTING_RESET_ALL,
  ROUTING_RESET_MIDIUSB,
  ROUTING_RESET_INTELLITHRU,
  ROUTING_INTELLITHRU_OFF
};

// Transformation pipe
typedef struct {
    uint8_t pId;    // FN_TRANSPIPE_NOPIPE means no pipe
    uint8_t byPass; // 1 = byPass. 0 = execute
    uint8_t par1;
    uint8_t par2;
    uint8_t par3;
    uint8_t par4;
} __packed midiTransPipe_t;

// Transformation pipeline
typedef struct {
    midiTransPipe_t pipeline[MIDI_TRANS_PIPELINE_SIZE];
    uint16_t attachedCablesMsk;
    uint16_t attachedJacksMsk;
    uint16_t attachedIthruJacksMsk;
} __packed midiTransPipeline_t;


// Routing & transformation rules structures
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

///////////////////////////////////////////////////////////////////////////////
// BUS MODE
///////////////////////////////////////////////////////////////////////////////
#define PIN_SDA PB7
#define PIN_SCL PB6

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
#define B_MASTER_READY_TIMEOUT 10000

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

// Bus data types for transfers
enum BusDataType {
  B_DTYPE_MIDI_ROUTING_RULES_CABLE,
  B_DTYPE_MIDI_ROUTING_RULES_SERIAL,
  B_DTYPE_MIDI_ROUTING_RULES_INTELLITHRU,
  B_DTYPE_MIDI_TRANSPIPE_SLOT_CABLES_MSK,
  B_DTYPE_MIDI_TRANSPIPE_SLOT_JACKS_MSK,
  B_DTYPE_MIDI_TRANSPIPE_SLOT_ITHRU_MSK,
  B_DTYPE_MIDI_TRANSPIPE,
  B_DTYPE_MIDI_ROUTING_INTELLITHRU_JACKIN_MSK,
  B_DTYPE_MIDI_ROUTING_INTELLITHRU_DELAY_PERIOD,
};

enum BusDeviceSate {
  B_STATE_READY,
  B_STATE_BUSY,
  B_STATE_KO,
  B_STATE_OK
};

///////////////////////////////////////////////////////////////////////////////
// EEPROM parameters
// The signature is used to check if EEPROM is correctly initialized
// The parameters version is the version of the current structure.
// Changing the version will cause a new initialization in CheckEEPROM();.
// The following structure start at the first address of the EEPROM
///////////////////////////////////////////////////////////////////////////////
#define EE_SIGNATURE "MDK"
#define EE_PRMVER 21

typedef struct {
        uint8_t         signature[3];
        uint16_t        size;
        uint8_t         majorVersion;
        uint8_t         minorVersion;
        uint8_t         prmVersion;
        uint8_t         TimestampedVersion[14];

        uint8_t         nextBootMode;
        boolean         debugMode;

        // I2C device
        uint8_t         I2C_DeviceId;

        // I2C BUS Mode state
        uint8_t         I2C_BusModeState;

        // Incoming events routing rules
        // Storage space is set to the max i.e. INTERFACE_MAX for all
        // To allow dynamic change of bus mode.

        midiRoutingRule_t midiRoutingRulesCable[USBCABLE_INTERFACE_MAX];
        midiRoutingRule_t midiRoutingRulesSerial[B_SERIAL_INTERFACE_MAX];

        // IntelliThru routing rules
        midiRoutingRuleJack_t midiRoutingRulesIntelliThru[B_SERIAL_INTERFACE_MAX];
        uint16_t              intelliThruJackInMsk;
        uint8_t               intelliThruDelayPeriod; // 1 to 255 periods of 15s.

        // Transformation pipelines slots.
        midiTransPipeline_t midiTransPipelineSlots[MIDI_TRANS_PIPELINE_SLOT_SIZE];

        uint16_t        vendorID;
        uint16_t        productID;
        uint8_t         productString[USB_MIDI_PRODUCT_STRING_SIZE+1]; // Unicode string - defined in usb_midi_device.h

} __packed EEPROM_Params_t;

///////////////////////////////////////////////////////////////////////////////
//  CORE FUNCTIONS PROTOTYPES
///////////////////////////////////////////////////////////////////////////////
void Timer2Handler(void);
void FlashAllLeds(uint8_t);
void SerialMidi_SendMsg(uint8_t *, uint8_t);
void SerialMidi_SendPacket(midiPacket_t *, uint8_t );
void SerialMidi_RouteMsg( uint8_t, midiXparser*  );
void SerialMidi_RouteSysEx( uint8_t , midiXparser* );
void SysExInternalParse(uint8_t, midiPacket_t *);
void RoutePacketToTarget(uint8_t , midiPacket_t *);
void ResetMidiRoutingRules(uint8_t);
uint8_t SysexInternalDumpConf(uint32_t , uint8_t ,uint8_t *);
void SysexInternalDumpToStream(uint8_t ) ;
void SysExSendMsgPacket(uint8_t *,uint16_t );
void SysExInternalProcess(uint8_t, uint8_t *);
void CheckBootMode();
void USBMidi_Init();
void USBMidi_Process();
void SerialMidi_Process();

#endif
