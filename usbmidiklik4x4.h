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

#define __O_SMALL __attribute__((optimize("-Os")))

#include "usb_midi_device.h"
#include "hardware_config.h"

// Millisec all purpose Timer (LED off notably)
#define TIMER_MILLIS_RATE_MICROS 1000
// LED ON duration in loop count. NB : 255 tick max * TIMER2_RATE_MICROS
#define LED_TICK_COUNT 5
// LED ON recovery time in msec when no dedicated LED for CONNECT USB
#define LED_CONNECT_USB_RECOVER_TIME_MILLIS 500

// Sysex internal buffer size
#define GLOBAL_DATA_BUFF_SIZE 64

// Default is stm32duino bootloader.
#define HID_BOOTLOADER

// Boot modes magic words
#ifdef HID_BOOTLOADER
  #define BOOT_BTL_MAGIC        0x424C
  #define BOOT_BTL_MAGIC_NOWAIT 0x0000
  #define BOOT_BTL_REGISTER     DR10
#else
  #define BOOT_BTL_MAGIC        0x424C
  #define BOOT_BTL_MAGIC_NOWAIT 0x424D
  #define BOOT_BTL_REGISTER     DR10
#endif

// Configuration mode magic word.
#define BOOT_CONFIG_MAGIC 0x3012
#define BOOT_MIDI_MAGIC   0x0000
#define BOOT_REGISTER     DR5

// LED Tick
typedef struct {
    uint8_t pin;
    uint8_t tick;
} __packed LEDTick_t;

// Process functions vector
typedef void(*procVectorFn_t)();

///////////////////////////////////////////////////////////////////////////////
// ROUTING RULES & TRANSFORMATION PIPELINES
///////////////////////////////////////////////////////////////////////////////

// Default number of 15 secs periods to start after USB midi inactivity
// Can be changed by SYSEX
#define DEFAULT_ITHRU_USB_IDLE_TIME_PERIOD 2

// Size of a pipeline (number of pipes)
#define TRANS_PIPELINE_SIZE 8

// Number of pipelines slots
#define TRANS_PIPELINE_SLOT_SIZE 8

// Number of virtual interface ports
#define VIRTUAL_INTERFACE_MAX 8

// Empty pipe (nb : below 0x80 - 7 bits value)
#define FN_TRANSPIPE_NOPIPE 0x7F

// Port types
#define PORT_TYPE_CABLE 0
#define PORT_TYPE_JACK 1
#define PORT_TYPE_VIRTUAL 2
#define PORT_TYPE_ITHRU 3

// Routing reset modes
#define ROUTING_RESET_ALL 0
#define ROUTING_RESET_MIDIUSB 1
#define ROUTING_RESET_INTELLITHRU 2
#define ROUTING_CLEAR_ALL 3

// Transformation pipe
typedef struct {
    uint8_t pId;    // FN_TRANSPIPE_NOPIPE means no pipe
    uint8_t byPass; // 1 = byPass. 0 = execute
    uint8_t par1;
    uint8_t par2;
    uint8_t par3;
    uint8_t par4;
} __packed transPipe_t;

// Transformation pipeline
typedef struct {
    transPipe_t pipeline[TRANS_PIPELINE_SIZE];
} __packed transPipeline_t;


// Routing & transformation rules structures
typedef struct {
      uint8_t  slot;
      uint16_t cbInTgMsk;
      uint16_t jkOutTgMsk;
      uint16_t vrInTgMsk;
} __packed routingRule_t;

// Routing structure used for IThru and Virtual IN
typedef struct {
      uint8_t  slot;
      union {
        uint16_t cbInTgMsk;
        uint16_t vrInTgMsk;
      };
      uint16_t jkOutTgMsk;
} __packed routingRuleAlt_t;

// Use this structure to send and receive packet to/from USB /serial/BUS
typedef union  {
    uint32_t i;
    uint8_t  packet[4];
} __packed midiPacket_t;

// MIDI CLOCK GENERATOR MANAGEMENT
// Compute a BPM where BPM is x 10 to manage half bpm.
// i.e. 1205 means 120.5 BPM
#define MIDI_CLOCKGEN_MAX VIRTUAL_INTERFACE_MAX/2
//# 1205 BPM means 120.5
#define DEFAULT_BPM 1200
#define MAX_BPM 3000
#define MIN_BPM 100
// MIDI TIME CODE
#define MTC_FPS 30
// SMPTE type  : 0 = 24 fps, 1 = 25 fps, 2 = 30 fps (Drop-Frame), 3 = 30 fps
#define MTC_SMPTE_TYPE 3
#define MTC_FRAME_TICK 1000000 / MTC_FPS

// midi Bpm Clock type. Overflow value is stored in tick.
// When mtc is true, a MIDI TIME CODE is sent
typedef struct {
  boolean   enabled;
  boolean   mtc;
  uint16_t  bpm;
} __packed bpmClock_t;

typedef struct {
    unsigned long tickBpm;
    unsigned long nextBpmTick;
} __packed bpmTick_t;

///////////////////////////////////////////////////////////////////////////////
// BUS MODE
///////////////////////////////////////////////////////////////////////////////

// Specific midi packet for master on BUS.
// packed clause is mandatory to reflect the real size!!!
typedef union {
  struct {
        uint8_t dest;
        midiPacket_t pk;
  } __packed mpk;
  uint8_t packet[5];
} __packed masterMidiPacket_t;

// I2C Bulk data transfer packet. 19 bytes.
// Size is data size >0 and <= 16
// Checksum is a xor of all 16 data bytes
typedef struct {
    uint8_t size;
    uint8_t data[16];
    uint8_t cksum;
} __packed I2C_bulkDataPacket_t;

#define I2C_LED_TIMER_MILLIS 500

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
};

// Bus data types for transfers
enum BusDataType {
  B_DTYPE_ROUTING_RULE,
  B_DTYPE_ROUTING_RULE_ALT,
  B_DTYPE_BPM_CLOCK,
  B_DTYPE_MIDI_TRANSPIPE,
  B_DTYPE_ROUTING_ITHRU_JACKIN_MSK,
  B_DTYPE_ROUTING_ITHRU_USB_IDLE_TIME_PERIOD,
  B_DTYPE_BULK_DATA,
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
#define EE_SIGNATURE "UMK"
#define EE_PRMVER 27

typedef struct {
        uint8_t         signature[3];
        uint16_t        size;
        uint8_t         majorVersion;
        uint8_t         minorVersion;
        uint8_t         prmVersion;
        uint8_t         TimestampedVersion[14];

        // I2C device
        uint8_t         I2C_DeviceId;

        // I2C BUS Mode state
        uint8_t         I2C_BusModeState;

        // Incoming events routing rules
        // Storage space is set to the max i.e. INTERFACE_MAX for all
        // To allow dynamic change of bus mode.

        routingRule_t rtRulesCable[16];
        routingRule_t rtRulesJack[B_SERIAL_INTERFACE_MAX];

        // Virtual routing rules : cable and jack only
        routingRuleAlt_t rtRulesVirtual[VIRTUAL_INTERFACE_MAX];

        // IntelliThru routing rules jack and virtual in only
        routingRuleAlt_t rtRulesIthru[B_SERIAL_INTERFACE_MAX];
        uint16_t         ithruJackInMsk;
        uint8_t          ithruUSBIdleTimePeriod; // 1 to 255 periods of 15s.

        // Transformation pipelines slots.
        transPipeline_t pipelineSlot[TRANS_PIPELINE_SLOT_SIZE];

        // Midiclock virtuals
        bpmClock_t  bpmClocks[MIDI_CLOCKGEN_MAX] ;

        uint16_t        vendorID;
        uint16_t        productID;
        uint8_t         productString[USB_MIDI_PRODUCT_STRING_SIZE+1]; // Unicode string - defined in usb_midi_device.h

} __packed EEPROM_Prm_t;

///////////////////////////////////////////////////////////////////////////////
//  CORE FUNCTIONS PROTOTYPES
///////////////////////////////////////////////////////////////////////////////
int     memcmpcpy ( void * , void * , size_t );
void    TimerMillisHandler();
void    LED_Init();
void    LED_TurnOn(volatile LEDTick_t *);
void    LED_TurnOff(volatile LEDTick_t *);
boolean LED_Flash(volatile LEDTick_t *);
void    LED_Update();
void    FlashAllLeds(uint8_t);
void    SerialMidi_SendMsg(uint8_t *, uint8_t);
void    SerialMidi_SendPacket(midiPacket_t *, uint8_t );
void    SerialMidi_RouteMsg( uint8_t, midiXparser*  );
void    SerialMidi_RouteSysEx( uint8_t , midiXparser* );
void    RoutePacketToTarget(uint8_t , midiPacket_t *);
void    MidiClockGenerator();
boolean SetMidiBpmClock(uint8_t, uint16_t);
boolean SetMidiEnableClock(uint8_t , boolean );
uint8_t MidiTimeCodeGetFrameByte();
void    ResetMidiRoutingRules(uint8_t);
boolean USBMidi_SendSysExPacket(uint8_t,const uint8_t *,uint16_t );
uint16_t GetAndClearBootMagicWord();
void    SetBootMagicWord(uint16_t);
void    CheckBootMode();
void    USBMidi_Init();
void    USBMidi_Process();
void    SerialMidi_Process();

///////////////////////////////////////////////////////////////////////////////
// EXTERNAL SHARED FUNCTIONS PROTOTYPES (MODULES)
///////////////////////////////////////////////////////////////////////////////
void __O_SMALL ShowBufferHexDump(uint8_t* , uint16_t, uint8_t nl=16 ) ;

void I2C_SlavesRoutingSyncFromMaster();
void __O_SMALL I2C_ShowActiveDevice() ;

boolean TransPacketPipeline_ClearSlot(uint8_t);
boolean TransPacketPipeline_CopySlot(uint8_t ,uint8_t ) ;
boolean TransPacketPipeline_AttachPort(uint8_t ,uint8_t ,uint8_t );
boolean TransPacketPipe_AddToSlot(uint8_t , transPipe_t *);
boolean TransPacketPipe_InsertToSlot(uint8_t , uint8_t , transPipe_t *,boolean);
boolean TransPacketPipe_ClearSlotIndexPid(uint8_t , boolean ,uint8_t);
boolean TransPacketPipe_ByPass(uint8_t , uint8_t ,uint8_t);
void __O_SMALL ShowPipelineSlot(uint8_t s)  ;
void __O_SMALL SerialPrintf(const char *format, ...)  ;

#endif
