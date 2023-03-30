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
#define OPTION_CONFIGUI 1


#include "build_number_defines.h"
#if OPTION_CONFIGUI
  #include <string.h>
#endif
#include <stdarg.h>

#include <libmaple/nvic.h>
#include "libmaple/flash.h"
#include "libmaple/pwr.h"
#include "libmaple/rcc.h"
#include "libmaple/bkp.h"

#include <Wire_slave.h>
#include <midiXparser.h>
#include "usbmidiklik4x4.h"
#include "usb_midi.h"
#include "ringbuffer.h"

///////////////////////////////////////////////////////////////////////////////
// GLOBALS
///////////////////////////////////////////////////////////////////////////////

// EEPROMS parameters
EEPROM_Prm_t EE_Prm;

// Default Boot modes magic word
uint16_t bootMagicWord = BOOT_MIDI_MAGIC;

// Serial interfaces Array
HardwareSerial * serialHw[SERIAL_INTERFACE_MAX] = {SERIALS_PLIST};

// LED Management
volatile LEDTick_t LED_ConnectTick= { LED_CONNECT,0};

// LEDs ticks for Connect, MIDIN and MIDIOUT
#ifdef HAS_MIDITECH_HARDWARE
  // LED must be declared in the same order as hardware serials
  #define LED_MIDI_SIZE 4
  volatile LEDTick_t LED_MidiInTick[LED_MIDI_SIZE] = {
    {D4,0},{D5,0},{D6,0},{D7,0}
  };

  volatile LEDTick_t LED_MidiOutTick[LED_MIDI_SIZE] = {
    {D36,0},{D37,0},{D16,0},{D17,0}
  };
#endif

// Midi Clocks initialize
bpmTick_t bpmTicks[MIDI_CLOCKGEN_MAX] ;

// USB Midi object & globals
USBMidi MidiUSB;

volatile bool					midiUSBCx      = false ;
volatile bool         midiUSBIdle    = false ;
bool 					        isSerialBusy   = false ;

// MIDI Parsers for serial 1 to n
midiXparser midiSerial[SERIAL_INTERFACE_MAX];

// Multi purpose data buffer.
uint8_t globalDataBuffer[GLOBAL_DATA_BUFF_SIZE] ;

// Intelligent midi thru mode
volatile bool midiIthruActive        = false ;
unsigned long ithruUSBIdlelMillis    = DEFAULT_ITHRU_USB_IDLE_TIME_PERIOD * 15000;

// Bus Mode globals
volatile uint8_t I2C_Command         = B_CMD_NONE;

// True if events received from Master when slave
volatile boolean I2C_MasterIsActive    = false;

// Master to slave synchonization globals
volatile boolean I2C_SlaveSyncStarted  = false;
volatile boolean I2C_SlaveSyncDoUpdate = false;

// Array of active devices
uint8_t I2C_DeviceIdActive[B_MAX_NB_DEVICE-1]; // Minus the master
uint8_t I2C_DeviceActiveCount=0;

// Templated RingBuffers to manage I2C slave reception/transmission outside I2C ISR
// Volatile by default and RESERVED TO SLAVE
RingBuffer<uint8_t,B_RING_BUFFER_PACKET_SIZE> I2C_QPacketsFromMaster;
RingBuffer<uint8_t,B_RING_BUFFER_MPACKET_SIZE> I2C_QPacketsToMaster;

// Vector for process run in main Loop to avoid testing flags in a pure loop
// approch. All function pointer are added when they are only necessary in Setup.
// Check carefully the array size...
uint8_t procVectorFnCount = 0;
procVectorFn_t procVectorFn[6] ;

///////////////////////////////////////////////////////////////////////////////
//  CODE MODULES
//-----------------------------------------------------------------------------
// Due to the unusual make process of Arduino platform, modules are included
// directly here as "h" type. This allows a better code separation and readability.
///////////////////////////////////////////////////////////////////////////////
// DO NOT REMOVE OR CHANGE THE ORDER !

#include "mod_macros.h"
#include "mod_eeprom.h"
#include "mod_intsysex.h"
#if OPTION_CONFIGUI
  #include "mod_configui.h"
#endif
#include "mod_i2cbus.h"
#include "mod_miditransfn.h"

///////////////////////////////////////////////////////////////////////////////
//  CORE FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// memcmpcpy : copy if different
///////////////////////////////////////////////////////////////////////////////
int memcmpcpy ( void * pDest, void * pSrc, size_t sz )
{

  int r = 0;
  if ( ( r = memcmp(pDest,pSrc,sz) ) ) {
    memcpy(pDest,pSrc,sz);
  };

  return r;

}
///////////////////////////////////////////////////////////////////////////////
// Timer2 interrupt handler - 1 millisec timer
///////////////////////////////////////////////////////////////////////////////
void TimerMillisHandler(void)
{
  LED_Update();
}

///////////////////////////////////////////////////////////////////////////////
// LED MANAGEMENT - Init PINS
///////////////////////////////////////////////////////////////////////////////
void LED_Init()
{

  // Initialize LED arrays

  // LED connect
  gpio_set_mode(PIN_MAP[LED_ConnectTick.pin].gpio_device, PIN_MAP[LED_ConnectTick.pin].gpio_bit, GPIO_OUTPUT_PP);
  LED_ConnectTick.tick = 0;

  // LEDs IN/OUT if available
#ifdef HAS_MIDITECH_HARDWARE
  for (uint8_t i=0 ; i != LED_MIDI_SIZE ; i++ ) {
    gpio_set_mode(PIN_MAP[LED_MidiInTick[i].pin].gpio_device, PIN_MAP[LED_MidiInTick[i].pin].gpio_bit, GPIO_OUTPUT_PP);
    gpio_set_mode(PIN_MAP[LED_MidiOutTick[i].pin].gpio_device, PIN_MAP[LED_MidiOutTick[i].pin].gpio_bit, GPIO_OUTPUT_PP);

    LED_MidiOutTick[i].tick = 0;
    LED_MidiInTick[i].tick = 0;
  }
#endif

}

///////////////////////////////////////////////////////////////////////////////
// LED MANAGEMENT - Turn ON / OFF A LED (Faster with GPIO functions)
///////////////////////////////////////////////////////////////////////////////
void LED_TurnOn(volatile LEDTick_t *ledTick)
{
  gpio_write_bit(PIN_MAP[ledTick->pin].gpio_device, PIN_MAP[ledTick->pin].gpio_bit, LOW);
}

void LED_TurnOff(volatile LEDTick_t *ledTick)
{
  gpio_write_bit(PIN_MAP[ledTick->pin].gpio_device, PIN_MAP[ledTick->pin].gpio_bit, HIGH);
}

///////////////////////////////////////////////////////////////////////////////
// LED MANAGEMENT - FLASH A LED DURING LED_TICK_COUNT*TIMER2_RATE_MICROS
///////////////////////////////////////////////////////////////////////////////
boolean LED_Flash(volatile LEDTick_t *ledTick)
{
    if ( ! ledTick->tick ) {
      gpio_write_bit(PIN_MAP[ledTick->pin].gpio_device, PIN_MAP[ledTick->pin].gpio_bit, LOW);
      ledTick->tick = LED_TICK_COUNT;
      return true;
    }
    ledTick->tick = LED_TICK_COUNT;
    return false;
}

///////////////////////////////////////////////////////////////////////////////
// LED MANAGEMENT - CHECK TO SWITCH OFF FLASHED LEDS AT TIMER2_RATE_MICROS RATE
///////////////////////////////////////////////////////////////////////////////
void LED_Update()
{
    // LED connect
    if ( LED_ConnectTick.tick ) {
      if ( !( --LED_ConnectTick.tick ) ) {
        // LED OFF
        gpio_write_bit(PIN_MAP[LED_ConnectTick.pin].gpio_device, PIN_MAP[LED_ConnectTick.pin].gpio_bit, HIGH);
      }
    }
#ifdef LED_MIDI_SIZE
    for (uint8_t i=0 ; i != LED_MIDI_SIZE ; i++ ) {
      if ( LED_MidiInTick[i].tick ) {
        if ( !(--LED_MidiInTick[i].tick) ) {
          // LED OFF
          gpio_write_bit(PIN_MAP[LED_MidiInTick[i].pin].gpio_device, PIN_MAP[LED_MidiInTick[i].pin].gpio_bit, HIGH);
        }
      }
      if ( LED_MidiOutTick[i].tick ) {
        if ( !(--LED_MidiOutTick[i].tick) ) {
          // LED OFF
          gpio_write_bit(PIN_MAP[LED_MidiOutTick[i].pin].gpio_device, PIN_MAP[LED_MidiOutTick[i].pin].gpio_bit, HIGH);
        }
      }
    }
#endif
}

///////////////////////////////////////////////////////////////////////////////
// FlashAllLeds . 0 = Alls. 1 = In. 2 = Out
///////////////////////////////////////////////////////////////////////////////
void FlashAllLeds(uint8_t mode)
{
  for ( uint8_t f=0 ; f!= 4 ; f++ ) {
		#ifdef LED_MIDI_SIZE
			for ( uint8_t i=0 ; i != LED_MIDI_SIZE ; i++ ) {
					if ( mode == 0 || mode ==1 ) FLASH_LED_IN(i);
					if ( mode == 0 || mode ==2 ) FLASH_LED_OUT(i);
			}
		#else
			FLASH_LED_OUT(0);
		#endif
		delay(100);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Send a midi msg to serial. 0 is Serial1.
///////////////////////////////////////////////////////////////////////////////
void SerialMidi_SendMsg(uint8_t *msg, uint8_t serialNo)
{
  if (serialNo >= SERIAL_INTERFACE_MAX ) return;

	uint8_t msgLen = midiXparser::getMidiStatusMsgLen(msg[0]);

	if ( msgLen > 0 ) {
	  serialHw[serialNo]->write(msg,msgLen);
		FLASH_LED_OUT(serialNo);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Send a USB midi packet to ad-hoc serial
///////////////////////////////////////////////////////////////////////////////
void SerialMidi_SendPacket(midiPacket_t *pk, uint8_t serialNo)
{
  if (serialNo >= SERIAL_INTERFACE_MAX ) return;

	uint8_t msgLen = USBMidi::CINToLenTable[pk->packet[0] & 0x0F] ;

 	if ( msgLen > 0 ) {
		serialHw[serialNo]->write(&pk->packet[1],msgLen);
		FLASH_LED_OUT(serialNo);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Prepare a pseudo packet from serial midi and route it to the right target
///////////////////////////////////////////////////////////////////////////////
void SerialMidi_RouteMsg( uint8_t cable, midiXparser* xpMidi )
{
    midiPacket_t pk = { .i = 0 };
    uint8_t msgLen = xpMidi->getMidiMsgLen();
    uint8_t msgType = xpMidi->getMidiMsgType();

    pk.packet[0] = cable << 4;
    memcpy(&pk.packet[1],&(xpMidi->getMidiMsg()[0]),msgLen);

    // Real time single byte message CIN F->
    if ( msgType == midiXparser::realTimeMsgTypeMsk ) pk.packet[0]   += 0xF;
    else

    // Channel voice message => CIN A-E
    if ( msgType == midiXparser::channelVoiceMsgTypeMsk )
        pk.packet[0]  += ( (xpMidi->getMidiMsg()[0]) >> 4);
    else

    // System common message CIN 2-3
    if ( msgType == midiXparser::systemCommonMsgTypeMsk ) {

        // 5 -  single-byte system common message (Tune request is the only case)
        if ( msgLen == 1 ) pk.packet[0] += 5;

        // 2/3 - two/three bytes system common message
        else pk.packet[0] += msgLen;
    }

    else return; // We should never be here !

    RoutePacketToTarget( PORT_TYPE_JACK,&pk);
}

///////////////////////////////////////////////////////////////////////////////
// Parse sysex flows and make a packet for USB
// ----------------------------------------------------------------------------
// We use the midiXparser 'on the fly' mode, allowing to tag bytes as "captured"
// when they belong to a midi SYSEX message, without storing them in a buffer.
///////////////////////////////////////////////////////////////////////////////
void SerialMidi_RouteSysEx( uint8_t cable, midiXparser* xpMidi )
{
  static midiPacket_t pk[SERIAL_INTERFACE_MAX];
  static uint8_t packetLen[SERIAL_INTERFACE_MAX];
  static bool firstCall = true;

  uint8_t readByte = xpMidi->getByte();

  // Initialize everything at the first call
  if (firstCall ) {
    firstCall = false;
    memset(pk,0,sizeof(midiPacket_t)*SERIAL_INTERFACE_MAX);
    memset(packetLen,0,sizeof(uint8_t)*SERIAL_INTERFACE_MAX);
  }

  // Normal End of SysEx or : End of SysEx with error.
  // Force clean end of SYSEX as the midi usb driver
  // will not understand if we send the packet as is
  if ( xpMidi->wasSysExMode() ) {
      // Force the eox byte in case we have a SYSEX error.
      packetLen[cable]++;
      pk[cable].packet[ packetLen[cable] ] = midiXparser::eoxStatus;
      // CIN = 5/6/7  sysex ends with one/two/three bytes,
      pk[cable].packet[0] = (cable << 4) + (packetLen[cable] + 4) ;
      RoutePacketToTarget( PORT_TYPE_JACK,&pk[cable]);
      packetLen[cable] = 0;
      pk[cable].i = 0;
			return;
  } else

  // Fill USB sysex packet
  if ( xpMidi->isSysExMode() ) {
	  packetLen[cable]++;
	  pk[cable].packet[ packetLen[cable] ] = readByte ;

	  // Packet complete ?
	  if (packetLen[cable] == 3 ) {
	      pk[cable].packet[0] = (cable << 4) + 4 ; // Sysex start or continue
	      RoutePacketToTarget( PORT_TYPE_JACK,&pk[cable]);
	      packetLen[cable] = 0;
	      pk[cable].i = 0;
	  }
	}
}

///////////////////////////////////////////////////////////////////////////////
// THE MIDI PACKET ROUTER
//-----------------------------------------------------------------------------
// Route a packet from a midi IN jack / USB OUT to
// a midi OUT jacks / USB IN  or I2C remote serial midi on another device
///////////////////////////////////////////////////////////////////////////////
void RoutePacketToTarget(uint8_t portType,  midiPacket_t *pk)
{

  if ( portType != PORT_TYPE_CABLE && portType != PORT_TYPE_JACK && portType != PORT_TYPE_VIRTUAL) return;

  // NB : we use the same routine to route USB and jack serial/ I2C .
	// The Cable can be the serial port # if coming from local serial
  uint8_t port  = pk->packet[0] >> 4;
  uint8_t cin   = pk->packet[0] & 0x0F ;

  // ROUTING rules masks
	uint16_t cbInTargets  = 0;
	uint16_t jkOutTargets = 0;
  uint16_t vrInTargets  = 0;
  uint8_t  attachedSlot = 0;

  // Save midiIthruActive state as it could be changed in an interrupt
  boolean ithru = midiIthruActive;

  FLASH_LED_IN(port);

  // A midi packet from physical serial jack ?
  if ( portType == PORT_TYPE_JACK ) {
    // Check at the physical level (i.e. not the bus)
    if ( port >= SERIAL_INTERFACE_MAX ) return;

    // If bus mode active, the local port# must be translated according
		// to the device Id, before routing
    if (EE_Prm.I2C_BusModeState == B_ENABLED ) {
			port = GET_BUS_SERIALNO_FROM_LOCALDEV(EE_Prm.I2C_DeviceId,port);
      // Rebuild packet header with source port translated
      pk->packet[0] = cin + ( port << 4) ;
    }

    // IntelliThru active ? If so, take the good routing rules
    if ( ithru ) {
      jkOutTargets = EE_Prm.rtRulesIthru[port].jkOutTgMsk;
      vrInTargets  = EE_Prm.rtRulesIthru[port].vrInTgMsk;
      attachedSlot = EE_Prm.rtRulesIthru[port].slot;
    }
    // else Standard jack rules
    else {
      cbInTargets  = EE_Prm.rtRulesJack[port].cbInTgMsk;
      jkOutTargets = EE_Prm.rtRulesJack[port].jkOutTgMsk;
      vrInTargets  = EE_Prm.rtRulesJack[port].vrInTgMsk;
      attachedSlot = EE_Prm.rtRulesJack[port].slot;
    }
  }
  // A midi packet from USB cable out ?
  else if ( portType == PORT_TYPE_CABLE ) {
    if ( port >= USBCABLE_INTERFACE_MAX ) return;
    cbInTargets  = EE_Prm.rtRulesCable[port].cbInTgMsk;
    jkOutTargets = EE_Prm.rtRulesCable[port].jkOutTgMsk;
    vrInTargets  = EE_Prm.rtRulesCable[port].vrInTgMsk;
    attachedSlot = EE_Prm.rtRulesCable[port].slot;
  }
  // A midi packet from a virtual port ?
  else if ( portType == PORT_TYPE_VIRTUAL ) {
    if ( port >= VIRTUAL_INTERFACE_MAX ) return;
    cbInTargets  = EE_Prm.rtRulesVirtual[port].cbInTgMsk;
    jkOutTargets = EE_Prm.rtRulesVirtual[port].jkOutTgMsk;
    attachedSlot = EE_Prm.rtRulesVirtual[port].slot;
  }

	// Sysex is a particular case when routing or modifying packets.
	// Internal sysex must be sent to Jack/Cable port 0. These ports 0 must be ALWAYS
  // available whatever routing is to insure that internal sysex will be always interpreted.
  // Internal sysex packets can't be looped back, because that will corrupt the flow.
  // A slave must not interpret sysex when active on bus to stay  synchronized with the master.
  // The port#0 is mandatory a port of the master or a port of a slave without bus.
  // Only virtual port can be 0 when bus mode active.
  if (port == 0 && portType != PORT_TYPE_VIRTUAL && !slotLockMsk) {
    // CIN 5 exception : tune request. It is not a sysex packet !
		if ( cin <= 7 && cin >= 4 && pk->packet[1] != midiXparser::tuneRequestStatus) {
      if (SysExInternal_Parse(portType, pk,globalDataBuffer))
            SysExInternal_Process(portType,globalDataBuffer);
    }
	}

  // 1/ Apply pipeline if any.  Drop packet if a pipe returned false
  if ( attachedSlot && !TransPacketPipelineExec(portType, attachedSlot, pk) ) return ;

  // 2/ Apply virtual port routing if a target match
  uint8_t t=0;
  while ( vrInTargets && t != VIRTUAL_INTERFACE_MAX ) {
    if ( vrInTargets & 1 ) {
      midiPacket_t pk2 = { .i = pk->i }; // packet copy
      pk2.packet[0] = ( t << 4 ) + cin;
      RoutePacketToTarget(PORT_TYPE_VIRTUAL,  &pk2);
    }
    t++; vrInTargets >>= 1;
  }

  // 3/ Apply serial jack routing if a target match
  t=0;
  while ( jkOutTargets && t != SERIAL_INTERFACE_COUNT ) {
    if ( jkOutTargets & 1 ) {
      // Route via the bus or local serial if bus mode disabled
      if (EE_Prm.I2C_BusModeState == B_ENABLED ) I2C_BusSerialSendMidiPacket(pk, t);
      else SerialMidi_SendPacket(pk,t);
    }
    t++; jkOutTargets >>= 1;
  }

  // Stop here if IntelliThru active (no USB active but maybe connected)
  // or no USB connection (owned by the master).
  // If we are a slave, the master should have notified us
  // Intellithru is always activated by the master in bus mode!.

  if ( ithru || !midiUSBCx ) return;

	// 4/ Apply cable routing rules only if USB connected and thru mode inactive
  t=0;
  while ( cbInTargets && t != USBCABLE_INTERFACE_MAX ) {
    if ( cbInTargets & 1 ) {
        midiPacket_t pk2 = { .i = pk->i }; // packet copy to change the dest cable
        pk2.packet[0] = ( t << 4 ) + cin;
        // Only the master has USB midi privilege in bus MODE
        // Everybody else if an usb connection is active
        if ( !(IS_SLAVE && IS_BUS_E) ) {
            MidiUSB.writePacket(&pk2.i);
        } else
        // A slave in bus mode ?
        // We need to add a master packet to the Master's queue.
        {
            masterMidiPacket_t mpk;
            mpk.mpk.dest = PORT_TYPE_CABLE;
            // Copy the midi packet to the master packet
            mpk.mpk.pk.i = pk2.i;
            I2C_QPacketsToMaster.write(mpk.packet,sizeof(masterMidiPacket_t));
        }
    }
    t++; cbInTargets >>= 1;
  }
  // All Routing Done !
}

///////////////////////////////////////////////////////////////////////////////
// MIDI CLOCK GENERATOR TO VIRTUAL PORTS
// Clocks are mandatory attached to their respective virtual port.
///////////////////////////////////////////////////////////////////////////////
void MidiClockGenerator()
{
  static unsigned long nextMTCFrameTick = 0;
  uint8_t frameByte = 0;
  boolean sendMTC = false;

  // MTC Frame byte
  if ( micros() > nextMTCFrameTick ) {
       nextMTCFrameTick = micros() + MTC_FRAME_TICK;
       frameByte = MidiTimeCodeGetFrameByte();
       sendMTC = true;
  }

  for ( uint8_t i = 0 ;  i != MIDI_CLOCKGEN_MAX ; i++ ) {
    // MTC
    if ( EE_Prm.bpmClocks[i].mtc && sendMTC ) {
       midiPacket_t MtcPk = {.packet = { (uint8_t)(0x02 + (i<<4)),0XF1, frameByte,0x00 } };
       RoutePacketToTarget(PORT_TYPE_VIRTUAL, &MtcPk);
    }
    // Midi Clock
    if ( EE_Prm.bpmClocks[i].enabled ) {
       // Generate a midi timingClock status packet
       if ( micros() > bpmTicks[i].nextBpmTick ) {
          // Change virtual port clk 0 to port 0, clk1 port1, ...
          midiPacket_t timingClockPk = { .packet= {0x0F ,0XF8,0,0} };
          timingClockPk.packet[0] += (i<< 4);
          RoutePacketToTarget(PORT_TYPE_VIRTUAL, &timingClockPk);
          bpmTicks[i].nextBpmTick += bpmTicks[i].tickBpm ;
       }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Set a midi clock bpm or 7F for all.
///////////////////////////////////////////////////////////////////////////////
boolean SetMidiBpmClock(uint8_t clockNo, uint16_t bpm)
{
  if (clockNo != 0x7F && clockNo >= MIDI_CLOCKGEN_MAX ) return false;
  if ( (bpm != 0 && bpm < MIN_BPM) || bpm > MAX_BPM  ) return false;

  if ( clockNo == 0x7F) {
    for ( uint8_t i=0 ; i !=MIDI_CLOCKGEN_MAX ; i++ ) {
      // If bpm = 0, then current bpm is kept.
      if (bpm) EE_Prm.bpmClocks[i].bpm = bpm;
      bpmTicks[i].tickBpm = ( (60000000 / EE_Prm.bpmClocks[i].bpm ) / 24 ) * 10;
      bpmTicks[i].nextBpmTick = micros() + bpmTicks[i].tickBpm;
    }
    return true;
  }
  // a valid bpm value is required here
  if (! bpm) return false;
  EE_Prm.bpmClocks[clockNo].bpm = bpm;
  bpmTicks[clockNo].tickBpm = ( (60000000 / EE_Prm.bpmClocks[clockNo].bpm ) / 24 ) * 10;
  bpmTicks[clockNo].nextBpmTick = micros() + bpmTicks[clockNo].tickBpm;

  return true;
}
///////////////////////////////////////////////////////////////////////////////
// Enable/Disable a midi clock or 7F for all .
///////////////////////////////////////////////////////////////////////////////
boolean SetMidiEnableClock(uint8_t clockNo, boolean enable)
{
  if (clockNo != 0x7F && clockNo >= MIDI_CLOCKGEN_MAX ) return false;

  if ( clockNo == 0x7F) {
    for ( uint8_t i=0 ; i !=MIDI_CLOCKGEN_MAX ; i++ ) {
        EE_Prm.bpmClocks[i].enabled = enable;
        bpmTicks[i].nextBpmTick = micros() + bpmTicks[i].tickBpm;
    }
    return true;
  }
  bpmTicks[clockNo].nextBpmTick = micros() + bpmTicks[clockNo].tickBpm;
  EE_Prm.bpmClocks[clockNo].enabled = enable;
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Generate the midi time code (MTC) frame byte
///////////////////////////////////////////////////////////////////////////////
uint8_t MidiTimeCodeGetFrameByte()
{
  //static unsigned long nextFrameTick = 0;
  static uint8_t hh=0, mm=0 , ss=0, frmType=0, frmCount=0;
  // MTC packet F1 0nnn dddd
  // 0nnn = frame Type - dddd = 4 bits data

  uint8_t frameByte = 0;
  // frame type
  frameByte = frmType << 4;
  //   0 = Frame count LS nibble
  if ( frmType == 0 ) frameByte += frmCount & 0x0F ;
  //   1 = Frame count MS nibble
  else if ( frmType == 1 ) frameByte += frmCount >> 4  ;
  //   2 = Seconds LS nibble
  else if ( frmType == 2 ) frameByte += ss & 0x0F ;
  //   3 = Seconds count MS nibble
  else if ( frmType == 3 ) frameByte += ss  >> 4 ;
  //   4 = Minutes count LS nibble
  else if ( frmType == 4 ) frameByte += mm & 0x0F ;
  //   5 = Minutes count MS nibble
  else if ( frmType == 5 ) frameByte += mm  >> 4 ;
  //   6 = Hours count LS nibble
  else if ( frmType == 6 ) frameByte += hh & 0x0F ;
  //   7 = Hours count MS nibble and SMPTE Type : 0nnn x yy d
  //  Where nnn is 7. x is unused and set to 0.
  //  d is bit 4 of the Hours Time. yy tells the SMPTE Type as follows:
  // 0 = 24 fps, 1 = 25 fps, 2 = 30 fps (Drop-Frame), 3 = 30 fps
  if ( frmType++ == 7 ) { frameByte += (hh  >> 4) + (MTC_SMPTE_TYPE<<1) ; frmType = 0; }

  if ( ++frmCount == MTC_FPS ) { frmCount = 0 ;
     if ( ++ss > 59 ) { ss = 0 ;
       if ( ++mm > 59 ) { mm = 0 ;
         if ( ++hh > 23 ) hh = 0;
       } //mm
     } // ss
  } //frmCount

  return frameByte;
}

///////////////////////////////////////////////////////////////////////////////
// Reset routing rules to default factory
// ROUTING_RESET_ALL         : Factory defaults
// ROUTING_RESET_MIDIUSB     : Midi USB , serial, virtual routing to defaults
// ROUTING_RESET_INTELLITHRU : Intellithru to factory defaults
// ROUTING_INTELLITHRU_OFF   : Stop IntelliThru
// ROUTING_CLEAR_ALL         : Erase all routing and pipeline rules
///////////////////////////////////////////////////////////////////////////////
void ResetMidiRoutingRules(uint8_t mode)
{

  // Clear all pipelines slots
  if (mode == ROUTING_RESET_ALL || mode == ROUTING_CLEAR_ALL ) {
      TransPacketPipeline_ClearSlot(0x7F);
  }

  if (mode == ROUTING_RESET_ALL || mode == ROUTING_RESET_MIDIUSB || mode == ROUTING_CLEAR_ALL ) {
    // Virtual
    for ( uint8_t i = 0 ; i != VIRTUAL_INTERFACE_MAX ; i++ ) {
      EE_Prm.rtRulesVirtual[i].slot = 0;
      EE_Prm.rtRulesVirtual[i].cbInTgMsk = 0 ;
      EE_Prm.rtRulesVirtual[i].jkOutTgMsk = 0  ;
    }
  }

  if (mode == ROUTING_CLEAR_ALL ) {
    for ( uint8_t i = 0 ; i != USBCABLE_INTERFACE_MAX ; i++ ) {

      // Cables
      EE_Prm.rtRulesCable[i].slot = 0;
      EE_Prm.rtRulesCable[i].cbInTgMsk = 0 ;
      EE_Prm.rtRulesCable[i].jkOutTgMsk = 0 ;
      EE_Prm.rtRulesCable[i].vrInTgMsk = 0  ;

    }
    // Jack serial
    for ( uint8_t i = 0 ; i != B_SERIAL_INTERFACE_MAX ; i++ ) {
      EE_Prm.rtRulesJack[i].slot = 0;
      EE_Prm.rtRulesJack[i].cbInTgMsk = 0 ;
      EE_Prm.rtRulesJack[i].jkOutTgMsk = 0  ;
      EE_Prm.rtRulesJack[i].vrInTgMsk = 0  ;
    }

    // "Intelligent thru" serial mode
	  for ( uint8_t i = 0 ; i != B_SERIAL_INTERFACE_MAX ; i++ ) {
	    EE_Prm.rtRulesIthru[i].slot = 0;
	    EE_Prm.rtRulesIthru[i].jkOutTgMsk = 0 ;
      EE_Prm.rtRulesIthru[i].vrInTgMsk = 0  ;
		}
		EE_Prm.ithruJackInMsk = 0;
	  EE_Prm.ithruUSBIdleTimePeriod = DEFAULT_ITHRU_USB_IDLE_TIME_PERIOD ;
	}


	if (mode == ROUTING_RESET_ALL || mode == ROUTING_RESET_MIDIUSB) {

	  for ( uint8_t i = 0 ; i != USBCABLE_INTERFACE_MAX ; i++ ) {
			// Cables
	    EE_Prm.rtRulesCable[i].slot = 0;
	    EE_Prm.rtRulesCable[i].cbInTgMsk = 0 ;
	    EE_Prm.rtRulesCable[i].jkOutTgMsk = 1 << i ;
      EE_Prm.rtRulesCable[i].vrInTgMsk = 0  ;
		}

		for ( uint8_t i = 0 ; i != B_SERIAL_INTERFACE_MAX ; i++ ) {
			// Jack serial
	    EE_Prm.rtRulesJack[i].slot = 0;
	    EE_Prm.rtRulesJack[i].cbInTgMsk = 1 << i ;
	    EE_Prm.rtRulesJack[i].jkOutTgMsk = 0  ;
      EE_Prm.rtRulesJack[i].vrInTgMsk = 0  ;
	  }

  }

	if (mode == ROUTING_RESET_ALL || mode == ROUTING_RESET_INTELLITHRU) {
	  // "Intelligent thru" serial mode

	  for ( uint8_t i = 0 ; i != B_SERIAL_INTERFACE_MAX ; i++ ) {
	    EE_Prm.rtRulesIthru[i].slot = 0;
	    EE_Prm.rtRulesIthru[i].jkOutTgMsk = 0 ;
      EE_Prm.rtRulesIthru[i].vrInTgMsk = 0  ;
		}
    // Default IN 1 -> OUT 1,2 (split) , IN 2,3 -> OUT 3 (merge)
    EE_Prm.rtRulesIthru[0].jkOutTgMsk = B0011 ;
    EE_Prm.rtRulesIthru[1].jkOutTgMsk = B0100 ;
    EE_Prm.rtRulesIthru[2].jkOutTgMsk = B0100 ;

		EE_Prm.ithruJackInMsk = 0;
	  EE_Prm.ithruUSBIdleTimePeriod = DEFAULT_ITHRU_USB_IDLE_TIME_PERIOD ;

	}

}

///////////////////////////////////////////////////////////////////////////////
// Send a SYSEX midi message to USB Cable 0
///////////////////////////////////////////////////////////////////////////////
boolean USBMidi_SendSysExPacket( uint8_t cable, const uint8_t sxBuff[],uint16_t sz)
{
  midiPacket_t pk { .i = 0};
  uint8_t b=0;
  bool startSx=false;
  bool endSx=false;

  if (cable > 0x0F) return false;
  if ( sxBuff[0] != 0xF0 || sxBuff[sz-1] != 0xF7) return false;

  // Build sysex packets
  // Multiple Sysyex messages can be embedded in the buffer :
  // F0 nn ... nn F7 F0 nn ... nn F7 so we have to care about that.

  for ( uint16_t i = 0; i != sz ; i ++ ) {
      // Check integrity
      if ( sxBuff[i] == 0xF0) startSx = true;
      if ( sxBuff[i] == 0xF7) endSx = true;
      if (startSx) {
        pk.packet[++b] = sxBuff[i];
        if ( b == 3 || endSx ) {
          pk.packet[0]  = (endSx ?  b + 4 : 4 ) + (cable << 4);
          MidiUSB.writePacket(&pk.i);
          if (endSx) startSx = endSx = false;
          b=0; pk.i = 0;
        }
      }
  }

  FLASH_LED_OUT(0);
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Get/ Set magic boot mode
// DR5 backup register is used for UMK4x4
///////////////////////////////////////////////////////////////////////////////
uint16_t GetAndClearBootMagicWord()
{
  uint16_t magicWord = 0x0000;

  RCC_BASE->APB1ENR |=  (RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN) ;
  // read magic word in register register
  magicWord = BKP_BASE->BOOT_REGISTER ;
  // Reset magic word
  // Enable write access to the backup registers and the RTC
  PWR_BASE->CR |= PWR_CR_DBP;
  // write register
  BKP_BASE->BOOT_REGISTER = BOOT_MIDI_MAGIC; // Default;
  // Disable write
  PWR_BASE->CR &= ~PWR_CR_DBP;
  RCC_BASE->APB1ENR &=  ~(RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN) ;
  return magicWord;
}

void SetBootMagicWord(uint16_t magicWord)
{
  if ( magicWord != BOOT_BTL_MAGIC &&
       magicWord != BOOT_CONFIG_MAGIC &&
       magicWord != BOOT_MIDI_MAGIC )

       return;

   // global
   bootMagicWord = magicWord;

   // Write the Magic word bootloader

   RCC_BASE->APB1ENR |=  (RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN) ;
   // Enable write access to the backup registers and the RTC
   PWR_BASE->CR |= PWR_CR_DBP;

   // write register
   // if bootloader then write hid_boolaoder magic word to DR4/10
   // Write also DR5 to come back to config mode.
   if (magicWord == BOOT_BTL_MAGIC ) {
     BKP_BASE->BOOT_BTL_REGISTER = BOOT_BTL_MAGIC;
     BKP_BASE->BOOT_REGISTER = BOOT_CONFIG_MAGIC;
   }
   // usual case. No Wait.
   else {
     BKP_BASE->BOOT_BTL_REGISTER = BOOT_BTL_MAGIC_NOWAIT;
     BKP_BASE->BOOT_REGISTER = bootMagicWord;
   }

   // Disable write
   PWR_BASE->CR &= ~PWR_CR_DBP;
   RCC_BASE->APB1ENR &=  ~(RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN) ;

}

///////////////////////////////////////////////////////////////////////////////
// Check what is the current boot mode.
// Will never come back if config mode.
///////////////////////////////////////////////////////////////////////////////
void CheckBootMode()
{
	// Does the config menu boot mode is active ?
	// if so, prepare the next boot in MIDI mode and jump to menu

  // Read the boot magic word if default.
  // If a new build was uploaded, we force config mode.
    if (bootMagicWord == BOOT_CONFIG_MAGIC || GetAndClearBootMagicWord() == BOOT_CONFIG_MAGIC) {

      // Next boot on Midi
      SetBootMagicWord(BOOT_MIDI_MAGIC);

			#ifdef HAS_MIDITECH_HARDWARE
				// Assert DISC PIN (PA8 usually for Miditech) to enable USB
				gpio_set_mode(PIN_MAP[PA8].gpio_device, PIN_MAP[PA8].gpio_bit, GPIO_OUTPUT_PP);
				gpio_write_bit(PIN_MAP[PA8].gpio_device, PIN_MAP[PA8].gpio_bit, 1);
			#endif

			// start USB serial
			Serial.begin(115200);
			delay(500);

      // Start Wire as a master for config & tests purpose in the menu
      Wire.begin();
      Wire.setClock(B_FREQ) ;

			// wait for a serial monitor to be connected.
			// 3 short flash

			while (!Serial) {
          LED_Flash(&LED_ConnectTick);delay(300);
          LED_Flash(&LED_ConnectTick);delay(300);
          LED_Flash(&LED_ConnectTick);delay(300);
				}
			digitalWrite(LED_CONNECT, LOW);
#if OPTION_CONFIGUI
			ShowConfigMenu(); // INFINITE LOOP
#endif
	}

}

///////////////////////////////////////////////////////////////////////////////
// MIDI USB initiate connection if master
// + Set USB descriptor strings
///////////////////////////////////////////////////////////////////////////////
void USBMidi_Init()
{
	usb_midi_set_vid_pid(EE_Prm.vendorID,EE_Prm.productID);
	usb_midi_set_product_string((char *) &EE_Prm.productString);

	MidiUSB.begin() ;
  // Note : Usually around 4 s to fully detect USB Midi on the host
  while (! MidiUSB.isConnected() ) delay(500);
}

///////////////////////////////////////////////////////////////////////////////
// MIDI USB Loop Process
///////////////////////////////////////////////////////////////////////////////
void USBMidi_Process()
{
  // Try to connect/reconnect USB if we detect a high level on USBDM
	// This is to manage the case of a powered device without USB active or suspend mode for ex.
  static unsigned long ledCxMillis = 0;
  static unsigned long lastPacketMillis = 0;

	if ( MidiUSB.isConnected() ) {

    unsigned long lastPollMillis = millis();

    if (lastPollMillis > ledCxMillis ) {
      LED_TurnOn(&LED_ConnectTick);
      ledCxMillis = lastPollMillis + LED_CONNECT_USB_RECOVER_TIME_MILLIS;
    }

    midiUSBCx = true;

		// Do we have a MIDI USB packet available ?
		if ( MidiUSB.available() ) {
			lastPacketMillis = lastPollMillis  ;
      ledCxMillis = lastPollMillis + LED_CONNECT_USB_RECOVER_TIME_MILLIS;
			midiUSBIdle = false;
			midiIthruActive = false;

			// Read a Midi USB packet .
			if ( !isSerialBusy ) {
				midiPacket_t pk ;
				pk.i = MidiUSB.readPacket();
				RoutePacketToTarget( PORT_TYPE_CABLE,  &pk );
			} else {
					isSerialBusy = false ;
			}
		}
    else
		if (!midiUSBIdle && lastPollMillis > ( lastPacketMillis + ithruUSBIdlelMillis ) )
				midiUSBIdle = true;
	}
	// Are we physically connected to USB
	else {
       if (midiUSBCx) LED_TurnOff(&LED_ConnectTick);
       midiUSBCx = false;
		   midiUSBIdle = true;
  }

	if ( midiUSBIdle && !midiIthruActive && EE_Prm.ithruJackInMsk) {
			midiIthruActive = true;
			FlashAllLeds(0); // All leds when Midi intellithru mode active
	}

}

///////////////////////////////////////////////////////////////////////////////
// MIDI SERIAL Loop Process
///////////////////////////////////////////////////////////////////////////////
void SerialMidi_Process()
{
	// LOCAL SERIAL JACK MIDI IN PROCESS
	for ( uint8_t s = 0; s< SERIAL_INTERFACE_MAX  ; s++ )
	{
				// Do we have any MIDI msg on Serial 1 to n ?
				if ( serialHw[s]->available() ) {
					 if ( midiSerial[s].parse( serialHw[s]->read() ) ) {
								// We manage sysEx "on the fly". Clean end of a sysexe msg ?
								if ( midiSerial[s].getMidiMsgType() == midiXparser::sysExMsgTypeMsk )
									SerialMidi_RouteSysEx(s, &midiSerial[s]) ;
								// Not a sysex. The message is complete.
								else {
                  SerialMidi_RouteMsg( s, &midiSerial[s]);
                }
					 }
					 else
					 // Acknowledge any sysex error
					 if ( midiSerial[s].isSysExError() )
						 SerialMidi_RouteSysEx(s, &midiSerial[s]) ;
					 else
					 // Check if a SYSEX mode active and send bytes on the fly.
					 if ( midiSerial[s].isSysExMode() && midiSerial[s].isByteCaptured() ) {
							SerialMidi_RouteSysEx(s, &midiSerial[s]) ;
					 }
				}

				// Manage Serial contention vs USB
				// When one or more of the serial buffer is full, we block USB read one round.
				// This implies to use non blocking Serial.write(buff,len).
				if (  midiUSBCx &&  !serialHw[s]->availableForWrite() ) isSerialBusy = true; // 1 round without reading USB
	}
}

///////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////
void setup()
{

    Serial.end();

    // Retrieve EEPROM parameters
    EE_PrmInit();

    // Reset LEDs counters.
    LED_Init();

    // Configure the general purpose TIMER.
    HardwareTimer *timerMillis = new HardwareTimer(2);

    // Configure the millisec timer ISR
    timerMillis->pause();
    timerMillis->setPeriod(TIMER_MILLIS_RATE_MICROS); // in microseconds
    // Set up an interrupt on channel 1
    timerMillis->setChannel1Mode(TIMER_OUTPUT_COMPARE);
    // Interrupt 1 count after each update
    timerMillis->setCompare(TIMER_CH4, 1);
    timerMillis->attachInterrupt(TIMER_CH4, TimerMillisHandler);
    timerMillis->refresh();
    timerMillis->resume();

    // recompute all midi clocks ticks...
    SetMidiBpmClock(0x7F, 0);

    // Check if we must go to configuration mode
		CheckBootMode();

    // MIDI MODE START HERE ==================================================

    ithruUSBIdlelMillis = EE_Prm.ithruUSBIdleTimePeriod * 15000;

    // MIDI SERIAL PORTS set Baud rates and parser inits
    // To compile with the 4 serial ports, you must use the right variant : STMF103RC
    // + Set parsers filters in the same loop.  All messages including on the fly SYSEX.

    for ( uint8_t s=0; s != SERIAL_INTERFACE_MAX ; s++ ) {
      serialHw[s]->begin(31250);
      midiSerial[s].setMidiMsgFilter( midiXparser::allMsgTypeMsk );
    }

		// I2C bus checks that could disable the bus mode
  	I2C_BusChecks();

    // PROCESSES.
    // 1. SerialMidi_Process
    // 2. MidiClockGenerator (if not slave on bus)
    // 3. USBMidi_Process
    // 4. MidiClockGenerator (if not slave on bus)
    // 5. I2C_ProcessSlave or I2C_ProcessMaster (if bus enabled)
    // 6. MidiClockGenerator (if master on bus )
    //
    // MidiClockGenerator is added after each process because we don't
    // want to wait the full loop, for accuracy.

    // 1. Add Serial process to process fn vector
    procVectorFn[procVectorFnCount++] = &SerialMidi_Process;

    // Midi USB only if available only for master when bus is enabled or master/slave
    if ( !(IS_SLAVE && IS_BUS_E)  ) {
        // 2. Add midi clock generator not for a slave on bus
        procVectorFn[procVectorFnCount++] = &MidiClockGenerator;
        // 3. Add midi USB process
        procVectorFn[procVectorFnCount++] = &USBMidi_Process;
        USBMidi_Init();
        // 4. add again midi clock generator for the best refresh rate possible
        procVectorFn[procVectorFnCount++] = &MidiClockGenerator;
  	}

    // 5. Add add-hoc I2C BUS process vector fn when bus is enabled
    if ( IS_BUS_E ) {
      if (IS_SLAVE ) procVectorFn[procVectorFnCount++] = &I2C_ProcessSlave;
      else
      if ( IS_MASTER )  {
        procVectorFn[procVectorFnCount++] = &I2C_ProcessMaster;
        // 6. Add midi clock generator again for the master.
        procVectorFn[procVectorFnCount++] = &MidiClockGenerator;
      }
    }

    // Start Wire (I2C) if bus mode enabled. AFTER MIDI !
    I2C_BusStartWire();

    // Show end of setup phase
    // Seems that using LED_Flash in combination with timer introduce
    // some conflicts with USB (cf USBMidi_Init above)...
    // This is a low level thing...in the core library probably.

    FlashAllLeds(0);

}

///////////////////////////////////////////////////////////////////////////////
// LOOP
///////////////////////////////////////////////////////////////////////////////
void loop()
{
    // Pure loop of functions in vector
    uint8_t p = procVectorFnCount;
    do {  procVectorFn[--p](); } while (p);
}
