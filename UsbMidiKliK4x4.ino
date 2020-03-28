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

#include "build_number_defines.h"
#include <string.h>
#include <libmaple/nvic.h>
#include <Wire_slave.h>
#include <PulseOutManager.h>
#include <midiXparser.h>
#include "usbmidiklik4x4.h"
#include "usb_midi.h"
#include "ringbuffer.h"

///////////////////////////////////////////////////////////////////////////////
// GLOBALS
///////////////////////////////////////////////////////////////////////////////

// EEPROMS parameters
EEPROM_Params_t EEPROM_Params;
// Timer
HardwareTimer timer(2);
// Serial interfaces Array
HardwareSerial * serialHw[SERIAL_INTERFACE_MAX] = {SERIALS_PLIST};
// Prepare LEDs pulse for Connect, MIDIN and MIDIOUT
// From MIDI SERIAL point of view
// Use a PulseOutManager factory to create the pulses
PulseOutManager flashLEDManager;
PulseOut* flashLED_CONNECT = flashLEDManager.factory(LED_CONNECT,LED_PULSE_MILLIS,LOW);

#ifdef HAS_MIDITECH_HARDWARE
  // LED must be declared in the same order as hardware serials
  #define LEDS_MIDI
  PulseOut* flashLED_IN[SERIAL_INTERFACE_MAX] =
	{
    flashLEDManager.factory(D4,LED_PULSE_MILLIS,LOW),
    flashLEDManager.factory(D5,LED_PULSE_MILLIS,LOW),
    flashLEDManager.factory(D6,LED_PULSE_MILLIS,LOW),
    flashLEDManager.factory(D7,LED_PULSE_MILLIS,LOW)
  };
  PulseOut* flashLED_OUT[SERIAL_INTERFACE_MAX] =
	{
    flashLEDManager.factory(D36,LED_PULSE_MILLIS,LOW),
    flashLEDManager.factory(D37,LED_PULSE_MILLIS,LOW),
    flashLEDManager.factory(D16,LED_PULSE_MILLIS,LOW),
    flashLEDManager.factory(D17,LED_PULSE_MILLIS,LOW)
  };
#endif

// Timer used to signal I2C events every 500 ms
PulseOut I2C_LedTimer(0xFF,500);

// USB Midi object & globals
USBMidi MidiUSB;
volatile bool					midiUSBCx      = false;
volatile bool         midiUSBIdle    = false;
bool                  midiUSBLaunched= false;
bool 					        isSerialBusy   = false ;
unsigned long         midiUSBLastPacketMillis    = 0;
// MIDI Parsers for serial 1 to n
midiXparser midiSerial[SERIAL_INTERFACE_MAX];

uint8_t sysExInternalHeader[] = {SYSEX_INTERNAL_HEADER} ;
uint8_t sysExInternalIdentityRqReply[] = {SYSEX_INTERNAL_IDENTITY_RQ_REPLY};
uint8_t sysExInternalBuffer[SYSEX_INTERNAL_BUFF_SIZE] ;


// Intelligent midi thru mode
volatile bool intelliThruActive = false;
unsigned long intelliThruDelayMillis = DEFAULT_INTELLIGENT_MIDI_THRU_DELAY_PERIOD * 15000;

// Bus Mode globals
uint8_t I2C_DeviceIdActive[B_MAX_NB_DEVICE-1]; // Minus the master
uint8_t I2C_DeviceActiveCount=0;

boolean I2C_MasterReady = false;
volatile unsigned long I2C_MasterReadyTimeoutMillis = 0;
volatile uint8_t I2C_Command = B_CMD_NONE;

// Master to slave synchonization globals
volatile boolean I2C_SlaveSyncStarted = false;
volatile boolean I2C_SlaveSyncDoUpdate = false;

// Templated RingBuffers to manage I2C slave reception/transmission outside I2C ISR
// Volatile by default and RESERVED TO SLAVE
RingBuffer<uint8_t,B_RING_BUFFER_PACKET_SIZE> I2C_QPacketsFromMaster;
RingBuffer<uint8_t,B_RING_BUFFER_MPACKET_SIZE> I2C_QPacketsToMaster;

///////////////////////////////////////////////////////////////////////////////
//  CODE MODULES
//-----------------------------------------------------------------------------
// Due to the unusual make process of Arduino platform, modules are included
// directly here as "h" type. This allows a better code separation and readability.
///////////////////////////////////////////////////////////////////////////////
// DO NOT REMOVE OR CHANGE THE ORDER !

#include "mod_macros.h"
#include "mod_eeprom.h"
#include "mod_configui.h"
#include "mod_i2cbus.h"
#include "mod_miditransfn.h"

///////////////////////////////////////////////////////////////////////////////
//  CORE FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// memcmpcpy : copy if different
///////////////////////////////////////////////////////////////////////////////
int memcmpcpy ( void * pDest, void * pSrc, size_t sz ) {

  int r = 0;
  if ( r = memcmp(pDest,pSrc,sz) ) {
    memcpy(pDest,pSrc,sz);
  };

  return r;

}
///////////////////////////////////////////////////////////////////////////////
// Timer2 interrupt handler
///////////////////////////////////////////////////////////////////////////////
void Timer2Handler(void)
{
     // Update LEDS & timer
     flashLEDManager.update(millis());
     I2C_LedTimer.update(millis());
     #ifdef DEBUG_MODE
     I2C_DebugTimer.update(millis());
     #endif
}

///////////////////////////////////////////////////////////////////////////////
// FlashAllLeds . 0 = Alls. 1 = In. 2 = Out
///////////////////////////////////////////////////////////////////////////////
void FlashAllLeds(uint8_t mode)
{
  for ( uint8_t f=0 ; f!= 4 ; f++ ) {
		#ifdef LEDS_MIDI
			for ( uint8_t i=0 ; i< SERIAL_INTERFACE_MAX ; i++ ) {
					if ( mode == 0 || mode ==1 ) flashLED_IN[i]->start();
					if ( mode == 0 || mode ==2 ) 	flashLED_OUT[i]->start();
			}
		#else
			flashLED_CONNECT->start();
		#endif
		delay(100);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Send a midi msg to serial MIDI. 0 is Serial1.
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
// Send a USB midi packet to ad-hoc serial MIDI
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
// Prepare a packet and route it to the right USB midi cable
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

    RoutePacketToTarget( FROM_SERIAL,&pk);
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
      RoutePacketToTarget( FROM_SERIAL,&pk[cable]);
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
	      RoutePacketToTarget( FROM_SERIAL,&pk[cable]);
	      packetLen[cable] = 0;
	      pk[cable].i = 0;
	  }
	}
}

///////////////////////////////////////////////////////////////////////////////
// PARSE INTERNAL SYSEX, either for serial or USB
// ----------------------------------------------------------------------------
// Internal sysex must be transmitted on the first cable/midi jack (whatever routing).
// Internal sysex are purely ignored on other cables or jack.
///////////////////////////////////////////////////////////////////////////////
 void SysExInternalParse(uint8_t source, midiPacket_t *pk)
{
		static unsigned sysExInternalMsgIdx = 0;
		static bool 	sysExInternalHeaderFound = false;

		uint8_t cin   = pk->packet[0] & 0x0F ;
    uint8_t cable = pk->packet[0] >> 4;

		// Only SYSEX and concerned packet on cable or serial 1
    if (cable > 0) return;
		if (cin < 4 || cin > 7) return;
		if (cin == 4 && pk->packet[1] != 0xF0 && sysExInternalMsgIdx<3 ) return;
		if (cin > 4  && sysExInternalMsgIdx<3) return;

		uint8_t pklen = ( cin == 4 ? 3 : cin - 4) ;
		uint8_t ev = 1;

		for ( uint8_t i = 0 ; i< pklen ; i++ ) {
			if (sysExInternalHeaderFound) {
				// Start storing the message in the msg buffer
				// If Message too big. don't store...
				if ( sysExInternalBuffer[0] <  sizeof(sysExInternalBuffer)-1  ) {
						if (pk->packet[ev] != 0xF7) {
							sysExInternalBuffer[0]++;
							sysExInternalBuffer[sysExInternalBuffer[0]]  = pk->packet[ev];
						}
						ev++;
				}
			}	else

			if ( sysExInternalHeader[sysExInternalMsgIdx] == pk->packet[ev] ) {
				sysExInternalMsgIdx++;
				ev++;
				if ( sysExInternalMsgIdx >= sizeof(sysExInternalHeader) ) {
					sysExInternalHeaderFound = true;
					sysExInternalBuffer[0] = 0; // Len of the sysex buffer
				}
			}
			else {
				// No match
				sysExInternalMsgIdx = 0;
				sysExInternalHeaderFound = false;
				return;
			}
		}

		// End of SYSEX for a valid message ? => Process
		if (cin != 4  && sysExInternalHeaderFound ) {
			sysExInternalMsgIdx = 0;
			sysExInternalHeaderFound = false;
			SysExInternalProcess(source,sysExInternalBuffer);
		}
}

///////////////////////////////////////////////////////////////////////////////
// THE MIDI PACKET ROUTER
//-----------------------------------------------------------------------------
// Route a packet from a midi IN jack / USB OUT to
// a midi OUT jacks / USB IN  or I2C remote serial midi on another device
///////////////////////////////////////////////////////////////////////////////
void RoutePacketToTarget(uint8_t source,  midiPacket_t *pk)
{
  static boolean pipeLineLock = false;

  if ( source != FROM_USB && source != FROM_SERIAL ) return;

  // NB : we use the same routine to route USB and serial/ I2C .
	// The Cable can be the serial port # if coming from local serial
  uint8_t sourcePort  = pk->packet[0] >> 4;
  uint8_t cin         = pk->packet[0] & 0x0F ;

  // ROUTING rules pointers
	uint16_t *cableInTargets ;
	uint16_t *serialOutTargets ;
	uint8_t  *inFilters ;

  // Pipeline slot
  uint8_t   attachedPipelineSlot = 0;

  // Save intelliThruActive state as it could be changed in an interrupt
  boolean ithru = intelliThruActive;

  FLASH_LED_IN(sourcePort);

  // A midi packet from serial jack ?
  if ( source == FROM_SERIAL ) {
    // Check at the physical level (i.e. not the bus)
    if ( sourcePort >= SERIAL_INTERFACE_MAX ) return;

    // If bus mode active, the local port# must be translated according
		// to the device Id, before routing
    if (EEPROM_Params.I2C_BusModeState == B_ENABLED ) {
			sourcePort = GET_BUS_SERIALNO_FROM_LOCALDEV(EEPROM_Params.I2C_DeviceId,sourcePort);
      // Rebuild packet header with source port translated
      pk->packet[0] = cin + ( sourcePort << 4) ;
    }

    // IntelliThru active ? If so, take the good routing rules
    if ( ithru ) {
      if ( ! EEPROM_Params.intelliThruJackInMsk ) return; // Double check.
      serialOutTargets = &EEPROM_Params.midiRoutingRulesIntelliThru[sourcePort].jackOutTargetsMsk;
      inFilters = &EEPROM_Params.midiRoutingRulesIntelliThru[sourcePort].filterMsk;
      attachedPipelineSlot = TransPacketPipeline_FindAttachedSlot( PORT_TYPE_ITHRU, sourcePort );
    }
    // else Standard jack rules
    else {
      cableInTargets = &EEPROM_Params.midiRoutingRulesSerial[sourcePort].cableInTargetsMsk;
      serialOutTargets = &EEPROM_Params.midiRoutingRulesSerial[sourcePort].jackOutTargetsMsk;
      inFilters = &EEPROM_Params.midiRoutingRulesSerial[sourcePort].filterMsk;
      attachedPipelineSlot = TransPacketPipeline_FindAttachedSlot( PORT_TYPE_JACK, sourcePort );
    }
  }
  // A midi packet from USB cable out ?
  else {
    if ( sourcePort >= USBCABLE_INTERFACE_MAX ) return;
    cableInTargets = &EEPROM_Params.midiRoutingRulesCable[sourcePort].cableInTargetsMsk;
    serialOutTargets = &EEPROM_Params.midiRoutingRulesCable[sourcePort].jackOutTargetsMsk;
    inFilters = &EEPROM_Params.midiRoutingRulesCable[sourcePort].filterMsk;
    attachedPipelineSlot = TransPacketPipeline_FindAttachedSlot( PORT_TYPE_CABLE, sourcePort );
  }

	// Sysex is a particular case when using packets.
	// Internal sysex Jack 1/Cable = 0 ALWAYS!! are checked whatever filters are
	// This insures that the internal sysex will be always interpreted.
	// If the MCU is resetted, the msg will not be sent
	uint8_t  msgType=0;

	if (cin >= 4 && cin <= 7  ) {
		if (sourcePort == 0) SysExInternalParse(source, pk);
		msgType =  midiXparser::sysExMsgTypeMsk;
	} else {
			msgType =  midiXparser::getMidiStatusMsgTypeMsk(pk->packet[1]);
	}

	// 1/ Apply high level midi filters before pipeline
	if (! (msgType & *inFilters) ) return;

  // 2/ Apply pipeline if any
  if ( !pipeLineLock && attachedPipelineSlot ) {
    if ( EEPROM_Params.midiTransPipelineSlots[attachedPipelineSlot].pipeline[0].pId != FN_TRANSPIPE_NOPIPE) {
      pipeLineLock = true; // Avoid infinite loop
      boolean r = TransPacketPipelineExec(source, attachedPipelineSlot, pk) ;
      pipeLineLock = false;
      if (!r) return; // Drop packet
    }
  }

  // 3/ Apply serial jack routing if a target match
  if ( *serialOutTargets) {
				for (	uint16_t t=0; t != SERIAL_INTERFACE_COUNT ; t++)
					if ( (*serialOutTargets & ( 1 << t ) ) ) {
								// Route via the bus
								if (EEPROM_Params.I2C_BusModeState == B_ENABLED ) {
                     I2C_BusSerialSendMidiPacket(pk, t);
								}
								// Route to local serial if bus mode disabled
								else SerialMidi_SendPacket(pk,t);
					}
	}

  // Stop here if IntelliThru active (no USB active but maybe connected)
  // or no USB connection (owned by the master).
  // If we are a slave, the master should have notified us
  // Intellithru is always activated by the master in bus mode!.

  if ( ithru || !midiUSBCx ) return;

	// 4/ Apply cable routing rules from serial or USB
	// Only if USB connected and thru mode inactive
  if (  *cableInTargets  ) {
      midiPacket_t pk2 = { .i = pk->i }; // packet copy to change the dest cable
			for (uint8_t t=0; t != USBCABLE_INTERFACE_MAX ; t++) {
	      if ( *cableInTargets & ( 1 << t ) ) {
	          pk2.packet[0] = ( t << 4 ) + cin;
            // Only the master has USB midi privilege in bus MODE
            // Everybody else if an usb connection is active
            if (! B_IS_SLAVE ) {
                MidiUSB.writePacket(&pk2.i);
            } else
            // A slave in bus mode ?
            // We need to add a master packet to the Master's queue.
            {
                masterMidiPacket_t mpk;
                mpk.mpk.dest = TO_USB;
                // Copy the midi packet to the master packet
                mpk.mpk.pk.i = pk2.i;
                I2C_QPacketsToMaster.write(mpk.packet,sizeof(masterMidiPacket_t));
            }

	          #ifdef LEDS_MIDI
	          flashLED_IN[t]->start();
	          #endif
	    	}
			}
	}
  // Routing Done !

}

///////////////////////////////////////////////////////////////////////////////
// Reset routing rules to default factory
// ROUTING_RESET_ALL         : Factory defaults
// ROUTING_RESET_MIDIUSB     : Midi USB and serial routing to defaults
// ROUTING_RESET_INTELLITHRU : Intellithru to factory defaults
// ROUTING_INTELLITHRU_OFF   : Stop IntelliThru
///////////////////////////////////////////////////////////////////////////////
void ResetMidiRoutingRules(uint8_t mode)
{

  // Clear all pipelines
  if (mode == ROUTING_RESET_ALL) TransPacketPipeline_ClearSlot(0x7F);

	if (mode == ROUTING_RESET_ALL || mode == ROUTING_RESET_MIDIUSB) {

	  for ( uint8_t i = 0 ; i != USBCABLE_INTERFACE_MAX ; i++ ) {

			// Cables
	    EEPROM_Params.midiRoutingRulesCable[i].filterMsk = midiXparser::allMsgTypeMsk;
	    EEPROM_Params.midiRoutingRulesCable[i].cableInTargetsMsk = 0 ;
	    EEPROM_Params.midiRoutingRulesCable[i].jackOutTargetsMsk = 1 << i ;

		}

		for ( uint8_t i = 0 ; i != B_SERIAL_INTERFACE_MAX ; i++ ) {

			// Jack serial
	    EEPROM_Params.midiRoutingRulesSerial[i].filterMsk = midiXparser::allMsgTypeMsk;
	    EEPROM_Params.midiRoutingRulesSerial[i].cableInTargetsMsk = 1 << i ;
	    EEPROM_Params.midiRoutingRulesSerial[i].jackOutTargetsMsk = 0  ;
	  }

    // Reset pipeline attached ports
    for (uint8_t s=0 ; s != MIDI_TRANS_PIPELINE_SLOT_SIZE ; s++ ) {
      EEPROM_Params.midiTransPipelineSlots[s].attachedPortsMsk[PORT_TYPE_CABLE] = 0 ;
      EEPROM_Params.midiTransPipelineSlots[s].attachedPortsMsk[PORT_TYPE_JACK] = 0;
  	}

  }

	if (mode == ROUTING_RESET_ALL || mode == ROUTING_RESET_INTELLITHRU) {
	  // "Intelligent thru" serial mode
	  for ( uint8_t i = 0 ; i != B_SERIAL_INTERFACE_MAX ; i++ ) {
	    EEPROM_Params.midiRoutingRulesIntelliThru[i].filterMsk = midiXparser::allMsgTypeMsk;
	    EEPROM_Params.midiRoutingRulesIntelliThru[i].jackOutTargetsMsk = 0B1111 ;
		}
		EEPROM_Params.intelliThruJackInMsk = 0;
	  EEPROM_Params.intelliThruDelayPeriod = DEFAULT_INTELLIGENT_MIDI_THRU_DELAY_PERIOD ;

    // Reset pipeline attached ports
    for (uint8_t s=0 ; s != MIDI_TRANS_PIPELINE_SLOT_SIZE ; s++ )
      EEPROM_Params.midiTransPipelineSlots[s].attachedPortsMsk[PORT_TYPE_ITHRU] = 0;
	}

	// Disable "Intelligent thru" serial mode
	if (mode == ROUTING_INTELLITHRU_OFF ) {
		for ( uint8_t i = 0 ; i != B_SERIAL_INTERFACE_MAX ; i++ ) {
			EEPROM_Params.intelliThruJackInMsk = 0;
		}
	}

}

///////////////////////////////////////////////////////////////////////////////
// Return current configuration as a SYSEX buffer
///////////////////////////////////////////////////////////////////////////////
uint8_t SysexInternalDumpConf(uint32_t fnId, uint8_t port,uint8_t *buff) {

  uint8_t src;
  uint8_t dest = 0 ;
  uint16_t msk ;
  uint8_t i;
  uint8_t c;
  uint8_t *buff2 = buff;

  memcpy(buff2,sysExInternalHeader,sizeof(sysExInternalHeader));
  buff2+=sizeof(sysExInternalHeader);
  *buff2 = fnId >> 24;

  switch (fnId) {

    // Function 0B - Change USB Product String
    case 0x0B000000:
          strcpy((char*)++buff2,(char*)EEPROM_Params.productString);
          buff2+=strlen((char*)EEPROM_Params.productString)-1;
          break;

    // Function 0C - Change USB Vendor ID and Product ID
    case 0x0C000000:
          *(++buff2) = EEPROM_Params.vendorID >> 12;
          *(++buff2) = (EEPROM_Params.vendorID & 0x0F00) >> 8;
          *(++buff2) = (EEPROM_Params.vendorID & 0x00F0) >> 4;
          *(++buff2) = (EEPROM_Params.vendorID & 0x000F) ;
          *(++buff2) = EEPROM_Params.productID >> 12;
          *(++buff2) = (EEPROM_Params.productID & 0x0F00) >> 8;
          *(++buff2) = (EEPROM_Params.productID & 0x00F0) >> 4;
          *(++buff2) = (EEPROM_Params.productID & 0x000F) ;
          break;

    // Function 0E - Intellithru midi routing rules
    // 02 Timeout
    case 0x0E020000:
          *(++buff2) = 0X02;
          *(++buff2) = EEPROM_Params.intelliThruDelayPeriod;
          break;

     // Function 0E - Intellithru midi routing rules
     // 03 Routing rules
     case 0x0E030000:
          *(++buff2) = 0X03;
          *(++buff2) = port;
          *(++buff2) = EEPROM_Params.midiRoutingRulesIntelliThru[port].filterMsk;
          c = 0;
          for ( i=0; i != 16 ; i++) {
     						if ( EEPROM_Params.midiRoutingRulesIntelliThru[port].jackOutTargetsMsk & ( 1 << i) ) {
                      *(++buff2) = i;
                      c++;
                }
     		  }
          if (c == 0 ) return 0;
          break;

     // Function 0F - USB/Serial Midi midi routing rules
     // 02 Midi filter
     case 0x0F020000: // Cable
     case 0x0F020100: // Serial
         src  = (fnId & 0x0000FF00) >> 8;
         if (src > 1  ) return 0;
         *(++buff2) = 0X02;
         *(++buff2) = src;
         *(++buff2) = port;
         if (src) {
            *(++buff2) = EEPROM_Params.midiRoutingRulesSerial[port].filterMsk;
         }
         else {
            *(++buff2) = EEPROM_Params.midiRoutingRulesCable[port].filterMsk;
         }

         break;

     // Function 0F - USB/Serial Midi midi routing rules
     // 01 Routing rules
     case 0x0F010000: // Cable to Cable
     case 0x0F010001: // Cable to Serial
     case 0x0F010100: // Serial to Cable
     case 0x0F010101: // Serial to Serial
     src  = (fnId & 0x0000FF00) >> 8;
     dest = (fnId & 0x000000FF) ;
     if (src > 1 || dest > 1 ) return 0;
     *(++buff2) = 0X01;
     *(++buff2) = src;
     *(++buff2) = port;
     *(++buff2) = dest;

     if (src ) {
       msk = dest ? EEPROM_Params.midiRoutingRulesSerial[port].jackOutTargetsMsk : EEPROM_Params.midiRoutingRulesSerial[port].cableInTargetsMsk;
     } else {
       msk = dest ? EEPROM_Params.midiRoutingRulesCable[port].jackOutTargetsMsk : EEPROM_Params.midiRoutingRulesCable[port].cableInTargetsMsk;
     } ;

     c = 0;
     for ( i = 0 ; i != 16  ; i++) {
         if ( msk & ( 1 << i) ) {
           *(++buff2) = i;
           c++;
         }
     }
     if (c == 0 ) return 0;
     break;
  }
  *(++buff2) = 0xF7;
  return buff2-buff+1;
}


///////////////////////////////////////////////////////////////////////////////
// Send a SYSEX dump to the appropriate destination stream
// 0 : Serial USB
// 1 : Serial port 0
// 2 : USB Cable 0
///////////////////////////////////////////////////////////////////////////////
void SysexInternalDumpToStream(uint8_t dest) {

  uint16_t l;

  // Function 0B - Change USB Product String
  l = SysexInternalDumpConf(0x0B000000, 0, sysExInternalBuffer);
  if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
  else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
  else if ( l && dest == 2 ) SysExSendMsgPacket(sysExInternalBuffer,l);

  // Function 0C - Change USB Vendor ID and Product ID
  l = SysexInternalDumpConf(0x0C000000, 0, sysExInternalBuffer);
  if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
  else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
  else if ( l && dest == 2 ) SysExSendMsgPacket(sysExInternalBuffer,l);

  // Function 0E - Intellithru midi routing rules - 02 Timeout
  l = SysexInternalDumpConf(0x0E020000, 0, sysExInternalBuffer);
  if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
  else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
  else if ( l && dest == 2 ) SysExSendMsgPacket(sysExInternalBuffer,l);

  for ( uint8_t j=0; j != 16 ; j++) {
      // Function 0E - Intellithru midi routing rules - 03 Routing rules
      l = SysexInternalDumpConf(0x0E030000, j, sysExInternalBuffer);
      if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
      else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
      else if ( l && dest == 2 ) SysExSendMsgPacket(sysExInternalBuffer,l);
  }

  for ( uint8_t j=0; j != 16 ; j++) {
      // Function 0F - USB/Serial Midi midi routing rules - 02 Midi filter Cable
      l = SysexInternalDumpConf(0x0F020000, j, sysExInternalBuffer);
      if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
      else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
      else if ( l && dest == 2 ) SysExSendMsgPacket(sysExInternalBuffer,l);
  }

  for ( uint8_t j=0; j != 16 ; j++) {
      // Function 0F - USB/Serial Midi midi routing rules - 02 Midi filter Serial
      l = SysexInternalDumpConf(0x0F020100, j, sysExInternalBuffer);
      if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
      else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
      else if ( l && dest == 2 ) SysExSendMsgPacket(sysExInternalBuffer,l);
  }
  for ( uint8_t j=0; j != 16 ; j++) {
      // Function 0F - USB/Serial Midi midi routing rules - Cable to Cable
      l = SysexInternalDumpConf(0x0F010000, j, sysExInternalBuffer);
      if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
      else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
      else if ( l && dest == 2 ) SysExSendMsgPacket(sysExInternalBuffer,l);
  }

  for ( uint8_t j=0; j != 16 ; j++) {
      // Function 0F - USB/Serial Midi midi routing rules - Cable to Serial
      l = SysexInternalDumpConf(0x0F010001, j, sysExInternalBuffer);
      if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
      else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
      else if ( l && dest == 2 ) SysExSendMsgPacket(sysExInternalBuffer,l);
  }

  for ( uint8_t j=0; j != 16 ; j++) {
      // Function 0F - USB/Serial Midi midi routing rules - Serial to Cable
      l = SysexInternalDumpConf(0x0F010100, j, sysExInternalBuffer);
      if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
      else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
      else if ( l && dest == 2 ) SysExSendMsgPacket(sysExInternalBuffer,l);
   }

   for ( uint8_t j=0; j != 16 ; j++) {
      // Function 0F - USB/Serial Midi midi routing rules - Serial to Serial
      l = SysexInternalDumpConf(0x0F010101, j, sysExInternalBuffer);
      if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
      else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
      else if ( l && dest == 2 ) SysExSendMsgPacket(sysExInternalBuffer,l);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Send a SYSEX midi message to USB Cable 0
///////////////////////////////////////////////////////////////////////////////
void SysExSendMsgPacket( uint8_t buff[],uint16_t sz) {
  midiPacket_t pk { .i = 0};
  uint8_t b=0;
  bool endPk;
  // Build sysex packets
  for ( uint16_t i = 0; i != sz ; i ++ ) {
    pk.packet[++b] = buff[i];
    endPk = ( i+2 > sz );
    if (b == 3 ||  endPk ) {
        pk.packet[0]  = endPk ?  b + 4 : 4 ;
        MidiUSB.writePacket(&pk.i);
        FLASH_LED_OUT(0);
        b=0; pk.i = 0;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Process internal USBMidiKlik SYSEX
// ----------------------------------------------------------------------------
// MidiKlik SYSEX are of the following form :
//
// F0            SOX Start Of Sysex
// 77 77 78      USBMIDIKliK header
// <xx>          USBMIDIKliK sysex command
// <dddddd...dd> data
// F7            EOX End of SYSEX
//
// SOX, Header and EOX are not stored in sxBuff.
//
// Fn   Description                           Data
//
// 0x05 Sysex configuration dump
// 0x06 General Information
// 0x08 Reboot in config mode
// 0x0A Hard reset interface
// 0x0B Change USB Product string
// 0X0C Change USB VID / PID
// 0X0E Set Intelligent Midi thru mode
// 0X0F Change midi routing rule
// 0x10 Bus mode management
// 0x11 Midi transformation pipelines
// ----------------------------------------------------------------------------
// sxBuff[0] length of the message (func code + data)
// sxBuff[1] function code
// sxBuff[2] data without EOX
///////////////////////////////////////////////////////////////////////////////
void SysExInternalProcess(uint8_t source, uint8_t sxBuff[])
{
  uint8_t msgLen = sxBuff[0];
  uint8_t cmdId  = sxBuff[1];

  switch (cmdId) {

    // ---------------------------------------------------------------------------
    // Function 05 Sysex configuration dump
    // Example : F0 77 77 78 05 F7
    // ---------------------------------------------------------------------------
    case 0x05:
      if (source == FROM_USB && midiUSBCx) {
        // send to USB , cable 0
        SysexInternalDumpToStream(2);
      } else
      if (source == FROM_SERIAL ) {
        // Send to serial port 0 being the only possible for sysex
        SysexInternalDumpToStream(1);
      }
      break;

    // ---------------------------------------------------------------------------
    // Function 06 subId 0x01 - Identity request.
    // Example : F0 77 77 78 06 01 F7
    // ---------------------------------------------------------------------------
    case 0x06:
      if ( sxBuff[2] == 0x01 && msgLen == 2 ) {

          if (source == FROM_USB && midiUSBCx) {
            // send to USB , cable 0
            SysExSendMsgPacket(sysExInternalIdentityRqReply,sizeof(sysExInternalIdentityRqReply));
          } else
          if (source == FROM_SERIAL ) {
            // Send to serial port 0 being the only possible for sysex
            serialHw[0]->write(sysExInternalIdentityRqReply,sizeof(sysExInternalIdentityRqReply));
          }
      }
      break;

    // ---------------------------------------------------------------------------
    // Function 08 - Reboot in serial configuration mode
    // Example : F0 77 77 78 08 F7
    // ---------------------------------------------------------------------------
		case 0x08:
			// Set serial boot mode & Write the whole param struct
			EEPROM_Params.nextBootMode = bootModeConfigMenu;
      EEPROM_ParamsSave();
      nvic_sys_reset();
			break;

    // ---------------------------------------------------------------------------
    // Function 0A - Reboot unit
    // Example : F0 77 77 78 0A F7
    // ---------------------------------------------------------------------------
    case 0x0A:
      nvic_sys_reset();
      break;

    // ---------------------------------------------------------------------------
    // Function 0B - Change USB Product String
    // F0 77 77 78 0B <string bytes> F7
    // ---------------------------------------------------------------------------
    // Copy the received string to the USB Product String Descriptor
    // For MIDI protocol compatibility, and avoid a sysex encoding,
    // accentuated ASCII characters, below 128 are non supported.
    // Size if defined by USB_MIDI_PRODUCT_STRING_SIZE in UsbMidiKliK4x4.h
    // ---------------------------------------------------------------------------
    case 0x0B:
      if (msgLen < 2) break;
      if ( (msgLen-1) > USB_MIDI_PRODUCT_STRING_SIZE  ) break;

      // Store the new string in EEPROM
      memset(&EEPROM_Params.productString,0, sizeof(EEPROM_Params.productString));
      memcpy(&EEPROM_Params.productString,&sxBuff[2],msgLen-1);
      EEPROM_ParamsSave();
      break;

    // ---------------------------------------------------------------------------
    // Function 0C - Change USB Vendor ID and Product ID
    // F0 77 77 78 0C <n1 n2 n3 n4 = VID nibbles> <n1 n2 n3 n4 = PID nibbles> F7
    // ---------------------------------------------------------------------------
    // To respect a simple encoding of 7 bits bytes, each hex digit must be
    // transmitted separately in a serialized way.
    // The following example will set  VID to 0X8F12 and PID to 0X9067 :
    // F0 77 77 78 0C 08 0F 01 02 09 00 06 07 F7
    //                8  F  1  2  9  0  6  7
    case 0x0C:
      if ( msgLen != 9 ) break;
      EEPROM_Params.vendorID = (sxBuff[2] << 12) + (sxBuff[3] << 8) +
                                     (sxBuff[4] << 4) + sxBuff[5] ;
      EEPROM_Params.productID= (sxBuff[6] << 12) + (sxBuff[7] << 8) +
                                     (sxBuff[8] << 4) + sxBuff[9] ;
      EEPROM_ParamsSave();
      break;

    // ---------------------------------------------------------------------------
    // Function 0E - Intellithru midi routing rules
    // ---------------------------------------------------------------------------
    // IntelligentThru can be activated when USB is sleeping or unavailable beyond a certain timeout.
    // F0 77 77 78 0E  < Routing rules command <command args>   >
    //
    // Commands are :
    //  00 Reset Intellithru to default
    //  01 Disable Intellithru
    //  02 Set  Intellithru timeout
    //      arg1 : 0xnn (number of 15s periods 1-127)
    //  03 Set thru mode jack routing
    //      arg1 - Midi In Jack = 0xnn (0-F)
    //      arg2 - Midi filter mask (binary OR)
    //                => zero if you want to inactivate intelliThru for this jack
    //                  channel Voice = 0001 (1), (binary OR)
    //                  system Common = 0010 (2), (binary OR)
    //                  realTime      = 0100 (4), (binary OR)
    //                  sysEx         = 1000 (8)
    //      arg3 - Serial midi Jack out targets
    //            <t1> <t2>...<tn>  0-F 16 targets maximum.
    // EOX = F7
    // Examples :
    // F0 77 77 78 0E 00 F7                    <= Reset to default
    // F0 77 77 78 0E 01 F7                    <= Disable
    // F0 77 77 78 0E 02 02 F7                 <= Set timeout to 30s
    // F0 77 77 78 0E 03 01 0F 00 01 02 03 F7  <= Route Midi In Jack 2 to Jacks out 1,2,3,4 All msg
    // F0 77 77 78 0E 03 03 04 03 04 F7        <= Route Midi In Jack 4 to Jacks out 4,5, real time only

    case 0x0E:
			if ( msgLen < 2 ) break;

			// reset to default midi thru routing
      if (sxBuff[2] == 0x00  && msgLen == 2) {
				 ResetMidiRoutingRules(ROUTING_RESET_INTELLITHRU);
			} else

			// Disable thru mode
			if (sxBuff[2] == 0x01  && msgLen == 3) {
					ResetMidiRoutingRules(ROUTING_INTELLITHRU_OFF);
			}	else

			// Set Delay
			// The min delay is 1 period of 15 secondes. The max is 127 periods of 15 secondes.
      if (sxBuff[2] == 0x02  && msgLen == 3) {
				if ( sxBuff[3] < 1 || sxBuff[3] > 0X7F ) break;
				EEPROM_Params.intelliThruDelayPeriod = sxBuff[3];
			}	else

      // Set routing : Midin 2 mapped to all events. All 4 ports.
			// 0E 03 01 0F 00 01 02 03
			// 0E 03 01 0F 00
			if (sxBuff[2] == 3 ) {
					if (msgLen < 4 ) break;
					if ( msgLen > SERIAL_INTERFACE_COUNT + 4 ) break;

					uint8_t src = sxBuff[3];
					uint8_t filterMsk = sxBuff[4];

					if ( src >= SERIAL_INTERFACE_COUNT) break;

					// Filter is 4 bits
					if ( filterMsk > 0x0F  ) break;

					// Disable Thru mode for this jack if no filter
					if ( filterMsk == 0 && msgLen ==4 ) {
						EEPROM_Params.intelliThruJackInMsk &= ~(1 << src);
					}
					else {
							if ( filterMsk == 0 ) break;
							// Add midi in jack
							EEPROM_Params.intelliThruJackInMsk |= (1 << src);
							// Set filter
							EEPROM_Params.midiRoutingRulesIntelliThru[src].filterMsk = filterMsk;
							// Set Jacks
							if ( msgLen > 4)	{
								uint16_t msk = 0;
								for ( uint8_t i = 5 ; i != (msgLen+1)  ; i++) {
										if ( sxBuff[i] < SERIAL_INTERFACE_COUNT)
											msk |= 	1 << sxBuff[i] ;
								}
								EEPROM_Params.midiRoutingRulesIntelliThru[src].jackOutTargetsMsk = msk;
							}
					}
			}
			else	break;

			// Write the whole param struct
      EEPROM_ParamsSave();
			// reset globals for a real time update
			intelliThruDelayMillis = EEPROM_Params.intelliThruDelayPeriod * 15000;
      // Synchronize slaves routing rules if bus active and master
      if (B_IS_MASTER) I2C_SlavesRoutingSyncFromMaster();

      break;

    // ---------------------------------------------------------------------------
    // Function 0F - USB/Serial Midi midi routing rules
    // ---------------------------------------------------------------------------
    // F0 77 77 78 0F  < Routing rules command <command args>   >
    //
    // Commands are :
    //  00 Reset to default midi routing
    //  01 Set routing +
    //      arg1 - source type : <0x00 usb cable | 0x01 jack serial>
    //      arg2 - port id : id for cable or jack serial (0-F)
    //      arg3 - destination = <0x00 usb cable in | 0x01 jack serial out>
    //      arg4 - targets : <port 0 1 2...n> 16 max (0-F)
    //  02  Midi filter
    //      arg1 - source type : : <0x00 usb cable | 0x01 jack serial>
    //      arg2 - port id : id for cable or jack serial (0-F)
    //      arg3 - midi filter mask (binary OR)
    //                => zero if you want to inactivate intelliThru for this jack
    //                  channel Voice = 0001 (1), (binary OR)
    //                  system Common = 0010 (2), (binary OR)
    //                  realTime      = 0100 (4), (binary OR)
    //                  sysEx         = 1000 (8)
    // EOX = F7
      //
    // Examples :
    // F0 77 77 78 0F 00 F7                      <= reset to default midi routing
    // F0 77 77 78 0F 02 00 00 04 F7             <= Set filter to realtime events on cable 0
    // F0 77 77 78 0F 01 00 00 01 00 01 F7       <= Set Cable 0 to Jack 1,2
    // F0 77 77 78 0F 01 00 00 00 00 01 F7       <= Set Cable 0 to Cable In 0, In 01
    // F0 77 77 78 0F 01 00 00 01 00 01 F7       <= & jack 1,2 (2 msg)
    // F0 77 77 78 0F 01 01 01 01 00 01 02 03 F7 <= Set Serial jack In No 2 to serial jacks out 1,2,3,4

    case 0x0F:

      // reset to default routing

      if (sxBuff[2] == 0x00  && msgLen == 2) {
					ResetMidiRoutingRules(ROUTING_RESET_MIDIUSB);
      } else

			// Set filter

			if (sxBuff[2] == 0x02  && msgLen == 5) {

					uint8_t srcType = sxBuff[3];
					uint8_t src = sxBuff[4];
					uint8_t filterMsk = sxBuff[5];

					// Filter is 4 bits
				  if ( filterMsk > 0x0F  ) break;

					if (srcType == 0 ) { // Cable
						if ( src  >= USBCABLE_INTERFACE_MAX) break;
								EEPROM_Params.midiRoutingRulesCable[src].filterMsk = filterMsk;
					} else

					if (srcType == 1) { // Serial
						if ( src >= SERIAL_INTERFACE_COUNT) break;
							EEPROM_Params.midiRoutingRulesSerial[src].filterMsk = filterMsk;
					} else break;


			} else

			// Set Routing targets

			if (sxBuff[2] == 0x01  )
      {

				if (msgLen < 6) break;

				uint8_t srcType = sxBuff[3];
				uint8_t dstType = sxBuff[5];
				uint8_t src = sxBuff[4];

				if (srcType != 0 && srcType != 1 ) break;
				if (dstType != 0 && dstType != 1 ) break;
				if (srcType  == 0 && src >= USBCABLE_INTERFACE_MAX ) break;
				if (srcType  == 1 && src >= SERIAL_INTERFACE_COUNT) break;
				if (dstType  == 0 &&  msgLen > (USBCABLE_INTERFACE_MAX + 5) )  break;
				if (dstType  == 1 &&  msgLen > (SERIAL_INTERFACE_COUNT + 5) )  break;

				// Compute mask from the port list
				uint16_t msk = 0;
				for ( uint8_t i = 6 ; i != (msgLen+1)  ; i++) {
					  uint8_t b = sxBuff[i];
						if ( (dstType == 0 && b < USBCABLE_INTERFACE_MAX) ||
						     (dstType == 1 && b < SERIAL_INTERFACE_COUNT) ) {

									 msk |= 	1 << b ;
						}
				}// for

				// Set masks
				if ( srcType == 0 ) { // Cable
						if (dstType == 0) // To cable
							EEPROM_Params.midiRoutingRulesCable[src].cableInTargetsMsk = msk;
						else // To serial
							EEPROM_Params.midiRoutingRulesCable[src].jackOutTargetsMsk = msk;

				} else

				if ( srcType == 1 ) { // Serial
					if (dstType == 0)
						EEPROM_Params.midiRoutingRulesSerial[src].cableInTargetsMsk = msk;
					else
						EEPROM_Params.midiRoutingRulesSerial[src].jackOutTargetsMsk = msk;
				}

      } else
					return;

			// Write the whole param struct
			EEPROM_ParamsSave();

      // Synchronize slaves routing rules
      if (B_IS_MASTER) I2C_SlavesRoutingSyncFromMaster();

			break;


      // ---------------------------------------------------------------------------
      // Function 10 - Bus mode management
      // ---------------------------------------------------------------------------
      // F0 77 77 78 10  < command <command args>  >
      //
      // Commands are :
      //  00 Toggle bus mode <00 = off | 01 = ON>
      //  01 Set device id <deviceid 04-08>

      // Enable Bus mode
      case 0x10:

      // Toogle Bus mode
      if (sxBuff[2] == 0x00 && msgLen == 3 )  {

          if ( sxBuff[3] == 1  && EEPROM_Params.I2C_BusModeState == B_DISABLED)
                  EEPROM_Params.I2C_BusModeState = B_ENABLED;
          else if ( sxBuff[3] == 0 && EEPROM_Params.I2C_BusModeState == B_ENABLED )
                  EEPROM_Params.I2C_BusModeState = B_DISABLED;
          else break;

          EEPROM_ParamsSave();
          nvic_sys_reset();

      }
      else

      if (sxBuff[2] == 0x01 && msgLen == 3 )  {
          if ( sxBuff[3] > B_SLAVE_DEVICE_LAST_ADDR || sxBuff[3] < B_SLAVE_DEVICE_BASE_ADDR )
              break;

          if ( sxBuff[3] != EEPROM_Params.I2C_DeviceId ) {
            EEPROM_Params.I2C_DeviceId = sxBuff[3];
            EEPROM_ParamsSave();
            if ( EEPROM_Params.I2C_BusModeState == B_ENABLED )
                nvic_sys_reset();
          }
      }

      break;

      // ---------------------------------------------------------------------------
      // Function 11 - Midi transformation pipelines
      // ---------------------------------------------------------------------------
      // F0 77 77 78 11  < command <command args>  >
      //
      // Commands are :
      //  00 slot operation  <00 = Copy>   <source slot number 1-8> <dest slot number 1-8>
      //                     <01 = Clear>  <Slot number 1-8  0x7F = ALL SLOTS>
      //                     <02 = Attach> <port type : 0 cable | 1 jack serial | 2 Ithru> <port # 0-F> <Slot number 1-8>
      //                     <03 = Detach> <port type : 0 cable | 1 jack serial | 2 Ithru | 0x7F=ALL> [<port # 0-F>]
      //
      //  01 pipe operations <00 = Add pipe>            <slot number 1-8> <FN id> <par1> <par2> <par3> <par4>
      //                     <01 = Insert before>       <slot number 1-8> <pipe index 0-n> <FN id> <par1> <par2> <par3> <par4>
      //                     <02 = Clear Pipe by Index> <slot number 1-8> <pipe index 0-n>
      //                     <03 = Clear pipe by pipe Id>  <slot number 1-8> <pipe Id>
      //                     <04 = ByPass pipe by index>  <slot number> <pipe index 0-n> <byPass:0=no. 1=yes>

      case 0x11:

      // SLOTS OPERATIONS
      if (sxBuff[2] == 0x00 ) {

        // Copy slot
        if (sxBuff[3] == 0x00  && msgLen == 5) {
          if ( ! TransPacketPipeline_CopySlot(sxBuff[4],sxBuff[5]) )  break;
        } else

        // Clear slot <Slot number 1-8> <0x7F = ALL SLOTS>
        if (sxBuff[3] == 0x01  && msgLen == 4) {
            if ( ! TransPacketPipeline_ClearSlot(sxBuff[4]) )  break;
        } else

        // Attach port to slot
        if (sxBuff[3] == 0x02  && msgLen == 6) {
            if ( ! TransPacketPipeline_AttachPort(true,sxBuff[4],sxBuff[5],sxBuff[6]) )  break;
        } else

        // Detach port from any slot. 0x7F : detach all.
        if (sxBuff[3] == 0x03 ) {
            if ( msgLen == 4 && sxBuff[4] == 0x7F )
              if (! TransPacketPipeline_AttachPort(false,sxBuff[4],0,0 ))  break;
            else if ( msgLen == 5 )
              if (! TransPacketPipeline_AttachPort(false,sxBuff[4],sxBuff[5],0 ) )  break;
            else break;
        }
        else break;

        EEPROM_ParamsSave();

        // Synchronize slaves
        if (B_IS_MASTER) I2C_SlavesRoutingSyncFromMaster();
      }
      else

      // PIPE OPERATIONS
      if (sxBuff[2] == 0x01 ) {
        //  11 01  <00 = Add pipe>  <slot number 01-08> <FN id> <par1> <par2> <par3> <par4>
        if (sxBuff[3] == 0x00  && msgLen == 9) {
            midiTransPipe_t p;
            p.pId = sxBuff[5];
            p.byPass = 0;
            p.par1 = sxBuff[6]; p.par2 = sxBuff[7];
            p.par3 = sxBuff[8]; p.par4 = sxBuff[9];
            if ( ! TransPacketPipe_AddToSlot(sxBuff[4],&p) ) break ;
            EEPROM_ParamsSave();
            // Synchronize slaves
            if (B_IS_MASTER) I2C_SlavesRoutingSyncFromMaster();

        }
        else
        // 11 01 <01 = Insert before>  <slot number> <pipe index 0-n> <FN id> <par1> <par2> <par3> <par4>
        if (sxBuff[3] == 0x01  && msgLen == 10) {
          midiTransPipe_t p;
          p.pId = sxBuff[6];
          p.byPass = 0;
          p.par1 = sxBuff[7]; p.par2 = sxBuff[8];
          p.par3 = sxBuff[9]; p.par4 = sxBuff[10];
          if ( ! TransPacketPipe_InsertToSlot(sxBuff[4],sxBuff[5],&p) ) break ;
        }
        else
        // Clear pipe by index
        if (sxBuff[3] == 0x02  && msgLen == 5 ) {
          if ( ! TransPacketPipe_ClearSlotIndexPid(sxBuff[4],true,sxBuff[5]) ) break ;
        }
        else
        // Clear pipe first pId
        if (sxBuff[3] == 0x03  && msgLen == 5 ) {
          if ( ! TransPacketPipe_ClearSlotIndexPid(sxBuff[4],false,sxBuff[5]) ) break ;
        }
        else
        // ByPass pipe by index
        if (sxBuff[3] == 0x04  && msgLen == 6 ) {
          if ( ! TransPacketPipe_ByPass(sxBuff[4],sxBuff[5],sxBuff[6]) ) break ;
        }
        else break;

        EEPROM_ParamsSave();
        // Synchronize slaves
        if (B_IS_MASTER) I2C_SlavesRoutingSyncFromMaster();

      }

      break;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Check what is the current boot mode.
// Will never come back if config mode.
///////////////////////////////////////////////////////////////////////////////
void CheckBootMode()
{
	// Does the config menu boot mode is active ?
	// if so, prepare the next boot in MIDI mode and jump to menu
	if  ( EEPROM_Params.nextBootMode == bootModeConfigMenu ) {

      // Next boot on Midi
      EEPROM_Params.nextBootMode = bootModeMidi;
      EEPROM_ParamsSave();

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
					flashLED_CONNECT->start();delay(100);
					flashLED_CONNECT->start();delay(100);
					flashLED_CONNECT->start();delay(300);
				}
			digitalWrite(LED_CONNECT, LOW);

			ShowConfigMenu(); // INFINITE LOOP
	}
}

///////////////////////////////////////////////////////////////////////////////
// MIDI USB initiate connection if master
// + Set USB descriptor strings
///////////////////////////////////////////////////////////////////////////////
void USBMidi_Init()
{
	usb_midi_set_vid_pid(EEPROM_Params.vendorID,EEPROM_Params.productID);
	usb_midi_set_product_string((char *) &EEPROM_Params.productString);

	MidiUSB.begin() ;
  delay(4000); // Note : Usually around 4 s to fully detect USB Midi on the host
}

///////////////////////////////////////////////////////////////////////////////
// MIDI USB Loop Process
///////////////////////////////////////////////////////////////////////////////
void USBMidi_Process()
{
	// Try to connect/reconnect USB if we detect a high level on USBDM
	// This is to manage the case of a powered device without USB active or suspend mode for ex.
	if ( MidiUSB.isConnected() ) {

    if (! midiUSBCx) digitalWrite(LED_CONNECT, LOW);
    midiUSBCx = true;

		// Do we have a MIDI USB packet available ?
		if ( MidiUSB.available() ) {
			midiUSBLastPacketMillis = millis()  ;
			midiUSBIdle = false;
			intelliThruActive = false;

			// Read a Midi USB packet .
			if ( !isSerialBusy ) {
				midiPacket_t pk ;
				pk.i = MidiUSB.readPacket();
				RoutePacketToTarget( FROM_USB,  &pk );
			} else {
					isSerialBusy = false ;
			}
		} else
		if (!midiUSBIdle && millis() > ( midiUSBLastPacketMillis + intelliThruDelayMillis ) )
				midiUSBIdle = true;
	}
	// Are we physically connected to USB
	else {
       if (midiUSBCx) digitalWrite(LED_CONNECT, HIGH);
       midiUSBCx = false;
		   midiUSBIdle = true;
  }

	if ( midiUSBIdle && !intelliThruActive && EEPROM_Params.intelliThruJackInMsk) {
			intelliThruActive = true;
			FlashAllLeds(0); // All leds when Midi intellithru mode active
	}

}

///////////////////////////////////////////////////////////////////////////////
// I2C Loop Process for SERIAL MIDI
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

		// EEPROM initialization
	  // EEPROM.PageBase0 = EE_PAGE_BASE0;
	  // EEPROM.PageBase1 = EE_PAGE_BASE1;
	  // EEPROM.PageSize  = EE_PAGE_SIZE;

		//EEPROM.init();

		// Retrieve EEPROM parameters
    EEPROM_ParamsInit();

    #ifndef DEBUG_MODE
        EEPROM_Params.debugMode = false;
    #endif

    // Configure the TIMER2
    timer.pause();
    timer.setPeriod(TIMER2_RATE_MICROS); // in microseconds
    // Set up an interrupt on channel 1
    timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
    timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
    timer.attachCompare1Interrupt(Timer2Handler);
    timer.refresh();   // Refresh the timer's count, prescale, and overflow
    timer.resume();    // Start the timer counting

    // Start the LED Manager
    flashLEDManager.begin();

		CheckBootMode();

    // MIDI MODE START HERE ==================================================

    intelliThruDelayMillis = EEPROM_Params.intelliThruDelayPeriod * 15000;

    // MIDI SERIAL PORTS set Baud rates and parser inits
    // To compile with the 4 serial ports, you must use the right variant : STMF103RC
    // + Set parsers filters in the same loop.  All messages including on the fly SYSEX.

    for ( uint8_t s=0; s != SERIAL_INTERFACE_MAX ; s++ ) {
      serialHw[s]->begin(31250);
      midiSerial[s].setMidiMsgFilter( midiXparser::allMsgTypeMsk );
    }

		// I2C bus checks that could disable the bus mode
  	I2C_BusChecks();

    // Midi USB only if master when bus is enabled or master/slave
    if ( ! B_IS_SLAVE  ) {
        midiUSBLaunched = true;
        USBMidi_Init();
        #ifdef DEBUG_MODE
        if (EEPROM_Params.debugMode ) {
            DEBUG_SERIAL.end();
            DEBUG_SERIAL.begin(115200);
            DEBUG_SERIAL.flush();
            delay(500);
        }
        #endif
  	}

		I2C_BusStartWire();		// Start Wire if bus mode enabled. AFTER MIDI !

}

///////////////////////////////////////////////////////////////////////////////
// LOOP
///////////////////////////////////////////////////////////////////////////////
void loop()
{
		if ( midiUSBLaunched ) USBMidi_Process();

		SerialMidi_Process();

		// I2C BUS MIDI PACKET PROCESS
		if ( B_IS_SLAVE ) I2C_ProcessSlave();
		else if ( B_IS_MASTER ) I2C_ProcessMaster();
}
