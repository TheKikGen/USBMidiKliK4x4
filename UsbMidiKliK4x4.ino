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

#include "build_number_defines.h"
#include <string.h>
#include <libmaple/nvic.h>
#include <EEPROM.h>
#include <Wire_slave.h>
#include <PulseOutManager.h>
#include <midiXparser.h>
#include "UsbMidiKliK4x4.h"
#include "usb_midi.h"
#include "RingBuffer.h"


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

// Timer used to signal I2C events every 300 ms
PulseOut I2C_LedTimer(0xFF,500);

// USB Midi object & globals
USBMidi MidiUSB;
volatile bool					midiUSBCx      = false;
volatile bool         midiUSBIdle    = false;
bool          midiUSBLaunched= false;
bool 					isSerialBusy   = false ;
unsigned long midiUSBLastPacketMillis    = 0;
// MIDI Parsers for serial 1 to n
midiXparser midiSerial[SERIAL_INTERFACE_MAX];

// Sysex used to set some parameters of the interface.
// Be aware that the 0x77 manufacturer id is reserved in the MIDI standard (but not used)
// The second byte is usually an id number or a func code + the midi channel (any here)
// The Third is the product id
static  const uint8_t sysExInternalHeader[] = {SYSEX_INTERNAL_HEADER} ;
static  uint8_t sysExInternalBuffer[SYSEX_INTERNAL_BUFF_SIZE] ;

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


///////////////////////////////////////////////////////////////////////////////
//  CORE FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

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
	for ( uint8_t f=0 ; f< 4 ; f++ ) {
		#ifdef LEDS_MIDI
			for ( uint8_t i=0 ; i< SERIAL_INTERFACE_MAX ; i++ ) {
					if ( mode == 0 || mode ==1 ) flashLED_IN[i]->start();
					if ( mode == 0 || mode ==2 ) 	flashLED_OUT[i]->start();
			}
		#else
      mode = 0; // Avoid unused prm warning
			flashLED_CONNECT->start();
		#endif

			delay(100);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Send a midi msg to serial MIDI. 0 is Serial1.
///////////////////////////////////////////////////////////////////////////////
void SerialMidi_SendMsg(uint8_t const *msg, uint8_t serialNo)
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
void SerialMidi_SendPacket(const midiPacket_t *pk, uint8_t serialNo)
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
 void ParseSysExInternal(const midiPacket_t *pk)
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
			ProcessSysExInternal();
		}
}

///////////////////////////////////////////////////////////////////////////////
// THE MIDI PACKET ROUTER
//-----------------------------------------------------------------------------
// Route a packet from a midi IN jack / USB OUT to
// a midi OUT jacks / USB IN  or I2C remote serial midi on another device
///////////////////////////////////////////////////////////////////////////////
 void RoutePacketToTarget(uint8_t source,  const midiPacket_t *pk)
{
  // NB : we use the same routine to route USB and serial/ I2C .
	// The Cable can be the serial port # if coming from local serial
  uint8_t sourcePort  = pk->packet[0] >> 4;

	// Check at the physical level (i.e. not the bus)
  if ( source == FROM_USB && sourcePort >= USBCABLE_INTERFACE_MAX ) return;
	if ( source == FROM_SERIAL ) {
    if ( sourcePort >= SERIAL_INTERFACE_MAX ) return;
    // If bus mode active, the local port# must be translated according
		// to the device Id, before routing
    if (EEPROM_Params.I2C_BusModeState == B_ENABLED ) {
			sourcePort = GET_BUS_SERIALNO_FROM_LOCALDEV(EEPROM_Params.I2C_DeviceId,sourcePort);
    }
  }

  uint8_t cin   = pk->packet[0] & 0x0F ;

	FLASH_LED_IN(sourcePort);

	// Sysex is a particular case when using packets.
	// Internal sysex Jack 1/Cable 0 ALWAYS!! are checked whatever filters are
	// This insures that the internal sysex will be always interpreted.
	// If the MCU is resetted, the msg will not be sent
	uint8_t  msgType=0;

	if (cin >= 4 && cin <= 7  ) {
		if (sourcePort == 0) ParseSysExInternal(pk);
		msgType =  midiXparser::sysExMsgTypeMsk;
	} else {
			msgType =  midiXparser::getMidiStatusMsgTypeMsk(pk->packet[1]);
	}


	// ROUTING tables
	uint16_t *cableInTargets ;
	uint16_t *serialOutTargets ;
	uint8_t *inFilters ;

  // Save intelliThruActive and USBCx state as it could be changed in an interrupt
  // (when slave)
  boolean ithru = intelliThruActive;

  if (source == FROM_SERIAL ){
    // IntelliThru active ? If so, take the good routing rules
    if ( ithru ) {
			if ( ! EEPROM_Params.intelliThruJackInMsk ) return; // Double check.
      serialOutTargets = &EEPROM_Params.midiRoutingRulesIntelliThru[sourcePort].jackOutTargetsMsk;
      inFilters = &EEPROM_Params.midiRoutingRulesIntelliThru[sourcePort].filterMsk;
    }
    else {
      cableInTargets = &EEPROM_Params.midiRoutingRulesSerial[sourcePort].cableInTargetsMsk;
      serialOutTargets = &EEPROM_Params.midiRoutingRulesSerial[sourcePort].jackOutTargetsMsk;
      inFilters = &EEPROM_Params.midiRoutingRulesSerial[sourcePort].filterMsk;
    }
  }
  else if (source == FROM_USB ) {
      cableInTargets = &EEPROM_Params.midiRoutingRulesCable[sourcePort].cableInTargetsMsk;
      serialOutTargets = &EEPROM_Params.midiRoutingRulesCable[sourcePort].jackOutTargetsMsk;
      inFilters = &EEPROM_Params.midiRoutingRulesCable[sourcePort].filterMsk;
  }

  else return; // Error.

	// Apply midi filters
	if (! (msgType & *inFilters) ) return;

  // ROUTING FROM ANY SOURCE PORT TO SERIAL TARGETS //////////////////////////
	// A target match ?
  if ( *serialOutTargets) {
				for (	uint16_t t=0; t<SERIAL_INTERFACE_COUNT ; t++)
					if ( (*serialOutTargets & ( 1 << t ) ) ) {
								// Route via the bus
								if (EEPROM_Params.I2C_BusModeState == B_ENABLED ) {
                     I2C_BusSerialSendMidiPacket(pk, t);
								}
								// Route to local serial if bus mode disabled
								else SerialMidi_SendPacket(pk,t);
					}
	} // serialOutTargets

  // Stop here if IntelliThru active (no USB active but maybe connected)
  // Intellithru is always activated by the master in bus mode!.

  if ( ithru ) return;

  // Stop here if no USB connection (owned by the master).
  // If we are a slave, the master should have notified us
  if ( ! midiUSBCx ) return;

	// Apply cable routing rules from serial or USB
	// Only if USB connected and thru mode inactive

  if (  *cableInTargets  ) {
    	midiPacket_t pk2 = { .i = pk->i }; // packet copy to change the dest cable
			for (uint8_t t=0; t < USBCABLE_INTERFACE_MAX ; t++) {
	      if ( *cableInTargets & ( 1 << t ) ) {
	          pk2.packet[0] = ( t << 4 ) + cin;
            // Only the master has USB midi privilege in bus MODE
            // Everybody else if an usb connection is active
            if (! B_IS_SLAVE ) {
                MidiUSB.writePacket(&(pk2.i));
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

	if (mode == ROUTING_RESET_ALL || mode == ROUTING_RESET_MIDIUSB) {

	  for ( uint8_t i = 0 ; i < USBCABLE_INTERFACE_MAX ; i++ ) {

			// Cables
	    EEPROM_Params.midiRoutingRulesCable[i].filterMsk = midiXparser::allMsgTypeMsk;
	    EEPROM_Params.midiRoutingRulesCable[i].cableInTargetsMsk = 0 ;
	    EEPROM_Params.midiRoutingRulesCable[i].jackOutTargetsMsk = 1 << i ;
		}

		for ( uint8_t i = 0 ; i < B_SERIAL_INTERFACE_MAX ; i++ ) {

			// Jack serial
	    EEPROM_Params.midiRoutingRulesSerial[i].filterMsk = midiXparser::allMsgTypeMsk;
	    EEPROM_Params.midiRoutingRulesSerial[i].cableInTargetsMsk = 1 << i ;
	    EEPROM_Params.midiRoutingRulesSerial[i].jackOutTargetsMsk = 0  ;
	  }
	}

	if (mode == ROUTING_RESET_ALL || mode == ROUTING_RESET_INTELLITHRU) {
	  // "Intelligent thru" serial mode
	  for ( uint8_t i = 0 ; i < B_SERIAL_INTERFACE_MAX ; i++ ) {
	    EEPROM_Params.midiRoutingRulesIntelliThru[i].filterMsk = midiXparser::allMsgTypeMsk;
	    EEPROM_Params.midiRoutingRulesIntelliThru[i].jackOutTargetsMsk = 0B1111 ;
		}
		EEPROM_Params.intelliThruJackInMsk = 0;
	  EEPROM_Params.intelliThruDelayPeriod = DEFAULT_INTELLIGENT_MIDI_THRU_DELAY_PERIOD ;
	}

	// Disable "Intelligent thru" serial mode
	if (mode == ROUTING_INTELLITHRU_OFF ) {
		for ( uint8_t i = 0 ; i < B_SERIAL_INTERFACE_MAX ; i++ ) {
			EEPROM_Params.intelliThruJackInMsk = 0;
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
// SOX, Header and EOX are not stored in sysExInternalBuffer.
//
// Cmd Description                           Data
//
// 0x08 Reboot in config mode
// 0x0A Hard reset interface
// 0x0B Change USB Product string
// 0X0C Change USB VID / PID
// 0X0E Set Intelligent Midi thru mode
// 0X0F Change input routing rule
// ----------------------------------------------------------------------------
// sysExInternalBuffer[0] length of the message (func code + data)
// sysExInternalBuffer[1] function code
// sysExInternalBuffer[2] data without EOX
///////////////////////////////////////////////////////////////////////////////
void ProcessSysExInternal()
{

  uint8_t msgLen = sysExInternalBuffer[0];
  uint8_t cmdId  = sysExInternalBuffer[1];

  switch (cmdId) {

    // TEMP REBOOT IN CONFIG MODE
		// F0 77 77 78 08 F7
		case 0x08:
			// Set serial boot mode & Write the whole param struct
			EEPROM_Params.nextBootMode = bootModeConfigMenu;
      EEPROM_ParamsSave();
      nvic_sys_reset();
			break;

    // RESET USB MIDI INTERFACE -----------------------------------------------
    // F0 77 77 78 0A F7
    case 0x0A:
      nvic_sys_reset();
      break;

    // CHANGE MIDI PRODUCT STRING ---------------------------------------------
    // F0 77 77 78 0B <character array> F7
    case 0x0B:
      // Copy the receive message to the Product String Descriptor
      // For MIDI protocol compatibility, and avoid a sysex encoding,
      // Accentuated ASCII characters, below 128 non supported.

      if (msgLen < 2) break;

      if ( (msgLen-1) > USB_MIDI_PRODUCT_STRING_SIZE  ) {
          // Error : Product name too long
          break;
      }

      // Store the new sting in EEPROM bloc
      memset(&EEPROM_Params.productString,0, sizeof(EEPROM_Params.productString));
      memcpy(&EEPROM_Params.productString,&sysExInternalBuffer[2],msgLen-1);

      // Write the whole param struct
      EEPROM_ParamsSave();

      break;

    // VENDOR ID & PRODUCT ID -------------------------------------------------
    // F0 77 77 78 0C <n1 n2 n3 n4 = Vendor Id nibbles> <n1 n2 n3 n4 = Product Id nibbles> F7
    case 0x0C:
      // As MIDI data are 7 bits bytes, we must use a special encoding, to encode 8 bits values,
      // as light as possible. As we have here only 2 x 16 bits values to handle,
      // the encoding will consists in sending each nibble (4 bits) serialized in bytes.
      // For example sending VendorID and ProductID 0X8F12 0X9067 will be encoded as :
      //   0x08 0XF 0x1 0x2  0X9 0X0 0X6 0X7,  so the complete SYSEX message will be :
      // F0 77 77 78 0C 08 0F 01 02 09 00 06 07 F7

      if ( msgLen != 9 ) break;
      EEPROM_Params.vendorID = (sysExInternalBuffer[2] << 12) + (sysExInternalBuffer[3] << 8) +
                                     (sysExInternalBuffer[4] << 4) + sysExInternalBuffer[5] ;
      EEPROM_Params.productID= (sysExInternalBuffer[6] << 12) + (sysExInternalBuffer[7] << 8) +
                                     (sysExInternalBuffer[8] << 4) + sysExInternalBuffer[9] ;
      EEPROM_ParamsSave();

      break;

		// Intelligent MIDI THRU. -------------------------------------------------
		// When USB midi is not active beyond a defined timout, the MIDI THRU mode can be activated.
		// After that timout, every events from the MIDI INPUT Jack #n will be routed to outputs jacks 1-4,
		// accordingly with the midi thru mode serial jack targets mask.
		//
    // Header       = F0 77 77 78
		// Function     = 0E
		//
		// Action       =
		//  00 Reset to default
		//  01 Disable
		//  02 Set Delay <number of 15s periods 1-127>
		//  03 Set thu mode jack routing +
		//          . Midi In Jack = < Midi In Jack # = 0-F>
		//          . Midi Msg filter mask
		//                  zero if you want to inactivate intelliThru for this jack
		//                  channel Voice = 0001 (1),
		//                  system Common = 0010 (2),
		//                  realTime      = 0100 (4),
		//                  sysEx         = 1000 (8)
		//          . Serial midi Jack out targets 1 < Midi Out Jack # 1-n = 0-n>
		//          . Serial midi Jack out targets 2
		//          . Serial midi Jack out targets 3
		//                     ......
		//          . Serial midi Jack out targets n <= 16
		//
		// EOX = F7
		//
		// Examples :
		// F0 77 77 78 0E 00 F7    <= Reset to default
		// F0 77 77 78 0E 01 F7    <= Disable
		// F0 77 77 78 0E 02 02 F7 <= Set delay to 30s
		// F0 77 77 78 0E 03 01 0F 00 01 02 03 F7<= Set Midi In Jack 2 to Jacks out 1,2,3,4 All msg
		// F0 77 77 78 0E 03 03 0C 03 04 F7 <= Set Midi In Jack 4 to Jack 3,4, real time only

    case 0x0E:

			if ( msgLen < 2 ) break;

			// reset to default midi thru routing
      if (sysExInternalBuffer[2] == 0x00  && msgLen == 2) {
				 ResetMidiRoutingRules(ROUTING_RESET_INTELLITHRU);
			} else

			// Disable thru mode
			if (sysExInternalBuffer[2] == 0x01  && msgLen == 3) {
					ResetMidiRoutingRules(ROUTING_INTELLITHRU_OFF);
			}

			else
			// Set Delay
			// The min delay is 1 period of 15 secondes. The max is 127 periods of 15 secondes.
      if (sysExInternalBuffer[2] == 0x02  && msgLen == 3) {
				if ( sysExInternalBuffer[3] < 1 || sysExInternalBuffer[3] > 0X7F ) break;
				EEPROM_Params.intelliThruDelayPeriod = sysExInternalBuffer[3];
			}

			else
			// Set routing : Midin 2 mapped to all events. All 4 ports.
			// 0E 03 01 0F 00 01 02 03
			// 0E 03 01 0F 00

			if (sysExInternalBuffer[2] == 3 ) {

					if (msgLen < 4 ) break;
					if ( msgLen > SERIAL_INTERFACE_COUNT + 4 ) break;

					uint8_t src = sysExInternalBuffer[3];
					uint8_t filterMsk = sysExInternalBuffer[4];

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
								for ( uint8_t i = 5 ; i< (msgLen+1)  ; i++) {
										if ( sysExInternalBuffer[i] < SERIAL_INTERFACE_COUNT)
											msk |= 	1 << sysExInternalBuffer[i] ;
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

      // Synchronize slaves routing rules
      if (B_IS_MASTER) I2C_SlavesRoutingSyncFromMaster();

      break;

    // SET ROUTING TARGETS ----------------------------------------------------
		// Header       = F0 77 77 78
		// Function     = 0F
		// Action       =
		//  00 Reset to default midi routing
		//  01 Set routing +
    //          . source type     = <cable=0X0 | serial=0x1>
		//          . id              = id for cable or serial 0-F
		//          . destination type = <cable=0X0 | serial=0x1>
		//          . routing targets = <cable 0 1 2 n> or <jack 0 1 2 n> - 0-F
		//  02 Set filter msk :
    //          . source type     = <cable=0X0 | serial=0x1>
		//          . id              = id for cable or serial 0-F
		//          . midi filter mask
		// EOX = F7
		//
		// Midi filter mask is defined as a midiXparser message type mask.
		// noneMsgTypeMsk          = 0B0000,
		// channelVoiceMsgTypeMsk  = 0B0001,
		// systemCommonMsgTypeMsk  = 0B0010,
		// realTimeMsgTypeMsk      = 0B0100,
		// sysExMsgTypeMsk         = 0B1000,
		// allMsgTypeMsk           = 0B1111
		//
    // Examples :
		// F0 77 77 78 0F 00 F7          <= reset to default midi routing
		// F0 77 77 78 0F 02 00 00 04 F7 <= Set filter to realtime events on cable 0
    // F0 77 77 78 0F 01 00 00 01 00 01 F7 <= Set Cable 0 to Jack 1,2
		// F0 77 77 78 0F 01 00 00 00 00 01 F7 <= Set Cable 0 to Cable In 0, In 01
		// F0 77 77 78 0F 01 00 00 01 00 01 F7 <= & jack 1,2 (2 msg)
		// F0 77 77 78 0F 01 01 01 01 00 01 02 03 F7 <= Set Serial jack In No 2 to 4 serial jack out

    case 0x0F:

      // reset to default routing

      if (sysExInternalBuffer[2] == 0x00  && msgLen == 2) {
					ResetMidiRoutingRules(ROUTING_RESET_MIDIUSB);
      } else

			// Set filter

			if (sysExInternalBuffer[2] == 0x02  && msgLen == 5) {

					uint8_t srcType = sysExInternalBuffer[3];
					uint8_t src = sysExInternalBuffer[4];
					uint8_t filterMsk = sysExInternalBuffer[5];

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

			if (sysExInternalBuffer[2] == 0x01  )
      {

				if (msgLen < 6) break;

				uint8_t srcType = sysExInternalBuffer[3];
				uint8_t dstType = sysExInternalBuffer[5];
				uint8_t src = sysExInternalBuffer[4];

				if (srcType != 0 && srcType != 1 ) break;
				if (dstType != 0 && dstType != 1 ) break;
				if (srcType  == 0 && src >= USBCABLE_INTERFACE_MAX ) break;
				if (srcType  == 1 && src >= SERIAL_INTERFACE_COUNT) break;
				if (dstType  == 0 &&  msgLen > (USBCABLE_INTERFACE_MAX + 5) )  break;
				if (dstType  == 1 &&  msgLen > (SERIAL_INTERFACE_COUNT + 5) )  break;

				// Compute mask from the port list
				uint16_t msk = 0;
				for ( uint8_t i = 6 ; i< (msgLen+1)  ; i++) {
					  uint8_t b = sysExInternalBuffer[i];
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
		// Set EEPROM parameters for the STMF103RC
	  EEPROM.PageBase0 = 0x801F000;
	  EEPROM.PageBase1 = 0x801F800;
	  EEPROM.PageSize  = 0x800;
		//EEPROM.Pages     = 1;


		EEPROM.init();

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

    for ( uint8_t s=0; s < SERIAL_INTERFACE_MAX ; s++ ) {
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
