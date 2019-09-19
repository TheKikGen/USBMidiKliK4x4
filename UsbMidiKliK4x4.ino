/*
  USB MidiKliK 4X4 - USB MIDI 4 IN X 4 OUT firmware
  Based on the MIDITECH / MIDIPLUS 4X4 harware.
  Copyright (C) 2017/2018 by The KikGen labs.

  MAIN SOURCE

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

#include <libmaple/nvic.h>
#include <EEPROM.h>

#include <PulseOutManager.h>
#include <midiXparser.h>
#include "usb_midi.h"

#include "build_number_defines.h"
#include "hardware_config.h"
#include "UsbMidiKliK4x4.h"
#include "EEPROM_Params.h"

#if SERIAL_INTERFACE_MAX > 4
#error "SERIAL_INTERFACE IS 4 MAX"
#endif

// Serial interfaces Array
HardwareSerial * serialHw[SERIAL_INTERFACE_MAX] = {SERIALS_PLIST};

// EEPROMS parameters
EEPROM_Params_t EEPROM_Params;

// Timer
HardwareTimer timer(2);

// Sysex used to set some parameters of the interface.
// Be aware that the 0x77 manufacturer id is reserved in the MIDI standard (but not used)
// The second byte is usually an id number or a func code + the midi channel (any here)
// The Third is the product id
static  const uint8_t sysExInternalHeader[] = {SYSEX_INTERNAL_HEADER} ;
static  uint8_t sysExInternalBuffer[SYSEX_INTERNAL_BUFF_SIZE] ;


// MIDI USB packet lenght
static const uint8_t CINToLenTable[] =
{
	0,
	0,
	2, // 0x02
	3, // 0x03
	3, // 0x04
	1, // 0x05
	2, // 0x06
	3, // 0x07
	3, // 0x08
	3, // 0x09
	3, // 0x0A
	3, // 0x0B
	2, // 0x0C
	2, // 0x0D
	3, // 0x0E
	1  // 0x0F
};

// Prepare LEDs pulse for Connect, MIDIN and MIDIOUT
// From MIDI SERIAL point of view
// Use a PulseOutManager factory to create the pulses
PulseOutManager flashLEDManager;

PulseOut* flashLED_CONNECT = flashLEDManager.factory(LED_CONNECT,LED_PULSE_MILLIS,LOW);

#ifdef HAS_MIDITECH_HARDWARE
  // LED must be declared in the same order as hardware serials
  #define LEDS_MIDI
  PulseOut* flashLED_IN[SERIAL_INTERFACE_MAX] = {
    flashLEDManager.factory(D4,LED_PULSE_MILLIS,LOW),
    flashLEDManager.factory(D5,LED_PULSE_MILLIS,LOW),
    flashLEDManager.factory(D6,LED_PULSE_MILLIS,LOW),
    flashLEDManager.factory(D7,LED_PULSE_MILLIS,LOW)
  };

  PulseOut* flashLED_OUT[SERIAL_INTERFACE_MAX] = {
    flashLEDManager.factory(D36,LED_PULSE_MILLIS,LOW),
    flashLEDManager.factory(D37,LED_PULSE_MILLIS,LOW),
    flashLEDManager.factory(D16,LED_PULSE_MILLIS,LOW),
    flashLEDManager.factory(D17,LED_PULSE_MILLIS,LOW)
  };
#endif

// USB Midi object
USBMidi MidiUSB;

// MIDI Parsers for serial 1 to n
midiXparser midiSerial[SERIAL_INTERFACE_MAX];

// Default midi routing
uint8_t const defaultMidiCableRoutingTarget[MIDI_ROUTING_TARGET_MAX]  = {DEFAULT_MIDI_CABLE_ROUTING_TARGET};
uint8_t const defaultMidiSerialRoutingTarget[MIDI_ROUTING_TARGET_MAX] = {DEFAULT_MIDI_SERIAL_ROUTING_TARGET};
uint8_t const defaultMidiMsgFilterRoutingTarget[MIDI_ROUTING_TARGET_MAX]  = {DEFAULT_MIDI_MSG_ROUTING_FILTER};
uint8_t const defaultIntelligentMidiThruOut[MIDI_ROUTING_TARGET_MAX]  = {DEFAULT_INTELLIGENT_MIDI_THRU_OUT};

bool					midiUSBCx      = false;
bool          midiUSBActive  = false;
unsigned long midiUSBLastPacketMillis    = 0;
unsigned long intelligentMidiThruDelayMillis = DEFAULT_INTELLIGENT_MIDI_THRU_DELAY_PERIOD * 15000;

// Intelligent midi thru mode
bool midiThruModeActive = false;

///////////////////////////////////////////////////////////////////////////////
// Timer2 interrupt handler
///////////////////////////////////////////////////////////////////////////////
void Timer2Handler(void) {
     // Update LEDS
     flashLEDManager.update(millis());
}


///////////////////////////////////////////////////////////////////////////////
// FlashAllLeds . Nb flash = nb of Serial routing point
// Mode 0 = Alls
// Mode 1 = In
// Mode 2 = Out
///////////////////////////////////////////////////////////////////////////////
void FlashAllLeds(uint8_t mode) {


	for ( uint8_t f=0 ; f< MIDI_ROUTING_TARGET_MAX ; f++ ) {
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
static void SendMidiMsgToSerial(uint8_t const *msg, uint8_t serialNo) {

  if (serialNo >= SERIAL_INTERFACE_MAX ) return;

	uint8_t msgLen = midiXparser::getMidiStatusMsgLen(msg[0]);

	if ( msgLen > 0 ) {
	  if (msgLen >= 1 ) serialHw[serialNo]->write(msg[0]);
	  if (msgLen >= 2 ) serialHw[serialNo]->write(msg[1]);
	  if (msgLen >= 3 ) serialHw[serialNo]->write(msg[2]);
	  #ifdef LEDS_MIDI
	  flashLED_OUT[serialNo]->start();
	  #else
	  flashLED_CONNECT->start();
	  #endif
	}
}

///////////////////////////////////////////////////////////////////////////////
// Send a USB midi packet to ad-hoc serial MIDI
///////////////////////////////////////////////////////////////////////////////
static void SerialWritePacket(const midiPacket_t *pk, uint8_t serialNo) {

  //if (serialNo >= SERIAL_INTERFACE_MAX ) return;
  // Sendpacket to serial at the right size
  uint8_t msgLen = CINToLenTable[pk->packet[0] & 0x0F] ;

	// if (msgLen >= 1 ) serialHw[serialNo]->write(pk->packet[1]); //  1 byte blocking transmission
	// if (msgLen >= 2 ) serialHw[serialNo]->write(pk->packet[2]);
	// if (msgLen >= 3 ) serialHw[serialNo]->write(pk->packet[3]);

	serialHw[serialNo]->write(&pk->packet[1],msgLen);

  #ifdef LEDS_MIDI
  flashLED_OUT[serialNo]->start();
  #else
  flashLED_CONNECT->start();
  #endif
}

///////////////////////////////////////////////////////////////////////////////
// Send a serial midi msg to the right USB midi cable
///////////////////////////////////////////////////////////////////////////////
static void RouteStdMidiMsg( uint8_t cable, midiXparser* xpMidi ) {

    midiPacket_t usbMidiPacket;

    uint8_t msgLen = xpMidi->getMidiMsgLen();
    uint8_t msgType = xpMidi->getMidiMsgType();

    usbMidiPacket.i = 0;
    usbMidiPacket.packet[0] = cable << 4;
    memcpy(&usbMidiPacket.packet[1],&(xpMidi->getMidiMsg()[0]),msgLen);

    // Real time single byte message CIN F->
    if ( msgType == midiXparser::realTimeMsgTypeMsk ) usbMidiPacket.packet[0]   += 0xF;
    else

    // Channel voice message CIN A-E
    if ( msgType == midiXparser::channelVoiceMsgTypeMsk )
        usbMidiPacket.packet[0]  += ( (xpMidi->getMidiMsg()[0]) >> 4);

    else

    // System common message CIN 2-3
    if ( msgType == midiXparser::systemCommonMsgTypeMsk ) {

        // 5 -  single-byte system common message (Tune request is the only case)
        if ( msgLen == 1 ) usbMidiPacket.packet[0] += 5;

        // 2/3 - two/three bytes system common message
        else usbMidiPacket.packet[0] += msgLen;
    }

    else return; // We should never be here !

    RoutePacketToTarget( FROM_SERIAL,&usbMidiPacket);
}

///////////////////////////////////////////////////////////////////////////////
// Parse sysex flows and make a packet for USB
// ----------------------------------------------------------------------------
// We use the midiXparser 'on the fly' mode, allowing to tag bytes as "captured"
// when they belong to a midi SYSEX message, without storing them in a buffer.
///////////////////////////////////////////////////////////////////////////////
static void RouteSysExMidiMsg( uint8_t cable, midiXparser* xpMidi ) {

  static midiPacket_t usbMidiSysExPacket[SERIAL_INTERFACE_MAX];
  static uint8_t packetLen[SERIAL_INTERFACE_MAX];
  static bool firstCall = true;

  byte readByte = xpMidi->getByte();

  // Initialize everything at the first call
  if (firstCall ) {
    firstCall = false;
    memset(usbMidiSysExPacket,0,sizeof(midiPacket_t)*SERIAL_INTERFACE_MAX);
    memset(packetLen,0,sizeof(uint8_t)*SERIAL_INTERFACE_MAX);
  }

  // Normal End of SysEx or : End of SysEx with error.
  // Force clean end of SYSEX as the midi usb driver
  // will not understand if we send the packet as is
  if ( xpMidi->wasSysExMode() ) {
      // Force the eox byte in case we have a SYSEX error.
      packetLen[cable]++;
      usbMidiSysExPacket[cable].packet[ packetLen[cable] ] = midiXparser::eoxStatus;
      // CIN = 5/6/7  sysex ends with one/two/three bytes,
      usbMidiSysExPacket[cable].packet[0] = (cable << 4) + (packetLen[cable] + 4) ;
      RoutePacketToTarget( FROM_SERIAL,&usbMidiSysExPacket[cable]);
      packetLen[cable] = 0;
      usbMidiSysExPacket[cable].i = 0;
			return;
  } else

  // Fill USB sysex packet
  if ( xpMidi->isSysExMode() ) {
	  packetLen[cable]++;
	  usbMidiSysExPacket[cable].packet[ packetLen[cable] ] = readByte ;

	  // Packet complete ?
	  if (packetLen[cable] == 3 ) {
	      usbMidiSysExPacket[cable].packet[0] = (cable << 4) + 4 ; // Sysex start or continue
	      RoutePacketToTarget( FROM_SERIAL,&usbMidiSysExPacket[cable]);
	      packetLen[cable] = 0;
	      usbMidiSysExPacket[cable].i = 0;
	  }
	}
}

///////////////////////////////////////////////////////////////////////////////
// PARSE INTERNAL SYSEX, either for serial or USB
// ----------------------------------------------------------------------------
// Internal sysex must be transmitted on the first cable/midi jack (whatever routing).
// Internal sysex are purely ignored on other cables or jack.
///////////////////////////////////////////////////////////////////////////////
static void ParseSysExInternal(const midiPacket_t *pk) {

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
// midi packet Router
//-----------------------------------------------------------------------------
// Route a packet from one MIDI IN / USB OUT to n MIDI OUT/USB IN
// ----------------------------------------------------------------------------
// 8 targets for the cable USB OUT and Serial MIDI IN are possible
// Routing targets tables are stored in 2 bytes / 8 bits.
// Bit 0-3 : Serial1 - Serial4
// Bit 4-7 : Cable 0 - Cable 4
///////////////////////////////////////////////////////////////////////////////
static void RoutePacketToTarget(uint8_t source, const midiPacket_t *pk) {

  uint8_t cable = pk->packet[0] >> 4;
  if (cable >= MIDI_ROUTING_TARGET_MAX ) return;

  uint8_t cin   = pk->packet[0] & 0x0F ;

	// Flash the LED IN (if available)
	#ifdef LEDS_MIDI
	flashLED_IN[cable]->start();
	#else
	flashLED_CONNECT->start();
	#endif

	// Sysex is a particular case when using packets.
	// Internal sysex Jack 1/Cable 0 ALWAYS!! are checked whatever filters are
	// This insures that the internal sysex will be always interpreted.
	// If the MCU is resetted, the msg will not be sent
	uint8_t  msgType=0;

	if (cin >= 4 && cin <= 7  ) {
		if (cable == 0) ParseSysExInternal(pk);
		msgType =  midiXparser::sysExMsgTypeMsk;
	} else {
			msgType =  midiXparser::getMidiStatusMsgTypeMsk(pk->packet[1]);
	}

	uint8_t outTargets;
	uint8_t inFilters;


	// midi Thru Mode Active ?
	// If so, take the routing rules from midi thru mode

	if ( midiThruModeActive ) {
		 outTargets = EEPROM_Params.intelligentMidiThruOut[cable] & 0x0F;
		 inFilters = EEPROM_Params.intelligentMidiThruOut[cable] >> 4;
	} else
	// Targets :
	// Bit 0-3 : Jack Serial 1 - Serial 4
	// Bit 4-7 : Cable  0 - Cable 4

	if (source == FROM_USB ) {
		 outTargets = EEPROM_Params.midiCableRoutingTarget[cable] ;
		 inFilters = EEPROM_Params.midiMsgFilterRoutingTarget[cable] >> 4;
	}
	else if (source == FROM_SERIAL ){
		 outTargets = EEPROM_Params.midiSerialRoutingTarget[cable];
		 inFilters = EEPROM_Params.midiMsgFilterRoutingTarget[cable] & 0x0F;
		 if ( !midiUSBCx  ) outTargets &= 0x0F;// Blank USB targets if not connected
	}
	else return; // Error.

	// Apply filters
	if (! (msgType & inFilters) ) return;

	uint8_t t;

	if ( outTargets & 0x0F) {
				for (t=0; t<SERIAL_INTERFACE_MAX ; t++)
					if ( (outTargets & ( 1 << t ) ) ) SerialWritePacket(pk,t);
	}


	// SERIAL JACKS with contention management
  // uint8_t jackTargets = outTargets & 0x0F;
	// if ( jackTargets ) {
	// 	uint8_t msgLen = CINToLenTable[cin] ;
	// 	do {
	// 		for (t=0; t<SERIAL_INTERFACE_MAX ; t++) {
	// 			if ( (jackTargets & ( 1 << t ) ) ){ //}&& serialHw[t]->availableForWrite()) {
	// 				serialHw[t]->write(&pk->packet[1],msgLen);
	// 				jackTargets &= ~( 1 << t );
	// 			}
	// 		}
	// 	} while (jackTargets);
	// }
	// // // We have not itme for contention management here....else we
	// // could miss somme USB packets...
	//
	// if ( outTargets & 0x0F ) {
	// 	for (t=0; t<SERIAL_INTERFACE_MAX ; t++) {
	// 		if (outTargets & ( 1 << t ) ) SerialWritePacket(pk,t);
	// 	}
	// }


	// USB Cable targets from serial or USB
	// Only if USB connected and thru mode inactive
	// Because we use MIDI_ROUTING_TARGET_MAX as a limit, it is possible
	// to route a 3 UART midi serial to 4 USB cables.
  if ( midiUSBCx && !midiThruModeActive && (outTargets & 0xF0 )  ) {
    	midiPacket_t lpk = { .i = pk->i }; ; // packet copy to change the dest cable
			outTargets = outTargets >> 4;
			for (t=0; t<=MIDI_ROUTING_TARGET_MAX ; t++) {
	      if (outTargets & ( 1 << t ) ) {
	          lpk.packet[0] = ( t << 4 ) + cin;
	          MidiUSB.writePacket(&(lpk.i));    // Send to USB
	          #ifdef LEDS_MIDI
	          flashLED_IN[t]->start();
	          #endif
	    	}
			}
	}
}

///////////////////////////////////////////////////////////////////////////////
// Reset routing rules to default factory
// 0 : All
// 1 : Midi routing
// 2 : Midi thru mode
///////////////////////////////////////////////////////////////////////////////
void SetRoutingToDefaultFactory(uint8_t mode) {

	if (mode == 0 || mode == 2) {
		EEPROM_Params.intelligentMidiThruIn = DEFAULT_INTELLIGENT_MIDI_THRU_IN ;
		EEPROM_Params.intelligentMidiThruDelayPeriod = DEFAULT_INTELLIGENT_MIDI_THRU_DELAY_PERIOD ;
		memcpy( EEPROM_Params.intelligentMidiThruOut,defaultIntelligentMidiThruOut,sizeof(defaultIntelligentMidiThruOut));
	}

	if ( mode == 0  || mode == 1) {
		// Routing targets
		memcpy( EEPROM_Params.midiCableRoutingTarget,defaultMidiCableRoutingTarget,sizeof(defaultMidiCableRoutingTarget));
		memcpy( EEPROM_Params.midiSerialRoutingTarget,defaultMidiSerialRoutingTarget,sizeof(defaultMidiSerialRoutingTarget));
		// Filters are midiXparser format.  Bits 0-3 : Serial - 4-7 : Cable
		//    noneMsgType = 0B0000, channelVoiceMsgType = 0B0001, systemCommonMsgType = 0B0010,
		//    realTimeMsgType = 0B0100, sysExMsgType = 0B1000
		memcpy( EEPROM_Params.midiMsgFilterRoutingTarget, defaultMidiMsgFilterRoutingTarget,sizeof(defaultMidiMsgFilterRoutingTarget));
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
static void ProcessSysExInternal() {

  uint8_t msgLen = sysExInternalBuffer[0];
  uint8_t cmdId  = sysExInternalBuffer[1];

  switch (cmdId) {

    // TEMP REBOOT IN CONFIG MODE
		// F0 77 77 78 08 F7
		case 0x08:
			// Set serial boot mode & Write the whole param struct
			EEPROM_Params.nextBootMode = bootModeConfigMenu;
      EEPROM_writeBlock(0, (uint8*)&EEPROM_Params, sizeof(EEPROM_Params));
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
      EEPROM_writeBlock(0, (uint8*)&EEPROM_Params, sizeof(EEPROM_Params));

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
      EEPROM_writeBlock(0, (uint8*)&EEPROM_Params,sizeof(EEPROM_Params) );

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
		// 					. Midi In Jack = < Midi In Jack # 1-4 = 0-3>
    //          . Midi Msg filter mask (can't be zero) :
    //                  channel Voice = 0001 (1),
    //                  system Common = 0010 (2),
    //                  realTime      = 0100 (4),
    //                  sysEx         = 1000 (8)
    //          . Serial midi Jack out targets Mask 4bits 1-F
		//                  Bit0 = Jack1, bit 3 = Jack 4
		// EOX = F7
		//
		// Examples :
		// F0 77 77 78 0E 00 F7    <= Reset to default
		// F0 77 77 78 0E 01 F7    <= Disable
		// F0 77 77 78 0E 02 02 F7 <= Set delay to 30s
		// F0 77 77 78 0E 03 01 0F 0F F7 <= Set Midi In Jack 2 to Jacks out 1,2,3,4 All msg
		// F0 77 77 78 0E 03 03 04 0C F7 <= Set Midi In Jack 4 to Jack 3,4, real time only

    case 0x0E:

			if ( msgLen < 2 ) break;

			// reset to default midi thru routing
      if (sysExInternalBuffer[2] == 0x00  && msgLen == 2) {
				SetRoutingToDefaultFactory(2);

			} else

			// Disable thru mode
			if (sysExInternalBuffer[2] == 0x01  && msgLen == 3) {
				EEPROM_Params.intelligentMidiThruIn = 0;
			}

			else
			// Set Delay
			// The min delay is 1 period of 15 secondes. The max is 127 periods of 15 secondes.
      if (sysExInternalBuffer[2] == 0x02  && msgLen == 3) {
				if ( sysExInternalBuffer[3] < 1 || sysExInternalBuffer[3] > 0X7F ) break;
				EEPROM_Params.intelligentMidiThruDelayPeriod = sysExInternalBuffer[3];
			}

			else
			// Set routing
			// 0E 03 01 0F 0F F7 <= Set Midi In Jack 2 to Jacks out 1,2,3,4 All msg
			if (msgLen == 5 && sysExInternalBuffer[2] == 3 ) {

					if ( sysExInternalBuffer[3] > SERIAL_INTERFACE_MAX) break;
					if ( sysExInternalBuffer[4] > 0x0F || sysExInternalBuffer[5]> 0x0F ) break;

					// Disable Thru mode for this jack if no filter or not jack out
					if ( sysExInternalBuffer[4] == 0 || sysExInternalBuffer[5] == 0) {
						EEPROM_Params.intelligentMidiThruIn &= ~(1 << sysExInternalBuffer[3]);
					} else {
						EEPROM_Params.intelligentMidiThruIn |= (1 << sysExInternalBuffer[3]);
					}

					// Set filter and jacks out
					EEPROM_Params.intelligentMidiThruOut[sysExInternalBuffer[3]] =
							sysExInternalBuffer[5] + (sysExInternalBuffer[4] << 4);
			}
			else	break;

			// Write the whole param struct
      EEPROM_writeBlock(0, (uint8*)&EEPROM_Params,sizeof(EEPROM_Params) );

			// reset globals for a real time update
			intelligentMidiThruDelayMillis = EEPROM_Params.intelligentMidiThruDelayPeriod * 15000;
      // midiUSBActive = true;
      // midiUSBLastPacketMillis = millis()  ;

      break;

    // SET ROUTING TARGETS ----------------------------------------------------
    // To configure the routing for an input, you must set some bits of the target byte to 1 :
    // Bits 0-3 are corresponding respectively to Serial Midi out Jack targets 1-4
    // Bits 4-7 are corresponding respectively to USB Cables targets IN 0-3.
    // Sysex message structure :
		//
		// Header       = F0 77 77 78
		// Function     = 0F
		// Action       =
		//  00 Reset to default midi routing
		//  01 Set routing +
    //          . source type     = <cable=0X0 | serial=0x1>
		//          . id              = id for cable or serial 0-3
    //          . Midi Msg filter mask
		//          . routing targets = <cable mask> , <jack serial mask>
		// EOX = F7
    //
		// Filter is defined as a midiXparser message type mask.
		// noneMsgTypeMsk          = 0B0000,
		// channelVoiceMsgTypeMsk  = 0B0001,
		// systemCommonMsgTypeMsk  = 0B0010,
		// realTimeMsgTypeMsk      = 0B0100,
		// sysExMsgTypeMsk         = 0B1000,
		// allMsgTypeMsk           = 0B1111
		//
    // Examples :
		// F0 77 77 78 0F 00 F7                 <= reset to default midi routing
    // F0 77 77 78 0F 01 00 00 0F 00 03 F7 <= Set Cable 0 to Jack 1,2, all midi msg
		// F0 77 77 78 0F 01 00 00 0F 01 03 F7 <= Set Cable 0 to Cable In 0, Jack 1,2, all midi msg
		// F0 77 77 78 0F 01 01 01 04 00 0F F7 <= Set Serial jack In 2 to all serial jack out, realtime msg only
		// F0 77 77 78 0F 01 01 00 01 03 03 F7 <= Set Serial jack In 1 to 1,2 serial jack out,cable in 0,1, channel voice msg only

    case 0x0F:
      // reset to default routing
      if (sysExInternalBuffer[2] == 0x00  && msgLen == 2) {
					SetRoutingToDefaultFactory(1);
      } else
      // Set targets
      if (sysExInternalBuffer[2] == 0x01 && msgLen == 7 )
      {

					if ( sysExInternalBuffer[4]>= MIDI_ROUTING_TARGET_MAX) break;
          if ( sysExInternalBuffer[5] > 0xF || sysExInternalBuffer[6] > 0xF || sysExInternalBuffer[7] > 0xF)  break;

					uint8_t source      = sysExInternalBuffer[4];
					uint8_t filtersMsk  = sysExInternalBuffer[5];
					uint8_t ruleMsk     = (sysExInternalBuffer[6] << 4) + sysExInternalBuffer[7];
					uint8_t* filterRouting = &EEPROM_Params.midiMsgFilterRoutingTarget[source];


          // Filter masks are stored in one byte :
					// bits 0:3 serial filter .  bits 4:7 usb cable filter
					
					// Cable
          if (sysExInternalBuffer[3] == 0x00 ) {
						*filterRouting = (*filterRouting & 0x0F ) | ( filtersMsk << 4);
            EEPROM_Params.midiCableRoutingTarget[source] = ruleMsk;
          } else
          // Serial
          if (sysExInternalBuffer[3] == 0x01 ) {
						 *filterRouting = (*filterRouting & 0xF0 ) | filtersMsk ;
						 EEPROM_Params.midiSerialRoutingTarget[source] = ruleMsk ;
          }

      } else
					return;

			// Write the whole param struct
			EEPROM_writeBlock(0, (uint8*)&EEPROM_Params, sizeof(EEPROM_Params));

			// reset globals for a real time update
			// midiUSBActive = true;
			// midiUSBLastPacketMillis = millis()  ;

			break;

  }

}

///////////////////////////////////////////////////////////////////////////////
// CHECK EEPROM
//----------------------------------------------------------------------------
// Retrieve global parameters from EEPROM, or Initalize it
// If factorySetting is true, all settings will be forced to factory default
//////////////////////////////////////////////////////////////////////////////
void CheckEEPROM(bool factorySettings=false) {

  // Set EEPROM parameters for the STMF103RC
  EEPROM.PageBase0 = 0x801F000;
  EEPROM.PageBase1 = 0x801F800;
  EEPROM.PageSize  = 0x800;


  // Read the EEPROM parameters structure
  EEPROM_readBlock(0, (uint8_t *)&EEPROM_Params, sizeof(EEPROM_Params) );

  // If the signature is not found, of not the same version of parameters structure,
  // or new build, then initialize (factory settings)
  if (  factorySettings ||
        memcmp( EEPROM_Params.signature,EE_SIGNATURE,sizeof(EEPROM_Params.signature) ) ||
        EEPROM_Params.prmVer != EE_PRMVER ||
        memcmp( EEPROM_Params.TimestampedVersion,TimestampedVersion,sizeof(EEPROM_Params.TimestampedVersion) )
     )
  {
    memset( &EEPROM_Params,0,sizeof(EEPROM_Params) );
    memcpy( EEPROM_Params.signature,EE_SIGNATURE,sizeof(EEPROM_Params.signature) );

    EEPROM_Params.prmVer = EE_PRMVER;

    memcpy( EEPROM_Params.TimestampedVersion,TimestampedVersion,sizeof(EEPROM_Params.TimestampedVersion) );

    // Default boot mode when new firmware uploaded
    EEPROM_Params.nextBootMode = bootModeConfigMenu;

		SetRoutingToDefaultFactory(0); // All

    EEPROM_Params.vendorID  = USB_MIDI_VENDORID;
    EEPROM_Params.productID = USB_MIDI_PRODUCTID;

    memcpy(EEPROM_Params.productString,USB_MIDI_PRODUCT_STRING,sizeof(USB_MIDI_PRODUCT_STRING));

    //Write the whole param struct
    EEPROM_writeBlock(0, (uint8*)&EEPROM_Params,sizeof(EEPROM_Params) );

  }

}
///////////////////////////////////////////////////////////////////////////////
// EEPROM EMULATION UTILITIES
///////////////////////////////////////////////////////////////////////////////
int EEPROM_writeBlock(uint16_t ee, const uint8_t *bloc, uint16_t size )
{
    uint16_t i;
    for (i = 0; i < size; i++) EEPROM.write(ee+i, *(bloc+i));

    return i;
}

int EEPROM_readBlock(uint16_t ee, uint8_t *bloc, uint16_t size )
{
    uint16_t i;
    for (i = 0; i < size; i++) *(bloc +i) = EEPROM.read(ee+i);
    return i;
}
///////////////////////////////////////////////////////////////////////////////
// Get an Int8 From a hex char.  letters are minus.
// Warning : No control of the size or hex validity!!
///////////////////////////////////////////////////////////////////////////////
static uint8_t GetInt8FromHexChar(char c) {
	return (uint8_t) ( c <= '9' ? c - '0' : c - 'a' + 0xa );
}

///////////////////////////////////////////////////////////////////////////////
// Get an Int16 From a 4 Char hex array.  letters are minus.
// Warning : No control of the size or hex validity!!
///////////////////////////////////////////////////////////////////////////////
static uint16_t GetInt16FromHex4Char(char * buff) {
	char val[4];

	val[0] = GetInt8FromHexChar(buff[0]);
	val[1] = GetInt8FromHexChar(buff[1]);
	val[2] = GetInt8FromHexChar(buff[2]);
	val[3] = GetInt8FromHexChar(buff[3]);

	return GetInt16FromHex4Bin(val);
}

///////////////////////////////////////////////////////////////////////////////
// Get an Int16 From a 4 hex digit binary array.
// Warning : No control of the size or hex validity!!
///////////////////////////////////////////////////////////////////////////////
static uint16_t GetInt16FromHex4Bin(char * buff) {
	return (uint16_t) ( (buff[0] << 12) + (buff[1] << 8) + (buff[2] << 4) + buff[3] );
}

///////////////////////////////////////////////////////////////////////////////
// USB serial getdigit
///////////////////////////////////////////////////////////////////////////////
static char USBSerialGetDigit() {
  char c;
  while ( ( c = USBSerialGetChar() ) <'0' && c > '9');
  return c;
}
///////////////////////////////////////////////////////////////////////////////
// USB serial getchar
///////////////////////////////////////////////////////////////////////////////
static char USBSerialGetChar() {
  while (!Serial.available() >0);
  char c = Serial.read();
  // Flush
  while (Serial.available()>0) Serial.read();
  return c;
}

///////////////////////////////////////////////////////////////////////////////
// "Scanf like" for hexadecimal inputs
///////////////////////////////////////////////////////////////////////////////

static uint8_t USBSerialScanHexChar(char *buff, uint8_t len,char exitchar,char sepa) {

	uint8_t i = 0, c = 0;

	while ( i < len ) {
		c = USBSerialGetChar();
		if ( (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'  ) ) {
			Serial.write(c);
			if (sepa) Serial.write(sepa);
			buff[i++]	= GetInt8FromHexChar(c);
		} else if (c == exitchar && exitchar !=0 ) break;
	}
	return i;
}

///////////////////////////////////////////////////////////////////////////////
// Get a choice from a question
///////////////////////////////////////////////////////////////////////////////
char getChoice(const char * qLabel, char * choices) {
	char c;
	char * yn = "yn";
	char * ch;

	if ( *choices == 0 ) ch = yn; else ch = choices;
	Serial.print(qLabel);
	Serial.print(" (");
	Serial.print(ch);
	Serial.print(") ? ");

	while (1) {
		c = USBSerialGetChar();
		uint8_t i=0;
		while (ch[i] )
				if ( c == ch[i++] ) { Serial.write(c); return c; }
	}
}

///////////////////////////////////////////////////////////////////////////////
// Show the midi routing map
///////////////////////////////////////////////////////////////////////////////
void ShowMidiRouting(uint8_t rt) {
		uint8_t bFilter=0;
		uint8_t *routing;
		uint8_t *filters;
		uint8_t i,j;

		Serial.println("|-------------------------------------------|");
		Serial.print("| ");
		Serial.print(rt ? "Jack " : "Cable");
		Serial.print("| Msg Filter   |");
		Serial.print(rt == 2 ? "          " : " Cable IN ") ;
		Serial.println("| Jack OUT |");
		Serial.print("| ");
		Serial.print( rt ? "IN # ":"OUT# ");
		Serial.print("| Ch Sc Rt Sx  |");
		Serial.print( rt ==2 ?" (No USB) " :" 1 2 3 4  ");
		Serial.println("| 1 2 3 4  |");
		Serial.println("|------+--------------+----------+----------|");

		// Cable
		if (rt == 0) {
		   bFilter = 4;
			 routing = EEPROM_Params.midiCableRoutingTarget;
			 filters = EEPROM_Params.midiMsgFilterRoutingTarget;
		} else

		// Jacks
		if (rt == 1) {
			 bFilter = 0;
			 routing = EEPROM_Params.midiSerialRoutingTarget;
			 filters = EEPROM_Params.midiMsgFilterRoutingTarget;
		} else

		// Thru mode
		if (rt == 2 ){
			bFilter = 4;
			routing = EEPROM_Params.intelligentMidiThruOut;
			filters = routing;
		} else return;


		for ( i=0; i< MIDI_ROUTING_TARGET_MAX ; i++) {

			Serial.print("|  ");
			if (rt == 2 && (EEPROM_Params.intelligentMidiThruIn & (1 << i)) == 0  )
				 Serial.print(".");
			else
			   Serial.print(i+1);
			Serial.print("-> |  ");

			// Filters
			for ( j=bFilter; j < bFilter+4 ; j++) {
			   if ( ( filters[i] & ( 1 << j) )  )
			     Serial.print("X  ");
			   else Serial.print(".  ");
			 }
			Serial.print("| ");

			// Cable In (but Midi Thru mode)
			if (rt !=2) {
				for ( j=4; j < 8 ; j++) {
							if ( routing[i] & ( 1 << j) )
								Serial.print("X ");
							else Serial.print(". ");
				}
			}
			else Serial.print("        ");
			Serial.print(" | ");

			// Jack Out
			for ( j=0; j < 4 ; j++) {
				if ( routing[i] & ( 1 << j) )
					Serial.print("X ");
				else Serial.print(". ");
			}
			Serial.println(" |");
		}

}

///////////////////////////////////////////////////////////////////////////////
// Show current EEPROM settings
///////////////////////////////////////////////////////////////////////////////
static void ShowCurrentSettings() {
	uint8_t i,j;

  Serial.println("-===========================================-");
	Serial.println("                CURRENT SETTINGS");
  Serial.println("---------------------------------------------");
	Serial.print("Magic number   : ");
	Serial.write(EEPROM_Params.signature , sizeof(EEPROM_Params.signature));
	Serial.print( EEPROM_Params.prmVer);
	Serial.print("-")	;
	Serial.println( (char *)EEPROM_Params.TimestampedVersion);
	Serial.print("Next BootMode  : ");
	Serial.println(EEPROM_Params.nextBootMode);
	Serial.print("Vendor Id      : ");
	Serial.println( EEPROM_Params.vendorID,HEX);
	Serial.print("Product Id     : ");
  Serial.println( EEPROM_Params.productID,HEX);
	Serial.print("Product string : ");
	Serial.write(EEPROM_Params.productString, sizeof(EEPROM_Params.productString));
	Serial.println("");
	Serial.print("Sysex header   : ");
	for (i=0; i < sizeof(sysExInternalHeader); i++) {
			Serial.print(sysExInternalHeader[i],HEX);Serial.print(" ");
	}

	Serial.println("");
  Serial.println("-===========================================-");
	Serial.println("|                MIDI ROUTING               |");

	ShowMidiRouting(0);  // Cable
	ShowMidiRouting(1);  // Jack

	 Serial.println("|-------------------------------------------|");
	 Serial.print("|      Intelligent Thru mode (");
	 Serial.print(EEPROM_Params.intelligentMidiThruIn ? " active " : "inactive");
	 Serial.println(")     |");

	 ShowMidiRouting(2);  // Thru Mode

   Serial.println("-===========================================-");
   Serial.print("  Intelligent Midi Thru USB timeout : ");
   Serial.print(EEPROM_Params.intelligentMidiThruDelayPeriod*15);Serial.println("s");
   Serial.println("-===========================================-");
   Serial.println("");
}

///////////////////////////////////////////////////////////////////////////////
// Setup a Midi routing target on screen
///////////////////////////////////////////////////////////////////////////////
uint8_t ConfigMidiRootTargets(bool isJackIn,bool isJackOut,uint8_t jackIn) {
	uint8_t targets = 0;
	for ( uint8_t i=0 ; i< MIDI_ROUTING_TARGET_MAX ; i++ ){
		Serial.print("Route ");
		Serial.print(isJackIn? "Jack IN#":"Cable OUT#");
		Serial.print(jackIn+1);
		Serial.print(" to ");
		Serial.print(isJackOut ?"Jack OUT#":"Cable IN#");
		Serial.print(i+1);
		if ( getChoice("","") == 'y') targets |= (1 << i);;
		Serial.println();
	}
	return targets;
}
///////////////////////////////////////////////////////////////////////////////
// Setup a Midi filter routing  on screen
///////////////////////////////////////////////////////////////////////////////
uint8_t ConfigMidiFilter(bool isJackIn,uint8_t jackIn) {
	Serial.print("Set filter Midi messages for ");
	Serial.print(isJackIn? "Jack IN#":"Cable OUT#");
	Serial.print(jackIn+1); Serial.println(" :");
	uint8_t flt = 0;
  Serial.println("");
	if ( getChoice("Channel Voice   ","") == 'y')
			flt |= midiXparser::channelVoiceMsgTypeMsk;
	Serial.println("");
	if ( getChoice("System Common   ","") == 'y')
			flt |= midiXparser::systemCommonMsgTypeMsk;
	Serial.println("");
	if ( getChoice("Realtime        ","") == 'y' )
			flt |= midiXparser::realTimeMsgTypeMsk;
	Serial.println("");
	if ( getChoice("System Exclusive","") == 'y')
			flt |= midiXparser::sysExMsgTypeMsk;
	Serial.println("");
	return flt;
}

///////////////////////////////////////////////////////////////////////////////
// INFINITE LOOP - CONFIG ROOT MENU
//----------------------------------------------------------------------------
// Allow USBMidLKLIK configuration by using a menu as a user interface
// from the USB serial port.
///////////////////////////////////////////////////////////////////////////////
void ConfigRootMenu()
{
	char choice=0;
	char buff [32];
	uint8_t i,j;
	char c;
  boolean showMenu = true;
	for ( ;; )
	{
		if (showMenu) {
  		Serial.println("");
  		Serial.println("");
  		Serial.print("USBMIDIKliK 4x4 MENU");
  		Serial.print(" - ");Serial.println(HARDWARE_TYPE);
  		Serial.println("(c)TheKikGen Labs");
      Serial.println("");

  		Serial.println("0.Show current settings              e.Reload settings from EEPROM");
  		Serial.println("1.Midi USB Cable OUT routing         f.Restore all factory settings");
  		Serial.println("2.Midi IN Jack routing               r.Reset routing to factory default");
  		Serial.println("3.Intelligent Thru IN Jack routing   s.Save & quit");
  		Serial.println("4.Intelligent Thru USB timeout       x.Abort");
  		Serial.println("5.USB Vendor ID & Product ID");
  		Serial.println("6.USB product string");
		}
    showMenu = true;
		Serial.println("");
		Serial.print("=>");
		choice = USBSerialGetChar();
		Serial.println(choice);
    Serial.println("");

		switch (choice)
		{

			// Show current settings
			case '0':
				ShowCurrentSettings();
        showMenu = false;
			break;

			// Cables midi routing
			case '1':
				Serial.println("-- Set USB cable Out routing --");
				Serial.print("Enter Cable OUT # (1-4 / 0 to exit) :");
				while ( (choice = USBSerialGetDigit() - '0' ) > MIDI_ROUTING_TARGET_MAX ) ;
				Serial.println((uint8_t)choice);
				if (choice > 0) {
					uint8_t cableOut =  choice - 1;

					// Filters
					uint8_t flt = ConfigMidiFilter(false,cableOut);
					if ( flt == 0 ) {
						Serial.println("No filters set. No change made.");
						break;
					}
					Serial.println("");

					// Cable
					uint8_t cTargets = ConfigMidiRootTargets(false,false,cableOut);
					Serial.println("");

					// Midi jacks
					uint8_t jTargets = ConfigMidiRootTargets(false,true,cableOut);

					if ( jTargets + cTargets == 0 ) {
						Serial.println("No targets set. No change made.");
						break;
					}
					EEPROM_Params.midiCableRoutingTarget[cableOut] = jTargets | (cTargets<<4);
					flt = (EEPROM_Params.midiMsgFilterRoutingTarget[cableOut] & 0x0F) | flt<<4;
					EEPROM_Params.midiMsgFilterRoutingTarget[cableOut] = flt;
				}

				break;

			// Midi IN jack routing
			case '2':
				Serial.println("-- Set Jack IN routing --");
				Serial.print("Enter Jack IN # (1-4 / 0 to exit) :");
				while ( (choice = USBSerialGetDigit() - '0' ) > MIDI_ROUTING_TARGET_MAX ) ;
				Serial.println((uint8_t)choice);
				if (choice > 0) {
					uint8_t jackIn =  choice - 1;

					// Filters
					uint8_t flt = ConfigMidiFilter(true,jackIn);
					if ( flt == 0 ) {
						Serial.println("No filters set. No change made.");
						break;
					}
					Serial.println("");

					// Cable
					uint8_t cTargets = ConfigMidiRootTargets(true,false,jackIn);
					Serial.println("");

					// Midi jacks
					uint8_t jTargets = ConfigMidiRootTargets(true,true,jackIn);
					if ( jTargets + cTargets == 0 ) {
						Serial.println("No targets set. No change made.");
						break;
					}

					EEPROM_Params.midiSerialRoutingTarget[jackIn] = jTargets | (cTargets<<4);
					flt = (EEPROM_Params.midiMsgFilterRoutingTarget[jackIn] & 0xF0) + flt;
					EEPROM_Params.midiMsgFilterRoutingTarget[jackIn] = flt;
				}

				break;

				// Intelligent MIDI Thru settings
				case '3':
					Serial.println("-- Set intelligent Midi Thru mode --");
					Serial.print("Enter Jack IN # (1-4 / 0 to exit) :");
					while ( (choice = USBSerialGetDigit() - '0' ) > MIDI_ROUTING_TARGET_MAX ) ;
					Serial.println((uint8_t)choice);
					if (choice > 0) {
						uint8_t jackIn =  (uint8_t)choice - 1;
						Serial.print("Jack #"); Serial.print(jackIn+1);
						if ( getChoice(" : enable thru mode routing","") != 'y') {
							Serial.println("");
							Serial.print("Jack IN #");Serial.print(jackIn+1);
							Serial.println(" disabled.");
							EEPROM_Params.intelligentMidiThruIn &= ~(1 << jackIn);
							break;
						}
						Serial.println();
						EEPROM_Params.intelligentMidiThruIn |= (1 << jackIn);
						if ( (EEPROM_Params.intelligentMidiThruOut[jackIn] & 0xF)  &&
						EEPROM_Params.intelligentMidiThruOut[jackIn] & 0xF0 )
						{
							if ( getChoice("Keep existing Midi routing & filtering","") == 'y')
							break;
						}

						Serial.println();
						// Filters
						uint8_t flt = ConfigMidiFilter(true,jackIn);
						if ( flt == 0 ) {
							Serial.print("No filters set - Jack IN #");Serial.print(jackIn+1);Serial.println(" disabled.");
							EEPROM_Params.intelligentMidiThruIn &= ~(1 << jackIn);
							break;
						}
						flt = flt<<4;

						// Jacks
						Serial.println("");
						uint8_t targets = ConfigMidiRootTargets(true,true,jackIn);
						if ( targets == 0 ) {
							Serial.print("No Jack(s) OUT set. Jack IN #");Serial.print(jackIn+1);Serial.println(" disabled.");
							EEPROM_Params.intelligentMidiThruIn &= ~(1 << jackIn);
							break;
						}
						EEPROM_Params.intelligentMidiThruOut[jackIn] = flt + targets;
					}

					break;

			// USB Timeout <number of 15s periods 1-127>
			case '4':
				Serial.println("Enter the number of 15s delay periods for USB timeout (001-127 / 000 to exit) :");
				i = 0; j = 0;
				while ( i < 3 ) {
					c  = USBSerialGetDigit();Serial.write(c);
					j +=  ( (uint8_t) (c - '0' ) ) * pow(10 ,2-i);
					i++;
				}
				if (j == 0 or j >127 )
				Serial.println(". No change made. Incorrect value.");
				else {
					EEPROM_Params.intelligentMidiThruDelayPeriod = j;
					Serial.print(" <= Delay set to ");Serial.print(j*15);Serial.println("s");
				}
				break;

			// Change VID & PID
			case '5':
        Serial.println("Enter VID - PID, in hex (0-9,a-f) :");
				USBSerialScanHexChar( (char *) buff, 4, 0, 0);
				EEPROM_Params.vendorID  = GetInt16FromHex4Bin((char*)buff);
				Serial.print("-");
				USBSerialScanHexChar( (char * ) buff, 4, 0, 0);
				EEPROM_Params.productID = GetInt16FromHex4Bin((char*)buff);
				break;

			// Change the product string
			case '6':
				Serial.println("Enter product string - ENTER to terminate :");
				i = 0;
				while ( i < USB_MIDI_PRODUCT_STRING_SIZE && (c = USBSerialGetChar() ) !=13 ) {
					if ( c >= 32 && c < 127 ) {
						Serial.write(c);
						buff[i++]	= c;
					}
				}
				if ( i > 0 ) {
					memset(EEPROM_Params.productString,0,sizeof(EEPROM_Params.productString) );
					memcpy(EEPROM_Params.productString,buff,i);
				}
				break;

			// Reload the EEPROM parameters structure
			case 'e':
				if ( getChoice("Reload current settings from EEPROM","") == 'y' ) {
					CheckEEPROM();
          Serial.println("");
					Serial.println("Settings reloaded from EEPROM.");
				}
				break;

			// Restore factory settings
			case 'f':
				if ( getChoice("Restore all factory settings","") == 'y' ) {
          Serial.println("");
					if ( getChoice("Your own settings will be erased. Are you really sure","") == 'y' ) {
						CheckEEPROM(true);
            Serial.println("");
						Serial.println("Factory settings restored.");
					}
				}
				break;

			// Default midi routing
			case 'r':
				if ( getChoice("Reset all Midi and thru mode routing to default","") == 'y')
				SetRoutingToDefaultFactory(0); // All
				break;

			// Save & quit
			case 's':
				ShowCurrentSettings();
				if ( getChoice("Save settings and exit to midi mode","") == 'y' ) {
					//Write the whole param struct
					EEPROM_writeBlock(0, (uint8*)&EEPROM_Params,sizeof(EEPROM_Params) );
					delay(100);
					nvic_sys_reset();
				}
				break;

			// Abort
			case 'x':
				if ( getChoice("Abort","") == 'y' ) nvic_sys_reset();
				break;
		}

	}
}

///////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////
void setup() {

    // Retrieve EEPROM parameters
    CheckEEPROM();

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

    // Does the config menu boot mode is active ?
    // if so, prepare the next boot in MIDI mode and jump to menu
    if  ( EEPROM_Params.nextBootMode == bootModeConfigMenu ) {
        EEPROM_Params.nextBootMode = bootModeMidi;
        EEPROM_writeBlock(0, (uint8*)&EEPROM_Params,sizeof(EEPROM_Params) );

        #ifdef HAS_MIDITECH_HARDWARE
          // Assert DISC PIN (PA8 usually for Miditech) to enable USB
          gpio_set_mode(PIN_MAP[PA8].gpio_device, PIN_MAP[PA8].gpio_bit, GPIO_OUTPUT_PP);
          gpio_write_bit(PIN_MAP[PA8].gpio_device, PIN_MAP[PA8].gpio_bit, 1);
        #endif

        // start USB serial
        Serial.begin();

        // wait for a serial monitor to be connected.
        // 3 short flash

        while (!Serial) {
            flashLED_CONNECT->start();delay(100);
            flashLED_CONNECT->start();delay(100);
            flashLED_CONNECT->start();delay(300);
          }
        digitalWrite(LED_CONNECT, LOW);
        ConfigRootMenu(); // INFINITE LOOP
    }

    // MIDI MODE START HERE ==================================================
    intelligentMidiThruDelayMillis = EEPROM_Params.intelligentMidiThruDelayPeriod * 15000;

    // Set USB descriptor strings
    usb_midi_set_vid_pid(EEPROM_Params.vendorID,EEPROM_Params.productID);
    usb_midi_set_product_string((char *) &EEPROM_Params.productString);

    // MIDI SERIAL PORTS set Baud rates
    // To compile with the 4 serial ports, you must use the right variant : STMF103RC
    // + Set parsers filters in the same loop.  All messages including on the fly SYSEX.

    for ( uint8_t s=0; s < SERIAL_INTERFACE_MAX ; s++ ) {
      serialHw[s]->begin(31250);
      midiSerial[s].setMidiMsgFilter( midiXparser::allMsgTypeMsk );
    }

    // MIDI USB initiate connection

    MidiUSB.begin() ;
    delay(500);
    digitalWrite(LED_CONNECT,MidiUSB.isConnected() ?  LOW : HIGH);
		midiUSBActive = true;
}

///////////////////////////////////////////////////////////////////////////////
// LOOP
///////////////////////////////////////////////////////////////////////////////
void loop() {

		static bool isSerialBusy = false ;

		// Try to connect/reconnect USB if we detect a high level on USBDM
    // This is to manage the case of a powered device without USB active or suspend mode for ex.

    if ( midiUSBCx = MidiUSB.isConnected() ) {

			// Do we have a MIDI USB packet available ?
	    if ( MidiUSB.available() ) {
	      midiUSBLastPacketMillis = millis()  ;
				midiUSBActive = true;

				// Do we come back from midiThru mode ?
				midiThruModeActive = false;

				// Read a Midi USB packet .
        if ( !isSerialBusy ) {
					uint32_t pk = MidiUSB.readPacket();
        	// Route Packet to the appropriate cable and serial out
        	RoutePacketToTarget( FROM_USB,  (const midiPacket_t *) &pk );
				} else {
						isSerialBusy = false ;
				}

	    } else
			if (midiUSBActive && millis() > ( midiUSBLastPacketMillis + intelligentMidiThruDelayMillis ) )
					midiUSBActive = false;

		}
		// Are we physically connected to USB
		else {

			 // USB devices are detected by the host because they have a pull-up resistor
			 // on one of the data lines
			 // - D- for a low-speed device, and D+ for a full speed device (or a high speed device; the high speed is discovered in the signalling).

			 // // Assert PA11 (USBDM) to check USB hardware connection
       // // A small lag on midi serial can surround if the connection is set
       // if (  gpio_read_bit(PIN_MAP[PIN_USBDM].gpio_device,PIN_MAP[PIN_USBDM].gpio_bit) ) {
       //    flashLED_CONNECT->start();
       //    MidiUSB.end() ;
       //    delay(200);
       //    flashLED_CONNECT->start();
       //    MidiUSB.begin() ;
       //    delay(500);
       // }
			 midiUSBActive = false;
    }

    // SET CONNECT LED STATUS. We use gpio instead digitalWrite to be really fast, in that case....
    // We lost the Arduino compatibility here...
    // We do that only when dedicated LEDs exist for MIDI (i.e. not the Blue Pill)...
    #ifdef LEDS_MIDI
    gpio_write_bit(PIN_MAP[LED_CONNECT].gpio_device,PIN_MAP[LED_CONNECT].gpio_bit, midiUSBCx ? 0 : 1 );
    #endif

		if ( !midiUSBActive && !midiThruModeActive && EEPROM_Params.intelligentMidiThruIn) {
				midiThruModeActive = true;
				#ifdef LEDS_MIDI
				FlashAllLeds(0); // All leds when Midi thru mode active
				#endif
		}

		// SERIAL MIDI PROCESS

		for ( uint8_t s = 0; s< SERIAL_INTERFACE_MAX ; s++ )
		{
		    // Do we have any MIDI msg on Serial 1 to n ?
		    if ( serialHw[s]->available() ) {
		       if ( midiSerial[s].parse( serialHw[s]->read() ) ) {
						 		// We manage sysEx "on the fly". Clean end of a sysexe msg ?
						 		if ( midiSerial[s].getMidiMsgType() == midiXparser::sysExMsgTypeMsk )
									RouteSysExMidiMsg(s, &midiSerial[s]) ;

								// Not a sysex. The message is complete.
								else
		            	RouteStdMidiMsg( s, &midiSerial[s]);
		       }
		       else
					 // Acknowledge any sysex error
					 if ( midiSerial[s].isSysExError() )
					 	 RouteSysExMidiMsg(s, &midiSerial[s]) ;
					 else
					 // Check if a SYSEX mode active and send bytes on the fly.
		       if ( midiSerial[s].isSysExMode() && midiSerial[s].isByteCaptured() ) {
						 	RouteSysExMidiMsg(s, &midiSerial[s]) ;
					 }
		    }

				// Manage Serial contention vs USB
				// When one or more of the serial buffer is full, we block USB read one round.
				// This implies to use non blocking Serial.write(buff,len).
				if (  midiUSBCx &&  !serialHw[s]->availableForWrite() ) isSerialBusy = true; // 1 round without reading USB
	  }

}
