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

#include <PulseOutManager.h>
#include <midiXparser.h>
#include "usb_midi.h"
#include <libmaple/nvic.h>
#include <EEPROM.h>
#include "EEPROM_Params.h"
#include "build_number_defines.h"
#include "UsbMidiKliK4x4.h"

// Serial interfaces Array
HardwareSerial * serialInterface[SERIAL_INTERFACE_MAX] = {SERIALS_PLIST};

// EEPROMS parameters
EEPROM_Params_t EEPROM_Params;

// Timer
HardwareTimer timer(2);

// Sysex used to set some parameters of the interface.
// Be aware that the 0x77 manufacturer id is reserved in the MIDI standard (but not used)
// The second byte is usually an id number or a func code + the midi channel (any here)
// The Third is the product id
static  uint8_t sysExInternalHeader[] = {SYSEX_INTERNAL_HEADER} ;
static  uint8_t sysExInternalBuffer[SYSEX_INTERNAL_BUFF_SIZE] ;

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

// When MIDI SERIAL is inactive beyond the timeout..
bool midiSerialActive = false;
unsigned long midiSerialLastPacketMillis = 0;

// MIDI Parsers for serial 1 to n
midiXparser serialMidiParser[SERIAL_INTERFACE_MAX];

// Default midi routing
uint8_t defaultMidiCableRoutingTarget[MIDI_ROUTING_TARGET_MAX]  = {DEFAULT_MIDI_CABLE_ROUTING_TARGET};
uint8_t defaultMidiSerialRoutingTarget[MIDI_ROUTING_TARGET_MAX] = {DEFAULT_MIDI_SERIAL_ROUTING_TARGET};

bool          midiUSBActive  = false;
unsigned long midiUSBLastPacketMillis    = 0;
unsigned long intelligentMidiThruDelayMillis = DEFAULT_INTELLIGENT_MIDI_THRU_DELAY_PERIOD * 15000;

// Functions prototypes
void Timer2Handler(void);
static void SendMidiMsgToSerial(uint8_t const *, uint8_t);
static void SerialWritePacket(const midiPacket_t *, uint8_t);
static void SendMidiSerialMsgToUsb( uint8_t, midiXparser* ) ;
static void PrepareSysExPacket( uint8_t , midiXparser*  ) ;
static void ParseSysExInternal(const midiPacket_t *) ;
static void RoutePacketToTarget(uint8_t, const midiPacket_t *) ;
static void ProcessSysExInternal() ;
void CheckEEPROM();
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

///////////////////////////////////////////////////////////////////////////////
// Timer2 interrupt handler
///////////////////////////////////////////////////////////////////////////////
void Timer2Handler(void) {
     // Update LEDS
     flashLEDManager.update(millis());
}

///////////////////////////////////////////////////////////////////////////////
// Send a midi msg to serial MIDI. 0 is Serial1.
///////////////////////////////////////////////////////////////////////////////
static void SendMidiMsgToSerial(uint8_t const *msg, uint8_t serialNo) {

  if (serialNo >= SERIAL_INTERFACE_MAX ) return;
  serialInterface[serialNo]->write(msg[0]);
  serialInterface[serialNo]->write(msg[1]);
  serialInterface[serialNo]->write(msg[2]);
  #ifdef LEDS_MIDI
  flashLED_OUT[serialNo]->start();
  #else
  flashLED_CONNECT->start();
  #endif
}

///////////////////////////////////////////////////////////////////////////////
// Send a USB midi packet to ad-hoc serial MIDI
///////////////////////////////////////////////////////////////////////////////
static void SerialWritePacket(const midiPacket_t *pk, uint8_t serialNo) {

  uint8_t cin   = pk->packet[0] & 0x0F ;
  if (serialNo >= SERIAL_INTERFACE_MAX ) return;

  // Sendpacket to serial at the right size
  switch (cin) {
          // 1 byte
          case 0x05: case 0x0F:
            serialInterface[serialNo]->write(pk->packet[1]);

            #ifdef LEDS_MIDI
            flashLED_OUT[serialNo]->start();
            #else
            flashLED_CONNECT->start();
            #endif

            break;

          // 2 bytes
          case 0x02: case 0x06: case 0x0C: case 0x0D:
            serialInterface[serialNo]->write(pk->packet[1]);
            serialInterface[serialNo]->write(pk->packet[2]);
            #ifdef LEDS_MIDI
            flashLED_OUT[serialNo]->start();
            #else
            flashLED_CONNECT->start();
            #endif

            break;

          // 3 bytes
          case 0x03: case 0x07: case 0x04: case 0x08:
          case 0x09: case 0x0A: case 0x0B: case 0x0E:
            serialInterface[serialNo]->write(pk->packet[1]);
            serialInterface[serialNo]->write(pk->packet[2]);
            serialInterface[serialNo]->write(pk->packet[3]);
            #ifdef LEDS_MIDI
            flashLED_OUT[serialNo]->start();
            #else
            flashLED_CONNECT->start();
            #endif

            break;

          default :
            return;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Send a serial midi msg to the right USB midi cable
///////////////////////////////////////////////////////////////////////////////
static void SendMidiSerialMsgToUsb( uint8_t cable, midiXparser* serialMidiParser ) {

    midiPacket_t usbMidiPacket;

    uint8_t msgLen = serialMidiParser->getMidiMsgLen();
    uint8_t msgType = serialMidiParser->getMidiMsgType();

    usbMidiPacket.i = 0;
    usbMidiPacket.packet[0] = cable << 4;
    memcpy(&usbMidiPacket.packet[1],&(serialMidiParser->getMidiMsg()[0]),msgLen);

    // Real time single byte message CIN F->
    if ( msgType == midiXparser::realTimeMsgType ) usbMidiPacket.packet[0]   += 0xF;
    else

    // Channel voice message CIN A-E
    if ( msgType == midiXparser::channelVoiceMsgType )
        usbMidiPacket.packet[0]  += ( (serialMidiParser->getMidiMsg()[0]) >> 4);

    else

    // System common message CIN 2-3
    if ( msgType == midiXparser::systemCommonMsgType ) {

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
// SYSEX Error (not correctly terminated by 0xF7 for example) are cleaned up,
// to restore a correct parsing state.
///////////////////////////////////////////////////////////////////////////////
static void PrepareSysExPacket( uint8_t cable, midiXparser* serialMidiParser ) {

  static midiPacket_t usbMidiSysExPacket[SERIAL_INTERFACE_MAX];
  static uint8_t packetLen[SERIAL_INTERFACE_MAX];
  static bool firstCall = true;

  byte readByte = serialMidiParser->getByte();

  // Initialize everything at the first call
  if (firstCall ) {
    firstCall = false;
    memset(usbMidiSysExPacket,0,sizeof(midiPacket_t)*SERIAL_INTERFACE_MAX);
    memset(packetLen,0,sizeof(uint8_t)*SERIAL_INTERFACE_MAX);
  }

  // Normal End of SysEx or : End of SysEx with error.
  // Force clean end of SYSEX as the midi usb driver
  // will not understand if we send the packet as is
  if ( readByte == midiXparser::eoxStatus || serialMidiParser->isSysExError() ) {
      // Force the eox byte in case we have a SYSEX error.
      packetLen[cable]++;
      usbMidiSysExPacket[cable].packet[ packetLen[cable] ] = midiXparser::eoxStatus;
      // CIN = 5/6/7  sysex ends with one/two/three bytes,
      usbMidiSysExPacket[cable].packet[0] = (cable << 4) + (packetLen[cable] + 4) ;
      RoutePacketToTarget( FROM_SERIAL,&usbMidiSysExPacket[cable]);
      packetLen[cable] = 0;
      usbMidiSysExPacket[cable].i = 0;
  }

  // Stop if not in sysexmode anymore here !
  // The SYSEX error could be caused by another SOX, or Midi status...,
  if ( ! serialMidiParser->isSysExMode() ) return;

  // Fill USB sysex packet
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
// 8 targets by midi input are possible (cable USB IN or Serial MIDI IN)
// Routing targets are stored in 2 tables of 1 byte/ 8 bits.
// When followings bits are set to 1, the midi message will be routed from
// the corresponding Serial IN / USB cable IN to :
// Bit 0-3 : Serial1 - Serial4
// Bit 4-7 : Cable 0 - Cable 4
//
// Example MIDI-MERGE :
// To make SERIAL IN 1 and 2 to be merged to CABLE IN 1,
// AND MIDI OUT 3, You must configure the serial table as following :
//
// midiSerialRoutingTarget[0] = B00010100;
// midiSerialRoutingTarget[1] = B00010100;
//
// Example MIDI-SPLITTER :
// To make USB MIDI OUT 1 to be split to SERIAL IN 1/2 and 3 :
//
// midiCableRoutingTarget[0] = B00000111;
//
// The default configuration will route cables 1-4 to Serial 1-4, as
// the standard configuration of a MIDIPLUS/MIDITECH interface.
///////////////////////////////////////////////////////////////////////////////
static void RoutePacketToTarget(uint8_t source, const midiPacket_t *pk) {

  uint8_t cable = pk->packet[0] >> 4;
  if (cable >= SERIAL_INTERFACE_MAX ) return;

  uint8_t cin   = pk->packet[0] & 0x0F ;

  uint8_t targets =
    ( source == FROM_SERIAL ?  EEPROM_Params.midiSerialRoutingTarget[cable]:
                   EEPROM_Params.midiCableRoutingTarget[cable]);

  // Flash the LED IN (if available)
  #ifdef LEDS_MIDI
  flashLED_IN[cable]->start();
  #else
  flashLED_CONNECT->start();
  #endif

  // Find targets
  // Bit 0-3 : Serial 1 - Serial 4
  // Bit 4-7 : Cable  0 - Cable 4

  // Serial targets
  for (uint8_t t=0; t<=3 ; t++) {
      if (targets & ( 1 << t ) ) SerialWritePacket(pk,t );
  }

  // usb cable targets
  // only if USB connected
  if ( MidiUSB.isConnected()) {
    targets = targets >> 4;
    midiPacket_t lpk = { .i = pk->i }; ; // Copy to change the cable
    for (uint8_t t=0; t<=3 ; t++) {
        if (targets & ( 1 << t ) ) {
          lpk.packet[0] = ( t << 4 ) + cin;
          MidiUSB.writePacket(&(lpk.i));    // Send to USB
          #ifdef LEDS_MIDI
          flashLED_IN[t]->start();
          #endif
        }
    }
  }
  // Internal sysex
  if (cin >= 4 && cin <= 7 && cable == 0 ) {
     ParseSysExInternal(pk);
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

      if ( (msgLen-1) > MIDI_PRODUCT_STRING_SIZE  ) {
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
    // F0 77 77 78 0E
    // < (High nible  = Midi In Jack # 1-4 or 0 to disable>) (low nible = midiMsg filter mask)>
    //
    // Midi Msg filter mask (can't be zero) :
    //    channel Voice = 0001 (1),
    //    system Common = 0010 (2),
    //    realTime      = 0100 (4),
    //    sysEx         = 1000 (8)
    //
    // <Serial targets Mask 4bits 1-F>
    // <number of 15s periods 1-127>
    // F7
    case 0x0E:

      // When USB midi is not active beyond the defined timout, the MIDI THRU mode can be activated.
      // Serial targets is a 4 bits value where bit 0-3 map serial Midi out jacks  1-4 and can't be zero.
      // The min delay is 1 period of 15 secondes. The max is 127 periods of 15 secondes.
      // After that delay, Every events from the MIDI INPUT Jack #n will be routed to outputs jacks 1-4,
      // accordingly with the serial targets mask.
      // For example, to set the MIDI IN 3 jack to be the input, realtime msg only, 4 outputs, 2 mn delay (8 periods) :
      // F0 77 77 78 0E 34 0F 08 F7

      if ( msgLen != 4 ) break;
      if ( (sysExInternalBuffer[2] >> 4 ) > SERIAL_INTERFACE_MAX) break;
      if ( (sysExInternalBuffer[2] & 0x0F)  == 0 ) break;
      if ( (sysExInternalBuffer[2] & 0x0F)  > 8  ) break;
      if ( sysExInternalBuffer[3] > 0xF || sysExInternalBuffer[3] == 0 ) break;
      if ( sysExInternalBuffer[4] == 0  || sysExInternalBuffer[4] > 127 ) break;

      EEPROM_Params.intelligentMidiThruIn = sysExInternalBuffer[2];
      EEPROM_Params.intelligentMidiThruOut = sysExInternalBuffer[3];
      EEPROM_Params.intelligentMidiThruDelayPeriod = sysExInternalBuffer[4];

      // reset globals for a real time update
      intelligentMidiThruDelayMillis = EEPROM_Params.intelligentMidiThruDelayPeriod * 15000;
      midiUSBActive = true;
      midiUSBLastPacketMillis = millis()  ;

      // Write the whole param struct
      EEPROM_writeBlock(0, (uint8*)&EEPROM_Params,sizeof(EEPROM_Params) );

      break;


    // SET ROUTING TARGETS ----------------------------------------------------
    // To configure the routing for an input, you must set some bits of the target byte to 1 :
    // Bits 0-3 are corresponding respectively to Serial Midi out Jack targets 1-4
    // Bits 4-7 are corresponding respectively to USB Cables targets IN 0-3.
    // Sysex message structure :
    //
    // F0 77 77 78 <0x0F> <0x01 = set> <cable=0X0 | serial=0x1> <id:0-3> <target nibble cable> <target nibble serial> F7
    //
    // F0 77 77 78 <0x0F> <0x00 = default> F7
    //
    // For example, the following routing rule set MIDI IN JACK1/JACK2 to be merged to cable 0 only :
    //
    //   F0 77 77 78 0F 01 01 00 01 00 F7
    //   F0 77 77 78 0F 01 01 01 01 00 F7
    //
    // The following sysex will restore default routing for all inputs : F0 77 77 78 0F 00 F7

    case 0x0F:

      // reset to default routing
      if (sysExInternalBuffer[2] == 0x00 ) {
           memcpy(&EEPROM_Params.midiCableRoutingTarget,&defaultMidiCableRoutingTarget,sizeof(defaultMidiCableRoutingTarget));
           memcpy(&EEPROM_Params.midiSerialRoutingTarget,&defaultMidiSerialRoutingTarget,sizeof(defaultMidiSerialRoutingTarget));

           // Write the whole param struct
           EEPROM_writeBlock(0, (uint8*)&EEPROM_Params,sizeof(EEPROM_Params) );

      } else

      // Set targets
      if (sysExInternalBuffer[2] == 0x01 && msgLen == 6 )
      {

          if ( sysExInternalBuffer[4]> MIDI_ROUTING_TARGET_MAX) break;
          if ( sysExInternalBuffer[5] > 0xF || sysExInternalBuffer[6] > 0xF)  break;

          // Cable
          if (sysExInternalBuffer[3] == 0x00 ) {
              EEPROM_Params.midiCableRoutingTarget[sysExInternalBuffer[4]] = (sysExInternalBuffer[5] << 4 ) + sysExInternalBuffer[6];
          } else
          // Serial
          if (sysExInternalBuffer[3] == 0x01 ) {
             EEPROM_Params.midiSerialRoutingTarget[sysExInternalBuffer[4]] = (sysExInternalBuffer[5] << 4 ) + sysExInternalBuffer[6];
          }
          // Write the whole param struct
          EEPROM_writeBlock(0, (uint8*)&EEPROM_Params, sizeof(EEPROM_Params));
      }
      break;
  }

}

///////////////////////////////////////////////////////////////////////////////
// CHECK EEPROM
//----------------------------------------------------------------------------
// Retrieve global parameters from EEPROM, or Initalize it
//////////////////////////////////////////////////////////////////////////////
void CheckEEPROM() {

  // Set EEPROM parameters for the STMF103RC

  EEPROM.PageBase0 = 0x801F000;
  EEPROM.PageBase1 = 0x801F800;
  EEPROM.PageSize  = 0x800;

  // Read the EEPROM parameters structure
  EEPROM_readBlock(0, (uint8 *)&EEPROM_Params, sizeof(EEPROM_Params) );

  // If the signature is not found, of not the same version of parameters structure,
  // or new build, then initialize
  if (
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

    // Intelligent Midi Thru.
    // intelligentMidiThru ==0 => Disabled
    EEPROM_Params.intelligentMidiThruIn = DEFAULT_INTELLIGENT_MIDI_THRU_IN;
    EEPROM_Params.intelligentMidiThruDelayPeriod = DEFAULT_INTELLIGENT_MIDI_THRU_DELAY_PERIOD ;
    EEPROM_Params.intelligentMidiThruOut = DEFAULT_INTELLIGENT_MIDI_THRU_OUT;

    // Routing targets
    memcpy( EEPROM_Params.midiCableRoutingTarget,defaultMidiCableRoutingTarget,sizeof(defaultMidiCableRoutingTarget));
    memcpy( EEPROM_Params.midiSerialRoutingTarget,defaultMidiSerialRoutingTarget,sizeof(defaultMidiSerialRoutingTarget));

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
int EEPROM_writeBlock(uint16 ee, const uint8 *bloc, uint16 size )
{
    uint16 i;
    for (i = 0; i < size; i++) EEPROM.write(ee+i, *(bloc+i));

    return i;
}

int EEPROM_readBlock(uint16 ee, uint8 *bloc, uint16 size )
{
    uint16 i;
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
// Show current EEPROM settings
///////////////////////////////////////////////////////////////////////////////
static void ShowCurrentSettings() {
	uint8_t i,j;

  Serial.print("\n================ CURRENT SETTINGS ================");

  Serial.print("\n\nMagic - V - Build : ");
  Serial.write(EEPROM_Params.signature , sizeof(EEPROM_Params.signature));
	Serial.write(" - "); Serial.print( EEPROM_Params.prmVer);
  Serial.write(" - "); Serial.print( (char *)EEPROM_Params.TimestampedVersion);

  Serial.print("\nBootMode          : "); Serial.print(EEPROM_Params.nextBootMode);
  Serial.print("\nVID - PID - STR   : ");
	Serial.print( EEPROM_Params.vendorID,HEX);  Serial.write(" - ");
  Serial.print( EEPROM_Params.productID,HEX); Serial.write(" - ");
	Serial.write(EEPROM_Params.productString, sizeof(EEPROM_Params.productString));

  Serial.print("\n\n------------ Midi routing ------------");
  Serial.print("\n\nCable            Cable IN  |  Midi OUT");
  Serial.print("\nUSB OUT #  --->  1 2 3 4   |  1 2 3 4");

  for (i=0; i< MIDI_ROUTING_TARGET_MAX ; i++) {
  Serial.print("\n   ");Serial.print(i+1); Serial.print("       --->  ");
    for ( j=4; j < 8 ; j++) {
      if ( EEPROM_Params.midiCableRoutingTarget[i] & ( 1 << j) )
        Serial.print("X ");
      else Serial.print(". ");
    }
    Serial.print("  |  ");
    for ( j=0; j < 4 ; j++) {
      if ( EEPROM_Params.midiCableRoutingTarget[i] & ( 1 << j) )
        Serial.print("X ");
      else Serial.print(". ");
    }
  }

   Serial.print("\n\nMIDI JACK        Cable IN  |  Midi OUT");
   Serial.print("\n   IN #    --->  1 2 3 4   |  1 2 3 4");
   for (i=0; i< MIDI_ROUTING_TARGET_MAX ; i++) {
    Serial.print("\n   ");Serial.print(i+1); Serial.print("       --->  ");
     for ( j=4; j < 8 ; j++) {
       if ( EEPROM_Params.midiSerialRoutingTarget[i] & ( 1 << j) )
         Serial.print("X ");
       else Serial.print(". ");
     }
     Serial.print("  |  ");
     for ( j=0; j < 4 ; j++) {
       if ( EEPROM_Params.midiSerialRoutingTarget[i] & ( 1 << j) )
         Serial.print("X ");
       else Serial.print(". ");
     }
   }

   Serial.print("\n\n INTELLIGENT THRU | Filter       |  Midi OUT");
   Serial.print  ("\nJACK IN # 1 2 3 4 | Ch Sc Rt Sx  |  1 2 3 4");
   Serial.print("\n          ");

   for ( j=4; j < 8 ; j++) {
      if ( EEPROM_Params.intelligentMidiThruIn & ( 1 << j) )
           Serial.print("X ");
      else Serial.print(". ");
   }
   Serial.print("|  ");

   for ( j=0; j < 4 ; j++) {
     if ( EEPROM_Params.intelligentMidiThruIn & ( 1 << j) )
       Serial.print("X  ");
     else Serial.print(".  ");
   }

   Serial.print("|  ");
   for ( j=0; j < 4 ; j++) {
     if ( EEPROM_Params.intelligentMidiThruOut & ( 1 << j) )
       Serial.print("X ");
     else Serial.print(". ");
   }

   Serial.print("\n\nIntelligent Midi Thru delay (USB timeout) : ");
   Serial.print(EEPROM_Params.intelligentMidiThruDelayPeriod*15);Serial.print("s");
   Serial.print("\n(Intelligent Midi Thru is ");
   Serial.print(EEPROM_Params.intelligentMidiThruIn & 0xF0 ? "active)." : "inactive).");

   Serial.print("\n\n==================================================\n");

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
	for ( ;; )
		{
      Serial.print("\n\nUSBMIDIKliK 4x4 MENU");
      Serial.print(" - ");Serial.println(HARDWARE_TYPE);
			Serial.println("(c)TheKikGen Labs\n");
			Serial.println("0.Show current settings");
			Serial.println("1.Reload settings");
			Serial.println("2.USB product string");
			Serial.println("3.USB Vendor ID & Product ID");
			Serial.println("4.Intelligent Midi Thru MIDI filters");
      Serial.println("5.Intelligent Midi Thru delay for USB timeout");
      Serial.println("6.Intelligent Midi Thru IN Jack routing");
			Serial.println("7.Midi USB Cable OUT routing");
      Serial.println("8.Midi IN Jack routing");
      Serial.println("9.Reset routing to factory default");
			Serial.println("s.Save & quit");
			Serial.println("x.Abort");
			Serial.print("=>");

      choice = USBSerialGetChar();
      Serial.println(choice);

 	    switch (choice)
		 	{
        // Show current settings
  	 		case '0':
		 			ShowCurrentSettings();
		 			break;

		 		case '1':
		 		// Reload the EEPROM parameters structure
          Serial.print("\nReload current settings from EEPROM  (y/n) ? \n");
          c = USBSerialGetChar();Serial.write(c);
          if ( c == 'y') {
            CheckEEPROM();
		 			  Serial.print("\nSettings reloaded from EEPROM.\n");
          }
		 			break;

        // Change the product string
		 		case '2':
		 			Serial.print("\nEnter product string - ENTER to terminate :\n");
		 			i = 0;
		 			while ( i < MIDI_PRODUCT_STRING_SIZE && (c = USBSerialGetChar() ) !=13 ) {
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

        // Change VID & PID
   			case '3':
					Serial.print("\nEnter VID - PID, in hex (0-9,a-f) :\n");
					USBSerialScanHexChar( (char *) buff, 4, 0, 0);
					EEPROM_Params.vendorID  = GetInt16FromHex4Bin((char*)buff);
					Serial.print("-");
					USBSerialScanHexChar( (char * ) buff, 4, 0, 0);
					EEPROM_Params.productID = GetInt16FromHex4Bin((char*)buff);
					break;

        // Change Midi filter
        case '4':
          i = 0;
          Serial.print("\nFilter MIDI JACK IN messages :\n");
          Serial.print("\nChannel Voice    (y/n) ? ");
 				  choice = USBSerialGetChar();Serial.write(choice);
          if ( choice == 'y') i += 1;

          Serial.print("\nSystem Common    (y/n) ? ");
 				  choice = USBSerialGetChar();Serial.write(choice);
          if ( choice == 'y') i += 2;

          Serial.print("\nRealtime         (y/n) ? ");
 				  choice = USBSerialGetChar();Serial.write(choice);
          if ( choice == 'y') i += 4;

          Serial.print("\nSystem Exclusive (y/n) ? ");
 				  choice = USBSerialGetChar();Serial.write(choice);
          if ( choice == 'y') i += 8;

          if ( i == 0 ) {
            Serial.print("\nNo change made. Filter can't be null.");
          } else {
            EEPROM_Params.intelligentMidiThruIn =
              (EEPROM_Params.intelligentMidiThruIn & 0xF0) + i;
          }
					break;

        // <number of 15s periods 1-127>
        case '5':
            Serial.print("\nEnter the number of 15s delay periods for USB timeout (001-127 / 000 to exit) :\n");
            i = 0; j = 0;
            while ( i < 3 ) {
                c  = USBSerialGetDigit();Serial.write(c);
                j +=  ( (uint8_t) (c - '0' ) ) * pow(10 ,2-i);
                i++;
            }
            if (j == 0 or j >127 )
              Serial.print(". No change made. Incorrect value.\n");
            else {
                EEPROM_Params.intelligentMidiThruDelayPeriod = j;
                Serial.print(" <= Delay set to ");Serial.print(j*15);Serial.print("s\n");
            }
            break;

        // Intelligent MIDI Thru settings
        case '6':
            j = 0;
            Serial.print("\nActivate intelligent Midi Thru for :\n");
            for ( i=0 ; i< MIDI_ROUTING_TARGET_MAX ; i++ ){
              Serial.print("\nMidi Jack IN ");Serial.print(i+1);
              Serial.print(" (y/n) ? ");
              choice = USBSerialGetChar();Serial.write(choice);
              if ( choice == 'y') j += (1 << (i+4));
            }
            EEPROM_Params.intelligentMidiThruIn =
              (EEPROM_Params.intelligentMidiThruIn & 0x0F) + j;

            j = 0;
            Serial.print("\nConnect :\n");
            for ( i=0 ; i< MIDI_ROUTING_TARGET_MAX ; i++ ){
              Serial.print("\nMidi Jack OUT ");Serial.print(i+1);
              Serial.print(" (y/n) ? ");
              choice = USBSerialGetChar();Serial.write(choice);
              if ( choice == 'y') j += (1 << i);
            }

            if ( j == 0 ) {
              Serial.print("\nNo change made for Midi Jack out.");
            } else {
              EEPROM_Params.intelligentMidiThruOut  = j;
            }

  					break;

        // Cables midi routing
        case '7':
              Serial.print("\nEnter USB cable OUT # (usually 1-4/ 0 to exit)  :\n");
              while ( (choice = USBSerialGetDigit() - '0' ) > MIDI_ROUTING_TARGET_MAX ) ;
              if (choice > 0) {
                Serial.print("\nSet routing for USB Cable OUT #");Serial.print((uint8_t)choice);
                j = 0;
                for ( i=0 ; i< MIDI_ROUTING_TARGET_MAX ; i++ ){
                  Serial.print("\nC");Serial.print((uint8_t)choice);
                  Serial.print(" => Route to USB cable IN # ");Serial.print(i+1);
                  Serial.print(" (y/n) ? ");
                  c = USBSerialGetChar();Serial.write(c);
                  if ( c == 'y') j += (1 << (i+4));
                }
                Serial.println();
                for ( i=0 ; i< MIDI_ROUTING_TARGET_MAX ; i++ ){
                  Serial.print("\nC");Serial.print((uint8_t)choice);
                  Serial.print(" => Route to serial Midi Jack OUT ");Serial.print(i+1);
                  Serial.print(" (y/n) ? ");
                  c = USBSerialGetChar();Serial.write(c);
                  if ( c == 'y') j += (1 << i);
                }
                EEPROM_Params.midiCableRoutingTarget[choice-1] = j;
              }

    					break;

        // Midi IN jack routing
        case '8':
                Serial.print("\nEnter MIDI IN Jack # (usually 1-4/ 0 to exit)  :\n");
                while ( (choice = USBSerialGetDigit() - '0' ) > MIDI_ROUTING_TARGET_MAX ) ;
                if (choice > 0) {
                  Serial.print("\nSet routing for MIDI IN JACK #");Serial.print((uint8_t)choice);
                  j = 0;
                  for ( i=0 ; i< MIDI_ROUTING_TARGET_MAX ; i++ ){
                    Serial.print("\nS");Serial.print((uint8_t)choice);
                    Serial.print(" => Route to USB cable IN # ");Serial.print(i+1);
                    Serial.print(" (y/n) ? ");
                    c = USBSerialGetChar();Serial.write(c);
                    if ( c == 'y') j += (1 << (i+4));
                  }
                  Serial.println();
                  for ( i=0 ; i< MIDI_ROUTING_TARGET_MAX ; i++ ){
                    Serial.print("\nS");Serial.print((uint8_t)choice);
                    Serial.print(" => Route to serial Midi Jack OUT ");Serial.print(i+1);
                    Serial.print(" (y/n) ? ");
                    c = USBSerialGetChar();Serial.write(c);
                    if ( c == 'y') j += (1 << i);
                  }
                  EEPROM_Params.midiSerialRoutingTarget[choice-1] = j;
                }

      					break;

        // Default midi routing
        case '9':
                Serial.print("\nReset MIDI routing to default (y/n) ? \n");
                c = USBSerialGetChar();Serial.write(c);
                if ( c == 'y') {
                  EEPROM_Params.intelligentMidiThruIn = DEFAULT_INTELLIGENT_MIDI_THRU_IN;
                  EEPROM_Params.intelligentMidiThruDelayPeriod = DEFAULT_INTELLIGENT_MIDI_THRU_DELAY_PERIOD ;
                  EEPROM_Params.intelligentMidiThruOut = DEFAULT_INTELLIGENT_MIDI_THRU_OUT;
                  memcpy( EEPROM_Params.midiCableRoutingTarget,defaultMidiCableRoutingTarget,sizeof(defaultMidiCableRoutingTarget));
                  memcpy( EEPROM_Params.midiSerialRoutingTarget,defaultMidiSerialRoutingTarget,sizeof(defaultMidiSerialRoutingTarget));
                }
                break;

        // Save & quit
			  case 's':
					ShowCurrentSettings();
					Serial.print("\nSave settings and exit to midi mode (y/n) ?");
					choice = USBSerialGetChar();Serial.write(choice);
					if ( choice == 'y') {
            //Write the whole param struct
            EEPROM_writeBlock(0, (uint8*)&EEPROM_Params,sizeof(EEPROM_Params) );
            nvic_sys_reset();
					}
					break;

        // Abort
		 		case 'x':
          Serial.print("\nAbort (y/n) ?");
          choice = USBSerialGetChar();Serial.write(choice);
          if ( choice == 'y') nvic_sys_reset();
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
      serialInterface[s]->begin(31250);
      serialMidiParser[s].setMidiChannelFilter(midiXparser::allChannel);
      serialMidiParser[s].setMidiMsgFilter( midiXparser::allMidiMsg );
      serialMidiParser[s].setSysExFilter(true,0);
    }

    // MIDI USB initiate connection
    MidiUSB.begin() ;
    delay(500);
    digitalWrite(LED_CONNECT,MidiUSB.isConnected() ?  LOW : HIGH);

}

///////////////////////////////////////////////////////////////////////////////
// LOOP
///////////////////////////////////////////////////////////////////////////////

void loop() {

    // Try to connect/reconnect USB if we detect a high level on USBDM
    // This is to manage the case of a powered device without USB active (suspend mode for ex.)
    uint8_t cx = MidiUSB.isConnected();
    if ( !cx ) {
       // Assert PA11 (USBDM) to check USB hardware connection
       // A small lag on midi serial can surround if the connection is set
       if (  gpio_read_bit(PIN_MAP[PIN_USBDM].gpio_device,PIN_MAP[PIN_USBDM].gpio_bit) ) {
          flashLED_CONNECT->start();
          MidiUSB.end() ;
          delay(200);
          flashLED_CONNECT->start();
          MidiUSB.begin() ;
          delay(500);
       }
    }

    // SET CONNECT LED STATUS. We use gpio instead digitalWrite to be really fast, in that case....
    // We lost the Arduino compatibility here...
    // We do that only when dedicated LEDs exist for MIDI (i.e. not the Blue Pill)...
    #ifdef LEDS_MIDI
    gpio_write_bit(PIN_MAP[LED_CONNECT].gpio_device,PIN_MAP[LED_CONNECT].gpio_bit, cx ? 0 : 1 );
    #endif

    // Do we have a MIDI USB packet available ?
    if ( cx && MidiUSB.available() ) {

      midiUSBActive = true;
      midiUSBLastPacketMillis = millis()  ;

      // Read packet only if serial room.
      // As we ignore destination due to dynamic routing, we must check all
      // possible serial ports. That implies that even if a packet must be
      // routed from USB to USB, serial must have room. Not an issue but good to know.
      uint8_t s=  0;
      do {
        if ( serialInterface[s]->availableForWrite() < 4 ) break;
      } while (++s < SERIAL_INTERFACE_MAX );
      // All serial ports have room for the packet
      if (s == SERIAL_INTERFACE_MAX) {
        uint32 pk = MidiUSB.readPacket();
        // Route Packet to the appropriate cable and serial out
        RoutePacketToTarget( FROM_USB,  (const midiPacket_t *) &pk );
      }
    }

    // Midi USB timeout
    else if ( midiUSBActive && millis() > ( midiUSBLastPacketMillis + intelligentMidiThruDelayMillis )  ) {
      flashLED_CONNECT->start();delay(100);
      flashLED_CONNECT->start();delay(100);
      flashLED_CONNECT->start();delay(100);
      midiUSBActive  = false;
    }


    // Do we have any MIDI msg on Serial 1 to n ?
    for (uint8_t s=0; s <SERIAL_INTERFACE_MAX ; s++ ) {
        if ( serialInterface[s]->available() ) {

              midiSerialActive = true;
              midiSerialLastPacketMillis = millis();

             if ( serialMidiParser[s].parse( serialInterface[s]->read() ) ) {

                  if ( midiUSBActive )
                    SendMidiSerialMsgToUsb( s, &serialMidiParser[s]);
                  else {
                    // If USB was set inactive after timeout, switch to Intelligent Thru mode
                    // NB : Bit testing for midi msg rely on msgType, matching value 1 to 8.
                    // This was done for speed optimization but could break if enum value change
                    // in the midiXparser class. 1:channel, 2:systemCo., 4:realTime, 8:Sysex
                    if ( EEPROM_Params.intelligentMidiThruOut
                         && ( EEPROM_Params.intelligentMidiThruIn & ( 1 << (s+4) ) )
                         && ( EEPROM_Params.intelligentMidiThruIn & serialMidiParser[s].getMidiMsgType() )
                       )
                    {
                      // Set the new routing rules for Intelligent Midi Thru serial MIDI IN
                      // broadcast the message then restore the routing rule
                      uint8_t savedRoutingRule = EEPROM_Params.midiSerialRoutingTarget[s];
                      EEPROM_Params.midiSerialRoutingTarget[s] = EEPROM_Params.intelligentMidiThruOut;
                      SendMidiSerialMsgToUsb( s, &serialMidiParser[s]);
                      EEPROM_Params.midiSerialRoutingTarget[s]=savedRoutingRule;
                    }
                    else
                      SendMidiSerialMsgToUsb( s, &serialMidiParser[s]);
                  }
             }
             else

             // Check if a SYSEX msg is currently sent or terminated
             // as we proceed on the fly.
             if ( serialMidiParser[s].isByteCaptured() &&
                  ( serialMidiParser[s].isSysExMode() ||
                    serialMidiParser[s].getByte() == midiXparser::eoxStatus ||
                    serialMidiParser[s].isSysExError()  ) )
             {
                // Process for eventual SYSEX unbuffered on the fly
                PrepareSysExPacket(s, &serialMidiParser[s]) ;
             }
        }    // Midi Serial timeout
        else if ( millis() > ( midiSerialLastPacketMillis + MIDI_SERIAL_TIMEOUT_MILLIS ) )
                midiSerialActive  = false;
  }

}
