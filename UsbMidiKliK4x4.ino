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

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.

   Licence : MIT.
*/

#include <PulseOutManager.h>
#include <midiXparser.h>
#include "usb_midi.h"
#include <libmaple/nvic.h>

// Timer
#define TIMER2_RATE_MICROS 1000
HardwareTimer timer(2);

// Serial interfaces Array
#define SERIAL_INTERFACE_MAX  4
HardwareSerial * serialInterface[SERIAL_INTERFACE_MAX] = {&Serial1,&Serial2,&Serial3,&Serial4};

// Sysex used to set some parameters of the interface.
// Be aware that the 0x77 manufacturer id is reserved in the MIDI standard (but not used)
// The second byte is usually an id number or a func code + the midi channel (any here)
// The Third is the product id
#define SYSEX_INTERNAL_BUFF_SIZE 32
static  uint8_t sysExInternalHeader[] = { 0x77,0x77,0x78} ;
static  uint8_t sysExInternalBuffer[SYSEX_INTERNAL_BUFF_SIZE] ;

// Prepare LEDs pulse for MIDIN and MIDIOUT
// From MIDI SERIAL point of view

// LED light duration in milliseconds
#define LED_PULSE_MILLIS  5

// Use a PulseOutManager factory to create the pulses
PulseOutManager flashLEDManager;

// LED must be declared in the same order as hardware serials
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

#define LED_CONNECT   D41
PulseOut* flashLED_CONNECT = flashLEDManager.factory(LED_CONNECT,LED_PULSE_MILLIS,LOW);

// USB Midi object
USBMidi MidiUSB;

// MIDI Parsers for serial 1 to 4
midiXparser serialMidiParser[4];

///////////////////////////////////////////////////////////////////////////////
// MIDI Routing tables
// ---------------------------------------------------------------------------
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
// the standard configuraiton of a MIDIPLUS/MIDITECH interface.
///////////////////////////////////////////////////////////////////////////////

#define MIDI_ROUTING_TARGET_MAX 4
#define FROM_SERIAL 0
#define FROM_USB    1

// Routing from an USB cable IN
#define DEFAULT_MIDI_CABLE_ROUTING_TARGET  0B00000001,0B00000010,0B00000100,0B00001000

// Routing from an serial MIDI IN
#define DEFAULT_MIDI_SERIAL_ROUTING_TARGET 0B00010000,0B00100000,0B01000000,0B10000000

uint8_t midiCableRoutingTarget[MIDI_ROUTING_TARGET_MAX]  = {DEFAULT_MIDI_CABLE_ROUTING_TARGET};
uint8_t midiSerialRoutingTarget[MIDI_ROUTING_TARGET_MAX] = {DEFAULT_MIDI_SERIAL_ROUTING_TARGET};

///////////////////////////////////////////////////////////////////////////////
// Timer2 interrupt handler
///////////////////////////////////////////////////////////////////////////////
void Timer2Handler(void) {

     // Update LEDS
     flashLEDManager.update(millis());
}

///////////////////////////////////////////////////////////////////////////////
// Send a USB midi packet to ad-hoc serial MIDI
///////////////////////////////////////////////////////////////////////////////
void sendMidiUsbPacketToSerial(uint32_t packet, uint8_t serialNo) {

  union EVENT_t midiPacket = { .i = packet };
  //uint8_t cable = midiPacket.packet[0] >> 4;

  uint8_t cin   = midiPacket.packet[0] & 0x0F ;
  //if (cable >= SERIAL_INTERFACE_MAX ) return;

  if (serialNo >= SERIAL_INTERFACE_MAX ) return;

  // Sendpacket to serial at the right size
  switch (cin) {
          // 1 byte
          case 0x05: case 0x0F:
            serialInterface[serialNo]->write(midiPacket.packet[1]);
            flashLED_OUT[serialNo]->start();
            break;

          // 2 bytes
          case 0x02: case 0x06: case 0x0C: case 0x0D:
            serialInterface[serialNo]->write(midiPacket.packet[1]);
            serialInterface[serialNo]->write(midiPacket.packet[2]);
            flashLED_OUT[serialNo]->start();
            break;

          // 3 bytes
          case 0x03: case 0x07: case 0x04: case 0x08:
          case 0x09: case 0x0A: case 0x0B: case 0x0E:
            serialInterface[serialNo]->write(midiPacket.packet[1]);
            serialInterface[serialNo]->write(midiPacket.packet[2]);
            serialInterface[serialNo]->write(midiPacket.packet[3]);
            flashLED_OUT[serialNo]->start();
            break;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Send a serial midi msg to the right USB midi cable
///////////////////////////////////////////////////////////////////////////////
void sendMidiSerialMsgToUsb( uint8_t cable, midiXparser* serialMidiParser ) {

    union EVENT_t usbMidiPacket;

    uint8_t msgLen = serialMidiParser->getMidiMsgLen();
    uint8_t msgType = serialMidiParser->getMidiMsgType();

    memset(&usbMidiPacket.packet[0],0,4);
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

    //MidiUSB.writePacket(usbMidiPacket.i);    // Send to USB
    routePacketToTarget( FROM_SERIAL,usbMidiPacket.i);
}

///////////////////////////////////////////////////////////////////////////////
// Scan and parse sysex flows
// ----------------------------------------------------------------------------
// We use the midiXparser 'on the fly' mode, allowing to tag bytes as "captured"
// when they belong to a midi SYSEX message, without storing them in a buffer.
// SYSEX Error (not correctly terminated by 0xF7 for example) are cleaned up,
// to restore a correct parsing state.
///////////////////////////////////////////////////////////////////////////////

void scanMidiSerialSysExToUsb( uint8_t cable, midiXparser* serialMidiParser ) {
  static union EVENT_t usbMidiSysExPacket[SERIAL_INTERFACE_MAX];
  static uint8_t packetLen[SERIAL_INTERFACE_MAX];
  static bool firstCall = true;
  static unsigned sysExInternalMsgIdx = 0;
	static bool 		sysExInternalHeaderFound = false;

  byte readByte = serialMidiParser->getByte();

  // Initialize everything at the first call
  if (firstCall ) {
    firstCall = false;
    memset(&usbMidiSysExPacket[0],0,4*SERIAL_INTERFACE_MAX);
    memset(&packetLen[0],0,sizeof(uint8_t)*SERIAL_INTERFACE_MAX);
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
      routePacketToTarget( FROM_SERIAL,usbMidiSysExPacket[cable].i);
      packetLen[cable] = 0;
      memset(&(usbMidiSysExPacket[cable].packet[0]),0,4);

      // Internal SYSEX to manage ?
      if ( sysExInternalHeaderFound  && sysExInternalBuffer[0] > 0   ) {
        // Process internal Sysex
        ProcessSysExInternal();
      }
      sysExInternalHeaderFound = false;
      sysExInternalMsgIdx = 0;
      sysExInternalBuffer[0] = 0;
  }

  // Stop if not in sysexmode anymore here !
  // The SYSEX error could be caused by another SOX, or Midi status...,
  if ( ! serialMidiParser->isSysExMode() ) return;

  // Fill USB sysex packet
  packetLen[cable]++;
  usbMidiSysExPacket[cable].packet[ packetLen[cable] ] = readByte ;

	// Is it an internal SYSEX message for the MIDI interface   ?

  if ( readByte == sysExInternalHeader[sysExInternalMsgIdx] || sysExInternalHeaderFound ) {
      if (sysExInternalHeaderFound) {
        // Start storing the message in the msg buffer
        // If Message too big. don't store...
        if ( sysExInternalBuffer[0] <  sizeof(sysExInternalBuffer)-1  ) {
            sysExInternalBuffer[0]++;
            sysExInternalBuffer[sysExInternalBuffer[0]]  = readByte;
        }

      }
      else {
        sysExInternalMsgIdx++;
        if (sysExInternalMsgIdx >= sizeof(sysExInternalHeader) ) {
            sysExInternalHeaderFound = true;
            sysExInternalBuffer[0] = 0;
        }
        else sysExInternalHeaderFound = false;
      }
  } else sysExInternalMsgIdx =0;

  // Packet complete ?
  if (packetLen[cable] == 3 ) {
      usbMidiSysExPacket[cable].packet[0] = (cable << 4) + 4 ; // Sysex start or continue
      routePacketToTarget( FROM_SERIAL,usbMidiSysExPacket[cable].i);
      //MidiUSB.writePacket(usbMidiSysExPacket[cable].i);    // Send to USB
      //flashLED_IN[cable]->start();
      packetLen[cable] = 0;
      memset(&(usbMidiSysExPacket[cable].packet[0]),0,4);
  }
}

///////////////////////////////////////////////////////////////////////////////
// midi packet Router
//-----------------------------------------------------------------------------
// Route a packet from one MIDI IN / USB OUT to n MIDI OUT/USB IN
///////////////////////////////////////////////////////////////////////////////

void routePacketToTarget(uint8_t source,uint32_t packet) {

  union EVENT_t midiPacket = { .i = packet };
  uint8_t cable = midiPacket.packet[0] >> 4;
  if (cable >= SERIAL_INTERFACE_MAX ) return;

  uint8_t cin   = midiPacket.packet[0] & 0x0F ;

  uint8_t targets =
    ( source == FROM_SERIAL ? midiSerialRoutingTarget[cable]:midiCableRoutingTarget[cable]);

  // Find targets
  // Bit 0-3 : Serial 1 - Serial 4
  // Bit 4-7 : Cable  0 - Cable 4

  // Serial targets
  for (uint8_t t=0; t<=3 ; t++) {

      if (targets & ( 1 << t ) ) sendMidiUsbPacketToSerial(midiPacket.i,t );

  }

  targets = targets >> 4;

  // usb cable targets
  for (uint8_t t=0; t<=3 ; t++) {

      if (targets & ( 1 << t ) ) {
        midiPacket.packet[0] = ( t << 4 ) + cin;
        MidiUSB.writePacket(midiPacket.i);    // Send to USB
        flashLED_IN[t]->start();
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
// 0xA Hard reset interface
// 0xB Set a new product string		< product String 30 x7 bits char max >
// 0xC Set VendorID & ProductID   <n1n2n3n4=VendorId > <n1n2n3n4=ProductId>
// ----------------------------------------------------------------------------
// sysExInternalBuffer[0] length of the message (func code + data)
// sysExInternalBuffer[1] function code
// sysExInternalBuffer[2] data without EOX
///////////////////////////////////////////////////////////////////////////////
static void ProcessSysExInternal() {

	uint8_t msgLen = sysExInternalBuffer[0];
	uint8_t cmdId  = sysExInternalBuffer[1];
	uint8_t i,j;

	switch (cmdId) {

		// RESET USB MIDI INTERFACE
		// F0 77 77 77 0A F7
		case 0x0A:
			nvic_sys_reset();
			break;

    // SET ROUTING TARGETS
    // To configure the routing for an input, you must set some bits of the target byte to 1 : 
    // Bits 0-3 are corresponding repesctively to Serial Midi out Jack targets 1-4
    // Bits 4-7 are corresponding respectively to USB Cables targets IN 0-3.
    // Sysex message structure : 
    //
    // F0 77 77 78 <0x0F> <0x01 = set> <cable=0X0 | serial=0x1> <id:0-4> <target nibble cable> <target nibble serial> F7
    //
    // F0 77 77 78 <0x0F> <0x00 = default> F7  
    //
    // For example, the following routing rule set MIDI IN JACK1/JACK2 to be merged to cable 0 :
    //
    //   F0 77 77 78 0F 01 01 00 01 00 F7
    //   F0 77 77 78 0F 01 01 01 01 00 F7
    //
    // The following sysex will restore default routing for all inputs : F0 77 77 78 0F 00 F7
    
    case 0x0F:

      // reset to default routing
      if (sysExInternalBuffer[2] == 0x00 ) {
           uint8_t xmidiCableRoutingTarget[MIDI_ROUTING_TARGET_MAX]  = {DEFAULT_MIDI_CABLE_ROUTING_TARGET};
           uint8_t xmidiSerialRoutingTarget[MIDI_ROUTING_TARGET_MAX] = {DEFAULT_MIDI_SERIAL_ROUTING_TARGET};
           memcpy(&midiCableRoutingTarget,&xmidiCableRoutingTarget,MIDI_ROUTING_TARGET_MAX);
           memcpy(&midiSerialRoutingTarget,&xmidiSerialRoutingTarget,MIDI_ROUTING_TARGET_MAX);       
      } else
      
      // Set targets
      if (sysExInternalBuffer[2] == 0x01 ) {
                    
          if ( sysExInternalBuffer[4]>= MIDI_ROUTING_TARGET_MAX) break;
          if ( sysExInternalBuffer[5] > 0xF || sysExInternalBuffer[6] > 0xF)  break;
          
          // Cable
          if (sysExInternalBuffer[3] == 0x00 ) {
              
              midiCableRoutingTarget[sysExInternalBuffer[4]] = (sysExInternalBuffer[5] << 4 ) + sysExInternalBuffer[6];
            
          } else

          // Serial
          if (sysExInternalBuffer[3] == 0x01 ) {

             midiSerialRoutingTarget[sysExInternalBuffer[4]] = (sysExInternalBuffer[5] << 4 ) + sysExInternalBuffer[6];
          }                  
      }
      
      break;


	}

};

///////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////
void setup() {

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

    // MIDI SERIAL PORTS set Baud rates
    // To compile with the 4 serial ports, you must use the right variant : STMF103RC
    // + Set parsers filters in the same loop.  All messages but SYSEX.

    for ( uint8_t s=0; s < SERIAL_INTERFACE_MAX ; s++ ) {
      serialInterface[s]->begin(31250);
      serialMidiParser[s].setMidiChannelFilter(midiXparser::allChannel);
      serialMidiParser[s].setMidiMsgFilter( midiXparser::allMidiMsg );
      serialMidiParser[s].setSysExFilter(true,0);
    }

    // START USB
    MidiUSB.begin() ;
    // Wait and signal the state by flashing the POWER LED then restart
    for (uint8_t i=1; i<=5 &&  !MidiUSB.isConnected() ; i++ ) {
        flashLED_CONNECT->start(); delay(500);
        flashLED_CONNECT->start(); delay(500);
        flashLED_CONNECT->start(); delay(500);
        flashLED_CONNECT->start(); delay(100);
        flashLED_CONNECT->start(); delay(100);
        flashLED_CONNECT->start(); delay(100);
    }
    // Force the POWER LED STATE (LOW logic)
    digitalWrite(LED_CONNECT,MidiUSB.isConnected() ?  LOW : HIGH);

}

///////////////////////////////////////////////////////////////////////////////
// LOOP
///////////////////////////////////////////////////////////////////////////////

void loop() {

    static uint8_t s=0;

    // Reflect the USB connection status
    if ( ! MidiUSB.isConnected() ) digitalWrite(LED_CONNECT, HIGH);

    // Do we have a MIDI USB packet available ?
    if ( MidiUSB.available() ) {

      // Route Packet to the appropriate cable and serial out
      routePacketToTarget( FROM_USB,MidiUSB.readPacket());

    }

    // Do we have any MIDI msg on Serial 1 to 4 ?

    if ( serialInterface[s]->available() ) {

         if ( serialMidiParser[s].parse( serialInterface[s]->read() ) ) {
              sendMidiSerialMsgToUsb( s, &serialMidiParser[s]);
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
            scanMidiSerialSysExToUsb(s, &serialMidiParser[s]) ;
         }
    }

    if ( ++s >= SERIAL_INTERFACE_MAX ) s = 0;
}
