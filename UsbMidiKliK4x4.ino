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

// Timer
#define TIMER2_RATE_MICROS 1000
HardwareTimer timer(2);

// Serial interfaces Array
#define SERIAL_INTERFACE_MAX  4
HardwareSerial * serialInterface[SERIAL_INTERFACE_MAX] = {&Serial1,&Serial2,&Serial3,&Serial4};

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
// Routing from an USB cable IN
#define DEFAULT_MIDI_CABLE_ROUTING_TARGET  B00000001,B00000010,B00000100,B00001000
// Routing from an serial MIDI IN
#define DEFAULT_MIDI_SERIAL_ROUTING_TARGET B00010000,B00100000,B01000000,B10000000

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
void sendMidiUsbPacketToSerial(uint32_t packet) {

  union EVENT_t midiPacket = { .i = packet };
  uint8_t cable = midiPacket.packet[0] >> 4;
  uint8_t cin   = midiPacket.packet[0] & 0x0F ;

  if (cable >= SERIAL_INTERFACE_MAX ) return;

  HardwareSerial *mySerial = serialInterface[cable];
  PulseOut* flashLED_O  = flashLED_OUT[cable];

  // Sendpacket to serial at the right size
  switch (cin) {
          // 1 byte
          case 0x05: case 0x0F:
            mySerial->write(midiPacket.packet[1]);
            flashLED_O->start();
            break;

          // 2 bytes
          case 0x02: case 0x06: case 0x0C: case 0x0D:
            mySerial->write(midiPacket.packet[1]);
            mySerial->write(midiPacket.packet[2]);
            flashLED_O->start();
            break;

          // 3 bytes
          case 0x03: case 0x07: case 0x04: case 0x08:
          case 0x09: case 0x0A: case 0x0B: case 0x0E:
            mySerial->write(midiPacket.packet[1]);
            mySerial->write(midiPacket.packet[2]);
            mySerial->write(midiPacket.packet[3]);
            flashLED_O->start();
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

    MidiUSB.writePacket(usbMidiPacket.i);    // Send to USB
    flashLED_IN[cable]->start();
}

///////////////////////////////////////////////////////////////////////////////
// Scan and parse sysex flows
// ----------------------------------------------------------------------------
// We use the midiXparser 'on the fly' mode, allowing to mark bytes "captured"
// when they belong to a midi SYSEX message, without storing them in a buffer.
// SYSEX Error (not correctly terminated by 0xF7 for example) are cleaned up,
// to restore a correct parsing state.
///////////////////////////////////////////////////////////////////////////////

void scanMidiSysExToUsb( uint8_t cable, midiXparser* serialMidiParser ) {
  static union EVENT_t usbMidiSysExPacket[SERIAL_INTERFACE_MAX];
  static uint8_t packetLen[SERIAL_INTERFACE_MAX];
  static bool firstCall = true;
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
      MidiUSB.writePacket(usbMidiSysExPacket[cable].i);    // Send to USB
      flashLED_IN[cable]->start();
      packetLen[cable] = 0;
      memset(&(usbMidiSysExPacket[cable].packet[0]),0,4);
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
      MidiUSB.writePacket(usbMidiSysExPacket[cable].i);    // Send to USB
      flashLED_IN[cable]->start();
      packetLen[cable] = 0;
      memset(&(usbMidiSysExPacket[cable].packet[0]),0,4);
  }
}



void RouteMidiUsbToTarget(uint32_t midiPacket) {



}

void routeMidiSerialToTarget() {

}


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

    // Reflect the USB connection status
    if ( ! MidiUSB.isConnected() ) digitalWrite(LED_CONNECT, HIGH);

    // Do we have a MIDI USB packet available ?
    if ( MidiUSB.available() ) {

      // Send Packet to the appropriate serial
      sendMidiUsbPacketToSerial( MidiUSB.readPacket());

    }

    // Do we have any MIDI msg on Serial 1 to 4 ?
    for ( uint8_t s=0; s< SERIAL_INTERFACE_MAX ; s++ ) {

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
              scanMidiSysExToUsb(s, &serialMidiParser[s]) ;
           }
       }

   } // for
}
