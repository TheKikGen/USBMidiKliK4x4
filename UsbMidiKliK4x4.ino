/*

KigenLab - MIDI 4X4 firmware
THIS NOT A COPY OR A HACK OF ANY EXISTING MIDITECH/MIDIPLUS FIRMWARE.

It was entirely rewritten from the hardware study, based on the STM32F103RC MCU.

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

// MIDI Routing tables
// 8 targets by midi input are possible (cable USB IN or Serial MIDI IN)
//
// When followings bits are set to 1, the midi message will be route to :
//
// Bit 0-3 : Serial1 - Serial4 
// Bit 4-7 : Cable 0 - Cable 4

#define MIDI_ROUTING_TARGET_MAX 4
// Routing from an USB cable IN
#define DEFAULT_MIDI_CABLE_ROUTING_TARGET  B00000001,B00000010,B00000100,B00001000
// Routing from an serial MIDI IN
#define DEFAULT_MIDI_SERIAL_ROUTING_TARGET B00010000,B00100000,B01000000,B10000000

uint8_t midiCableRoutingTarget[MIDI_ROUTING_TARGET_MAX]  = {DEFAULT_MIDI_CABLE_ROUTING_TARGET}; 
uint8_t midiSerialRoutingTarget[MIDI_ROUTING_TARGET_MAX] = {DEFAULT_MIDI_SERIAL_ROUTING_TARGET}; 

// Timer2 interrupt handler
void Timer2Handler(void) {

     // Update LEDS
     flashLEDManager.update(millis());
}

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
      serialMidiParser[s].setRealTimeMsgFilter(midiXparser::allRealTimeMsgMsk);
      serialMidiParser[s].setChannelVoiceMsgFilter(midiXparser::allChannelVoiceMsgMsk);
      serialMidiParser[s].setSystemCommonMsgFilter(midiXparser::allSystemCommonMsgMsk);
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



void sendMidiUsbPacketToSerial(uint32_t packet) {

  union EVENT_t midiPacket = { .i = packet };
  if (midiPacket.p.cable > sizeof(serialInterface) ) return;

  HardwareSerial *mySerial = serialInterface[midiPacket.p.cable];
  PulseOut* flashLED_O  = flashLED_OUT[(uint8_t)midiPacket.p.cable];


  // Sendpacket to serial at the right size
  switch (midiPacket.p.cin) {
          // 1 byte
          case 0x05: case 0x0F:
            mySerial->write(midiPacket.p.midi0);
            flashLED_O->start();
            break;

          // 2 bytes
          case 0x02: case 0x06: case 0x0C: case 0x0D:
            mySerial->write(midiPacket.p.midi0);
            mySerial->write(midiPacket.p.midi1);
            flashLED_O->start();
            break;

          // 3 bytes
          case 0x03: case 0x07: case 0x04: case 0x08:
          case 0x09: case 0x0A: case 0x0B: case 0x0E:
            mySerial->write(midiPacket.p.midi0);
            mySerial->write(midiPacket.p.midi1);
            mySerial->write(midiPacket.p.midi2);
            flashLED_O->start();
            break;
  }
}


void sendMidiSerialMsgToUsb( uint8_t cable, midiXparser* serialMidiParser ) {

    union EVENT_t usbMidiPacket;

    usbMidiPacket.p.cable = cable;          
    usbMidiPacket.p.midi0 = serialMidiParser->getMidiMsg()[0];
    usbMidiPacket.p.midi1 = 0;
    usbMidiPacket.p.midi2 = 0;  
      
    // Single byte message CIN F->
    if ( serialMidiParser->getMidiMsgLen()  == 1 ) {
        usbMidiPacket.p.cin   = 0xF;  
    } 
    else 

    // Channel voice message CIN A-E
    if ( serialMidiParser->getMidiMsgType()  == midiXparser::channelVoiceMsgType ) {       
        usbMidiPacket.p.cin   = ( (serialMidiParser->getMidiMsg()[0]) >> 4);  
        usbMidiPacket.p.midi1 = serialMidiParser->getMidiMsg()[1];
        if ( serialMidiParser->getMidiMsgLen()  == 3 ) {
                  usbMidiPacket.p.midi2 = serialMidiParser->getMidiMsg()[2];
        }
    } 
    else
    
    // System common message CIN 2-3
    // 2/3 - two/three bytes system common message
    if ( serialMidiParser->getMidiMsgType()  == midiXparser::systemCommonMsgType ) {
        usbMidiPacket.p.cin = serialMidiParser->getMidiMsgLen();
        usbMidiPacket.p.midi1 = serialMidiParser->getMidiMsg()[1];
        if ( serialMidiParser->getMidiMsgLen()  == 3 ) {
                  usbMidiPacket.p.midi2 = serialMidiParser->getMidiMsg()[2];
        }
    }    
    
    else return; // We should never be here !
    
    MidiUSB.writePacket(usbMidiPacket.i);    // Send to USB                                 
    flashLED_IN[cable]->start();
}
          
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

        if ( serialInterface[s]->available() && serialMidiParser[s].parse( serialInterface[s]->read() ) ) {       
 
          sendMidiSerialMsgToUsb( s, &serialMidiParser[s]);
           
        }
        else if (!serialMidiParser[s].isByteCaptured() ) {

          // Process SYSEX unbuffered on the fly

          

          
          
        }      
    } // for
      

}


