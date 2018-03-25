/*
<<<<<<< HEAD

KigenLab - MIDI 4X4 firmware
THIS NOT A COPY OR A HACK OF ANY EXISTING MIDITECH/MIDIPLUS FIRMWARE.

It was entirely rewritten from the hardware study, based on the STM32F103RC MCU.

*/

#include <PulseOutManager.h>
#include "USBMIDI.h"

// Timer 
#define TIMER2_RATE_MICROS 1000
HardwareTimer timer(2);

// Prepare LEDs pulse for MIDIN and MIDIOUT
// From MIDI SERIAL point of view

// LED light duration in milliseconds
#define LED_PULSE_MILLIS  5

PulseOutManager flashLEDManager;

// Use the PulseOutManager factory to create the pulses
#define LED_CONNECT   D41
PulseOut* flashLED_IN1  = flashLEDManager.factory(D4,LED_PULSE_MILLIS,LOW);
PulseOut* flashLED_IN2  = flashLEDManager.factory(D5,LED_PULSE_MILLIS,LOW);
PulseOut* flashLED_IN3  = flashLEDManager.factory(D6,LED_PULSE_MILLIS,LOW);
PulseOut* flashLED_IN4  = flashLEDManager.factory(D7,LED_PULSE_MILLIS,LOW);

PulseOut* flashLED_OUT1 = flashLEDManager.factory(D36,LED_PULSE_MILLIS,LOW);
PulseOut* flashLED_OUT2 = flashLEDManager.factory(D37,LED_PULSE_MILLIS,LOW);
PulseOut* flashLED_OUT3 = flashLEDManager.factory(D16,LED_PULSE_MILLIS,LOW);
PulseOut* flashLED_OUT4 = flashLEDManager.factory(D17,LED_PULSE_MILLIS,LOW);
PulseOut* flashLED_CONNECT = flashLEDManager.factory(LED_CONNECT,LED_PULSE_MILLIS,LOW);

void Timer2Handler(void) {
     
     // Update LEDS 
     flashLEDManager.update(millis());
=======
  MIDI note player

  This sketch shows how to use Serial1 (pins 7 and 8) to send MIDI
  note data.  If this circuit is connected to a MIDI synth, it will
  play the notes F#-0 (0x1E) to F#-5 (0x5A) in sequence.


  The circuit:
  * Pin 7 connected to MIDI jack pin 5
  * MIDI jack pin 2 connected to ground
  * MIDI jack pin 4 connected to +5V through 220-ohm resistor
  Attach a MIDI cable to the jack, then to a MIDI synth, and play music.

  created 13 Jun 2006
  modified 2 Jul 2009
  by Tom Igoe

  http://www.arduino.cc/en/Tutorial/MIDI

  Ported to the Maple 27 May 2010
  by Bryan Newbold
*/


#define PULSE_RATE 1000    // in microseconds;

const uint8_t notes[] = {60, 62, 64, 65, 67, 69, 71, 72, 61, 63, 66, 68, 70};
const int numNotes = sizeof(notes)/sizeof(*notes);

// We'll use timer 2
HardwareTimer timer(2);

#include <PulseOutManager.h>
#include "USBMIDI.h"

// Declare Pulse on Arduino digital pins

PulseOutManager myPulseOutManager;

// Using the factory
PulseOut* FlashLED4 = myPulseOutManager.factory(D4,5,LOW);
PulseOut* FlashLED5 = myPulseOutManager.factory(D5,5,LOW);
PulseOut* FlashLED6 = myPulseOutManager.factory(D6,5,LOW);
PulseOut* FlashLED7 = myPulseOutManager.factory(D7,5,LOW);
PulseOut* FlashLED8 = myPulseOutManager.factory(D16,5,LOW);
PulseOut* FlashLED9 = myPulseOutManager.factory(D17,5,LOW);
PulseOut* FlashLED10 = myPulseOutManager.factory(D36,5,LOW);
PulseOut* FlashLED11 = myPulseOutManager.factory(D37,5,LOW);

//USBMidi MidiUSB;


void handler_led(void) {
        myPulseOutManager.update(millis());
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
}


void setup() {
<<<<<<< HEAD

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

    // MIDI SERIAL PORTS Baud rates
    // To compile with the the 4 serial ports, you must use the right variant : STMF103RC
=======
    // Set MIDI baud rate:
    Serial.begin(115200);
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
    
    Serial1.begin(31250);
    Serial2.begin(31250);
    Serial3.begin(31250);
    Serial4.begin(31250);
<<<<<<< HEAD
  
    // START USB  
    
    MidiUSB.begin() ;
   // Wait and signal the state by flashing the POWER LED then restart
   for (uint8_t i=1; i<=5 &&  !MidiUSB.isConnected() ; i++ ) {
        flashLED_CONNECT->start();
        delay(500);
        flashLED_CONNECT->start();
        delay(500);
        flashLED_CONNECT->start();
        delay(100);
        flashLED_CONNECT->start();
        delay(100);    
    }
    // Force the POWER LED STATE (LOW logic)
    digitalWrite(LED_CONNECT,MidiUSB.isConnected() ?  LOW : HIGH);    
    
}

void routeMidiUsbPacketToSerial(uint32_t packet) {

  union EVENT_t midiPacket ;
  
  midiPacket.i = packet;
  HardwareSerial *mySerial = NULL;
  PulseOut* flashLED_OUT = NULL;

  // The cable # will indicate on what serial port to send the packet 
  switch (midiPacket.p.cable) {

      case 0: 
          mySerial = &Serial1; 
          flashLED_OUT = flashLED_OUT1 ; 
          flashLED_IN1->start();
          break;
      case 1: 
          mySerial = &Serial2; 
          flashLED_OUT = flashLED_OUT2 ; 
          flashLED_IN2->start();
          break;
      case 2: 
          mySerial = &Serial3; 
          flashLED_OUT = flashLED_OUT3 ; 
          flashLED_IN3->start();
          break;
      case 3: 
          mySerial = &Serial4; 
          flashLED_OUT = flashLED_OUT4 ;
          flashLED_IN4->start(); 
          break;
      default: return; break;
  }

  // Sendpacket to serial at the right size
  switch (midiPacket.p.cin) {
          // 1 byte
          case 0x05: case 0x0F: 
            mySerial->write(midiPacket.p.midi0);
            flashLED_OUT->start();
            break;
  
          // 2 bytes
          case 0x02: case 0x06: case 0x0C: case 0x0D: 
            mySerial->write(midiPacket.p.midi0);
            mySerial->write(midiPacket.p.midi1);
            flashLED_OUT->start();
            break;
  
          // 3 bytes      
          case 0x03: case 0x07: case 0x04: case 0x08: 
          case 0x09: case 0x0A: case 0x0B: case 0x0E:
            mySerial->write(midiPacket.p.midi0);
            mySerial->write(midiPacket.p.midi1);
            mySerial->write(midiPacket.p.midi2);
            flashLED_OUT->start();
            break;                   
  }    
}

void loop() {

    
    // Reflect the USB connection status
    if ( ! MidiUSB.isConnected() ) digitalWrite(LED_CONNECT, HIGH);    

    // Do we have a MIDI packet available ?
    if ( MidiUSB.available() ) {
      // Send Packet to the appropriate serial 
      routeMidiUsbPacketToSerial( MidiUSB.readPacket());     
    }

}

=======




    myPulseOutManager.begin();

//pinMode(D41,OUTPUT);


//digitalWrite(D8,HIGH);
//
//pinMode(D8,OUTPUT);
//digitalWrite(D8,HIGH);
//
//pinMode(D12,OUTPUT);
//digitalWrite(D12,LOW);
//      
//      for(volatile unsigned int i=0;i<512;i++);// Only small delay seems to be needed, and USB pins will get configured in Serial.begin
//      
//pinMode(D12,INPUT);


    
    // Pause the timer while we're configuring it
    timer.pause();

    // Set up period
    timer.setPeriod(PULSE_RATE); // in microseconds

    // Set up an interrupt on channel 1
    timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
    timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
    timer.attachCompare1Interrupt(handler_led);

    // Refresh the timer's count, prescale, and overflow
    timer.refresh();

    // Start the timer counting
    timer.resume();

    
     //delay(15000);
    MidiUSB.begin();
    delay(2000);


// GPIOC->regs->CRL = (GPIOC->regs->CRL & 
// 0x00000F00) | 
// 0x88000080 |
// 0x00333003;
//0x00000F00 is bitmask to retain value of pins 2 and 5 in original state
//0x88000080 is bitmask to set inputs
//0x00333003 is bitmask to set outputs
    
}







void loop() {
  for (int i=0;i<numNotes;i++) {
    MidiUSB.sendNoteOn(0, notes[i], 127);
    delay(200);
    MidiUSB.sendNoteOff(0, notes[i], 127);
 
    // Play notes from F#-0 (0x1E) to F#-5 (0x5A):
    for (int note = 0x1E; note < 0x5A; note++) {
        // Note on channel 1 (0x90), some note value (note), middle
        // velocity (0x45):

        noteOn(0x90, note, 0x45);        
        delay(100);
        // Note on channel 1 (0x90), some note value (note), silent
        // velocity (0x00):
        noteOn(0x90, note, 0x00);
        delay(500);
        
    }
}
}
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
// Plays a MIDI note.  Doesn't check to see that cmd is greater than
// 127, or that data values are less than 127:
void noteOn(int cmd, int pitch, int velocity) {
    Serial1.write(cmd);
    Serial1.write(pitch);
    Serial1.write(velocity);
    
    Serial2.write(cmd);
    Serial2.write(pitch);
    Serial2.write(velocity);
    
    Serial3.write(cmd);
    Serial3.write(pitch);
    Serial3.write(velocity);

    Serial4.write(cmd);
    Serial4.write(pitch);
    Serial4.write(velocity);

<<<<<<< HEAD
   
    
=======
    FlashLED4->start();
    FlashLED5->start();
    FlashLED6->start();     
    FlashLED7->start();
    FlashLED8->start();
    FlashLED9->start();
    FlashLED10->start();
    FlashLED11->start();
    


>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
}

