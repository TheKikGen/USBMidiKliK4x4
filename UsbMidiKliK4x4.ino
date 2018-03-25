/*
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
}


void setup() {
    // Set MIDI baud rate:
    //Serial.begin(115200);
    
    Serial1.begin(31250);
    Serial2.begin(31250);
    Serial3.begin(31250);
    Serial4.begin(31250);




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
   

    MidiUSB.begin();

    
}




void loop() {
 
//    // Play notes from F#-0 (0x1E) to F#-5 (0x5A):
//    for (int note = 0x1E; note < 0x5A; note++) {
//        // Note on channel 1 (0x90), some note value (note), middle
//        // velocity (0x45):
//        
//      for (int c=0; c<4 ; c++ ) {
//            MidiUSB.sendNoteOn(c,0, note, 0x45);
//            delay(100);
//            MidiUSB.sendNoteOff(c,0, note, 0x00);
//      }
//
//      noteOn(0x90, note, 0x45);                
//      delay(100);
//      noteOn(0x90, note, 0x00);  
//                
//      delay(500);
//        
//    }
    MidiUSB.poll();
}
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

    FlashLED4->start();
    FlashLED5->start();
    FlashLED6->start();     
    FlashLED7->start();
    FlashLED8->start();
    FlashLED9->start();
    FlashLED10->start();
    FlashLED11->start();
    
}

