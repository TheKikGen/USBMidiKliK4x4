# USBMidiKliK4x4
A hack of the MIDIPLUS/MIDITECH 4x4 USB to MIDI interface

## The idea
I currently own 2 MIDI 4X4 from Miditech/Miduplus manufacturer. 
I thought it was possible to stack the 2 on the same PC to get 8x2 ports IN/Out.
But as theses devices have exactly the same product and vendor ID, and no serial, it is in fact impossible to do that.
So the only alternative was to buy a 8x8 interface with the same manufacturer, for the three times the price of the MIDI 4x4 that I did not.
<img border="0" src="https://medias.audiofanzine.com/images/normal/miditech-midiface-4x4-1642123.jpg"  />

More, recently, one of my interface started to work incorrectly, invoking to me a corrupted firmware. 
I asked to the Miditech and Midiplus support and they were enough kind to provide to me an updated firmware kit, but for the NEW version...
Happy to learn that my interface was the "old" one, however still sold on many web sites today...I tested that update package 
without any success...My version was not updatable at all they said.

I could'nt accept that a device (almost) still in order with USB,  8 ports 4IN - 4 OUT goes directly in the trash bin. 
Nothing to loose, I opened the box, and inside I discovered that the microcontroller was a STMF103RC, a very common uC on musical gears 
(the Arturia Minilab and the Novation launchkey and launchpad are using it for example). 
Here start the idea of a possible hack....

=> I have a board available with a programmable and powerfull chip,  native USB, plus all the circuitry for 4 MIDI IN / 4OUT. 
Why not rewriting a new firmware from scratch as I did for other uC like of the AVR family ?
I could even extend thtat firmware to enable merge mode, thru mode, specific routing and filtering modes. 
So, building an ultimate USB MIDI interface better than the original....

## First steps

The question : deso the interface issue was really a firmware corruption problem or a more serious one ?
The STMF103 has an internal bootloader working with the UART Serial 1. The bootloader mode is activated by mainting 
the Boot0 pin to HIGH and the Boot1 pin to LOW.  So I desoldered 2 resitors on the board because they were disabling that mode, 
made a small reset / boot1 HIGH circuit, and soldered the boot 0 to ground.  

I also connected TX and RX from the serial 1 to a small plug. That was easy because some large labelled TXn // RXnpads pads exist on the board.
MIDI4x4 board.  I connected that plug to an USB Serial TTL, and 2 H later, i was ready to upload a new firmware in the thing.

I started with a basic demo UART firmware "Hello world" sending a text on the Serial port 1 at 115200.  
And guess what : that worked at the first time  !

To preserve and reuse my existing software libraries, I choose to use STMDUINO, a very clever port of the STM32F 
ARM architecture to the Arduino.  I  have downlaoded the STMDuino bootloader2.0 and tested the 4x4 board as a STMF103RC with 
the Arduino IDE... Again, my MIDI demo sketch worked at the first compilation.

Now ready to go ! 
Features I plan to develop :

- Reproduce the standard behaviuour of the MIDI 4X4 original interface
- Full MIDI compliance (Running status, SYSEX...).
- Dual Bootloader supporting USB MIDI and DFU Mode to upgrade firmware easily
- Routing function via SYSEX :
. Merge : n MIDI IN merged to nMIDI OUT
. Thru  : MIDI OUT to n MIDI IN








