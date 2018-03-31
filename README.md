# USBMidiKliK4x4
A hack of the MIDIPLUS/MIDITECH 4x4 USB to MIDI interface

<img border="0" src="https://medias.audiofanzine.com/images/normal/miditech-midiface-4x4-1642123.jpg"  />

It is not possible (but some were successfull) to use 2 Miditech / Midiplus MIDI USB 4X4 on the same computer, because these device can't share the same product/vendor ID and serial without USB conflict. More, that usb midi interface is not updateable at all, according to my talk with the Miditech support.

The MCU of the Miditech / Midiplus 4x4 midi interface is a a STMF103RC, a very common uC ARM STM32 chip family used on most musical gears like the Arturia Minilab, the Novation Launchkey and Launchpad for example. 

To preserve and reuse my existing software libraries, I choose to use STMDUINO, a port of the Arduino platform to the STM32 family ARM architecture. By activating the STM32F103 bootloader mode maintaning the Boot0 pin to HIGH and the Boot1 pin to LOW (you need to desolder 2 resistors on the board because they are disabling that mode) you can upload though the UART1, a STM32DUINO generic bootloader, and use that interface as an "Arduino board" to program your own firmware with the Arduino standard IDE.

I have written this firmware entirely from scratch, without hacking anything from Miditech / Midiplus. 
The current version supports full USB midi 4xIN , 4XOUT plus routing features, big sysex, and is very stable.
Mdi routing enables "4 merge" mode, thru mode, split mode, etc.  

So I can say my own firmware modified USB interface 4X4 is now better than the original, and, the most important stackable with my existing one as I changed, obviously,  the Product ID !  

And the STM32F103 is really really fast...no lag at all, even at 300 BPM with 4IN/OUT working.

The code is easily adaptable to any other multi-jack USB interface.

## USB Midi hard reset with an internal SYSEX

To avoid unplugging the USB cable, you cand send this sysex TO A MIDI IN JACK (USB not implemented) that will do an harware reset programatically.  The full board and USB will be resetted. The sysex message structure is the following :

       F0 77 77 78 <sysex function id = 0x0A> F7
       
## Define router midi target with internals SYSEX

You can change the behaviour of the routing from USB to serial, USB to USB, serial to USB, serial to serial.
2 routing tables are availables :

- USB Cables IN (USB MIDI OUT considering the Host point of view) to any USB OUT (Host IN) cables and/or MIDI jacks OUT 
- Serial Jacks MIDI IN to any USB OUT (Host IN) cables and/or MIDI jacks OUT 
                                                                          
                  Source                                    Destination   Target byte 
                                                                            bit
         USB  Host MIDI OUT 1 o-----------------+       o Host MIDI IN 1     7                             
       Cables Host MIDI OUT 2 o--------------+  |       o Host MIDI IN 2     6      USB
              Host MIDI OUT 3 o-----------+  |  |       o Host MIDI IN 3     5     Cables
              Host MIDI OUT 4 o---------+ |  |  |       o Host MIDI IN 4     4
                                        | |  |  |       
                                        | |  |  +-------o MIDI OU JACK 1     3
              MIDI IN Jack 1  o         | |  +----------o MIDI OU JACK 2     2     Serial 
       Serial MIDI IN Jack 2  o         | +-------------o MIDI OU JACK 3     1
              MIDI IN Jack 3  o         +---------------o MIDI OU JACK 4     0
              MIDI IN Jack 4  o

fTo avoid unplugging the USB cable, you cand send this sysex TO A MIDI IN JACK (USB not implemented) that will do an harware reset programatically.  The full board and USB will be resetted. The sysex message structure is the following :



