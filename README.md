# USBMidiKliK4x4
A hack of the MIDIPLUS/MIDITECH 4x4 USB to MIDI interface

Chack also the wiki here : https://github.com/TheKikGen/USBMidiKliK4x4/wiki

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

## Changing the device ProductStringName with an internal SYSEX

it is posssible to change the USB device ProductStringName with a specific SYSEX. The new name is saved in the flash memory immediatly after receiving the SYSEX, so it persists even after powering off the device.   The message structure is the following :

       F0 <USB MidiKlik 4x4 header = 0x77 0x77 0x78> <sysex fn id = 0x0b> <USB Midi Product name > F7

Only Serial is parsed (but USB will be in a next version), so you must send the SYSEX to a MIDI IN jack.
You can use a tool like MIDI-OX to do that easily :
- connect the device to USB 
- connect the MIDIOUT JACK 1 to the MIDI IN JACK 1
- Open MIDI-OX and connect the USBMidiKliK MIDI-OUT 1 in the MIDI output device dialog box
- Open the SysEx windows in the "View" menu
- Enter the SYSEX msg in the command window and click on "Send Sysex" in the "CommandWindow" menu.  
- Unplug/plug the USB cable or send a Midi USB HardReset sysex (see below)
- Quit and reopen MIDI-OX and you should see a new name in the MIDI devices dialog box.

For example : the following SYSEX will change the name of the MIDI interface to "USB MidiKliK" :

       F0 77 77 77 0B 55 53 42 20 4D 69 64 69 4B 6C 69 4B F7

The product name is limited to 30 characters max, non accentuated (ascii code between 0 and 0x7F).

## Changing the USB VendorID and ProductID with an internal SYSEX

In the same way, you can also change the USB Vendor and Product Ids with a SYSEX. They are also saved in the flash memory and persist after power off. The sysex message structure is the following :

    F0 77 77 78 <func id = 0x0C> <n1n2n3n4 = Vendor Id nibbles> <n1n2n3n4 = Product Id nibbles> F7

As MIDI data are 7 bits bytes, a special encoding is used to handle VendorId and ProductID beeing 16 bits values.  To stay light, and because the message is very short, 2 x 16 bits values, the encoding will consists in sending each nibble (4 bits) serialized in a bytes train. For example sending VendorID and ProductID 0X8F12 0X9067 will be encoded as :
			
      0x08 0XF 0x1 0x2  0X9 0X0 0X6 0X7

so the complete SYSEX message will be :

      F0 77 77 78 0C 08 0F 01 02 09 00 06 07 F7

## Set the "intelligent MIDI Thru" 

When USB midi is not active beyond a defined delay in seconds, the "intelligent" MIDI THRU can be activated automatically.
In that mode, all midi messages received on the selected MIDI IN jack are broadcasted to all jacks outputs (1 to 4). 
If any USB midi event is received, the intelligent thru mode is stopped immediatly, and the standard routing is restored.
The sysex message structure is the following :

    F0 77 77 78 <func id = 0x0E> <n = MIDI IN Jack #, 1-4> <delay from 10 to 127> F7

The minimum delay is 10 seconds. The max is 127 seconds.  If the delay is zero, the midi thru mode is disabled.
For example, to set the MIDI IN 3 jack to be the input, when the delay is 2 mn (120 seconds = 0x78) :

    F0 77 77 78 0E 03 78 F7

## Define router midi target with internals SYSEX

You can change the behaviour of the routing from USB to serial, USB to USB, serial to USB, serial to serial.
2 routing tables are availables :

- USB Cables IN (USB MIDI OUT considering the Host point of view) to any USB OUT (Host IN) cables and/or MIDI jacks OUT 
- Serial Jacks MIDI IN to any USB OUT (Host IN) cables and/or MIDI jacks OUT 

Example of routing :
                  
      Inputs        Source                                  Routing Targets            Target byte
                                                                                           bit
       USB       Host MIDI OUT 1 o-----------------+  +----o Host MIDI IN 1 (cable 0)    4                           
       Cables    Host MIDI OUT 2 o--------------+  |  |    o Host MIDI IN 2 (cable 1)    5   USB
                 Host MIDI OUT 3 o-----------+  |  |  |    o Host MIDI IN 3 (cable 2)    6   Cables
                 Host MIDI OUT 4 o---------+ |  |  |  |    o Host MIDI IN 4 (cable 3)    7
                                     +-+---|-|--|--|--+ 
                                     | |   | |  |  +-------o MIDI OUT JACK 1             0
                 MIDI IN Jack 1  o---+ |   | |  +----------o MIDI OUT JACK 2             1   Serial 
       Serial    MIDI IN Jack 2  o-----+   | +-------------o MIDI OUT JACK 3             2
                 MIDI IN Jack 3  o         +---------------o MIDI OUT JACK 4             3
                 MIDI IN Jack 4  o

To configure the routing for an input, you must set some bits of the target byte to 1 :
Bits 0-3 are corresponding repesctively to Serial Midi out Jack targets 1-4
Bits 4-7 are corresponding respectively to USB Cables targets IN 0-3.

Sysex message structure :

              F0 77 77 78   <0x0F> 
                            <0x00 = reset | 0x01 = set> 
                            <0X0 = cable  | 0x01 = serial> 
                            <id:0-4> 
                            <target nibble cable> 
                            <target nibble serial> 
              F7
                     
For example, the following routing rule set MIDI IN JACK1/JACK2 to be merged to cable 0 :

       F0 77 77 78 0F 01 01 00 01 00 F7
       F0 77 77 78 0F 01 01 01 01 00 F7
       
The following sysex will restore default routing for all inputs :

       F0 77 77 78 0F 00 F7

Default routing is the following :

       USB MIDI OUT 1 o------------->o MIDI OUT JACK 1 
       USB MIDI OUT 2 o------------->o MIDI OUT JACK 2 
       USB MIDI OUT 3 o------------->o MIDI OUT JACK 3 
       USB MIDI OUT 4 o------------->o MIDI OUT JACK 4 

       USB MIDI IN  1 o<-------------o MIDI IN JACK 1 
       USB MIDI IN  2 o<-------------o MIDI IN JACK 2 
       USB MIDI IN  3 o<-------------o MIDI IN JACK 3 
       USB MIDI IN  4 o<-------------o MIDI IN JACK 4 

The new routing is saved in the flash memory immediatly after the update. So it persists after power off.
