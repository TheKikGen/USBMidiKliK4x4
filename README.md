# USBMidiKliK4x4 Dual - STMDUINO
A multi-port USB MIDI interface for the STMDUINO platform.

Check also the wiki here : https://github.com/TheKikGen/USBMidiKliK4x4/wiki

<img border="0" src="https://medias.audiofanzine.com/images/normal/miditech-midiface-4x4-1642123.jpg"  />

The story of this project starts with a hack of the MIDIPLUS/MIDITECH 4x4 USB to MIDI interface.
Needing more midi jacks, I bought a second Miditech interface, but I discovered it was not possible to use 2 Miditech / Midiplus MIDI USB 4X4 on the same computer to get 8x8. This is mainly because of identical product/vendor ID and serial, and , according to the Miditech support, as that usb midi interface is not updateable at all, I was stucked....That was motivating me enough to go deep in the detail, and try, at less to change the PID or VID.

The MCU of the Miditech / Midiplus 4x4 midi interface is a high density STM32F103RC, a common uC ARM STM32F1 chip family used on most musical gears like the Arturia Minilab, the Novation Launchkey and Launchpad for example. It is a very powerful chip, especially when you compare it to the Arduino Uno (ATMEGA328P). 

As a former AVR platform developper, I choose to use STMDUINO to preserve a part of my knowledge and to reuse my existing software libraries.  STMDUINO is a port of the famous Arduino platform for the Maple Mini and other STM32 F1 and F4 boards, continuing by Roger Clark where Leaflabs left off.

To hack the Miditech interface, I had to remove some resistors on the motherboard because they were disabling the low level bootloader thought the UART1. After an upload of a STM32DUINO generic bootloader, I was able to compile and load from the USB a "blink" hello world sketch with the Arduino standard IDE.

## USBMidiKliK4x4 firmware

This firmware was entirely written from scratch, without hacking, or reverse-engineering anything from Miditech / Midiplus. 
The current version supports full USB midi 4xIN , 4XOUT plus routing features, enabling configurables standalone mode, "4 merge" mode, thru mode, split mode, etc., huge sysex flow, configuration menu from serial USB, and is very fast and stable thanks to the STM32F103. 

So I can say my modified Miditech USB interface is now better than the original, and, the most important, stackable with my other one as I changed, obviously,  the Product ID !  I use them almost everyday. 

The code was mainly adapted from my other single USBMidiKlik project, developed on the AVR platform.

# USBMidiKliK4x4 Dual - SYSEX

## USB Midi hard reset with an internal SYSEX

To avoid unplugging the USB cable, you cand send this sysex that will do an harware reset programatically.  The full board and USB will be resetted. The sysex message structure is the following :

       F0 77 77 78 <sysex function id = 0x0A> F7

## USB Midi reboot in USB serial config menu mode

This sysex enables the configuration menu accessible from USB serial.  Immediatly after sending this sequence, the interface reboots in serial COM mode, allowing you to open a terminal to configure easily USBMIDIKLIK.

       F0 77 77 78 <sysex function id = 0x08> F7

When in serial mode, the menu is the following :

	USBMIDIKliK 4x4 MENU
	(c)TheKikGen Labs                                                                             

	0.Show current settings                                                         
	1.Reload settings                                                               
	2.USB product string                                                            
	3.USB Vendor ID & Product ID                                                    
	4.Intelligent Midi Thru MIDI filters                                            
	5.Intelligent Midi Thru delay for USB timeout                                   
	6.Intelligent Midi Thru IN Jack routing                                         
	7.Midi USB Cable OUT routing                                                    
	8.Midi IN Jack routing                                                          
	9.Reset routing to factory default                                              
	s.Save & quit                                                                   
	x.Abort                                                                         
	=>         

## Changing the device ProductStringName

it is posssible to change the USB device ProductStringName with a specific SYSEX (or from the configuration menu). The new name is saved in the flash memory immediatly after receiving the SYSEX, so it persists even after powering off the device.   The message structure is the following :

       F0 <USB MidiKlik 4x4 header = 0x77 0x77 0x78> <sysex fn id = 0x0b> <USB Midi Product name > F7

For example : the following SYSEX will change the name of the MIDI interface to "USB MidiKliK" :

       F0 77 77 77 0B 55 53 42 20 4D 69 64 69 4B 6C 69 4B F7

The product name is limited to 30 characters max, non accentuated (ascii code between 0 and 0x7F).

## Changing the USB VendorID and ProductID

In the same way, you can also change the USB Vendor and Product Ids with a SYSEX (or configuration menu). They are also saved in the flash memory and persist after power off. The sysex message structure is the following :

    F0 77 77 78 <func id = 0x0C> <n1n2n3n4 = Vendor Id nibbles> <n1n2n3n4 = Product Id nibbles> F7

As MIDI data are 7 bits bytes, a special encoding is used to handle VendorId and ProductID beeing 16 bits values.  To stay light, and because the message is very short, 2 x 16 bits values, the encoding will consists in sending each nibble (4 bits) serialized in a bytes train. For example sending VendorID and ProductID 0X8F12 0X9067 will be encoded as :
			
      0x08 0XF 0x1 0x2  0X9 0X0 0X6 0X7

so the complete SYSEX message will be :

      F0 77 77 78 0C 08 0F 01 02 09 00 06 07 F7

## Set the "intelligent MIDI Thru" 

When USB midi is not active beyond a defined delay , the "intelligent" MIDI THRU can be activated automatically.
In that mode, all midi messages received on the selected MIDI IN jacks are broadcasted to jacks outputs (1 to 4) accordingly to the serial bits mask specified.  If any USB midi event is received, the intelligent thru mode is stopped immediatly, and the standard routing is restored. The sysex message structure is the following :

    	F0 77 77 78 	<func id = 0x0E> 
    			< (bits 4-7 = Midi In Jack 1-4) (bits 0-3 = midiMsg filter mask) >
			<serial Midi out bit mask 1-F>
			<n = nb of 15s periods, 0-127> 
	F7
Midi Msg filter masks (can be combined but can't be zero) are  b0 = channel Voice, b1 = system Common, b2=realTime, b3=sysEx;		
The delay is defined by a number of 15 seconds periods. The min/max period number is 1-127 (31 mn).  
After that delay, Every events from the MIDI INPUT Jack #n will be routed to outputs jacks 1-4, accordingly with the serial targets mask. For example, to set the MIDI IN 3 jack to be the input, realtime msg only, 4 outputs, 2 mn delay (8 periods) :

	F0 77 77 78 0E 34 0F 08 F7

## Define midi routing rules with internals SYSEX

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

## Using another STMF32x board

You can obviously use this project to build a midi interface with other STM32F boards. 
I have succesfully tested USBMIDIKLIK4X4 on a 2$ "Blue pill", allowing 3x3 serial midi I/O. 
https://wiki.stm32duino.com/index.php?title=Blue_Pill

<img border="0" src="https://4.bp.blogspot.com/-2nP69Lwl-dU/WhrncwR_WdI/AAAAAAAAIAE/ugo2ail4EdAXxgveZqc_jh9kwQU6PXiUwCLcBGAs/s1600/stm32-arduino-ide.jpg"  />


