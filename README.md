### THE V2 IS AT RC1 STAGE.   

That major release offers the possibility to aggregate until 5 Bluepill boards to get a 15x15 IN/OUT ports with routing....
https://github.com/TheKikGen/USBMidiKliK4x4/wiki/USBMIDIKLIK-BUS-MODE-TO-AGGREGATE-5-boards-to-a-15x15-MIDI-USB-interface

NEWS : I'm selling proto PCB on Ebay here https://www.ebay.fr/itm/303287160112


# USBMidiKliK4x4
A multi-port USB MIDI interface for the STM32DUINO platform.

Check also the wiki here : https://github.com/TheKikGen/USBMidiKliK4x4/wiki

<a rel="license" href="http://creativecommons.org/licenses/by-nc/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc/4.0/">Creative Commons Attribution-NonCommercial 4.0 International License</a>.

The story of this project starts with a hack of the MIDIPLUS/MIDITECH 4x4 USB to MIDI interface.
Needing more midi jacks, I bought a second Miditech interface, but I discovered it was not possible to use 2 Miditech / Midiplus MIDI USB 4X4 on the same computer to get 8x8, and according to the Miditech support, as that usb midi interface was not updateable at all !
I was stucked....That was motivating me enough to write a totally new and better firmware : the UsbMidiKlik4x4 project was born.

## Bluepill and USBMidiKlik 3x3

I have realized a proto board, let's say a Bluepill "MIDI shield", to easily transform that cheap uC board to a powerfull 3 IN / 3 OUT USB MIDI interface, based on the exactly same firmware as USB MidiKlik 4x4.  Contact me for more details.

<img width="250" border="0" src="https://2.bp.blogspot.com/-wo1H27RQYiU/XDzO9VG3vdI/AAAAAAAAAWA/KehLjyXhLTg_nmjjmEkO7LZtY5H83Rr-ACLcBGAs/s1600/20190113_221557.jpg"  />

## USBMidiKliK4x4 firmware

This firmware was entirely written from scratch, without hacking, or reverse-engineering anything from Miditech / Midiplus. 
The current version supports full USB midi 4xIN , 4XOUT plus routing features, enabling configurables standalone mode, "4 merge" mode, thru mode, split mode, etc., huge sysex flow, configuration menu from serial USB, and is very fast and stable thanks to the STM32F103. 

So I can say my modified Miditech USB interface is now better than the original, and, the most important, stackable with my other one as I changed, obviously,  the Product ID !  I use them almost everyday. 

The code was mainly adapted from my other single USBMidiKlik project, developed on the AVR platform.

# USBMidiKliK4x4 Dual - SYSEX

The system exclusive messages format is the following :

	F0 77 77 78 <sysex function id > <data> F7

The F0 77 77 78 is the specific sysex header for USBMidiKlik4x4. Know that it is a totally unofficial header.
IMPORTANT : INTERNAL SYSEX ARE ONLY INTERPRETED ON CABLE 0 OR MIDI IN JACK 1.  

## Hardware reset (function 0x0A)

To avoid unplugging the USB cable, you cand send this sysex that will do an harware reset programatically.  The full board and USB will be resetted. The sysex message is the following :

       F0 77 77 78 0A F7

## Serial Configuration menu Bootmode (function 0x08)

This sysex enables the configuration menu accessible from the USB serial.  Immediatly after sending this sequence, the interface reboots in CDC serial COM mode, allowing you to open a terminal to configure easily USBMIDIKLIK.

       F0 77 77 78 08 F7

The following menu should appear after connecting to the right serial USB port , and pressing ENTER :

	USBMIDIKliK 4x4 MENU - BLUEPILL                                                 
	(c)TheKikGen Labs                                                               

	0.Show current settings              e.Reload settings from EEPROM              
	1.Midi USB Cable OUT routing         f.Restore all factory settings             
	2.Midi IN Jack routing               r.Reset routing to factory default         
	3.Intelligent Thru IN Jack routing   s.Save & quit                              
	4.Intelligent Thru USB timeout       x.Abort                                    
	5.USB Vendor ID & Product ID                                                    
	6.USB product string                                                            

	=>s                                                                             
	-===========================================-                                  
			CURRENT SETTINGS                                                
	---------------------------------------------                                   
	Magic number   : MDK7-1.181218.1605                                             
	Next BootMode  : 2                                                              
	Vendor Id      : 2912                                                           
	Product Id     : 1970                                                           
	Product string : USB MIDIKliK 4x4                                               
	Sysex header   : F0 77 77 78                                                    

	-===========================================-                                   
	|                MIDI ROUTING               |                                   
	|-------------------------------------------|                                   
	| Cable| Msg Filter   | Cable IN | Jack OUT |                                   
	| OUT# | Ch Sc Rt Sx  | 1 2 3 4  | 1 2 3 4  |                                   
	|------+--------------+----------+----------|                                   
	|  1-> |  X  X  X  X  | . . . .  | X . . .  |                                   
	|  2-> |  X  X  X  X  | . . . .  | . X . .  |                                   
	|  3-> |  X  X  X  X  | . . . .  | . . X .  |                                   
	|  4-> |  X  X  X  X  | . . . .  | . . . X  |                                   
	|-------------------------------------------|                                   
	| Jack | Msg Filter   | Cable IN | Jack OUT |                                   
	| IN # | Ch Sc Rt Sx  | 1 2 3 4  | 1 2 3 4  |                                   
	|------+--------------+----------+----------|                                   
	|  1-> |  X  X  X  X  | X . . .  | . . . .  |                                   
	|  2-> |  X  X  X  X  | . X . .  | . . . .  |                                   
	|  3-> |  X  X  X  X  | . . X .  | . . . .  |                                   
	|  4-> |  X  X  X  X  | . . . X  | . . . .  |                                   
	|-------------------------------------------|                                   
	|      Intelligent Thru mode (inactive)     |                                   
	|-------------------------------------------|                                   
	| Jack | Msg Filter   |          | Jack OUT |                                   
	| IN # | Ch Sc Rt Sx  | (No USB) | 1 2 3 4  |                                   
	|------+--------------+----------+----------|                                   
	|  .-> |  X  X  X  X  |          | X . . .  |                                   
	|  .-> |  X  X  X  X  |          | X X . .  |                                   
	|  .-> |  X  X  X  X  |          | X X X .  |                                   
	|  .-> |  X  X  X  X  |          | X X X X  |                                   
	-===========================================-                                   
	  Intelligent Midi Thru USB timeout : 30s                                       
	-===========================================-  

## Change the device ProductStringName

It is posssible to change the USB device ProductStringName with a specific SYSEX (or from the configuration menu). The new name is saved in the flash memory immediatly after receiving the SYSEX, so it persists even after powering off the device.
The message structure is the following :

       F0 <USB MidiKlik 4x4 header = 0x77 0x77 0x78> <sysex fn id = 0x0b> <USB Midi Product name > F7

For example : the following SYSEX will change the name of the MIDI interface to "USB MidiKliK" :

       F0 77 77 78 0B 55 53 42 20 4D 69 64 69 4B 6C 69 4B F7

The product name is limited to 30 characters max, non accentuated (ascii code between 0 and 0x7F).

## Change the USB VendorID and ProductID

In the same way, you can also change the USB Vendor and Product Ids with a SYSEX (or configuration menu). They are also saved in the flash memory and persist after power off. The sysex message structure is the following :

    F0 77 77 78 <func id = 0x0C> <n1n2n3n4 = Vendor Id nibbles> <n1n2n3n4 = Product Id nibbles> F7

As MIDI data are 7 bits bytes, a special encoding is used to handle VendorId and ProductID beeing 16 bits values.  To stay light, and because the message is very short, 2 x 16 bits values, the encoding will consists in sending each nibble (4 bits) serialized in a bytes train. For example sending VendorID and ProductID 0X8F12 0X9067 will be encoded as :
			
      0x08 0XF 0x1 0x2  0X9 0X0 0X6 0X7

so the complete SYSEX message will be :

      F0 77 77 78 0C 08 0F 01 02 09 00 06 07 F7

## Set the "intelligent MIDI Thru" 

When USB midi is not active beyond a defined delay , the "intelligent" MIDI THRU can be activated automatically.
In that mode, the routing rules are changed to the routing rules defined for the thru mode.
If any USB midi event is received, the intelligent thru mode is stopped immediatly, and the standard routing is restored. 

The sysex message structure is the following :

	Header       = F0 77 77 78
	Function     = 0E
	Action       = 00 Reset to default
		  OR   01 Disable
		  OR   02 Set Delay <number of 15s periods 1-127>
		  OR   03 Set thu mode jack routing +
		  		. Midi In Jack = < Midi In Jack # 1-4 = 0-3>
				. Midi Msg filter mask (can't be zero) :
				. Serial midi Jack out targets Mask 4bits 1-F
		                       Bit0 = Jack1, bit 3 = Jack 4
	EOX = F7
	
Message filter masks (can be combined but can't be zero) are :

	b0 = channel Voice    (0001)
	b1 = system Common    (0010)
	b2 = realTime         (0100)
	b3 = system exlcusive (1000)		
	
The delay is defined by a number of 15 seconds periods. The min/max period number is 1/127 (31 mn).  
After that delay, Every events from the MIDI INPUT Jack #n will be routed to outputs jacks 1-4, accordingly with the serial targets mask.  Examples :

	F0 77 77 78 0E 00 F7    <= Reset to default 
	F0 77 77 78 0E 01 F7    <= Disable midi thru mode
	F0 77 77 78 0E 02 02 F7 <= Set delay to 30s (2 periods)
	F0 77 77 78 0E 03 01 0F 0F F7 <= Set Midi In Jack 2 to Jacks out 1,2,3,4 All msg
	F0 77 77 78 0E 03 03 04 0C F7 <= Set Midi In Jack 4 to Jack 3,4, real time only

## Define midi routing rules

You can change the behaviour of the MIDI routing from MIDI USB to MIDI serial, USB to USB, serial to USB, serial to serial.
2 routing tables are availables :

- USB Cables IN (USB MIDI OUT considering the Host point of view) to any USB OUT (Host IN) cables and/or MIDI jacks OUT 
- Serial Jacks MIDI IN to any USB OUT (Host IN) cables and/or MIDI jacks OUT 

Example of routing :
                  
      Inputs        Source                                  Routing Targets             Target byte
                                                                                            bits
       USB       Host MIDI OUT 1 o-->(filter)---------+  +----o Host MIDI IN 1 (cbl 0) 4                           
       Cables    Host MIDI OUT 2 o-->(filter)------+  |  |    o Host MIDI IN 2 (cbl 1) 5   USB
       OUT       Host MIDI OUT 3 o-->(filter)---+  |  |  |    o Host MIDI IN 3 (cbl 2) 6   Cables
                 Host MIDI OUT 4 o-->(filter)-+ |  |  |  |    o Host MIDI IN 4 (cbl 3) 7   IN
                                        +-+---|-|--|--|--+  
                                        | |   | |  |  +-------o MIDI OUT JACK 1        0
                 MIDI IN Jack 1  o--(f)-+ |   | |  +----------o MIDI OUT JACK 2        1   Serial 
       Serial    MIDI IN Jack 2  o--(f)---+   | +-------------o MIDI OUT JACK 3        2   Jacks
        Jacks    MIDI IN Jack 3  o            +---------------o MIDI OUT JACK 4        3   OUT
       IN        MIDI IN Jack 4  o

The sysex message structure is the following :

	Header       = F0 77 77 78	
	Function     = 0F
	Action       = <00 Reset to default midi routing>
		   OR  <01 Set routing +
		   		. source type     = <cable=0X0 | serial=0x1>
				. id              = id for cable or serial 0-3
				. Midi Msg filter mask
				. routing targets = <cable mask> , <jack serial mask>
	EOX 	     = F7

8 targets by input (a cable USB OUT or a jack Serial MIDI IN) are possible :
. USB cable IN 0 to 3
. Jack midi OUT to an external gear 1 to 4

Routing targets tables are stored in 2 bytes / 8 bits, 1 for each input. Bit 0 is starting at the top right. 
. Bits 0-3 are corresponding respectively to Serial Midi out Jack targets 1-4
. Bits 4-7 are corresponding respectively to USB Cables targets IN 0-3.

To configure the routing rule for an input, you must set some bits of the target byte to 1. For example,
the mask 01010001 will activate the cable out 0 and 2, and jack serial 1.

Message filter is defined as the midi thu mode (see above).

Some examples :

	F0 77 77 78 0F 00 F7                <= reset to default midi routing
	F0 77 77 78 0F 01 00 00 0F 00 03 F7 <= Set Cable 0 to Jack 1,2, all midi msg
	F0 77 77 78 0F 01 00 00 0F 01 03 F7 <= Set Cable 0 to Cable In 0, Jack 1,2, all midi msg
	F0 77 77 78 0F 01 01 01 04 00 0F F7 <= Set Serial jack In 2 to all serial jack out, realtime msg only
	F0 77 77 78 0F 01 01 00 01 03 03 F7 <= Set Serial jack In 1 to 1,2 jack out,cable in 0,1, channel voice msg only

Default routing is :

       USB Cable OUT (0,3)  o------------->o MIDI OUT JACK (1,4) 
       USB Jack IN   (1,4)  o------------->o USB Cable IN (0,3)

The new routing is saved in the flash memory, and is activated  immediatly after the update. So it persists after power off.

## MPC LIVE midi port C & D enabling

Here is the SYSEX to emulate an Akai internal USB midi interface and get MPC Live port C+D accessible in standalone mode.

	F0 77 77 78 0C 00 09 0E 08 00 00 03 0B F7 F0 77 77 78 0B 4D 50 43 20 4C 69 76 65 20 43 6F 6E 74 72 6F 6C 6C 65 72 F7

The sequence must be sent to board via CABLE 0 OR MIDI IN JACK 1. 
