
# USBMidiKliK4x4
A multi-port USB MIDI interface for the STM32DUINO platform.

Check also the wiki here : https://github.com/TheKikGen/USBMidiKliK4x4/wiki

The story of this project starts with a hack of the MIDIPLUS/MIDITECH 4x4 USB to MIDI interface.
Needing more midi jacks, I bought a second Miditech interface, but I discovered it was not possible to use 2 Miditech / Midiplus MIDI USB 4X4 on the same computer to get 8x8, and according to the Miditech support, as that usb midi interface was not updateable at all !
I was stucked....That was motivating me enough to write a totally new and better firmware : the UsbMidiKlik4x4 project was born.

The current version V2 supports full USB midi until 16xIN , 16XOUT plus routing features, enabling configurables standalone mode, merge mode, thru mode, split mode, etc., huge sysex flow, configuration menu from serial USB, and is very fast and stable thanks to the STM32F103.  

More of that, from the V2, you can aggregate until 5 3x3 boards seen as one by activating the "Bus mode". 
https://github.com/TheKikGen/USBMidiKliK4x4/wiki/USBMIDIKLIK-BUS-MODE-TO-AGGREGATE-5-boards-to-a-15x15-MIDI-USB-interface

<img  border="0" width=700 src="https://github.com/TheKikGen/USBMidiKliK4x4/blob/master/doc/USBMIDIKLIK-I2C-BUS-MODE.jpg?raw=true"  />

<a rel="license" href="http://creativecommons.org/licenses/by-nc/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc/4.0/">Creative Commons Attribution-NonCommercial 4.0 International License</a>.

## USBMidiKliK4x4 features

+ 16 MIDI IN / 16 MDI OUT Support 
+ Support 15 JACK IN / 15 JACK OUT in bus mode
+ Configuration with SYSEX and/or and interactive menu from a serial USB terminal
+ Complex routing rules also configurables by sysex or from an interactive user menu
+ Routing rules allowing MIDI MERGE, SPLIT easily on any ports available.
+ Ability to route any USB IN to any USB OUT/ MIDI OUT 
+ Midi messages filtering : channel voice, system common, realtime, sysex
+ Support of very big SYSEX dump files transmission
+ Hardware reset remotely by Sysex 
+ Sysex to switch to configuration menu serial mode instead of MIDI USB
+ USB device ProductStringName, Vendor and Product Ids can be changed easily
+ STANDALONE Mode :  can be used as a standalone MIDI routing box without connecting any host to the USB
+ Intellithru mode : possibility to define a second level of routing rules when USB is idle or unavailable
+ Stackable :  several interfaces can be "stacked" in bus mode , to get 6x6, 9x9, 15x15 physical midi ports, 16x16 USB.

Configuration is stored in EEPROM and persists after power off. 

## Bluepill and USBMidiKlik 3x3

<img width="250" border="0" src="https://2.bp.blogspot.com/-wo1H27RQYiU/XDzO9VG3vdI/AAAAAAAAAWA/KehLjyXhLTg_nmjjmEkO7LZtY5H83Rr-ACLcBGAs/s1600/20190113_221557.jpg"  />

I have realized a proto board, let's say a Bluepill "MIDI shield", to easily transform that cheap uC board to a powerfull 3 IN / 3 OUT USB MIDI interface, based on the exactly same firmware as USB MidiKlik 4x4.

The board is USB Powered, so no additional power supply necessary. It works driverless as a class compliant device, with Windows XP SP3, Vista 32 and 64 Bit, Windows 7 / 8 / 10   32 and 64 Bit, and Mac OS X, Linux ALSA, Akai MPC Live/X/Force, IOS, Android. 

I'm selling some proto PCB I still have on Ebay here https://www.ebay.fr/itm/303287160112

# USBMidiKliK4x4 - SYSEX V2

The system exclusive messages format is the following :

	F0 77 77 78 <sysex function id > <data> F7

The F0 77 77 78 is the specific sysex header for USBMidiKlik4x4. Know that it is a totally unofficial header.
IMPORTANT : INTERNAL SYSEX ARE ONLY INTERPRETED ON CABLE 0 OR MIDI IN JACK 1.  


## Fn 05 - Sysex configuration dump
    
    F0 77 77 78 05 F7

This command will dump all the configuration in a sysex form, allowing the user to save it in a file for backup purpose.

## Fn 06 - Universal Identity request

    F0 77 77 78 06 01 F7

will reply with the universal SYSEX identity answer :

    0xF0,
    0x7E,0x7F,0x06,0x02, SYSEX_MANUFACTURER_ID, 
    SYSEX_PRODUCT_FAMILY,SYSEX_MODEL_NUMBER,
    VERSION_MAJOR,VERSION_MINOR,
    0x00,0X00,
    0xF7

## Fn 0A - Hardware reset

To avoid unplugging the USB cable, you cand send this sysex that will do an harware reset programatically.  The full board and USB will be resetted. The sysex message is the following :

       F0 77 77 78 0A F7

## Fn 08 - Serial Configuration menu Bootmode

This sysex enables the configuration menu accessible from the USB serial.  Immediatly after sending this sequence, the interface reboots in CDC serial COM mode, allowing you to open a terminal to configure easily USBMIDIKLIK.

       F0 77 77 78 08 F7

The following menu should appear after connecting to the right serial USB port , and pressing ENTER :

	USBMIDIKLIK 4x4 - BLUEPILL STMF103C8x - V2.0
	(c) TheKikGen Labs
	https://github.com/TheKikGen/USBMidiKliK4x4

	[0] View global settings         [a] Show active devices
	[1] View midi routing            [d] SYSEX settings dump
	[2] Usb VID PID
	[3] Usb product string
	[4] Cable OUT routing            [e] Reload settings
	[5] Jack IN routing              [f] Factory settings
	[6] IntelliThru routing          [r] Factory routing
	[7] IntelliThru timeout          [s] Save settings
	[8] Toggle bus mode              [z] Debug on Serial3
	[9] Set device Id                [x] Exit
	=>

## Fn 0B - Change the device ProductStringName

It is posssible to change the USB device ProductStringName with a specific SYSEX (or from the configuration menu). The new name is saved in the flash memory immediatly after receiving the SYSEX, so it persists even after powering off the device.
The message structure is the following :

       F0 <USB MidiKlik 4x4 header = 0x77 0x77 0x78> <sysex fn id = 0x0b> <USB Midi Product name > F7

For example : the following SYSEX will change the name of the MIDI interface to "USB MidiKliK" :

       F0 77 77 78 0B 55 53 42 20 4D 69 64 69 4B 6C 69 4B F7

The product name is limited to 30 characters max, non accentuated (ascii code between 0 and 0x7F).

## Fn 0C - Change the USB VendorID and ProductID

In the same way, you can also change the USB Vendor and Product Ids with a SYSEX (or configuration menu). They are also saved in the flash memory and persist after power off. The sysex message structure is the following :

    F0 77 77 78 <func id = 0x0C> <n1n2n3n4 = Vendor Id nibbles> <n1n2n3n4 = Product Id nibbles> F7

As MIDI data are 7 bits bytes, a special encoding is used to handle VendorId and ProductID beeing 16 bits values.  To stay light, and because the message is very short, 2 x 16 bits values, the encoding will consists in sending each nibble (4 bits) serialized in a bytes train. For example sending VendorID and ProductID 0X8F12 0X9067 will be encoded as :
			
      0x08 0XF 0x1 0x2  0X9 0X0 0X6 0X7

so the complete SYSEX message will be :

      F0 77 77 78 0C 08 0F 01 02 09 00 06 07 F7

## Fn 0E - Intellithru midi routing rules

When USB midi is not active beyond a defined delay , the "intelligent" MIDI THRU can be activated automatically.
In that mode, the routing rules are changed to the routing rules defined for the thru mode.
If any USB midi event is received, the intelligent thru mode is stopped immediatly, and the standard routing is restored. 

    F0 77 77 78 0E  < Routing rules command <command args>   > F7

Commands are :
    
    00 Reset Intellithru to default
    01 Disable Intellithru
    02 Set  Intellithru timeout
          arg1 : 0xnn (number of 15s periods 1-127)
    03 Set thru mode jack routing
          arg1 - Midi In Jack = 0xnn (0-F)
          arg2 - Midi filter mask (binary OR)
                  => zero if you want to inactivate intelliThru for this jack
                     channel Voice = 0001 (1), (binary OR)
                     system Common = 0010 (2), (binary OR)
                     realTime      = 0100 (4), (binary OR)
                      sysEx         = 1000 (8)
         arg3 - Serial midi Jack out targets
               <t1> <t2>...<tn>  0-F 16 targets maximum.

The timeout is defined by a number of 15 seconds periods. The min/max period number is 1/127 (31 mn).  
After that delay, Every events from the MIDI INPUT Jack #n will be routed to outputs jacks 1-4, accordingly with the serial targets mask.  

Examples :

    F0 77 77 78 0E 00 F7                    <= Reset to default
    F0 77 77 78 0E 01 F7                    <= Disable
    F0 77 77 78 0E 02 02 F7                 <= Set timeout to 30s
    F0 77 77 78 0E 03 01 0F 00 01 02 03 F7  <= Route Midi In Jack 2 to Jacks out 1,2,3,4 All msg
    F0 77 77 78 0E 03 03 04 03 04 F7        <= Route Midi In Jack 4 to Jacks out 4,5, real time only


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

    F0 77 77 78 0F  < Routing rules command <command args>   > F7
    
    
Commands are :

    00 Reset to default midi routing
    01 Set routing +
        arg1 - source type : <0x00 usb cable | 0x01 jack serial>
        arg2 - port id : id for cable or jack serial (0-F)
        arg3 - destination = <0x00 usb cable in | 0x01 jack serial out>
        arg4 - targets : <port 0 1 2...n> 16 max (0-F)
    02  Midi filter
        arg1 - source type : : <0x00 usb cable | 0x01 jack serial>
        arg2 - port id : id for cable or jack serial (0-F)
        arg3 - midi filter mask (binary OR)
                  => zero if you want to inactivate intelliThru for this jack
                    channel Voice = 0001 (1), (binary OR)
                    system Common = 0010 (2), (binary OR)
                    realTime      = 0100 (4), (binary OR)
                    sysEx         = 1000 (8)

In bus mode, up to 16 targets by input (a cable USB OUT or a jack Serial MIDI IN) are possible.
IN standard mode, you can adress 4 cables and 3 to 4 jacks.

Examples :

    F0 77 77 78 0F 00 F7                      <= reset to default midi routing
    F0 77 77 78 0F 02 00 00 04 F7             <= Set filter to realtime events on cable 0
    F0 77 77 78 0F 01 00 00 01 00 01 F7       <= Set Cable 0 to Jack 1,2
    F0 77 77 78 0F 01 00 00 00 00 01 F7       <= Set Cable 0 to Cable In 0, In 01
    F0 77 77 78 0F 01 00 00 01 00 01 F7       <= & jack 1,2 (2 msg)
    F0 77 77 78 0F 01 01 01 01 00 01 02 03 F7 <= Set Serial jack In No 2 to serial jacks out 1,2,3,4

Default routing is :

       USB Cable OUT (0,F)   o------------->o MIDI OUT JACK (0,15) 
       USB Jack IN   (1,16)  o------------->o USB Cable IN (0,15)

The routing is saved in the flash memory, and is activated  immediatly after the update. So it persists after power off.
