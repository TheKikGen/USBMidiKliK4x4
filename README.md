
        __ __| |           |  /_) |     ___|             |           |
           |   __ \   _ \  ' /  | |  / |      _ \ __ \   |      _` | __ \   __|
           |   | | |  __/  . \  |   <  |   |  __/ |   |  |     (   | |   |\__ \
          _|  _| |_|\___| _|\_\_|_|\_\\____|\___|_|  _| _____|\__,_|_.__/ ____/

# USBMidiKliK4x4
A multi-port USB MIDI interface for the STM32DUINO platform.

Download the last firmware here : https://github.com/TheKikGen/USBMidiKliK4x4/releases

Check also the wiki here : https://github.com/TheKikGen/USBMidiKliK4x4/wiki

The story of this project starts with a hack of the MIDIPLUS/MIDITECH 4x4 USB to MIDI interface.
Needing more midi jacks, I bought a second Miditech interface, but I discovered it was not possible to use 2 Miditech / Midiplus MIDI USB 4X4 on the same computer to get 8x8, and according to the Miditech support, as that usb midi interface was not updateable at all !
I was stucked....That was motivating me enough to write a totally new and better firmware : the UsbMidiKlik4x4 project was born.

The current version V2 supports full USB midi until 16xIN , 16XOUT plus routing features, enabling configurables standalone mode, merge mode, thru mode, split mode, etc., huge sysex flow, configuration menu from serial USB, and is very fast and stable thanks to the STM32F103.  More of that, you can aggregate until 5 3x3 boards seen as one by activating the "Bus mode".

<a rel="license" href="http://creativecommons.org/licenses/by-nc/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc/4.0/">Creative Commons Attribution-NonCommercial 4.0 International License</a>.

[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=thekikgen@gmail.com&lc=FR&item_name=Donation+to+TheKikGen+projects&no_note=0&cn=&currency_code=EUR&bn=PP-DonationsBF:btn_donateCC_LG.gif:NonHosted)




## USBMidiKliK4x4 features

+ 16 MIDI IN / 16 MIDI OUT Support
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

<img  border="0" src="https://github.com/TheKikGen/USBMidiKliK4x4/blob/master/doc/USBMIDIKLIK-I2C-BUS-MODE.jpg?raw=true"  />

Configuration is stored in EEPROM and persists after power off.

Last release of the firmware can be found here : https://github.com/TheKikGen/USBMidiKliK4x4/releases/
Update tool is included. Unzip and launch the UMK_BluePillUpdate.bat file (Windows only currently).   


## Bluepill and USBMidiKlik 3x3

<img width="250" border="0" src="https://2.bp.blogspot.com/-wo1H27RQYiU/XDzO9VG3vdI/AAAAAAAAAWA/KehLjyXhLTg_nmjjmEkO7LZtY5H83Rr-ACLcBGAs/s1600/20190113_221557.jpg"  />

I have realized a proto board, let's say a Bluepill "MIDI shield", to easily transform that cheap uC board to a powerfull 3 IN / 3 OUT USB MIDI interface, based on the exactly same firmware as USB MidiKlik 4x4.

The board is USB Powered, so no additional power supply necessary. It works driverless as a class compliant device, with Windows XP SP3, Vista 32 and 64 Bit, Windows 7 / 8 / 10   32 and 64 Bit, and Mac OS X, Linux ALSA, Akai MPC Live/X/Force, IOS, Android.

Some units are currently available at Tindie :
<img width="50" border="0" src="https://d2ss6ovg47m0r5.cloudfront.net/images/tindie-logo@2x.png" />  
https://www.tindie.com/products/thekikgenlabs/usb-midi-interface-3-io-with-routing-features/


I have also designed a variant version of this PCB without DIN 5 female, to allow a deported DIN 5 rail.
Contact me for more details.

<img width="800" border="0" src="https://github.com/TheKikGen/USBMidiKliK4x4/blob/master/doc/20191215_173355.jpg?raw=true"  />

# USBMidiKliK4x4 - Configuration

2 modes can be used to configure the USBMidiKlik midi : system exclusive messages or a configuration menu.

Sysex messages have the following format, and ARE ONLY INTERPRETED ON CABLE 0 OR MIDI IN JACK 1 :

	F0 77 77 78 <sysex function id > <data> F7

The "F0 77 77 78" is the specific sysex header for USBMidiKlik4x4. Know that it is a totally unofficial header.
You will find the sysex V2 documentation in the wiki :

https://github.com/TheKikGen/USBMidiKliK4x4/wiki/USBMidiKliK4x4-V2-SYSEX-documentation

The second way is to activate the configuration menu on the serial port with the following sysex :

       F0 77 77 78 08 F7

This sysex enables the serial configuration menu.  To send it, you can use for example, MIDIOX, or the "sendmidi" command line utility by gbevin :

       sendmidi dev "USB MIDIKliK 4x4" syx hex 77 77 78 08

You will find the last binary release of sendmidi here :  https://github.com/gbevin/SendMIDI/releases
       
Immediatly after sending this sequence with your prefered midi software, the interface will reboot in CDC serial COM mode, allowing you to open a terminal to configure easily USBMIDIKLIK. 
You can use any software like TERATERM, on Windows, or "screen" on MacOs.  The bauds rate must be set to 115200.

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

# NEWS

## Midi transformation pipelines (22 March 2020) ###

I'm currently working on a powerfull new feature allowing to modify an incoming midi message via a "pipeline". A pipeline can be seen as a chain of transformation functions, e.g.,  transpose notes, split, map channel to another, map CC to another, etc....  
The RC1 will be delivered around end of March.  Have a look at the MIDITRANS branch.

## Build 2.191207.1818 is ready (07 December 2019) ###

Update tool is included.  Unzip and launch the UMK_BluePillUpdate.bat file (Windows only currently).
The libusb driver is also provided with this package if it is not already installed on your PC.
https://github.com/TheKikGen/USBMidiKliK4x4/releases/tag/build_2.191207.1818

## THE V2 IS MERGED WITH THE MASTER BRANCH AS RC3.   

That major release offers now the possibility to aggregate until 5 Bluepill boards to get a 15x15 IN/OUT ports with routing....
https://github.com/TheKikGen/USBMidiKliK4x4/wiki/USBMIDIKLIK-BUS-MODE-TO-AGGREGATE-5-boards-to-a-15x15-MIDI-USB-interface

## MPC LIVE/X Midi ports hack has been moved to MPC-LiveXplore
https://github.com/TheKikGen/MPC-LiveXplore/wiki/Add-2-new-midi-ports-C-D---a-quick-guide-with-MIDI-OX

