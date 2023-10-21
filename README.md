
        __ __| |           |  /_) |     ___|             |           |
           |   __ \   _ \  ' /  | |  / |      _ \ __ \   |      _` | __ \   __|
           |   | | |  __/  . \  |   <  |   |  __/ |   |  |     (   | |   |\__ \
          _|  _| |_|\___| _|\_\_|_|\_\\____|\___|_|  _| _____|\__,_|_.__/ ____/

# NEWS

## New release V2.5.2 soon : Velocity curve transformation pipe
<img width="300" border="0" src="https://raw.githubusercontent.com/TheKikGen/USBMidiKliK4x4/master/doc/um4x4-velocity-orignew-curves.png"/>
2 news pipes will be added in that release :

* VLCURV1 : allows you to define a default 5 segments velocity curve by adjusting 4 in ranges to out values.    
* VLCURV2 : allows you to define a single segment of the velocity curve with a (midi in, midi out) pairs. You can use that to define a fully customized curve, by adding, for example, eight pipes that will form 9 segments. It can be combined with VLCURV1 to adjust only some specify segments of a the 4 values curve.
* VLCURV3 : allows you to choose a predefined velocity curve between 6 presets : compressed for hard player, medium velocity, compresser/expander, low velocity 1 & 2, top and bottom ends cut

## New UMK4x4 PCB v2.3 will be soon available at Tindie !
The KikGen Labs team is curently working on the new UM4x4 V2.3 based on the STM32F103 Bluepill. This new board uses 3,5MM jacks instead of midi DIN, with full MMA RP-051 type A compliance. 
Midi DIN is still possible obviously, under the form of a 4 mini jack to DINs rail board, so you can "patch" midi in and out as you want.<br>
<img width="300" border="0" src="https://github.com/TheKikGen/USBMidiKliK4x4/blob/master/doc/UMK4X4-board%20and%20components%20capture-2023-10-14.png?raw=true"  />   <img width="250" border="0" src="https://github.com/TheKikGen/USBMidiKliK4x4/blob/master/doc/um4x4-mini-jack-to-midi-board.png?raw=true"  />

## New release : V2.5.1 correcting the MSGFLTR pipe bug
This v2.5.1 release includes a bug correction in parameters validation of the MSGFLTR pipe that was blocking the function #2.
https://github.com/TheKikGen/USBMidiKliK4x4/releases/tag/v2.5.1

# USBMidiKliK4x4
A multi-port USB MIDI interface for the STM32DUINO platform.

Download the last firmware here : https://github.com/TheKikGen/USBMidiKliK4x4/releases

User manual is here : https://github.com/TheKikGen/USBMidiKliK4x4/wiki/UMK4x4-V2.5-User-Manual

Check also the wiki here : https://github.com/TheKikGen/USBMidiKliK4x4/wiki

The story of this project starts with a hack of the MIDIPLUS/MIDITECH 4x4 USB to MIDI interface.
Needing more midi jacks, I bought a second Miditech interface, but I discovered it was not possible to use 2 Miditech / Midiplus MIDI USB 4X4 on the same computer to get 8x8, and according to the Miditech support, as that usb midi interface was not updateable at all !
I was stucked....That was motivating me enough to write a totally new and better firmware : the UsbMidiKlik4x4 project was born.

The current version V2.5 supports full USB midi until 16xIN , 16XOUT plus routing features, enabling configurables standalone mode, merge mode, thru mode, split mode, midi transformation, midi clock, etc., huge sysex flow, configuration menu from serial USB, and is very fast and stable thanks to the STM32F103.  More of that, you can aggregate until 5 3x3 boards seen as one by activating the "Bus mode".

<a rel="license" href="http://creativecommons.org/licenses/by-nc/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc/4.0/">Creative Commons Attribution-NonCommercial 4.0 International License</a>.

If you are a regular user of USBMidiKlik, please consider making a donation to encourage our team, and to pay for the coffees swallowed during the long nights of coding ! 

[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=thekikgen@gmail.com&lc=FR&item_name=Donation+to+TheKikGen+projects&no_note=0&cn=&currency_code=EUR&bn=PP-DonationsBF:btn_donateCC_LG.gif:NonHosted)

## USBMidiKliK4x4 features
+ STM32F103C8 / STM32F103CB / STM32F103RB / STM32F103RC uC support
+ Bluepill and Miditech 4x4 board support
+ 16 MIDI USB IN/OUT
+ 15-16  MIDI JACK IN  / JACKS OUT *1
+ 8 Virtual internal IN/OUT
+ 4 midi clock and Midi Time Code generators
+ Full  remote configuration by SYSEX
+ Configuration user interface from a serial USB terminal
+ Support of large sysex flow
+ Complex routing engine allowing to route any IN to any OUT
+ Transformation pipes allowing dynamic modification of midi messages
+ Customisable USB device ProductStringName, Vendor and Product Ids
+ STANDALONE mode (no USB host needed)
+ INTELLITHRU mode allowing specific routing rules when USB is idle
+ Stackable interfaces in bus mode to get 6x6 to 15x15 physical midi ports

The "pipeline" feature allows you to modify an incoming midi message through a chain of transformation functions (a "pipe"), e.g.,  transpose notes, split, map channel to another, map CC to another, etc...New pipes can be easily added in order to obtain complex midi transformations without degrading performances. 

<img width="650" border="0" src="https://github.com/TheKikGen/USBMidiKliK4x4/wiki/pipelines/pipelines1.jpg"  />

4 Midi clock generator / 8 virtual ports are available within the routing engine. Below is an overview of the possible paths for a midi message packet, coming from a physical in port to a physical out port :
<img  width="650" border="0" src="https://github.com/TheKikGen/USBMidiKliK4x4/wiki/pipelines/umk4x4-routing-engine-overview.jpg"  />

The bus mode allows the aggregation of up to five interfaces using the I2C protocol, which will be seen as a single interface by the usb host and/or the midi routing engine (including standalone mode).

<img  width="650" border="0" src="https://github.com/TheKikGen/USBMidiKliK4x4/raw/master/doc/USBMIDIKLIK-I2C-BUS-MODE.jpg?raw=true"  />

Last release of the firmware can be found here : https://github.com/TheKikGen/USBMidiKliK4x4/releases/
Update tool is included. Unzip and launch the UMK_BluePillUpdate.bat file (Windows only currently).   

## USBMidiKlik 4x4 PCB ready to be used

<img width="400" border="0" src="https://github.com/TheKikGen/USBMidiKliK4x4/blob/master/doc/UMK4X4-board%20and%20components%20capture-2023-10-14.png?raw=true)"  />

[![The Kikgen Labs Tindie place](https://d2ss6ovg47m0r5.cloudfront.net/images/tindie-logo@2x.png)]( https://www.tindie.com/stores/thekikgenlabs/ ) 

A new version of the USBMidiKlik 4x4 PCB ready-to-use (based on a STM32F103) is available now at [ Tindie ](https://www.tindie.com/stores/thekikgenlabs) 

The board is USB Powered, so no additional power supply necessary. 
It works driverless as a class compliant device, with all Windows and Linux versions, Mac OS X, Akai MPC Live/X/Force, IOS, Android...

To simplify connectivity, this board uses now only 3.5mm jacks for MIDI and I2C : 
- 6 TRS jacks : 3 MIDI in, 3 out, compliants with the MMA RP-054 type A specifications.
- You can use TRS to Midi DIN conversion cables, or connect your MIDI device directly with a simple stereo cable.
- 4 TRS jacks are now used instead connectors for stacking multiple boards on the I2C bus with 1 pair of stereo cables (power and data).

You will find also at Tindie, a 4 ports "midi rail" PCB allowing MIDI DIN from/to TRS. This can help if your need MIDI DIN ports and/or to arange the order of the DIN ports.  
With a single board, you can set e.g. 2 MIDI DIN IN and 2 OUT or 3 IN, 1 OUT, or you can set a 16 DIN ports (8 IN, 8 OUT) with 4 PCB, etc..
With 8 midi rail boards and 5 UMK4x4 stacked boards, it is possbile to get 30 midi DIN ports (15 x IN + 16 x OUT).
NB: The UMK4x4 supports only 3 physical IN and 3 physical OUT. 

<img width="300" border="0" src="https://github.com/TheKikGen/USBMidiKliK4x4/blob/master/doc/20191215_173355.jpg?raw=true"  />
The first protoype of the bus mode I use every day !

## MKPI : UMK4x4 graphical frontend

Orzdk has developped a graphical UI for manipulating and configuring the USBMidiKliK settings in sysex mode, from a web browser.
It also allows to show visual representation of the routes and transformations that are currently applied in your USBMidiKlik.

https://github.com/orzdk/mkpi

<img width="650" border="0" src="https://github.com/orzdk/mkpi/raw/master/doc/screenshot.png" />  





# USBMidiKliK4x4 - Configuration overview

Prebuilt binaries are compiled for the tkg-hid-bootloader since the 2.5 version, so you need the tkg-hid-bootloader firmware here :
https://github.com/TheKikGen/stm32-tkg-hid-bootloader/blob/master/F1/bootloader_only_binaries/tkg_hid_generic_pc13.bin

You can then use TKG-FLASH.EXE provided in the release to flash the 128K or the 64k bluepill version.  
The 128K firmware should work on a 64K Bluepill most of the time. If it is not the case, try the 64K firmware version.  

2 modes can be used to configure the USBMidiKlik midi : system exclusive messages or a configuration menu. 
Sysex messages have the following format, and ARE ONLY INTERPRETED ON CABLE 0 OR MIDI IN JACK 1 :

	F0 77 77 78 <sysex function id > <data> F7

The "F0 77 77 78" is the specific sysex header for USBMidiKlik4x4. Know that it is a totally unofficial header.
You will find the sysex V2.5 documentation in the repository :

https://raw.githubusercontent.com/TheKikGen/USBMidiKliK4x4/master/UMK-4X4-SYSEX-IPL.TXT

The second way is to activate the configuration menu on the serial port with the following sysex :

       F0 77 77 78 06 08 F7

To send that Sysex, you can use for example, MIDIOX, or the "sendmidi" command line utility by gbevin :

       sendmidi dev "USB MIDIKliK 4x4" syx hex 77 77 78 06 08

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
Don't forget to save your settings with the "s" option before leaving the configuration menu with "x".

# WIKI

<ul class="m-0 p-0 list-style-none" data-filterable-for="wiki-pages-filter" data-filterable-type="substring" data-pjax="">
<li class="Box-row">
<strong><a class="d-block" href="https://github.com/TheKikGen/USBMidiKliK4x4/wiki">Wiki Home</a></strong>
</li>
<li class="Box-row">
<strong><a class="d-block" href="https://github.com/TheKikGen/USBMidiKliK4x4/wiki/UMK4x4-V2.5-User-Manual">UMK4x4 official User Manuel</a></strong>
</li>
<li class="Box-row">
<strong><a class="d-block" href="https://github.com/TheKikGen/USBMidiKliK4x4/wiki/The-Miditech-Midiplus-4x4-hack-and-STM32Duino">The Miditech/Midiplus hack story</a></strong>
</li>
<li class="Box-row">
<strong><a class="d-block" href="https://github.com/TheKikGen/USBMidiKliK4x4/wiki/USBMIDIKLIK-BUS-MODE">About the bus mode</a></strong>
</li>
<li class="Box-row">
<strong><a class="d-block" href="https://github.com/TheKikGen/USBMidiKliK4x4/wiki/Build-UsbMidiKlik4x4-from-sources">Build UMK4x4 from sources</a></strong>
</li>
        <li class="Box-row">
          <strong><a class="d-block" href="https://github.com/TheKikGen/USBMidiKliK4x4/wiki/How-to-get-a--TriThru-%C3%A0-la-Midisolutions">Use case : how to get a TriThru a la Midisolutions</a></strong>
        </li>
        <li class="Box-row">
          <strong><a class="d-block" href="https://github.com/TheKikGen/USBMidiKliK4x4/blob/master/UMK-4X4-SYSEX-IPL.TXT">Sysex implementation for firmware V2.5</a></strong>
        </li>
    </ul>


