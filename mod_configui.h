/*
__ __| |           |  /_) |     ___|             |           |
  |   __ \   _ \  ' /  | |  / |      _ \ __ \   |      _` | __ \   __|
  |   | | |  __/  . \  |   <  |   |  __/ |   |  |     (   | |   |\__ \
 _|  _| |_|\___| _|\_\_|_|\_\\____|\___|_|  _| _____|\__,_|_.__/ ____/
  -----------------------------------------------------------------------------
  USBMIDIKLIK 4X4 - USB Midi advanced firmware for STM32F1 platform.
  Copyright (C) 2019 by The KikGen labs.
  LICENCE CREATIVE COMMONS - Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)

  This file is part of the USBMIDIKLIK-4x4 distribution
  https://github.com/TheKikGen/USBMidiKliK4x4
  Copyright (c) 2019 TheKikGen Labs team.
  -----------------------------------------------------------------------------
  Disclaimer.

  This work is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License.
  To view a copy of this license, visit http://creativecommons.org/licenses/by-nc/4.0/
  or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

  NON COMMERCIAL - PERSONAL USE ONLY : You may not use the material for pure
  commercial closed code solution without the licensor permission.

  You are free to copy and redistribute the material in any medium or format,
  adapt, transform, and build upon the material.

  You must give appropriate credit, a link to the github site
  https://github.com/TheKikGen/USBMidiKliK4x4 , provide a link to the license,
  and indicate if changes were made. You may do so in any reasonable manner,
  but not in any way that suggests the licensor endorses you or your use.

  You may not apply legal terms or technological measures that legally restrict
  others from doing anything the license permits.

  You do not have to comply with the license for elements of the material
  in the public domain or where your use is permitted by an applicable exception
  or limitation.

  No warranties are given. The license may not give you all of the permissions
  necessary for your intended use.  This program is distributed in the hope that
  it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*/
///////////////////////////////////////////////////////////////////////////////
//  CONFIGURATION USER INTERFACE FUNCTIONS
//-----------------------------------------------------------------------------
// Due to the unusual make process of Arduino platform, this file is directly
// included in the main ino one.
///////////////////////////////////////////////////////////////////////////////
#pragma once

///////////////////////////////////////////////////////////////////////////////
//  FUNCTIONS PROTOTYPES
///////////////////////////////////////////////////////////////////////////////
void PrintCleanHEX(uint8_t) __attribute__((optimize("-Os")));
//Shared. See usbmidiKlik4x4.h
//void ShowBufferHexDump(uint8_t* , uint16_t, uint8_t nl=16 ) __attribute__((optimize("-Os")));
void ShowBufferHexDumpDebugSerial(uint8_t* , uint8_t nl=16 ) __attribute__((optimize("-Os")));
uint8_t GetInt8FromHexChar(char) __attribute__((optimize("-Os")));
uint16_t GetInt16FromHex4Char(char *) __attribute__((optimize("-Os")));
uint16_t GetInt16FromHex4Bin(char * ) __attribute__((optimize("-Os")));
uint16_t PowInt(uint8_t ,uint8_t)  __attribute__((optimize("-Os")));
uint16_t AsknNumber(uint8_t) __attribute__((optimize("-Os")));
char AskDigit() __attribute__((optimize("-Os")));
char AskChar()  __attribute__((optimize("-Os")));
uint8_t AsknHexChar(char *, uint8_t ,char,char) __attribute__((optimize("-Os")));
char AskChoice(const char * , const char * ) __attribute__((optimize("-Os")));
void ShowMask16(uint16_t ,uint8_t );
void ShowPipelineSlotBrowser(boolean mustLoop=true) ;
void ShowMidiRoutingLine(uint8_t ,uint8_t ) __attribute__((optimize("-Os"))) ;
void ShowMidiRouting(uint8_t) __attribute__((optimize("-Os"))) ;
void ShowMidiKliKHeader() __attribute__((optimize("-Os")));
void ShowGlobalSettings() __attribute__((optimize("-Os")));
uint16_t AskMidiRoutingTargets(uint8_t,uint8_t , uint8_t ) __attribute__((optimize("-Os")));
void AskMidiRouting(uint8_t) __attribute__((optimize("-Os")));
void AskProductString() __attribute__((optimize("-Os")));
void AskVIDPID() __attribute__((optimize("-Os")));
void MenuItems( const char * ) __attribute__((optimize("-Os")));
void ShowConfigMenu() __attribute__((optimize("-Os")));

///////////////////////////////////////////////////////////////////////////////
// Serial print a formated hex value
//////////////////////////////////////////////////////////////////////////////
void PrintCleanHEX(uint8_t hexVal)
{
          if ( hexVal < 0x10 ) Serial.print("0");
          Serial.print(hexVal,HEX);
}

///////////////////////////////////////////////////////////////////////////////
// DUMP a byte buffer to screen
//////////////////////////////////////////////////////////////////////////////
void ShowBufferHexDump(uint8_t* bloc, uint16_t sz,uint8_t nl)
{
	uint8_t j=0;
	uint8_t * pp = bloc;
	for (uint16_t i=0; i != sz; i++) {
				PrintCleanHEX(*pp);
				Serial.print(" ");
				if (nl && (++j > nl) ) { Serial.println(); j=0; }
				pp++;
	}
}

void ShowBufferHexDumpDebugSerial(uint8_t* bloc, uint16_t sz,uint8_t nl)
{
	uint8_t j=0;
	uint8_t * pp = bloc;
	for (uint16_t i=0; i != sz; i++) {
        if ( *pp < 0x10 ) DEBUG_SERIAL.print("0");
        DEBUG_SERIAL.print(*pp,HEX);
				DEBUG_SERIAL.print(" ");
				if (nl && (++j > nl) ) { DEBUG_SERIAL.println(); j=0; }
				pp++;
	}
}


///////////////////////////////////////////////////////////////////////////////
// Get an Int8 From a hex char.  letters are minus.
// Warning : No control of the size or hex validity!!
///////////////////////////////////////////////////////////////////////////////
uint8_t GetInt8FromHexChar(char c)
{
	return (uint8_t) ( c <= '9' ? c - '0' : c - 'a' + 0xa );
}

///////////////////////////////////////////////////////////////////////////////
// Get an Int16 From a 4 Char hex array.  letters are minus.
// Warning : No control of the size or hex validity!!
///////////////////////////////////////////////////////////////////////////////
uint16_t GetInt16FromHex4Char(char * buff)
{
	char val[4];

  for ( uint8_t i =0; i != sizeof(val) ; i++ )
    val[i] = GetInt8FromHexChar(buff[i]);

	return GetInt16FromHex4Bin(val);
}

///////////////////////////////////////////////////////////////////////////////
// Get an Int16 From a 4 hex digit binary array.
// Warning : No control of the size or hex validity!!
///////////////////////////////////////////////////////////////////////////////
uint16_t GetInt16FromHex4Bin(char * buff)
{
	return (uint16_t) ( (buff[0] << 12) + (buff[1] << 8) + (buff[2] << 4) + buff[3] );
}

///////////////////////////////////////////////////////////////////////////////
// A Pow function to not use the fat float math library one
///////////////////////////////////////////////////////////////////////////////
uint16_t PowInt(uint8_t p,uint8_t n) {

  if (n == 0) return 1;
  uint16_t pow = 1;
  while ( n--) pow = pow * p;
  return pow;
}


///////////////////////////////////////////////////////////////////////////////
// USB serial get a number of N digit (long)
///////////////////////////////////////////////////////////////////////////////
uint16_t AsknNumber(uint8_t n)
{
	uint16_t v=0;
	uint8_t choice;
	while (n--) {
		v += ( (choice = AskDigit() ) - '0' )*PowInt(10,n);
		Serial.print(choice - '0');
	}
	return v;
}

///////////////////////////////////////////////////////////////////////////////
// USB serial getdigit
///////////////////////////////////////////////////////////////////////////////
char AskDigit()
{
  char c;
  while ( ( c = AskChar() ) <'0' && c > '9');
  return c;
}

///////////////////////////////////////////////////////////////////////////////
// USB serial getchar
///////////////////////////////////////////////////////////////////////////////
char AskChar()
{
  while (!Serial.available() >0);
  char c = Serial.read();
  // Flush
  while (Serial.available()>0) Serial.read();
  return c;
}

///////////////////////////////////////////////////////////////////////////////
// "Scanf like" for hexadecimal inputs
///////////////////////////////////////////////////////////////////////////////
uint8_t AsknHexChar(char *buff, uint8_t len,char exitchar,char sepa)
{

	uint8_t i = 0, c = 0;

	while ( i < len ) {
		c = AskChar();
		if ( (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'  ) ) {
			Serial.write(c);
			if (sepa) Serial.write(sepa);
			buff[i++]	= GetInt8FromHexChar(c);
		} else if (c == exitchar && exitchar !=0 ) break;
	}
	return i;
}

///////////////////////////////////////////////////////////////////////////////
// Get a choice from a question
///////////////////////////////////////////////////////////////////////////////
char AskChoice(const char * qLabel, const char * choices)
{
	char c;
	const char * yn = "yn";
	const char * ch;

	if ( *choices == 0 ) ch = yn; else ch = choices;
	Serial.print(qLabel);
	Serial.print(" (");
	Serial.print(ch);
	Serial.print(") ? ");

	while (1) {
		c = AskChar();
		uint8_t i=0;
		while (ch[i] )
				if ( c == ch[i++] ) { Serial.write(c); return c; }
	}
}

///////////////////////////////////////////////////////////////////////////////
// Show a 16 bits mask value with "X" and "."
///////////////////////////////////////////////////////////////////////////////
void ShowMask16(uint16_t bitMsk,uint8_t maxValue)
{
  // IN targets ports
  for ( uint8_t p =0; p != 16 ; p++) {
        if (p >= maxValue ) {
          Serial.print(" ");
        } else {
            if ( bitMsk & ( 1 << p) ) Serial.print("X");
            else   Serial.print(".");
        }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Browse pipelines
///////////////////////////////////////////////////////////////////////////////
void ShowPipelineSlotBrowser(boolean mustLoop) {
  while (1) {
    Serial.print("Enter pipeline slot 1-8, or 0 to exit :");
    uint8_t choice = AsknNumber(1);
    Serial.println();
    if ( choice == 0 ) break;
    if ( choice < 1 || choice > MIDI_TRANS_PIPELINE_SLOT_SIZE ) {
      Serial.println("# error. Please try again :");
    } else {
      ShowPipelineSlot(choice);
      Serial.println();
      if (!mustLoop) break;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Show a routing table line
///////////////////////////////////////////////////////////////////////////////
void ShowMidiRoutingLine(uint8_t port,uint8_t portType)
{
  uint8_t attachedSlot = 0;
  uint16_t inTargetsMsk =0;
  uint16_t outTargetsMsk =0;

	if (portType == PORT_TYPE_CABLE ) {
    attachedSlot  = EEPROM_Params.midiRoutingRulesCable[port].attachedSlot;
    inTargetsMsk  = EEPROM_Params.midiRoutingRulesCable[port].cableInTargetsMsk;
    outTargetsMsk = EEPROM_Params.midiRoutingRulesCable[port].jackOutTargetsMsk;
  } else

  if ( portType == PORT_TYPE_JACK ) {
    attachedSlot  = EEPROM_Params.midiRoutingRulesJack[port].attachedSlot;
    inTargetsMsk  = EEPROM_Params.midiRoutingRulesJack[port].cableInTargetsMsk;
    outTargetsMsk = EEPROM_Params.midiRoutingRulesJack[port].jackOutTargetsMsk;
  } else
  if (portType == PORT_TYPE_VIRTUAL ) {
    attachedSlot  = EEPROM_Params.midiRoutingRulesVirtual[port].attachedSlot;
    inTargetsMsk  = EEPROM_Params.midiRoutingRulesVirtual[port].cableInTargetsMsk;
    outTargetsMsk = EEPROM_Params.midiRoutingRulesVirtual[port].jackOutTargetsMsk;
  }
  else return;

	// print a full new line
  // Port

  Serial.print("| ");
	if ( port+1 < 10 ) Serial.print( " ");Serial.print(port+1);

	Serial.print(" |    ");
  if ( attachedSlot)
    Serial.print(attachedSlot);
  else
    Serial.print(".");
  Serial.print("     | ");

	// Cable IN targets ports
  ShowMask16(inTargetsMsk,USBCABLE_INTERFACE_MAX);

  Serial.print(" | ");

  // Jack Out targets ports
  ShowMask16(outTargetsMsk,SERIAL_INTERFACE_COUNT);

  Serial.print(" |");

  // Ithru if jack serial line
  if ( portType == PORT_TYPE_JACK ) {
    // IThru mask
    Serial.print("    ");
    if ( EEPROM_Params.intelliThruJackInMsk & ( 1 << port) )
      Serial.print("X");
    else
      Serial.print(".");

    // pipeLine slot
    Serial.print("    |    ");

    if (  EEPROM_Params.midiRoutingRulesIntelliThru[port].attachedSlot)
      Serial.print( EEPROM_Params.midiRoutingRulesIntelliThru[port].attachedSlot);
    else
      Serial.print(".");
    Serial.print("   | ");

    // Ithu Out targets ports
    ShowMask16(EEPROM_Params.midiRoutingRulesIntelliThru[port].jackOutTargetsMsk,SERIAL_INTERFACE_COUNT);

    Serial.print(" |");
  }

  Serial.println();
}

///////////////////////////////////////////////////////////////////////////////
// Show the midi routing map
///////////////////////////////////////////////////////////////////////////////
void ShowMidiRouting(uint8_t portType)
{
	uint8_t maxPorts = 0;

 	// Cable
	if (portType == PORT_TYPE_CABLE) {
			Serial.print("CABLE OUT ROUTING");
			maxPorts = USBCABLE_INTERFACE_MAX;
	}

	// Serial
	else
	if (portType == PORT_TYPE_JACK) {
			Serial.print("MIDI IN JACK ROUTING ");
			maxPorts = SERIAL_INTERFACE_COUNT;
	}
  // Virtual
	else
	if (portType == PORT_TYPE_VIRTUAL) {
			Serial.print("VIRTUAL IN ROUTING ");
			maxPorts = VIRTUAL_INTERFACE_MAX;
	}
  else return;

	Serial.print(" (");Serial.print(maxPorts);
	Serial.println(" port(s) found)");

  //| Jk | Pipeline | Cable IN 1111111 | Jack OUT 1111111 |        ITHRU      Jack OUT  1111111 |
  //|    |   Slot   | 1234567890123456 | 1234567890123456 | Enabled |  Slot  | 1234567890123456 |

	Serial.println();
	Serial.print("| ");
  if (portType == PORT_TYPE_CABLE) Serial.print("Cb");
  else if (portType == PORT_TYPE_VIRTUAL) Serial.print("Vr");
  else Serial.print("Jk");

	Serial.print(" | Pipeline | Cable IN 1111111 | Jack OUT 1111111 |");
  if ( portType == PORT_TYPE_JACK) Serial.print("    <-- ITHRU -->  Jack OUT  1111111 |");
  Serial.println();
  Serial.print("|    |   Slot   | 1234567890123456 | 1234567890123456 |");
  if ( portType == PORT_TYPE_JACK) Serial.print(" Enabled |  Slot  | 1234567890123456 |");
  Serial.println();

	for (uint8_t p=0; p != maxPorts ; p++) ShowMidiRoutingLine(p,portType);

	Serial.println();
	if ( portType == PORT_TYPE_JACK) {
		Serial.print("IThru mode is ");
		Serial.println(EEPROM_Params.intelliThruJackInMsk ? "active." : "inactive.");
		Serial.print("USB idle detection time :");
    Serial.print(EEPROM_Params.intelliThruDelayPeriod*15);Serial.println("s");
	}
}

///////////////////////////////////////////////////////////////////////////////
// Show MidiKlik Header on screen
///////////////////////////////////////////////////////////////////////////////
void ShowMidiKliKHeader()
{
  Serial.print("USBMIDIKLIK 4x4 - ");
	Serial.print(HARDWARE_TYPE);
	Serial.print(" - V");
	Serial.print(VERSION_MAJOR);Serial.print(".");Serial.println(VERSION_MINOR);
	Serial.println("(c) TheKikGen Labs");
}

///////////////////////////////////////////////////////////////////////////////
// Show current EEPROM settings
///////////////////////////////////////////////////////////////////////////////
void ShowGlobalSettings()
{
	uint8_t i;
	Serial.println("GLOBAL SETTINGS");
	Serial.println();
	Serial.print("Firmware version    : ");
	Serial.print(EEPROM_Params.majorVersion);Serial.print(".");
	Serial.println(EEPROM_Params.minorVersion);
	Serial.print("Magic number        : ");
	Serial.write(EEPROM_Params.signature , sizeof(EEPROM_Params.signature));
	Serial.println(EEPROM_Params.prmVersion);

	Serial.print("Build               : ");
	Serial.println( (char *)EEPROM_Params.TimestampedVersion);

	Serial.print("Sysex header        : ");
	for (i=0; i != sizeof(sysExInternalHeader); i++) {
			Serial.print(sysExInternalHeader[i],HEX);Serial.print(" ");
	}
	Serial.println();
	Serial.print("Next BootMode       : ");
	Serial.println(EEPROM_Params.nextBootMode);

	Serial.print("Debug Mode          : ");
	Serial.println(EEPROM_Params.debugMode ? "ENABLED":"DISABLED");

	Serial.print("Hardware type       : ");
	Serial.println(HARDWARE_TYPE);

	Serial.print("EEPROM param. size  : ");
	Serial.print(sizeof(EEPROM_Params_t));
  Serial.print(" / ");
  Serial.println(EE_CAPACITY);

	Serial.print("I2C Bus mode        : ");
	if ( EEPROM_Params.I2C_BusModeState == B_DISABLED )
			Serial.println("DISABLED");
  else
			Serial.println("ENABLED");

	Serial.print("I2C Device ID       : ");
	Serial.print(EEPROM_Params.I2C_DeviceId);
	if ( EEPROM_Params.I2C_DeviceId == B_MASTERID )
			Serial.print(" (master)");
	else
			Serial.print(" (slave)");
	Serial.println();

	Serial.print("MIDI USB ports      : ");
	Serial.println(USBCABLE_INTERFACE_MAX);

	Serial.print("MIDI serial ports   : ");
	Serial.println(SERIAL_INTERFACE_COUNT);

  Serial.print("MIDI virtual ports  : ");
	Serial.println(VIRTUAL_INTERFACE_MAX);

	Serial.print("USB VID - PID       : ");
	Serial.print( EEPROM_Params.vendorID,HEX);
	Serial.print(" - ");
	Serial.println( EEPROM_Params.productID,HEX);

	Serial.print("USB Product string  : ");
	Serial.write(EEPROM_Params.productString, sizeof(EEPROM_Params.productString));
	Serial.println();
}

///////////////////////////////////////////////////////////////////////////////
// Setup a Midi routing target on screen
///////////////////////////////////////////////////////////////////////////////
uint16_t AskMidiRoutingTargets(uint8_t portType,uint8_t portTypeOut, uint8_t port)
{
	uint16_t targetsMsk = 0;
	uint8_t  portMax;

  if      (portTypeOut == PORT_TYPE_CABLE) portMax = USBCABLE_INTERFACE_MAX;
  else if (portTypeOut == PORT_TYPE_JACK ) portMax = SERIAL_INTERFACE_COUNT;
  else return 0;

  uint8_t choice;

	if (portType == PORT_TYPE_ITHRU && portTypeOut == PORT_TYPE_CABLE ) return 0;

	for ( uint8_t i=0 ; i != portMax  ; i++ ){
		Serial.print("Route ");
    if (portType == PORT_TYPE_CABLE ) Serial.print("Cable OUT#");
    else if (portType == PORT_TYPE_VIRTUAL ) Serial.print("Virtual IN#");
    else Serial.print("Jack IN#");

		if (port+1 < 10) Serial.print("0");
		Serial.print(port+1);
		Serial.print(" to ");
		Serial.print(portTypeOut == PORT_TYPE_CABLE ? "Cable IN#":"Jack OUT#");
		if (i+1 < 10) Serial.print("0");
		Serial.print(i+1);
		choice = AskChoice("  (x to exit)","ynx");
		if (choice == 'x') {Serial.println(); break;}
		else if ( choice == 'y') targetsMsk |= (1 << i);
		Serial.println();
	}
	return targetsMsk;
}

///////////////////////////////////////////////////////////////////////////////
// Setup a Midi routing rule on screen
///////////////////////////////////////////////////////////////////////////////
void AskMidiRouting(uint8_t portType)
{
	uint8_t  portMax;

  if      (portType == PORT_TYPE_CABLE) portMax = USBCABLE_INTERFACE_MAX;
  else if (portType == PORT_TYPE_JACK || portType == PORT_TYPE_ITHRU) portMax = SERIAL_INTERFACE_COUNT;
  else if (portType == PORT_TYPE_VIRTUAL) portMax = VIRTUAL_INTERFACE_MAX;
  else return ;

	uint8_t  choice;

	Serial.print("-- Set ");
  if      (portType == PORT_TYPE_CABLE) Serial.print("USB Cable OUT");
  else if (portType == PORT_TYPE_VIRTUAL) Serial.print("Virtual IN");
  else Serial.print("Jack IN");

	if (portType == PORT_TYPE_ITHRU ) Serial.print(" IntelliThru ");
	Serial.println(" routing --");

	Serial.print("Enter ");
	if      (portType == PORT_TYPE_CABLE) Serial.print("Cable OUT");
  else if (portType == PORT_TYPE_VIRTUAL) Serial.print("Virtual IN");
  else Serial.print("Jack IN");

	Serial.print (" # (01-");
	Serial.print( portMax > 9 ? "":"0");
	Serial.print( portMax);
	Serial.print(" / 00 to exit) :");
	while (1) {
		choice = AsknNumber(2);
		Serial.println();
		if ( choice > 0 && choice > portMax) {
			Serial.print("# error. Please try again :");
		} else break;
	}

  if (choice > 0) {
		uint8_t port =  choice - 1;
		Serial.println();

		if (portType == PORT_TYPE_ITHRU ) {
			Serial.print("Jack #");
			Serial.print( port+1 > 9 ? "":"0");Serial.print(port+1);
			if ( AskChoice(" : disable thru mode routing","") == 'y') {
				Serial.println();
				Serial.print("Jack IN #");Serial.print(port+1);
				Serial.println(" disabled.");
				EEPROM_Params.intelliThruJackInMsk &= ~(1 << port);
				return;
			}
      Serial.println();
      if ( EEPROM_Params.midiRoutingRulesIntelliThru[port].jackOutTargetsMsk ) {
        if (AskChoice("Keep existing routing ?","") == 'y') {
          Serial.println();
          EEPROM_Params.intelliThruJackInMsk |= (1 << port); // enable Ithru for this port
          return;
        }
        Serial.println();
      }
		}

    uint8_t attachedSlot;
    Serial.print("Enter pipeline slot 0-8, (0 = no slot) :");
    while (1) {
      attachedSlot = AsknNumber(1);
      Serial.println();
      if ( attachedSlot == 0 || attachedSlot <= MIDI_TRANS_PIPELINE_SLOT_SIZE ) break;
      Serial.print("# error. Please try again :");
    }

		uint16_t cTargets=0;
		uint16_t jTargets=0;

		if (portType != PORT_TYPE_ITHRU ) {
			// Cable
			cTargets = AskMidiRoutingTargets(portType,PORT_TYPE_CABLE,port);
		}
		// Midi jacks
		jTargets = AskMidiRoutingTargets(portType,PORT_TYPE_JACK,port);

		if (portType == PORT_TYPE_CABLE ) {
			EEPROM_Params.midiRoutingRulesCable[port].attachedSlot = attachedSlot;
			if ( jTargets + cTargets != 0 ) {
          EEPROM_Params.midiRoutingRulesCable[port].cableInTargetsMsk = cTargets;
			    EEPROM_Params.midiRoutingRulesCable[port].jackOutTargetsMsk = jTargets ;
      }
		} else
		if (portType == PORT_TYPE_JACK ) {
			EEPROM_Params.midiRoutingRulesJack[port].attachedSlot = attachedSlot;
      if ( jTargets + cTargets != 0 ) {
			    EEPROM_Params.midiRoutingRulesJack[port].cableInTargetsMsk = cTargets;
			    EEPROM_Params.midiRoutingRulesJack[port].jackOutTargetsMsk = jTargets ;
      }
		} else
    if (portType == PORT_TYPE_VIRTUAL ) {
      EEPROM_Params.midiRoutingRulesVirtual[port].attachedSlot = attachedSlot;
      if ( jTargets + cTargets != 0 ) {
          EEPROM_Params.midiRoutingRulesVirtual[port].cableInTargetsMsk = cTargets;
          EEPROM_Params.midiRoutingRulesVirtual[port].jackOutTargetsMsk = jTargets ;
      }
    } else
  	if (portType == PORT_TYPE_ITHRU ) {
			if ( jTargets != 0 ) {
        EEPROM_Params.midiRoutingRulesIntelliThru[port].jackOutTargetsMsk = jTargets ;
        EEPROM_Params.intelliThruJackInMsk |= (1 << port); // enable Ithru for this port
      }
      if( EEPROM_Params.midiRoutingRulesIntelliThru[port].jackOutTargetsMsk)
        EEPROM_Params.midiRoutingRulesIntelliThru[port].attachedSlot = attachedSlot;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// Setup Product String on screen
///////////////////////////////////////////////////////////////////////////////
void AskProductString()
{
	uint8_t i = 0;
	char c;
	char buff [32];

	Serial.println("Enter product string - ENTER to terminate :");
	while ( i < USB_MIDI_PRODUCT_STRING_SIZE && (c = AskChar() ) !=13 ) {
		if ( c >= 32 && c < 127 ) {
			Serial.write(c);
			buff[i++]	= c;
		}
	}
	if ( i > 0 ) {
		memset(EEPROM_Params.productString,0,sizeof(EEPROM_Params.productString) );
		memcpy(EEPROM_Params.productString,buff,i);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Setup PID and VID on screen
///////////////////////////////////////////////////////////////////////////////
void AskVIDPID()
{
	char buff [5];
	Serial.println("Enter VID - PID, in hex (0-9,a-f) :");
	AsknHexChar( (char *) buff, 4, 0, 0);
	EEPROM_Params.vendorID  = GetInt16FromHex4Bin((char*)buff);
	Serial.print("-");
	AsknHexChar( (char * ) buff, 4, 0, 0);
	EEPROM_Params.productID = GetInt16FromHex4Bin((char*)buff);
}

///////////////////////////////////////////////////////////////////////////////
// Show menu items with columns
///////////////////////////////////////////////////////////////////////////////
void MenuItems( const char * menuItems[]) {
  uint8_t c = 0;
  uint8_t l = 0;
  uint8_t nb = 0;

  // 3 columns of 10 items max
  const char * item[3][10];

  // Fill columns
  for ( c=0 ; c< 3 ; c++ ) {
    for ( l=0 ; l < 10 ; l++) {
      if ( strlen(menuItems[nb]) ) item[c][l] = menuItems[nb++];
      else {
        item[c][l] = 0;
      }
    }
  }

  // Print columns
  for ( l=0 ; l< 10 ; l++ ) {
    for ( c=0 ; c < 3 ; c++) {
      if ( *item[c][l] != 0 ) {
          if ( *item[c][l] != '.') {
              Serial.print("["); Serial.print(*item[c][l]);Serial.print("] ");
              Serial.print(item[c][l]+1);
          } else Serial.print("    ");
          for (uint8_t j = 0 ; j < 30-strlen(item[c][l]) ; j++) Serial.write(' ');
      }
    }
    Serial.println();
  }
}

///////////////////////////////////////////////////////////////////////////////
// INFINITE LOOP - CONFIG ROOT MENU
//----------------------------------------------------------------------------
// Allow USBMidLKLIK configuration by using a menu as a user interface
// from the USB serial port.
///////////////////////////////////////////////////////////////////////////////
void ShowConfigMenu()
{
	char choice=0;
	uint8_t i;
  boolean showMenu = true;
  static const char * configMenu[] =
  {
  "0Show global settings",
  "1Show midi routing",
  "vShow virtual routing",
  "2Usb VID PID",
  "3Usb product string",
  "4Cable OUT routing",
  "5Jack IN routing",
  "6IThru routing",
  "7IThru timeout",
  "tVirtual IN routing",
  "pShow pipeline",
  "8Toggle bus mode",
  "9Set device Id",
  "aShow active devices",
  "dSYSEX settings dump",
  ".",
  "eReload settings",
  "fFactory settings",
  "rFactory routing",
  "sSave settings",
  "zDebug mode",
  "xExit",
  "!Dump EEPROM",
  ":Dump Flash memory",
  "/Format EEPROM",
  ""
  };
	for ( ;; )
  {
		if (showMenu) {
      Serial.println();
  		ShowMidiKliKHeader();
      Serial.println();
      MenuItems(configMenu);
		}
    showMenu = true;
		Serial.print("=>");
		choice = AskChar();
		Serial.println(choice);
    Serial.println();

		switch (choice)
    {
      // Show global settings
			case '0':
				ShowGlobalSettings();
				Serial.println();
        showMenu = false;
			  break;

			// Show midi routing
			case '1':
			  ShowMidiRouting(PORT_TYPE_CABLE);
				Serial.println();
			  ShowMidiRouting(PORT_TYPE_JACK);
				Serial.println();
	      showMenu = false;
			  break;

			// Change VID & PID
			case '2':
				AskVIDPID();
				Serial.println();
				showMenu = false;
				break;

			// Change the product string
			case '3':
				AskProductString();
				Serial.println();
				showMenu = false;
				break;

			// Cables midi routing
			case '4':
				AskMidiRouting(PORT_TYPE_CABLE);
				Serial.println();
				break;

			// Jack IN midi routing
			case '5':
				AskMidiRouting(PORT_TYPE_JACK);
				Serial.println();
				break;

			// IntelliThru IN midi routing
			case '6':
				AskMidiRouting(PORT_TYPE_ITHRU);
				Serial.println();
				break;

			// USB Timeout <number of 15s periods 1-127>
			case '7':
				Serial.println("15s periods nb (001-127 / 000 to exit) :");
				i = AsknNumber(3);
				if (i == 0 || i >127 )
					Serial.println(". Error. No change made.");
				else {
					EEPROM_Params.intelliThruDelayPeriod = i;
					Serial.print(" <= Delay set to ");Serial.print(i*15);Serial.println("s");
				}
				Serial.println();
				showMenu = false;
				break;

				// Enable Bus mode
			case '8':
				if ( AskChoice("Enable bus mode","") == 'y' ) {
					EEPROM_Params.I2C_BusModeState = B_ENABLED;
					Serial.println();
					Serial.println("Bus mode enabled.");
					Serial.println();
				} else {
					EEPROM_Params.I2C_BusModeState = B_DISABLED;
					Serial.println();
					Serial.println("Bus mode disabled.");
					Serial.println();
				}
			  showMenu = false;
				break;

				// Set device ID.
			case '9':
	 			Serial.print("Device ID ( ");
				Serial.print(B_MASTERID < 9 ? "0": "");
				Serial.print(B_MASTERID);
				Serial.print(":Master, Slaves ");
				Serial.print(B_SLAVE_DEVICE_BASE_ADDR < 9 ? "0": "");
				Serial.print(B_SLAVE_DEVICE_BASE_ADDR);
				Serial.print("-");
				Serial.print(B_SLAVE_DEVICE_LAST_ADDR < 9 ? "0": "");
				Serial.print(B_SLAVE_DEVICE_LAST_ADDR);
				Serial.print(") :");
				while (1) {
					i = AsknNumber(2);
					if ( i != B_MASTERID ) {
						if ( i > B_SLAVE_DEVICE_LAST_ADDR || i < B_SLAVE_DEVICE_BASE_ADDR ) {
								Serial.println();
								Serial.print("Id error. Try again :");
						} else break;
					} else break;
				}
				EEPROM_Params.I2C_DeviceId = i;
				Serial.println();
				Serial.println();
				showMenu = false;

		    break;

			// Show active devices on the I2C Bus
			case 'a':
				Serial.println();
				I2C_ShowActiveDevice();
				Serial.println();
				showMenu = false;
				break;

      // Sysex config dump
			case 'd':
          SysexInternal_DumpFullConfToStream(2);
          Serial.println();
        break;

			// Reload the EEPROM parameters structure
			case 'e':
				if ( AskChoice("Reload settings ","") == 'y' ) {
					EEPROM_ParamsInit();
          Serial.println();
					Serial.println("Settings reloaded.");
					Serial.println();
				}
				showMenu = false;
				break;

			// Restore factory settings
			case 'f':
				if ( AskChoice("Restore factory settings","") == 'y' ) {
          Serial.println();
					if ( AskChoice("Sure","") == 'y' ) {
						EEPROM_ParamsInit(true);
            Serial.println();
						Serial.println("Factory settings restored.");
					}
          Serial.println();
				}
				showMenu = false;
				break;

      // Show pipeline
			case 'p':
        ShowPipelineSlotBrowser();
				showMenu = false;
				break;

			// Default midi routing
			case 'r':
				if ( AskChoice("Reload default midi routing","") == 'y')
					ResetMidiRoutingRules(ROUTING_RESET_ALL);
				Serial.println();
				showMenu = false;
				break;

			// Save & quit
			case 's':
				ShowGlobalSettings();
				Serial.println();
				if ( AskChoice("Save settings","") == 'y' ) {
					Serial.println();
					//Goto midi mode at the next boot
					EEPROM_Params.nextBootMode = bootModeMidi;
					EEPROM_ParamsSave();
					delay(100);
					showMenu = false;
				}
				break;

      // Show virtual midi routing
			case 't':
          ShowMidiRouting(PORT_TYPE_VIRTUAL);
          Serial.println();
  	      showMenu = false;
  			  break;

      // Virtual In midi routing
			case 'v':
  				AskMidiRouting(PORT_TYPE_VIRTUAL);
  				Serial.println();
  				break;

      // debug Mode
      case 'z':
        #ifdef DEBUG_MODE
        if ( AskChoice("Enable debug mode.","") == 'y' ) {
          EEPROM_Params.debugMode = true;
          Serial.println();
          Serial.println("Debug mode : M:Serial3, S:UsbSerial. 115200 bds.");
          Serial.println();
        } else {
          EEPROM_Params.debugMode = false;
          Serial.println();
          Serial.println("Debug mode disabled.");
          Serial.println();
        }
        #else
          Serial.println("Not available.");
          Serial.println();
        #endif
				showMenu = false;
        break;

      // Dump EEPROM
      case '!':
        EEPROM_FlashMemoryDump(EE_PAGE_BASE,EE_NBPAGE);
        break;

      // Navigate EEPROM
      case ':':
        {
            char c;
            uint16_t p=0;
            do {
              EEPROM_FlashMemoryDump(p,1);
              c = AskChoice("q/s to navigate or e(x)it","qsx");
              Serial.println();
              if (c == 'q' && p > 0) p--;
              else if ( c == 's' && p < (EE_FLASH_SIZE*1024 / EE_PAGE_SIZE -1 ) ) p++;
            }  while ( c != 'x');
        break;
        }

      // Format EEPROM
      case '/':
        if ( AskChoice("Format EEPROM, Sure","") == 'y' )
          EEPROM_Format();
        break;

			// Abort
			case 'x':
				if ( AskChoice("Exit","") == 'y' ) {
            Serial.println();
						Serial.end();
						nvic_sys_reset();
				}
				break;

    } // Switch
  } // for
}
