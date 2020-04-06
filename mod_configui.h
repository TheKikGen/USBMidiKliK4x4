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
void SerialPrintf(const char *format, ...);
uint16_t PowInt(uint8_t ,uint8_t)  __attribute__((optimize("-Os")));
uint16_t AsknNumber(uint8_t,boolean nl=true) __attribute__((optimize("-Os")));
char AskDigit() __attribute__((optimize("-Os")));
char AskChar()  __attribute__((optimize("-Os")));
uint8_t AsknHexChar(char *, uint8_t ,char,char) __attribute__((optimize("-Os")));
char AskChoice(const char * , const char *,boolean nl=true) __attribute__((optimize("-Os")));
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
// Strings const used in UI
//////////////////////////////////////////////////////////////////////////////
// _M : Major Case.  _B : Beautified.
const char* str_16DIGITS     = "1234567890123456";
const char* str_71DIGITS     = "1111111";
const char* str_MIDI_M       = "MIDI";
const char* str_IN_M         = "IN";
const char* str_OUT_M        = "OUT";
const char* str_CABLE_M      = "CABLE";
const char* str_CABLE_B      = "Cable";
const char* str_CABLE        = "cable";
const char* str_JACK_M       = "JACK";
const char* str_JACK_B       = "Jack";
const char* str_JACK         = "jack";
const char* str_VIRTUAL_M    = "VIRTUAL";
const char* str_VIRTUAL_B    = "Virtual";
const char* str_VIRTUAL      = "virtual";
const char* str_ROUTING      = "routing";
const char* str_ROUTING_M    = "ROUTING";
const char* str_PIPELINE_B   = "Pipeline";
const char* str_VIRT_B       = "Virt.";
const char* str_SLOT_B       = "Slot";
const char* str_ENABLE_B     = "Enable";
const char* str_ENABLED_B    = "Enabled";
const char* str_ENABLED      = "enabled";
const char* str_ENABLED_M    = "ENABLED";
const char* str_DISABLE_B    = "Disable";
const char* str_DISABLED_B   = "Disabled";
const char* str_DISABLED     = "disabled";
const char* str_DISABLED_M   = "DISABLED";
const char* str_USB_M        = "USB";
const char* str_PORTS        = "port(s)";
const char* str_PORT         = "port";
const char* str_ERROR_B      = "Error";
const char* str_TRYAGAIN_B   = "Try again";
const char* str_ITHRU_M      = "ITHRU";
const char* str_ITHRU_B      = "Ithru";
const char* str_DEVICE_ID_B  = "Device ID";
const char* str_SETTINGS     = "settings";
const char* str_SETTINGS_M   = "SETTINGS";
const char* str_DONE_B       = "Done";

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
// SerialPrintf : a light printf like to Serial.
// %s : strings
// $n : new line
// %x : hexa
// %(n)d %(n)x : 0 left pad fo size n
// %% : %
///////////////////////////////////////////////////////////////////////////////
void SerialPrintf(const char *format, ...)
{
  va_list varg;
  va_start(varg, format);

  while ( *format != 0 ) {
    if (*format == '%') {
      format++;
      if (*format == '%') Serial.print(*format);
      else if (*format == 's') Serial.print(va_arg(varg, char*)); // String
      else if (*format == 'd') Serial.print(va_arg(varg, int));  // long int
      else if (*format == 'x') Serial.print(va_arg(varg, int),HEX);  // hexa
      else if (*format >= '0' && *format <= '9' ) {
            char p;
            if ( *format == '0') { format++; p = '0';}
            else p = ' ';
            if ( *format >='1' & *format <= '9' ) {
              uint8_t pad = *format - '0';
              int value = va_arg(varg, int);
              uint8_t  base = 0;
              format++;
              if ( *format == 'd' ) base = 10;
              else if ( *format =='x' ) base = 16;
              if (base) {
                uint32_t pw   = base;
                while (pad--) {
                  if ( value < base ) while (pad) { Serial.print(p);pad--; }
                  else pw *= base;
                }
                if (base == 16) Serial.print(value,HEX); else Serial.print(value);
              }
            } else return;
      }
      else if (*format == 'n') Serial.println(); // new line
      else return;
    } else Serial.print(*format);
    format++;
  }
  va_end(varg);
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
uint16_t AsknNumber(uint8_t n,boolean nl)
{
	uint16_t v=0;
	uint8_t choice;
	while (n--) {
		v += ( (choice = AskDigit() ) - '0' )*PowInt(10,n);
		Serial.print(choice - '0');
	}
  if (nl) Serial.println();
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
char AskChoice(const char * qLabel, const char * choices,boolean nl)
{
	char c;
	const char * yn = "yn";
	const char * ch;

	if ( *choices == 0 ) ch = yn; else ch = choices;
	SerialPrintf("%s (%s) ? ",qLabel,ch);
	while (1) {
		c = AskChar();
		uint8_t i=0;
		while (ch[i] )
				if ( c == ch[i++] ) {
            Serial.write(c);
            if (nl) Serial.println();
            return c;
        }
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
    if ( choice == 0 ) break;
    if ( choice >= 1 && choice <= MIDI_TRANS_PIPELINE_SLOT_SIZE ) {
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
  uint16_t cbInTgMsk =0;
  uint16_t jkOutTgMsk =0;
  uint16_t vrOutTgMsk =0;

	if (portType == PORT_TYPE_CABLE ) {
    attachedSlot = EEPROM_Params.midiRoutingRulesCable[port].attachedSlot;
    cbInTgMsk    = EEPROM_Params.midiRoutingRulesCable[port].cableInTargetsMsk;
    jkOutTgMsk   = EEPROM_Params.midiRoutingRulesCable[port].jackOutTargetsMsk;
    vrOutTgMsk   = EEPROM_Params.midiRoutingRulesCable[port].virtualOutTargetsMsk;
  } else
  if ( portType == PORT_TYPE_JACK ) {
    attachedSlot = EEPROM_Params.midiRoutingRulesJack[port].attachedSlot;
    cbInTgMsk    = EEPROM_Params.midiRoutingRulesJack[port].cableInTargetsMsk;
    jkOutTgMsk   = EEPROM_Params.midiRoutingRulesJack[port].jackOutTargetsMsk;
    vrOutTgMsk   = EEPROM_Params.midiRoutingRulesJack[port].virtualOutTargetsMsk;
  } else
  if (portType == PORT_TYPE_VIRTUAL ) {
    attachedSlot = EEPROM_Params.midiRoutingRulesVirtual[port].attachedSlot;
    cbInTgMsk    = EEPROM_Params.midiRoutingRulesVirtual[port].cableInTargetsMsk;
    jkOutTgMsk   = EEPROM_Params.midiRoutingRulesVirtual[port].jackOutTargetsMsk;
    vrOutTgMsk   = EEPROM_Params.midiRoutingRulesVirtual[port].virtualOutTargetsMsk;
  }
  else return;

	// print a full new line
  // Port
  SerialPrintf("| %2d |    ",port+1);

  if ( attachedSlot)
    Serial.print(attachedSlot);
  else
    Serial.print(".");
  Serial.print("     | ");

	// Cable IN targets ports
  ShowMask16(cbInTgMsk,USBCABLE_INTERFACE_MAX);
  Serial.print(" | ");

  // Jack Out targets ports
  ShowMask16(jkOutTgMsk,SERIAL_INTERFACE_COUNT);
  Serial.print(" | ");

  // virtual Out targets ports
  ShowMask16(vrOutTgMsk,VIRTUAL_INTERFACE_MAX);
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
    Serial.print(" | ");
    ShowMask16(EEPROM_Params.midiRoutingRulesIntelliThru[port].virtualOutTargetsMsk,VIRTUAL_INTERFACE_MAX);
    Serial.print(" |");
  }

  Serial.println();
}

///////////////////////////////////////////////////////////////////////////////
// Show the midi routing map for a specific port type
///////////////////////////////////////////////////////////////////////////////
void ShowMidiRouting(uint8_t portType)
{
	uint8_t maxPorts = 0;

  // Cable
	if (portType == PORT_TYPE_CABLE) {
      SerialPrintf("%s %s",str_CABLE_M,str_OUT_M);
			maxPorts = USBCABLE_INTERFACE_MAX;
	}
	// Serial
	else
	if (portType == PORT_TYPE_JACK) {
      SerialPrintf("%s %s",str_JACK_M,str_IN_M);
			maxPorts = SERIAL_INTERFACE_COUNT;
	}
  // Virtual
	else
	if (portType == PORT_TYPE_VIRTUAL) {
      SerialPrintf("%s %s",str_VIRTUAL_M,str_IN_M);
			maxPorts = VIRTUAL_INTERFACE_MAX;
	}  else return;

  SerialPrintf(" %s (%d %s found)%n",str_ROUTING_M,maxPorts,str_PORTS);
  Serial.println();

//| Jk | Pipeline | Cable IN 1111111 | Jack OUT 1111111 | Virt.OUT 1111111 |    <-- ITHRU -->  Jack OUT  1111111 | Virt.OUT 1111111 |
//|    |   Slot   | 1234567890123456 | 1234567890123456 | 1234567890123456 | Enabled |  Slot  | 1234567890123456 | 1234567890123456 |
//|  1 |    .     | X............... | ...............  | X..............  |    .    |    .   | ...............  | X..............  |

	Serial.print("| ");
  if (portType == PORT_TYPE_CABLE) Serial.print("Cb");
  else if (portType == PORT_TYPE_VIRTUAL) Serial.print("Vr");
  else if (portType == PORT_TYPE_JACK) Serial.print("Jk");
  else return;

  SerialPrintf(" | %s | %s %s %s |",str_PIPELINE_B,str_CABLE_B,str_IN_M,str_71DIGITS);
  SerialPrintf(" %s %s %s |",str_JACK_B,str_OUT_M,str_71DIGITS);
  SerialPrintf(" %s%s %s |",str_VIRT_B,str_OUT_M,str_71DIGITS);
  if ( portType == PORT_TYPE_JACK) {
    SerialPrintf("    <-- %s -->  %s %s  %s |",str_ITHRU_M,str_JACK_B,str_OUT_M,str_71DIGITS);
    SerialPrintf(" %s%s %s |",str_VIRT_B,str_OUT_M,str_71DIGITS);
  }
  Serial.println();
  SerialPrintf("|    |   %s   | %s | %s | %s |",str_SLOT_B,str_16DIGITS,str_16DIGITS,str_16DIGITS);
  if ( portType == PORT_TYPE_JACK) {
    SerialPrintf(" %s |  %s  | %s | %s |",str_ENABLED_B,str_SLOT_B,str_16DIGITS,str_16DIGITS);
  }
  Serial.println();
	for (uint8_t p=0; p != maxPorts ; p++) ShowMidiRoutingLine(p,portType);
	Serial.println();
	if ( portType == PORT_TYPE_JACK) {
		SerialPrintf("% mode is %s.%n",str_ITHRU_B,EEPROM_Params.intelliThruJackInMsk ? str_ENABLED:str_DISABLED);
		SerialPrintf("%s idle detection time :%ds%n",str_USB_M,EEPROM_Params.intelliThruDelayPeriod*15);
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
	SerialPrintf("GLOBAL %s",str_SETTINGS);
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
	Serial.println(EEPROM_Params.debugMode ? str_ENABLED_M:str_DISABLED_M);

	Serial.print("Hardware type       : ");
	Serial.println(HARDWARE_TYPE);

	Serial.print("EEPROM param. size  : ");
	Serial.print(sizeof(EEPROM_Params_t));
  Serial.print(" / ");
  Serial.println(EE_CAPACITY);

	Serial.print("I2C Bus mode        : ");
  Serial.println(EEPROM_Params.I2C_BusModeState == B_DISABLED ? str_DISABLED_M:str_ENABLED_M);

	SerialPrintf("I2C %s       : ",str_DEVICE_ID_B);
	Serial.print(EEPROM_Params.I2C_DeviceId);
	if ( EEPROM_Params.I2C_DeviceId == B_MASTERID )
			Serial.print(" (master)");
	else
			Serial.print(" (slave)");
	Serial.println();

  // Midi USB ports
	SerialPrintf("%s %s %s    : %d%n",str_MIDI_M,str_USB_M,str_PORTS,USBCABLE_INTERFACE_MAX);
  // Midi jack port(s)
	SerialPrintf("%s %s %s   : %d%n",str_MIDI_M,str_JACK,str_PORTS,SERIAL_INTERFACE_COUNT);
 // Midi virtual port(s)
  SerialPrintf("%s virtual %s: %d%n",str_MIDI_M,str_PORTS,VIRTUAL_INTERFACE_MAX);

	SerialPrintf("%s VID - PID       : %04x - %04x%n",str_USB_M,EEPROM_Params.vendorID,EEPROM_Params.productID);

	SerialPrintf("%s Product string  : ",str_USB_M);
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
  else if (portTypeOut == PORT_TYPE_VIRTUAL) portMax = VIRTUAL_INTERFACE_MAX;
  else return 0;

  char choice;
	for ( uint8_t i=0 ; i != portMax  ; i++ ){
		Serial.print("Route ");
    if (portType == PORT_TYPE_CABLE ) SerialPrintf("%s %s %s",str_CABLE,str_OUT_M,str_PORT);
    else if (portType == PORT_TYPE_JACK ) SerialPrintf("%s %s %s",str_JACK,str_IN_M,str_PORT);
    else if (portType == PORT_TYPE_VIRTUAL ) SerialPrintf("%s %s %s",str_VIRTUAL,str_IN_M,str_PORT);
    else return 0;
    SerialPrintf("# %02d to ",port+1);

    if (portTypeOut == PORT_TYPE_CABLE ) SerialPrintf("%s %s %s",str_CABLE,str_IN_M,str_PORT);
    else if (portTypeOut == PORT_TYPE_JACK ) SerialPrintf("%s %s %s",str_JACK,str_OUT_M,str_PORT);
    else if (portTypeOut == PORT_TYPE_VIRTUAL ) SerialPrintf("%s %s %s",str_VIRTUAL,str_OUT_M,str_PORT);
    else return 0;
    SerialPrintf("# %02d",i+1);

		choice = AskChoice(" (x to exit)","ynx");
		if (choice == 'x') break;
		else if ( choice == 'y') targetsMsk |= (1 << i);
	}
	return targetsMsk;
}

///////////////////////////////////////////////////////////////////////////////
// Setup a Midi routing rule on screen
///////////////////////////////////////////////////////////////////////////////
void AskMidiRouting(uint8_t portType)
{
	uint8_t  portMax;
  uint8_t  choice;
  uint16_t cTargets=0;
  uint16_t jTargets=0;
  uint16_t vTargets=0;
  uint8_t  port =0;
  uint8_t  attachedSlot=0;

  if  (portType != PORT_TYPE_CABLE && portType != PORT_TYPE_JACK && portType != PORT_TYPE_ITHRU && portType != PORT_TYPE_VIRTUAL) return ;

  if  (portType == PORT_TYPE_CABLE) {
    portMax = USBCABLE_INTERFACE_MAX;
    SerialPrintf("%s %s",str_CABLE_M,str_OUT_M);
  }
  else if (portType == PORT_TYPE_JACK || portType == PORT_TYPE_ITHRU) {
    portMax = SERIAL_INTERFACE_COUNT;
    SerialPrintf("%s %s %s",str_JACK_M,str_IN_M,portType == PORT_TYPE_ITHRU ? str_ITHRU_M:"");
  }
  else if (portType == PORT_TYPE_VIRTUAL) {
    portMax = VIRTUAL_INTERFACE_MAX;
    SerialPrintf("%s %s",str_VIRTUAL_M,str_IN_M);
  }
  else return ;

	SerialPrintf (" %s - Enter %s# (01-%02d) / 00 to exit) :",str_ROUTING_M,str_PORT,portMax);
	while (1) {
		choice = AsknNumber(2);
		if ( choice > 0 && choice > portMax) {
			SerialPrintf("%s. %s :",str_ERROR_B,str_TRYAGAIN_B);
		} else break;
	}
  if (choice == 0) return;
  port =  choice - 1;
  Serial.println();

  attachedSlot = (portType == PORT_TYPE_CABLE) ? EEPROM_Params.midiRoutingRulesCable[port].attachedSlot :
                  (portType == PORT_TYPE_JACK) ? EEPROM_Params.midiRoutingRulesJack[port].attachedSlot:
                   (portType == PORT_TYPE_VIRTUAL) ? EEPROM_Params.midiRoutingRulesVirtual[port].attachedSlot:
                    (portType == PORT_TYPE_ITHRU)  ? EEPROM_Params.midiRoutingRulesIntelliThru[port].attachedSlot:0;

  // Slot
  SerialPrintf("Enter pipeline slot 0-8, (0 = no slot) [%d]:",attachedSlot);
  while (1) {
    attachedSlot = AsknNumber(1);
    if ( attachedSlot == 0 || attachedSlot <= MIDI_TRANS_PIPELINE_SLOT_SIZE ) break;
    SerialPrintf("%s. %s :",str_ERROR_B,str_TRYAGAIN_B);
  }
  // Targets

  Serial.println();
  if (portType != PORT_TYPE_ITHRU) {
      cTargets = AskMidiRoutingTargets(portType,PORT_TYPE_CABLE,port);
      Serial.println();
      jTargets = AskMidiRoutingTargets(portType,PORT_TYPE_JACK,port);
      Serial.println();
      vTargets = AskMidiRoutingTargets(portType,PORT_TYPE_VIRTUAL,port);
  } else {

      if ( EEPROM_Params.midiRoutingRulesIntelliThru[port].jackOutTargetsMsk ||
          EEPROM_Params.midiRoutingRulesIntelliThru[port].virtualOutTargetsMsk) {
          // Enable of disable an existing ITHRU routing
          SerialPrintf("Keep existing %s",str_ROUTING);
          if (AskChoice("","") == 'y') {
              Serial.print((EEPROM_Params.intelliThruJackInMsk & (1 << port)) ? str_DISABLE_B:str_ENABLE_B);
              SerialPrintf(" thru mode %s",str_ROUTING);
              if (AskChoice("","") == 'y') {
                EEPROM_Params.intelliThruJackInMsk ^= (1 << port);
                SerialPrintf("%s %s %s.%n",str_JACK_B,str_IN_M,(EEPROM_Params.intelliThruJackInMsk & (1 << port)) ? str_ENABLED:str_DISABLED);
              }
              return;
          }
      }
      jTargets = AskMidiRoutingTargets(PORT_TYPE_JACK,PORT_TYPE_JACK,port);
      Serial.println();
      vTargets = AskMidiRoutingTargets(PORT_TYPE_JACK,PORT_TYPE_VIRTUAL,port);
  }
  Serial.println();

	if (portType == PORT_TYPE_CABLE ) {
			EEPROM_Params.midiRoutingRulesCable[port].attachedSlot = attachedSlot;
	    EEPROM_Params.midiRoutingRulesCable[port].cableInTargetsMsk = cTargets;
	    EEPROM_Params.midiRoutingRulesCable[port].jackOutTargetsMsk = jTargets ;
      EEPROM_Params.midiRoutingRulesCable[port].virtualOutTargetsMsk = vTargets ;
  }
	else if (portType == PORT_TYPE_JACK ) {
			EEPROM_Params.midiRoutingRulesJack[port].attachedSlot = attachedSlot;
	    EEPROM_Params.midiRoutingRulesJack[port].cableInTargetsMsk = cTargets;
	    EEPROM_Params.midiRoutingRulesJack[port].jackOutTargetsMsk = jTargets ;
      EEPROM_Params.midiRoutingRulesJack[port].virtualOutTargetsMsk = vTargets ;
  }
	else if (portType == PORT_TYPE_VIRTUAL ) {
      EEPROM_Params.midiRoutingRulesVirtual[port].attachedSlot = attachedSlot;
      EEPROM_Params.midiRoutingRulesVirtual[port].cableInTargetsMsk = cTargets;
      EEPROM_Params.midiRoutingRulesVirtual[port].jackOutTargetsMsk = jTargets ;
      EEPROM_Params.midiRoutingRulesVirtual[port].virtualOutTargetsMsk = vTargets ;
  }
  else if (portType == PORT_TYPE_ITHRU ) {
        EEPROM_Params.midiRoutingRulesIntelliThru[port].attachedSlot = attachedSlot;
        EEPROM_Params.midiRoutingRulesIntelliThru[port].jackOutTargetsMsk = jTargets ;
        EEPROM_Params.midiRoutingRulesIntelliThru[port].virtualOutTargetsMsk = vTargets ;
        if ( jTargets + vTargets == 0 ) {
          EEPROM_Params.intelliThruJackInMsk &= ~(1 << port); // Disable
        } else EEPROM_Params.intelliThruJackInMsk |= (1 << port); // enable Ithru for this port
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
  "2Usb VID PID",
  "3Usb product string",
  "4Cable OUT routing",
  "5Jack IN routing",
  "6IThru routing",
  "7USB idle time",
  "vVirtual IN routing",
  "pShow pipeline",
  "8Toggle bus mode",
  "9Set device Id",
  "aShow active devices",
  "dSYSEX settings dump",
  ".",
  "eReload settings",
  "fFactory settings",
  "rReset routing",
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
        ShowMidiRouting(PORT_TYPE_VIRTUAL);
        Serial.println();
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
				Serial.print("15s periods nb (001-127 / 000 to exit) :");
				i = AsknNumber(3,false);
				if (i == 0 || i >127 )
          SerialPrintf(". %s. No change made.%n",str_ERROR_B);
				else {
					EEPROM_Params.intelliThruDelayPeriod = i;
					SerialPrintf(" <= %s idle time set to %ds%n",str_USB_M,i*15);
				}
				Serial.println();
				showMenu = false;
				break;

				// Enable Bus mode
			case '8':
        SerialPrintf("%s bus mode",(EEPROM_Params.I2C_BusModeState == B_ENABLED)?str_DISABLE_B:str_ENABLE_B);
        if (AskChoice("","") == 'y' ) {
					EEPROM_Params.I2C_BusModeState = B_ENABLED;
          SerialPrintf("%s !%n",str_DONE_B);
				}
        else EEPROM_Params.I2C_BusModeState = B_DISABLED;
			  showMenu = false;
				break;

				// Set device ID.
			case '9':
	 			SerialPrintf("%s ( %02d:Master, Slaves %02d-%02d ) :",str_DEVICE_ID_B,B_MASTERID,B_SLAVE_DEVICE_BASE_ADDR,B_SLAVE_DEVICE_LAST_ADDR);
				while (1) {
					i = AsknNumber(2);
					if ( i != B_MASTERID ) {
						if ( i > B_SLAVE_DEVICE_LAST_ADDR || i < B_SLAVE_DEVICE_BASE_ADDR ) {
								SerialPrintf("%s. %s :",str_ERROR_B,str_TRYAGAIN_B);
						} else break;
					} else break;
				}
				EEPROM_Params.I2C_DeviceId = i;
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
        SerialPrintf("Reload %s",str_SETTINGS);
        if (AskChoice("","") == 'y' ) {
					EEPROM_ParamsInit();
          SerialPrintf("%s !%n",str_DONE_B);
				}
				showMenu = false;
				break;

			// Restore factory settings
			case 'f':
        SerialPrintf("Restore factory %s",str_SETTINGS);
        if (AskChoice("","") == 'y' ) {
          if (AskChoice("Sure","") == 'y' ) {
						EEPROM_ParamsInit(true);
            SerialPrintf("%s !%n",str_DONE_B);
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
        if (AskChoice("Reload default routing","") == 'y') {
					ResetMidiRoutingRules(ROUTING_RESET_ALL);
          SerialPrintf("%s !%n",str_DONE_B);
        }
				Serial.println();
				showMenu = false;
				break;

			// Save & quit
			case 's':
				ShowGlobalSettings();
				Serial.println();
        SerialPrintf("Save %s",str_SETTINGS);
        if (AskChoice("","") == 'y' ) {
					//Goto midi mode at the next boot
					EEPROM_Params.nextBootMode = bootModeMidi;
					EEPROM_ParamsSave();
					delay(100);
          SerialPrintf("%s !%n",str_DONE_B);
					showMenu = false;
				}
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
