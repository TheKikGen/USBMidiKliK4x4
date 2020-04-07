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
//  INTERNAL SYSEX IMPLEMENTATION
//-----------------------------------------------------------------------------
// Due to the unusual make process of Arduino platform, this file is directly
// included in the main ino one.
///////////////////////////////////////////////////////////////////////////////
#pragma once

///////////////////////////////////////////////////////////////////////////////
//  FUNCTIONS PROTOTYPES
///////////////////////////////////////////////////////////////////////////////
boolean SysExInternal_Process(uint8_t, uint8_t *);
boolean SysExInternal_Parse(uint8_t, midiPacket_t *);
uint8_t SysexInternal_DumpConf(uint32_t , uint8_t *);
void    SysexInternal_DumpSendConfToStream(uint32_t sxAddr,uint8_t dest);
void    SysexInternal_DumpFullConfToStream(uint8_t dest);
void    SysExInternal_SendFnACK(uint8_t ,uint8_t ) ;

uint8_t SysExInternal_fnDumpConfig(uint8_t ,uint8_t *);
uint8_t SysExInternal_fnIdentityRequest(uint8_t ,uint8_t *);
uint8_t SysExInternal_fnConfigMode(uint8_t ,uint8_t *);
uint8_t SysExInternal_fnHardReset(uint8_t ,uint8_t *);
uint8_t SysExInternal_fnSetUsbProductString(uint8_t ,uint8_t *);
uint8_t SysExInternal_fnSetUsbPidVid(uint8_t ,uint8_t *);
uint8_t SysExInternal_fnIThruSettings(uint8_t ,uint8_t *);
uint8_t SysExInternal_fnMidiRoutingSettings(uint8_t ,uint8_t *);
uint8_t SysExInternal_fnBusModeSettings(uint8_t ,uint8_t *);
uint8_t SysExInternal_fnPipelinesSettings(uint8_t ,uint8_t *);


///////////////////////////////////////////////////////////////////////////////
// Sysex is used to set some parameters of the interface.
// Be aware that the 0x77 manufacturer id is reserved in the MIDI standard (but not used)
// The second byte is usually an id number or a func code + the midi channel (forced to 0x77 here)
// The Third is the product id
#define SYSEX_MANUFACTURER_ID 0x77
// UsbMidiKlik multi-port interface STM32F103 family
#define SYSEX_PRODUCT_FAMILY 0x0,0x01
#define SYSEX_MODEL_NUMBER 0x00,0x78
#define SYSEX_INTERNAL_HEADER 0xF0,SYSEX_MANUFACTURER_ID,0x77,0x78
#define SYSEX_INTERNAL_CMD_ACK SYSEX_INTERNAL_HEADER,0x06,0x03,0,0XF7
#define SYSEX_INTERNAL_IDENTITY_RQ_REPLY 0xF0,0x7E,0x7F,0x06,0x02,\
        SYSEX_MANUFACTURER_ID,SYSEX_PRODUCT_FAMILY,SYSEX_MODEL_NUMBER,VERSION_MAJOR,VERSION_MINOR,0x00,0X00,0xF7

const uint8_t sysExInternalHeader[] = {SYSEX_INTERNAL_HEADER} ;
const uint8_t sysExInternalIdentityRqReply[] = {SYSEX_INTERNAL_IDENTITY_RQ_REPLY};

// ACK.  Byte before F7 will usually contain 00 or error code when a command send back an ack..
uint8_t sysExInternalCommandACK[] = {SYSEX_INTERNAL_CMD_ACK};

enum SysExInternal_Error {
  SX_NO_ERROR,
  SX_NO_ACK,
  SX_NO_REBOOT,
  SX_ERROR_ANY,
  SX_ERROR_BAD_MSG_SIZE,
  SX_ERROR_BAD_PORT,
  SX_ERROR_BAD_PORT_TYPE,
  SX_ERROR_BAD_SLOT,
  SX_ERROR_BAD_VALUE,
  SX_ERROR_BAD_DEVICEID,
} ;

boolean sysExFunctionAckToggle = false;

///////////////////////////////////////////////////////////////////////////////
//  Midi SYSEX functions vector
// ----------------------------------------------------------------------------
// FN Description
//
// 05 Configuration sysex dump
// 06 General Information
// 08 Reboot in configuration mode
// 0A Hardware reset
// 0B Set USB product string
// 0C Set USB vendor id and product id
// 0E Intelligent thru mode settings
// 0F Midi routing settings
// 10 Bus mode settings
// 11 Midi transformation pipelines settings
///////////////////////////////////////////////////////////////////////////////
enum SysExInternal_FnId {
  FN_SX_DUMP              = 0X05,
  FN_SX_GEN_INFO          = 0X06,
  FN_SX_CONFIG_MODE       = 0x08,
  FN_SX_HARD_RESET        = 0X0A,
  FN_SX_USB_PR_STR_SET    = 0X0B,
  FN_SX_USB_VID_PID_SET   = 0X0C,
  FN_SX_ITHRU_SET         = 0X0E,
  FN_SX_MIDI_ROUTING_SET  = 0X0F,
  FN_SX_BUS_SET           = 0X10,
  FN_SX_PIPELINE_SET      = 0X11,
} ;

// Sysex function vector

typedef uint8_t (*SysExInternalFnP_t) (uint8_t portType,uint8_t *sxMsg) ;

typedef struct {
    uint8_t             fnId;
    SysExInternalFnP_t  fn;
    boolean             needACK;
    boolean             needStoreIfSucceed;
    boolean             needBusSyncIfSucceed;
    boolean             needResetIfSucceed;
} __packed SysExInternalFnVector_t;

#define FN_SX_VECTOR_SIZE 11

const SysExInternalFnVector_t SysExInternalFnVector[FN_SX_VECTOR_SIZE] = {
  { FN_SX_DUMP              ,&SysExInternal_fnDumpConfig,          false,false,false,false },
  { FN_SX_GEN_INFO          ,&SysExInternal_fnIdentityRequest,     true,false,false,false },
  { FN_SX_CONFIG_MODE       ,&SysExInternal_fnConfigMode,          false,true ,false,true  },
  { FN_SX_HARD_RESET        ,&SysExInternal_fnHardReset,           false,false,false,true  },
  { FN_SX_USB_PR_STR_SET    ,&SysExInternal_fnSetUsbProductString, true ,true ,false,false },
  { FN_SX_USB_VID_PID_SET   ,&SysExInternal_fnSetUsbPidVid,        true ,true ,false,false },
  { FN_SX_ITHRU_SET         ,&SysExInternal_fnIThruSettings,       true ,true ,true ,false },
  { FN_SX_MIDI_ROUTING_SET  ,&SysExInternal_fnMidiRoutingSettings, true ,true ,true ,false },
  { FN_SX_BUS_SET           ,&SysExInternal_fnBusModeSettings,     false,true ,false,true  },
  { FN_SX_PIPELINE_SET      ,&SysExInternal_fnPipelinesSettings,   true ,true ,true ,false },
};



///////////////////////////////////////////////////////////////////////////////
// Process internal USBMidiKlik SYSEX
// ----------------------------------------------------------------------------
// MidiKlik SYSEX are of the following form :
//
// F0 77 77 78   USBMIDIKliK header
// <xx>          USBMIDIKliK sysex command
// <dddddd...dd> data
// F7            EOX End of SYSEX
//
// sxMsg[0] length of the message (func id + data)
// sxMsg[1] function id
// sxMsg[2] data without EOX
///////////////////////////////////////////////////////////////////////////////
boolean SysExInternal_Process(uint8_t portType, uint8_t sxMsg[]) {

  if ( sxMsg[0] < 1 ) return false;
  for (uint8_t i=0 ; i < FN_SX_VECTOR_SIZE ; i++ ) {
      if ( SysExInternalFnVector[i].fnId == sxMsg[1] ) {
          uint8_t r =  SysExInternalFnVector[i].fn(portType,sxMsg);
          if ( sysExFunctionAckToggle && r != SX_NO_ACK && SysExInternalFnVector[i].needACK )
              SysExInternal_SendFnACK(portType,r);
          if ( r == SX_NO_ERROR ) {
            if ( SysExInternalFnVector[i].needStoreIfSucceed) EE_PrmSave();
            if ( SysExInternalFnVector[i].needBusSyncIfSucceed && (B_IS_MASTER) ) I2C_SlavesRoutingSyncFromMaster();
            if ( SysExInternalFnVector[i].needResetIfSucceed) nvic_sys_reset();
            return true;
          }
      }
  }

  return false;
}

///////////////////////////////////////////////////////////////////////////////
// Send ACK sysex msg.F0 77 77 78 06 03 <ack> F7
///////////////////////////////////////////////////////////////////////////////
void SysExInternal_SendFnACK(uint8_t portType,uint8_t errorCode) {

  sysExInternalCommandACK[6] = errorCode;
  if (portType == PORT_TYPE_CABLE && midiUSBCx) {
      // send to USB , cable 0
      USBMidi_SendSysExPacket(sysExInternalCommandACK,sizeof(sysExInternalCommandACK));
  } else
  if (portType == PORT_TYPE_JACK ) {
      // Send to serial port 0 being the only possible for sysex
      serialHw[0]->write(sysExInternalCommandACK,sizeof(sysExInternalCommandACK));
  }
}

///////////////////////////////////////////////////////////////////////////////
// PARSE INTERNAL SYSEX, either for serial or USB
// ----------------------------------------------------------------------------
// Internal sysex must be transmitted on the first cable/midi jack (whatever routing).
// Internal sysex are purely ignored on other cables or jack.
// Return true if an internal sysex message is ready
///////////////////////////////////////////////////////////////////////////////
boolean SysExInternal_Parse(uint8_t portType, midiPacket_t *pk,uint8_t sxMsg[])
{
    static unsigned sxMsgIdx = 0;
    static uint8_t  lastPortType = 0xFF;
		static bool 	  sxHeaderFound = false;

    // Check that parsing is done continueously on the same port
    // If not, reset parser.
    if ( sxMsgIdx && portType != lastPortType ) {
        sxMsgIdx = 0;  sxHeaderFound = false;
    }

    lastPortType = portType;

    // Cable/jack 0 only
    if ( (pk->packet[0] >> 4) > 0) return false;

		uint8_t cin   = pk->packet[0] & 0x0F ;

		// Only SYSEX and concerned packet on cable or serial 1

		if (cin > 7 || cin < 4 ) return false;
		if (cin == 4 && pk->packet[1] != 0xF0 && sxMsgIdx < 3 ) return false;
		if (cin > 4  && sxMsgIdx <3 ) return false;

		uint8_t pklen = ( cin == 4 ? 3 : cin - 4) ;
		uint8_t ev = 1;

    // Start storing the message in the msg buffer
    // If Message too big. don't store...

		for ( uint8_t i = 0 ; i< pklen ; i++ ) {

      if (sxHeaderFound) {
				if ( sxMsg[0] <  SYSEX_INTERNAL_BUFF_SIZE -1  ) {
						if (pk->packet[ev] != 0xF7) sxMsg[++sxMsg[0]]  = pk->packet[ev];
						ev++;
				}
			}	else

			if ( sysExInternalHeader[sxMsgIdx] == pk->packet[ev] ) {
				sxMsgIdx++; 	ev++;
				if ( sxMsgIdx >= sizeof(sysExInternalHeader) ) {
					sxHeaderFound = true;
					sxMsg[0] = 0; // Len of the sysex buffer
				}
			}
			else {
				// No match
				sxMsgIdx = 0; sxHeaderFound = false;
				return false;
			}
		}

		// End of SYSEX for a valid message ? => Process
		if (cin != 4  && sxHeaderFound ) {
			sxMsgIdx = 0;  sxHeaderFound = false;
      return true;
		}

    return false;
}


///////////////////////////////////////////////////////////////////////////////
// SYSEX VECTOR FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// 05 Configuration sysex dump
// F0 77 77 78 05 <fnId> <command id | 00 > <sub id1 |00 ><sub id2 | 00 > F7
// F0 77 77 78 05 E0 00 00 00
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnDumpConfig(uint8_t portType,uint8_t *sxMsg) {

  if (sxMsg[0] != 5) SX_ERROR_BAD_MSG_SIZE;

  // Make a pseudo 32 bits value to address sysex functions
  // In Little Endian: the value 0x 01 02 03 04 is stored as  0x 04 03 02 01
  uint32_t sxAddr =  sxMsg[2] + (sxMsg[3] << 8) + (sxMsg[4] << 16) + (sxMsg[5] << 24);
  uint8_t l = SysexInternal_DumpConf(sxAddr,sysExInternalBuffer) ;
  if (!l) return SX_ERROR_ANY;

  if (portType == PORT_TYPE_CABLE && midiUSBCx) {
    // send to USB , cable 0
    USBMidi_SendSysExPacket(sysExInternalBuffer,l);
    return SX_NO_ERROR;

  } else
  if (portType == PORT_TYPE_JACK ) {
    // Send to serial port 0 being the only possible for sysex
    serialHw[0]->write(sysExInternalBuffer,l);
    return SX_NO_ERROR;
  }
  return SX_ERROR_ANY;
}

///////////////////////////////////////////////////////////////////////////////
// 06 General Information
// 06 01 Identity request
// F0 77 77 78 06 01 F7
//
// 06 02 Sysex acknowledgment toggle (on/off)
// F0 77 77 78 06 02 F7
//
// 06 03 Sysex acknowledgment (received)
// F0 77 77 78 06 03 <ack> F7
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnIdentityRequest(uint8_t portType,uint8_t *sxMsg) {
  if ( sxMsg[2] == 0x01 && sxMsg[0] == 2 ) {
      if (portType == PORT_TYPE_CABLE && midiUSBCx) {
        // send to USB , cable 0
        USBMidi_SendSysExPacket(sysExInternalIdentityRqReply,sizeof(sysExInternalIdentityRqReply));
        return SX_NO_ACK; // Force no ACK in that case
      } else
      if (portType == PORT_TYPE_JACK ) {
        // Send to serial port 0 being the only possible for sysex
        serialHw[0]->write(sysExInternalIdentityRqReply,sizeof(sysExInternalIdentityRqReply));
        return SX_NO_ACK; // Force no ACK in that case
      }
  } else
  // Sysex acknowledgment toggle (on/off)
  if ( sxMsg[2] == 0x02 && sxMsg[0] == 2 ) {
    sysExFunctionAckToggle = ! sysExFunctionAckToggle;
    return SX_NO_ERROR;
  }

  return SX_ERROR_ANY;
}

///////////////////////////////////////////////////////////////////////////////
// 08 Reboot in configuration mode
// F0 77 77 78 08 F7
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnConfigMode(uint8_t portType,uint8_t *sxMsg) {
  // Set serial boot mode & Write the whole param struct
  if ( sxMsg[0] != 1 ) return SX_ERROR_BAD_MSG_SIZE;
  EE_Prm.nextBootMode = bootModeConfigMenu;
  return SX_NO_ERROR;
}

///////////////////////////////////////////////////////////////////////////////
// 0A Hardware reset
// F0 77 77 78 0A F7
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnHardReset(uint8_t portType,uint8_t *sxMsg) {
  if ( sxMsg[0] != 1 ) return SX_ERROR_BAD_MSG_SIZE;
  return SX_NO_ERROR;
}

///////////////////////////////////////////////////////////////////////////////
// 0B Set USB product string
// F0 77 77 78 0B <usb product string not accentuated> F7
// ---------------------------------------------------------------------------
// Copy the received string to the USB Product String Descriptor
// For MIDI protocol compatibility, and avoid a sysex encoding,
// accentuated ASCII characters, below 128 are non supported.
// Size without null termination is defined by USB_MIDI_PRODUCT_STRING_SIZE
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnSetUsbProductString(uint8_t portType,uint8_t *sxMsg) {

  if ( sxMsg[0] < 2 || (sxMsg[0]-1) > USB_MIDI_PRODUCT_STRING_SIZE )  return SX_ERROR_BAD_MSG_SIZE;

  // Store the new string in EEPROM
  memset(&EE_Prm.productString,0, sizeof(EE_Prm.productString));
  memcpy(&EE_Prm.productString,&sxMsg[2],sxMsg[0]-1);
  return SX_NO_ERROR;
}

///////////////////////////////////////////////////////////////////////////////
// 0C Set USB vendor ID and product ID
// F0 77 77 78 0C <vendor id:nn nn nn nn> <product id:nn nn nn nn> F7
// ---------------------------------------------------------------------------
// To respect a simple encoding of 7 bits bytes, each hex digit must be
// transmitted separately in a serialized way.
// The following example will set  VID to 0X8F12 and PID to 0X9067 :
// F0 77 77 78 0C 08 0F 01 02 09 00 06 07 F7
//                8  F  1  2  9  0  6  7
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnSetUsbPidVid(uint8_t portType,uint8_t *sxMsg) {
  if ( sxMsg[0] != 9 ) return SX_ERROR_BAD_MSG_SIZE;
  EE_Prm.vendorID = (sxMsg[2] << 12) + (sxMsg[3] << 8) +
                                 (sxMsg[4] << 4) + sxMsg[5] ;
  EE_Prm.productID= (sxMsg[6] << 12) + (sxMsg[7] << 8) +
                                 (sxMsg[8] << 4) + sxMsg[9] ;
  return SX_NO_ERROR;
}

///////////////////////////////////////////////////////////////////////////////
// 0E IntelligentThru routing is activated when USB is idle beyond <delay>
// ---------------------------------------------------------------------------
// 0E 00 Reset to default
// F0 77 77 78 0E 00 F7
//
// 0E 01 Disable all jackin - IThru off.
// F0 77 77 78 0E 01 F7
//
// 0E 02 Set USB idle delay
// F0 77 77 78 0E 02 < Number of 15s periods: 00-7F > F7
//
// 0E 03 Set jack routing
// F0 77 77 78 0E 03 <JackIn port > [<out port type><out ports list: nn...nn>] F7
// Allowed port type : jack=1 | virtual=2   port : 0-F   outportype/out ports list are optional.
// If only jackin is passed, this toggle on/off IThru if outpout ports exists.
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnIThruSettings(uint8_t portType,uint8_t *sxMsg) {

  uint8_t msgLen = sxMsg[0];
  uint8_t cmdId  = sxMsg[2];

  // reset to default midi thru routing
  if (cmdId == 0x00  && msgLen == 2) {
       ResetMidiRoutingRules(ROUTING_RESET_INTELLITHRU);
       return SX_NO_ERROR;
  } else

  // Disable all jackin ports = Off
  if (cmdId == 0x01  && msgLen == 2) {
       EE_Prm.ithruJackInMsk = 0;
       return SX_NO_ERROR;
  } else

  // Set USB Idle time. Min time is 1 period of 15 secondes. The max is 127 periods of 15 secondes.
  if (cmdId == 0x02  && msgLen == 3) {
      if ( sxMsg[3] < 1 || sxMsg[3] > 0X7F ) return SX_ERROR_BAD_VALUE;
      EE_Prm.ithruUSBIdleTimePeriod = sxMsg[3];
      return SX_NO_ERROR;
  }
  else

  // Set routing
  // F0 77 77 78 0E 03 <JackIn port > <out port type>[out ports list: nn...nn] F7
  //     (len) (id)(cmd)    01          01           0 1 2 3 4 5 6 7 8 9 A B C D E
  if (cmdId == 0x03 && msgLen >= 3  ) {
      uint8_t jackIn = sxMsg[3];

      if ( jackIn >= SERIAL_INTERFACE_COUNT) return SX_ERROR_BAD_PORT;

      // disable/enable (toggle) Intellithru for this port
      if (msgLen == 3 ) {
        if ( ( EE_Prm.rtRulesIthru[jackIn].jkOutTgMsk & (1 << jackIn) )||
             ( EE_Prm.rtRulesIthru[jackIn].vrOutTgMsk & (1 << jackIn) ) )
            EE_Prm.ithruJackInMsk ^= (1 << jackIn);  // Toggle
        else EE_Prm.ithruJackInMsk &= ~(1 << jackIn); // Off if no out ports
        return SX_NO_ERROR;
      }

      if (msgLen < 5 ) return SX_ERROR_BAD_MSG_SIZE;

      uint16_t *msk;
      uint8_t outPortType = sxMsg[4];

      if (outPortType == PORT_TYPE_JACK  ) {
        if ( msgLen > (SERIAL_INTERFACE_COUNT + 4) ) return SX_ERROR_BAD_MSG_SIZE;
        msk = &EE_Prm.rtRulesIthru[jackIn].jkOutTgMsk;
      }
      else
      if (outPortType == PORT_TYPE_VIRTUAL ) {
        if ( msgLen > (VIRTUAL_INTERFACE_MAX + 4) ) return SX_ERROR_BAD_MSG_SIZE;
        msk = &EE_Prm.rtRulesIthru[jackIn].vrOutTgMsk;
      }
      else return SX_ERROR_BAD_PORT_TYPE;

      // Set midi in jack msk
      EE_Prm.ithruJackInMsk |= (1 << jackIn);

      // Set target port out msk
      uint16_t newMsk = 0;
      for ( uint8_t i = 4 ; i <= msgLen  ; i++) {
          if ( sxMsg[i] < SERIAL_INTERFACE_COUNT )
                newMsk |= 	1 << sxMsg[i] ;
          else return SX_ERROR_BAD_PORT;
      }
      *msk = newMsk;

      // reset globals for a real time update
      intelliThruDelayMillis = EE_Prm.ithruUSBIdleTimePeriod * 15000;
      return SX_NO_ERROR;
  }
  return SX_ERROR_ANY;
}

///////////////////////////////////////////////////////////////////////////////
// 0F Midi routing
// ---------------------------------------------------------------------------
// 0F 00 Reset to default
// F0 77 77 78 0F 00 F7
//
// 0F 01 Set midi port routing
// F0 77 77 78 0F 01 <in port type> <in port> <out port type>[out ports list: nn...nn] F7
//
// 0F 01 Set midi port routing
// F0 77 77 78 0F 02 <in port> <out port type>[out ports list: nn...nn] F7 F7
// F0 77 77 78 0F 03 <JackIn port: 0-F > [jk out ports list: nn nn ... nn] F7
// port type : cable = 0 |jack=1 | virtual=2    port : 0-F    out ports list is optional.
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnMidiRoutingSettings(uint8_t portType,uint8_t *sxMsg) {
  uint8_t msgLen = sxMsg[0];
  uint8_t cmdId  = sxMsg[2];

  // reset to default routing
  if (cmdId == 0x00  && msgLen == 2) {
      ResetMidiRoutingRules(ROUTING_RESET_MIDIUSB);
      return SX_NO_ERROR;
  }
  else

  // Set Routing targets - (len) (fnid) (cmd) 00 0F 01  00 01 02 03
  if (cmdId == 0x01 && msgLen >= 5 ) {

    uint8_t inPortType  = sxMsg[3];
    uint8_t inPort      = sxMsg[4];
    uint8_t outPortType = sxMsg[5];

    if (inPortType == PORT_TYPE_CABLE ) {
      if ( inPort >= USBCABLE_INTERFACE_MAX ) return SX_ERROR_BAD_PORT;
    }
    else
    if (inPortType == PORT_TYPE_JACK ) {
      if ( inPort >= SERIAL_INTERFACE_COUNT ) return SX_ERROR_BAD_PORT;
    }
    else
    if (inPortType == PORT_TYPE_VIRTUAL ) {
      if ( inPort >= VIRTUAL_INTERFACE_MAX ) return SX_ERROR_BAD_PORT;
    }
    else return SX_ERROR_BAD_PORT_TYPE;

    if (outPortType == PORT_TYPE_CABLE ) {
      if (msgLen > (USBCABLE_INTERFACE_MAX + 5) ) return SX_ERROR_BAD_MSG_SIZE;
    }
    else
    if (outPortType == PORT_TYPE_JACK  ) {
      if ( msgLen > (SERIAL_INTERFACE_COUNT + 5) ) return SX_ERROR_BAD_MSG_SIZE;
    }
    else
    if (outPortType == PORT_TYPE_VIRTUAL ) {
      if ( msgLen > (VIRTUAL_INTERFACE_MAX + 5) ) return SX_ERROR_BAD_MSG_SIZE;
    }
    else return SX_ERROR_BAD_PORT_TYPE;

    uint16_t msk = 0;
    // If port list, Compute mask else no target
    if ( msgLen > 5 ) {
      for ( uint8_t i = 6 ; i <= msgLen  ; i++) {
          if ( (outPortType == PORT_TYPE_CABLE    && sxMsg[i] < USBCABLE_INTERFACE_MAX) ||
               (outPortType == PORT_TYPE_JACK     && sxMsg[i] < SERIAL_INTERFACE_COUNT) ||
               (outPortType == PORT_TYPE_VIRTUAL  && sxMsg[i] < VIRTUAL_INTERFACE_MAX) ) {
                 msk |= 	1 << sxMsg[i] ;
          }
      }// for
    }

    // Set masks Cable
    if (inPortType == PORT_TYPE_CABLE ) {
          if (outPortType == PORT_TYPE_CABLE)     EE_Prm.rtRulesCable[inPort].cbInTgMsk = msk;
          else if (outPortType == PORT_TYPE_JACK) EE_Prm.rtRulesCable[inPort].jkOutTgMsk = msk;
          else                                    EE_Prm.rtRulesCable[inPort].vrOutTgMsk = msk;
          return SX_NO_ERROR;
    }
    // Jack
    else if (inPortType == PORT_TYPE_JACK ) {
          if (outPortType == PORT_TYPE_CABLE)     EE_Prm.rtRulesJack[inPort].cbInTgMsk = msk;
          else if (outPortType == PORT_TYPE_JACK) EE_Prm.rtRulesJack[inPort].jkOutTgMsk = msk;
          else                                    EE_Prm.rtRulesJack[inPort].vrOutTgMsk = msk;
          return SX_NO_ERROR;
    }
    // Virtual
    else if (inPortType == PORT_TYPE_VIRTUAL ) {
          if (outPortType == PORT_TYPE_CABLE)     EE_Prm.rtRulesVirtual[inPort].cbInTgMsk = msk;
          else if (outPortType == PORT_TYPE_JACK) EE_Prm.rtRulesVirtual[inPort].jkOutTgMsk = msk;
          else                                    EE_Prm.rtRulesVirtual[inPort].vrOutTgMsk = msk;
          return SX_NO_ERROR;
    }
  } else

  // Clear all Routing rules, Ithru and pipelines
  if (cmdId == 0x02  && msgLen == 2) {
      ResetMidiRoutingRules(ROUTING_CLEAR_ALL);
      return SX_NO_ERROR;
  }

  return SX_ERROR_ANY;
}

///////////////////////////////////////////////////////////////////////////////
// 10 Bus mode
// ---------------------------------------------------------------------------
// 10 00 Enable/disable bus mode
// F0 77 77 78 10 00 < enable:1 | disable:0 > F7
// The device will reboot after the command is bus activated.
//
// 10 01 Set device ID
// F0 77 77 78 10 01 < deviceid:04-08 > F7
// deviceid must be set to 4 when master. The device will reboot after the command is bus active..
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnBusModeSettings(uint8_t portType,uint8_t *sxMsg) {
  uint8_t msgLen = sxMsg[0];
  uint8_t cmdId  = sxMsg[2];

  // Toogle Bus mode
  if (cmdId == 0x00 && msgLen == 3 )  {

      if ( sxMsg[3] == 1  && EE_Prm.I2C_BusModeState == B_DISABLED)
              EE_Prm.I2C_BusModeState = B_ENABLED;
      else if ( sxMsg[3] == 0 && EE_Prm.I2C_BusModeState == B_ENABLED ) {
              EE_Prm.I2C_BusModeState = B_DISABLED;
              return SX_NO_REBOOT;
      }
      else return SX_ERROR_ANY;
      return SX_NO_ERROR;
  }
  else

  // Set device Id
  if (cmdId == 0x01 && msgLen == 3 )  {
      if ( sxMsg[3] > B_SLAVE_DEVICE_LAST_ADDR || sxMsg[3] < B_SLAVE_DEVICE_BASE_ADDR ) return SX_ERROR_BAD_DEVICEID;
      if ( sxMsg[3] != EE_Prm.I2C_DeviceId ) {
        EE_Prm.I2C_DeviceId = sxMsg[3];
        if (EE_Prm.I2C_BusModeState = B_ENABLED) return SX_NO_ERROR;
        return SX_NO_REBOOT;
      }
  }
  return SX_ERROR_ANY;
}

///////////////////////////////////////////////////////////////////////////////
// 11 Midi transformation pipelines
// ---------------------------------------------------------------------------
// 11 00 Slot operation - 00 slot copy
// F0 77 77 78 11 00 00 < src slot:01-08 > < dest slot: 01-08 > F7
//
// 11 00 Slot operation - 01 clear slot
// F0 77 77 78 11 00 01 < slot: 01-08 | All slot:7F > F7
//
// 11 00 Slot operation - 02 slot attach port
// F0 77 77 78 11 00 02 < in port type> < in port: 0-F> <slot: 0-8> F7
// port type : cable = 0 | jack = 1 | virtual = 2 | ithru = 3
// When slot = 0, the port is considered as detached from any slot.
//
// 11 01 Pipe operation - 00 add pipe
// F0 77 77 78 11 01 00 <slot:1-8> <pipe id: nn> <prm: nn nn nn nn > F7
//
// 11 01 Pipe operation - 01 insert pipe before
// F0 77 77 78 11 01 01 <slot:1-8> <pipe idx:nn> <pipe id:nn> <prm:nn nn nn nn > F7
//
// 11 01 Pipe operation - 02 replace pipe
// F0 77 77 78 11 01 02 <slot:1-8> <pipe idx:nn> <pipe id:nn> <prm:nn nn nn nn > F7
//
// 11 01 Pipe operation - 03 clear pipe index
// F0 77 77 78 11 01 03 <slot:1-8> <pipe idx:nn> F7
//
// 11 01 Pipe operation - 04 clear first pipe id
// F0 77 77 78 11 01 04 <slot:1-8> <pipe id:nn> F7
//
// 11 01 Pipe operation - 05 pipe bypass index
// F0 77 77 78 11 01 05 <slot:1-8> <pipe index:nn> <no bypass!0 | bypass:1> F7
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnPipelinesSettings(uint8_t portType,uint8_t *sxMsg) {
  uint8_t msgLen    = sxMsg[0];
  uint8_t cmdId     = sxMsg[2];
  uint8_t cmdSubId  = sxMsg[3];

  // SLOTS OPERATIONS
  if (cmdId == 0x00 ) {
    // Copy slot
    if (cmdSubId == 0x00  && msgLen == 5)
      return TransPacketPipeline_CopySlot(sxMsg[4],sxMsg[5]) ? SX_NO_ERROR : SX_ERROR_ANY ;
    else    // Clear slot <Slot number 1-8> <0x7F = ALL SLOTS>
    if (cmdSubId == 0x01  && msgLen == 4)
      return TransPacketPipeline_ClearSlot(sxMsg[4]) ? SX_NO_ERROR : SX_ERROR_ANY;
    else  // Attach/Detach port to slot
    if (cmdSubId == 0x02  && msgLen == 6)
      return TransPacketPipeline_AttachPort(sxMsg[4],sxMsg[5],sxMsg[6]) ? SX_NO_ERROR : SX_ERROR_ANY;
    else return SX_ERROR_ANY;
  }
  else

  // PIPE OPERATIONS
  if (cmdId == 0x01 ) {
    //  11 01  <00 = Add pipe>  <slot number 01-08> <FN id> <par1> <par2> <par3> <par4>
    if (cmdSubId == 0x00  && msgLen == 9) {
        transPipe_t p;
        p.pId = sxMsg[5];  p.byPass = 0;
        p.par1 = sxMsg[6]; p.par2 = sxMsg[7];
        p.par3 = sxMsg[8]; p.par4 = sxMsg[9];
        return TransPacketPipe_AddToSlot(sxMsg[4],&p) ? SX_NO_ERROR : SX_ERROR_ANY;
    }
    else
    // 11 01 <01 = Insert before>  <slot number> <pipe index 0-n> <FN id> <par1> <par2> <par3> <par4>
    if (cmdSubId == 0x01  && msgLen == 10) {
      transPipe_t p;
      p.pId = sxMsg[6];  p.byPass = 0;
      p.par1 = sxMsg[7]; p.par2 = sxMsg[8];
      p.par3 = sxMsg[9]; p.par4 = sxMsg[10];
      return TransPacketPipe_InsertToSlot(sxMsg[4],sxMsg[5],&p,false)  ? SX_NO_ERROR : SX_ERROR_ANY;
    }
    else
    // 11 01 <02 = Replace> <slot number> <pipe index 0-n> <FN id> <par1> <par2> <par3> <par4>
    if (cmdSubId == 0x02  && msgLen == 10) {
      transPipe_t p;
      p.pId = sxMsg[6];  p.byPass = 0;
      p.par1 = sxMsg[7]; p.par2 = sxMsg[8];
      p.par3 = sxMsg[9]; p.par4 = sxMsg[10];
      return TransPacketPipe_InsertToSlot(sxMsg[4],sxMsg[5],&p,true)  ? SX_NO_ERROR : SX_ERROR_ANY;
    }
    else
    // Clear pipe by index
    if (cmdSubId == 0x03  && msgLen == 5 )
      return TransPacketPipe_ClearSlotIndexPid(sxMsg[4],true,sxMsg[5])  ? SX_NO_ERROR : SX_ERROR_ANY;
    else
    // Clear pipe first pId
    if (cmdSubId == 0x04  && msgLen == 5 )
      return TransPacketPipe_ClearSlotIndexPid(sxMsg[4],false,sxMsg[5])  ? SX_NO_ERROR : SX_ERROR_ANY;
    else
    // ByPass pipe by index
    if (cmdSubId == 0x05  && msgLen == 6 )
      return TransPacketPipe_ByPass(sxMsg[4],sxMsg[5],sxMsg[6]) ? SX_NO_ERROR : SX_ERROR_ANY;
  }

  return SX_ERROR_ANY;
}

///////////////////////////////////////////////////////////////////////////////
// Send a specific address SYSEX dump to the appropriate destination stream
// 0 : Cable 0
// 1 : Jack port 0
// 2 : HexDump to serial 0
///////////////////////////////////////////////////////////////////////////////
void SysexInternal_DumpSendConfToStream(uint32_t sxAddr,uint8_t dest) {

  uint16_t l;
  // Build a stream according to the address
  l = SysexInternal_DumpConf(sxAddr, sysExInternalBuffer);
  if ( l && dest == 2 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
  else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
  else if ( l && dest == 0 ) USBMidi_SendSysExPacket(sysExInternalBuffer,l);
}
///////////////////////////////////////////////////////////////////////////////
// Generate a full sysex config dump to the appropriate destination stream
// 0 : Cable 0
// 1 : Jack port 0
// 2 : HexDump to serial 0
///////////////////////////////////////////////////////////////////////////////
void SysexInternal_DumpFullConfToStream(uint8_t dest) {

  // Usb string , PID, VID
  SysexInternal_DumpSendConfToStream(0x0B000000,dest);
  SysexInternal_DumpSendConfToStream(0x0C000000,dest);
  // Ithru USB Idle, routing,
  SysexInternal_DumpSendConfToStream(0x0E020000,dest);
  for (  uint32_t i=0; i!= B_SERIAL_INTERFACE_MAX ; i++)
        SysexInternal_DumpSendConfToStream(0x0E030000 + (i << 8 ), dest);

  // Cable out Routing
  for (  uint32_t i=0; i!= USBCABLE_INTERFACE_MAX ; i++)
        SysexInternal_DumpSendConfToStream(0x0F010000 + i , dest);

  // jack In Routing,
  for (  uint32_t i=0; i!= B_SERIAL_INTERFACE_MAX ; i++)
        SysexInternal_DumpSendConfToStream(0x0F010100 + i , dest);

  // Virtual In Routing,
  for (  uint32_t i=0; i!= VIRTUAL_INTERFACE_MAX ; i++)
        SysexInternal_DumpSendConfToStream(0x0F010200 + i , dest);

  // Cable out attached slot
  for (  uint32_t i=0; i!= USBCABLE_INTERFACE_MAX ; i++)
        SysexInternal_DumpSendConfToStream(0x11000000 + i , dest);

  // Jack In attached slot
  for (  uint32_t i=0; i!= B_SERIAL_INTERFACE_MAX ; i++)
        SysexInternal_DumpSendConfToStream(0x11000100 + i , dest);

  // Virtual In attached slot
  for (  uint32_t i=0; i!= VIRTUAL_INTERFACE_MAX ; i++)
        SysexInternal_DumpSendConfToStream(0x11000200 + i , dest);

  // Ithru attached slot
  for (  uint32_t i=0; i!= B_SERIAL_INTERFACE_MAX ; i++)
        SysexInternal_DumpSendConfToStream(0x11000300 + i , dest);

  // PIPES Dump : 11 01 <slot> < pipe index>
  for (  uint32_t s=1; s <= TRANS_PIPELINE_SLOT_SIZE ; s++) {
    for (  uint32_t p=0; p != TRANS_PIPELINE_SIZE ; p++)
      SysexInternal_DumpSendConfToStream(0x11010000 + (s << 8) + p, dest);
  }

  // Bus settings. Commented because reset the board.
  //SysexInternal_DumpSendConfToStream(0x10000000, dest);
}

///////////////////////////////////////////////////////////////////////////////
// Return current configuration as a SYSEX buffer
///////////////////////////////////////////////////////////////////////////////
uint8_t SysexInternal_DumpConf(uint32_t sxAddr, uint8_t *buff) {

  uint8_t *buff2 = buff;

  // Copy header
  memcpy(buff2,sysExInternalHeader,sizeof(sysExInternalHeader));
  buff2+=sizeof(sysExInternalHeader);
  // Copy FnId - Little endian
  uint8_t fnId = (sxAddr>>24) ; // fn000000 => 000000fn little endian

  *buff2 = fnId ;

  // 0B Set USB product string
  // Dump : 0B
  // Generate F0 77 77 78 0B <usb product string:nn...nn> F7
  if ( sxAddr == 0x0B000000 ) {
    strcpy((char*)++buff2,(char*)EE_Prm.productString);
    buff2+=strlen((char*)EE_Prm.productString)-1;
    *(++buff2) = 0xF7;
    return buff2-buff+1;
  }

  // 0C Set USB vendor ID and product ID
  // Dump : 0C
  // Generate F0 77 77 78 0C <vendor id:nn nn nn nn> <product id:nn nn nn nn> F7
  if ( sxAddr == 0x0C000000 ) {
      *(++buff2) = EE_Prm.vendorID >> 12;
      *(++buff2) = (EE_Prm.vendorID & 0x0F00) >> 8;
      *(++buff2) = (EE_Prm.vendorID & 0x00F0) >> 4;
      *(++buff2) = (EE_Prm.vendorID & 0x000F) ;
      *(++buff2) = EE_Prm.productID >> 12;
      *(++buff2) = (EE_Prm.productID & 0x0F00) >> 8;
      *(++buff2) = (EE_Prm.productID & 0x00F0) >> 4;
      *(++buff2) = (EE_Prm.productID & 0x000F) ;
      *(++buff2) = 0xF7;
      return buff2-buff+1;
  }

  // 0E Intelligent thru - 02 Set USB idle
  // Dump : 0E 02
  // Generate F0 77 77 78 0E 02 < Number of 15s periods: 00-7F > F7
  if ( sxAddr == 0x0E020000 ) {
      *(++buff2) = 0X02;
      *(++buff2) = EE_Prm.ithruUSBIdleTimePeriod;
      *(++buff2) = 0xF7;
      return buff2-buff+1;
  }

  // 0E Intelligent thru - 03 Set jack routing
  // Dump 0E 03 <Jack In>
  // Generate F0 77 77 78 0E 03 <JackIn port: 0-F> [jk out ports list: nn...nn] F7
  if ( sxAddr >= 0x0E030000 && sxAddr <= 0x0E030F00 ) {
      *(++buff2) = 0X03;
      uint8_t jackIn = ( (sxAddr<<16) >> 24 ) ;
      *(++buff2) = jackIn;
      uint8_t jackOut=0;
      for (  uint8_t i=0; i!= B_SERIAL_INTERFACE_MAX ; i++) {
      	     if ( EE_Prm.rtRulesIthru[jackIn].jkOutTgMsk & ( 1 << i) ) {
              *(++buff2) = i;
              jackOut++;
           }
      }
      if (!jackOut) return 0;
      *(++buff2) = 0xF7;
      // If out ports and no jack in, then generate a "disable" but keep the routing
      if ( ! (EE_Prm.ithruJackInMsk & (1 << jackIn) ) ) {
        memcpy(++buff2,sysExInternalHeader,sizeof(sysExInternalHeader));
        buff2+=sizeof(sysExInternalHeader);
        *buff2 = fnId ;
        *(++buff2) = 0X03; // cmdId
        *(++buff2) = jackIn;
        *(++buff2) = 0xF7;
      }

      return buff2-buff+1;
  }

  // 0F Midi routing - 01 Set midi port routing
  // Dump : 0F 01 <inport type> <inport>
  // Generate F0 77 77 78 0F 01 <inport type> <inport> <outport type>[outports list: nn...nn] F7
  if ( sxAddr >= 0x0F010000 && sxAddr <= 0x0F01020F ) {
      uint8_t inPortType = ( (sxAddr<<16) >> 24 );
      uint8_t inPort = ( sxAddr & 0x0000000FF );
      uint16_t cmsk = 0;
      uint16_t jmsk = 0;

      *(++buff2) = 0X01; // cmdId
      *(++buff2) = inPortType;
      *(++buff2) = inPort;

      if (inPortType == 0 ) {  // Cable
        cmsk = EE_Prm.rtRulesCable[inPort].cbInTgMsk;
        jmsk = EE_Prm.rtRulesCable[inPort].jkOutTgMsk;
      }
      else if (inPortType == 1 ){  // jack
        cmsk = EE_Prm.rtRulesJack[inPort].cbInTgMsk;
        jmsk = EE_Prm.rtRulesJack[inPort].jkOutTgMsk;
      }
      else if (inPortType == 2 ) {  // Virtual
        cmsk = EE_Prm.rtRulesVirtual[inPort].cbInTgMsk;
        jmsk = EE_Prm.rtRulesVirtual[inPort].jkOutTgMsk;
      } else return 0;

      if ( ! (cmsk + jmsk) ) return 0;

      // Cable targets
      if ( cmsk) {
        *(++buff2) = 0 ; // outport Type cable
        for ( uint8_t  i = 0 ; i != USBCABLE_INTERFACE_MAX  ; i++)
            if ( cmsk & ( 1 << i) ) *(++buff2) = i;
        if (jmsk) {
          // Add a new header for jack out targets
          *(++buff2) = 0xF7;
          memcpy(++buff2,sysExInternalHeader,sizeof(sysExInternalHeader));
          buff2+=sizeof(sysExInternalHeader);
          *buff2 = fnId ;
          *(++buff2) = 0X01; // cmdId
          *(++buff2) = inPortType;
          *(++buff2) = inPort;
        }
      }
      if (jmsk) {
        *(++buff2) = 1 ; // outport Type jack
        for ( uint8_t i = 0 ; i != B_SERIAL_INTERFACE_MAX  ; i++)
            if ( jmsk & ( 1 << i) )  *(++buff2) = i;
      }

      *(++buff2) = 0xF7;
      return buff2-buff+1;
  }

 // 10 Bus mode
 // Dump : 10
 // Generate :
 // F0 77 77 78 10 00 < enable:1 | disable:0 > F7
 // F0 77 77 78 10 01 < deviceid:04-08 > F7

  if ( sxAddr == 0x10000000 ) {

    *(++buff2) = 0X00; // cmdId : mode
    *(++buff2) = EE_Prm.I2C_BusModeState ; // Mode value

    // Add a new header
    *(++buff2) = 0xF7;
    memcpy(++buff2,sysExInternalHeader,sizeof(sysExInternalHeader));
    buff2+=sizeof(sysExInternalHeader);
    *buff2 = fnId;
    *(++buff2) = 0X01; // cmdId : deviceId
    *(++buff2) = EE_Prm.I2C_DeviceId ; // Device Id value

    *(++buff2) = 0xF7;
    return buff2-buff+1;

  }


  // 11 Midi transformation pipelines - Attached slots to port
  // Dump : 11 00 < in port type> < in port: 0-F>
  // Generates : F0 77 77 78 11 00 02 < in port type> < in port: 0-F> <slot: 0-8> F7

  if ( sxAddr >= 0x11000000 && sxAddr <= 0x1100030F ) {

    uint8_t inPortType = ( (sxAddr<<16) >> 24 );
    uint8_t inPort = ( sxAddr & 0x0000000FF );

    // Attached ports
    // F0 77 77 78 11 00 02 < in port type> < in port: 0-F> <slot: 0-8> F7
    *(++buff2) = 0X00; // Slot operations
    *(++buff2) = 0X02; // Attach port
    *(++buff2) = inPortType;
    *(++buff2) = inPort;
    if ( inPortType == 0) *(++buff2) = EE_Prm.rtRulesCable[inPort].slot;
    else if ( inPortType == 1) *(++buff2) = EE_Prm.rtRulesJack[inPort].slot;
    else if ( inPortType == 2) *(++buff2) = EE_Prm.rtRulesVirtual[inPort].slot;
    else if ( inPortType == 3) *(++buff2) = EE_Prm.rtRulesIthru[inPort].slot;
    else return 0;

    // Don't take unattached ports to reduce sysex size
    if (*buff2 == 0) return 0;

    *(++buff2) = 0xF7;
    return buff2-buff+1;
  }

  // Pipes - Add in slot
  // 11 01 Pipe operation - 00 add pipe
  // Dump : 11 01 <slot> < pipe index>
  // Generates
  // F0 77 77 78 11 01 00 <slot:1-8> <pipe id: nn> <prm: nn nn nn nn > F7
  // F0 77 77 78 11 01 05 <slot:1-8> <pipe index:nn> <no bypass!0 | bypass:1> F7

  if ( sxAddr >= 0x11010100 && sxAddr <= 0x11017F7F ) {
    uint8_t slot = ( (sxAddr<<16) >> 24 ) ;
    uint8_t pipeIndex = ( sxAddr & 0x0000000FF );
    if (slot < 1 || slot > TRANS_PIPELINE_SLOT_SIZE ) return 0;
    if (pipeIndex > TRANS_PIPELINE_SIZE ) return 0;
    if ( EE_Prm.pipelineSlot[--slot].pipeline[pipeIndex].pId == FN_TRANSPIPE_NOPIPE) return 0;
    *(++buff2) = 0X01; // Pipe operations
    *(++buff2) = 0X00; // Add Pipe
    *(++buff2) = slot+1;
    *(++buff2) = EE_Prm.pipelineSlot[slot].pipeline[pipeIndex].pId;
    *(++buff2) = EE_Prm.pipelineSlot[slot].pipeline[pipeIndex].par1;
    *(++buff2) = EE_Prm.pipelineSlot[slot].pipeline[pipeIndex].par2;
    *(++buff2) = EE_Prm.pipelineSlot[slot].pipeline[pipeIndex].par3;
    *(++buff2) = EE_Prm.pipelineSlot[slot].pipeline[pipeIndex].par4;
    // Add a new header to generate the bypass command
    *(++buff2) = 0xF7;
    memcpy(++buff2,sysExInternalHeader,sizeof(sysExInternalHeader));
    buff2+=sizeof(sysExInternalHeader);
    *buff2 = fnId;
    *(++buff2) = 0X01; // Pipe operations
    *(++buff2) = 0X05; // cmdId : ByPass
    *(++buff2) = slot+1;
    *(++buff2) = pipeIndex;
    *(++buff2) = EE_Prm.pipelineSlot[slot].pipeline[pipeIndex].byPass;

    *(++buff2) = 0xF7;
    return buff2-buff+1;
  }

  return 0;
}
