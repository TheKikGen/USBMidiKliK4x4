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
uint8_t SysexInternal_DumpConf(uint32_t , uint8_t ,uint8_t *);
void    SysexInternal_DumpToStream(uint8_t ) ;
void SysExInternal_SendCommandACK(uint8_t ,uint8_t ) ;

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
#define SYSEX_INTERNAL_CMD_ACK SYSEX_INTERNAL_HEADER,0x7F,00,0XF7
#define SYSEX_INTERNAL_IDENTITY_RQ_REPLY 0xF0,0x7E,0x7F,0x06,0x02,\
        SYSEX_MANUFACTURER_ID,SYSEX_PRODUCT_FAMILY,SYSEX_MODEL_NUMBER,VERSION_MAJOR,VERSION_MINOR,0x00,0X00,0xF7

const uint8_t sysExInternalHeader[] = {SYSEX_INTERNAL_HEADER} ;
const uint8_t sysExInternalIdentityRqReply[] = {SYSEX_INTERNAL_IDENTITY_RQ_REPLY};

// ACK.  Byte before F7 will usually contain 00 or error code when a command send back an ack..
uint8_t sysExInternalCommandACK[] = {SYSEX_INTERNAL_CMD_ACK};

enum SysExInternal_Error {
  SX_ERROR_NO_ERROR,
  SX_ERROR_ANY,
  SX_ERROR_BAD_MSG_SIZE,
  SX_ERROR_BAD_PORT,
  SX_ERROR_BAD_SLOT,
  SX_ERROR_BAD_VALUE,
  SX_ERROR_BAD_DEVICEID,
} ;

///////////////////////////////////////////////////////////////////////////////
//  Midi SYSEX functions vector
///////////////////////////////////////////////////////////////////////////////

// 0x05 Sysex configuration dump
// 0x06 General Information. Identity request.
// 0x08 Reboot in config mode
// 0x0A Hard reset interface
// 0x0B Set USB Product string
// 0X0C Set USB VID / PID
// 0X0E Intelligent Midi thru mode settings
// 0X0F Midi routing settings
// 0x10 Bus mode settings
// 0x11 Midi transformation pipelines settings

enum SysExInternal_FnId {
  FN_SX_DUMP              = 0X05,
  FN_SX_IDREQUEST         = 0X06,
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

typedef uint8_t (*SysExInternalFnP_t) (uint8_t source,uint8_t *sxMsg) ;

typedef struct {
    uint8_t             fnId;
    SysExInternalFnP_t  fn;
    boolean             needACK;
    boolean             needStoreIfSucceed;
    boolean             needBusSyncIfSucceed;
    boolean             needResetIfSucceed;
} __packed SysExInternalFnVector_t;

#define FN_SX_VECTOR_SIZE 10

const SysExInternalFnVector_t SysExInternalFnVector[FN_SX_VECTOR_SIZE] = {
  { FN_SX_IDREQUEST         ,&SysExInternal_fnIdentityRequest,     false,false,false,false },
  { FN_SX_DUMP              ,&SysExInternal_fnDumpConfig,          false,false,false,false },
  { FN_SX_CONFIG_MODE       ,&SysExInternal_fnConfigMode,          false,true ,false,true  },
  { FN_SX_HARD_RESET        ,&SysExInternal_fnHardReset,           false,false,false,true  },
  { FN_SX_USB_PR_STR_SET    ,&SysExInternal_fnSetUsbProductString, false,true ,false,false },
  { FN_SX_USB_VID_PID_SET   ,&SysExInternal_fnSetUsbPidVid,        false,true ,false,false },
  { FN_SX_ITHRU_SET         ,&SysExInternal_fnIThruSettings,       false,true ,true,false  },
  { FN_SX_MIDI_ROUTING_SET  ,&SysExInternal_fnMidiRoutingSettings, false,true ,true,false  },
  { FN_SX_BUS_SET           ,&SysExInternal_fnBusModeSettings,     false,true ,false,true  },
  { FN_SX_PIPELINE_SET      ,&SysExInternal_fnPipelinesSettings,   true,true ,true,false  },
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
boolean SysExInternal_Process(uint8_t source, uint8_t sxMsg[]) {

  if ( sxMsg[0] < 1 ) return false;
  // If slave on Bus : No Sysex
  if ( B_IS_SLAVE ) return false;

  for (uint8_t i=0 ; i < FN_SX_VECTOR_SIZE ; i++ ) {
      if ( SysExInternalFnVector[i].fnId == sxMsg[1] ) {
          uint8_t r =  SysExInternalFnVector[i].fn(source,sxMsg);
          if ( SysExInternalFnVector[i].needACK) SysExInternal_SendCommandACK(source,r);
          if ( r == SX_ERROR_NO_ERROR ) {
            if ( SysExInternalFnVector[i].needStoreIfSucceed) EEPROM_ParamsSave();
            if ( SysExInternalFnVector[i].needResetIfSucceed) nvic_sys_reset();
            if ( SysExInternalFnVector[i].needBusSyncIfSucceed && (B_IS_MASTER) ) I2C_SlavesRoutingSyncFromMaster();
            return true;
          }
      }
  }

  return false;
}

///////////////////////////////////////////////////////////////////////////////
// Send ACK sysex msg.
///////////////////////////////////////////////////////////////////////////////
void SysExInternal_SendCommandACK(uint8_t source,uint8_t errorCode) {

  sysExInternalCommandACK[5] = errorCode;
  if (source == FROM_USB && midiUSBCx) {
      // send to USB , cable 0
      USBMidi_SendSysExPacket(sysExInternalCommandACK,sizeof(sysExInternalCommandACK));
  } else
  if (source == FROM_SERIAL ) {
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
boolean SysExInternal_Parse(uint8_t source, midiPacket_t *pk,uint8_t sxMsg[])
{
    static unsigned sxMsgIdx = 0;
		static bool 	  sxHeaderFound = false;

    // Cable 0 only
    if ( (pk->packet[0] >> 4) > 0) return false;

		uint8_t cin   = pk->packet[0] & 0x0F ;

		// Only SYSEX and concerned packet on cable or serial 1

		if (cin < 4 || cin > 7) return false;
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
// Bulk sysex dump of current configuration TODO
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnDumpConfig(uint8_t source,uint8_t *sxMsg) {
  if (source == FROM_USB && midiUSBCx) {
    // send to USB , cable 0
    SysexInternal_DumpToStream(2);
    return SX_ERROR_NO_ERROR;
  } else
  if (source == FROM_SERIAL ) {
    // Send to serial port 0 being the only possible for sysex
    SysexInternal_DumpToStream(1);
    return SX_ERROR_NO_ERROR;
  }
  return SX_ERROR_ANY;
}

///////////////////////////////////////////////////////////////////////////////
// Identity request.  Example : F0 77 77 78 06 01 F7
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnIdentityRequest(uint8_t source,uint8_t *sxMsg) {
  if ( sxMsg[2] == 0x01 && sxMsg[0] == 2 ) {
      if (source == FROM_USB && midiUSBCx) {
        // send to USB , cable 0
        USBMidi_SendSysExPacket(sysExInternalIdentityRqReply,sizeof(sysExInternalIdentityRqReply));
        return SX_ERROR_NO_ERROR;
      } else
      if (source == FROM_SERIAL ) {
        // Send to serial port 0 being the only possible for sysex
        serialHw[0]->write(sysExInternalIdentityRqReply,sizeof(sysExInternalIdentityRqReply));
        return SX_ERROR_NO_ERROR;
      }
  }
  return SX_ERROR_ANY;
}

///////////////////////////////////////////////////////////////////////////////
// Reboot in configuration mode.  Example : F0 77 77 78 08 F7
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnConfigMode(uint8_t source,uint8_t *sxMsg) {
  // Set serial boot mode & Write the whole param struct
  if ( sxMsg[0] != 1 ) return SX_ERROR_BAD_MSG_SIZE;
  EEPROM_Params.nextBootMode = bootModeConfigMenu;
  return SX_ERROR_NO_ERROR;
}

///////////////////////////////////////////////////////////////////////////////
// Hard reset. Example : F0 77 77 78 0A F7
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnHardReset(uint8_t source,uint8_t *sxMsg) {
  if ( sxMsg[0] != 1 ) return SX_ERROR_BAD_MSG_SIZE;
  return SX_ERROR_NO_ERROR;
}

///////////////////////////////////////////////////////////////////////////////
// Change USB Product String
// F0 77 77 78 0B <string bytes> F7
// ---------------------------------------------------------------------------
// Copy the received string to the USB Product String Descriptor
// For MIDI protocol compatibility, and avoid a sysex encoding,
// accentuated ASCII characters, below 128 are non supported.
// Size if defined by USB_MIDI_PRODUCT_STRING_SIZE in UsbMidiKliK4x4.h
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnSetUsbProductString(uint8_t source,uint8_t *sxMsg) {

  if ( sxMsg[0] < 2 || (sxMsg[0]-1) > USB_MIDI_PRODUCT_STRING_SIZE )  return SX_ERROR_BAD_MSG_SIZE;

  // Store the new string in EEPROM
  memset(&EEPROM_Params.productString,0, sizeof(EEPROM_Params.productString));
  memcpy(&EEPROM_Params.productString,&sxMsg[2],sxMsg[0]-1);
  return SX_ERROR_NO_ERROR;
}

///////////////////////////////////////////////////////////////////////////////
// Change USB Vendor ID and Product ID
// F0 77 77 78 0C <n1 n2 n3 n4 = VID nibbles> <n1 n2 n3 n4 = PID nibbles> F7
// ---------------------------------------------------------------------------
// To respect a simple encoding of 7 bits bytes, each hex digit must be
// transmitted separately in a serialized way.
// The following example will set  VID to 0X8F12 and PID to 0X9067 :
// F0 77 77 78 0C 08 0F 01 02 09 00 06 07 F7
//                8  F  1  2  9  0  6  7
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnSetUsbPidVid(uint8_t source,uint8_t *sxMsg) {
  if ( sxMsg[0] != 9 ) return SX_ERROR_BAD_MSG_SIZE;
  EEPROM_Params.vendorID = (sxMsg[2] << 12) + (sxMsg[3] << 8) +
                                 (sxMsg[4] << 4) + sxMsg[5] ;
  EEPROM_Params.productID= (sxMsg[6] << 12) + (sxMsg[7] << 8) +
                                 (sxMsg[8] << 4) + sxMsg[9] ;
  return SX_ERROR_NO_ERROR;
}

///////////////////////////////////////////////////////////////////////////////
// IntelligentThru - Activated when USB is idle.
// ---------------------------------------------------------------------------
// F0 77 77 78 0E 02 04 F7
//  00 Reset Intellithru to default
//  01 Disable Intellithru
//  02 Set  Intellithru timeout <0xnn (number of 15s periods 1-127)>
//  03 Set thru mode jack routing < Jack In = (0-F)> <Attached slot 0-8 . => zero if no slot (detach).>
//      <t1> <t2>...<tn>  0-F 16 jack out targets maximum. => Disable intellithru if no jack out
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnIThruSettings(uint8_t source,uint8_t *sxMsg) {

  uint8_t msgLen = sxMsg[0];
  uint8_t cmdId  = sxMsg[2];

  // reset to default midi thru routing
  if (cmdId == 0x00  && msgLen == 2) {
       ResetMidiRoutingRules(ROUTING_RESET_INTELLITHRU);
       return SX_ERROR_NO_ERROR;
  } else

  // Disable thru mode
  if (cmdId == 0x01  && msgLen == 2) {
      ResetMidiRoutingRules(ROUTING_INTELLITHRU_OFF);
      return SX_ERROR_NO_ERROR;
  }	else

  // Set Delay. Min delay is 1 period of 15 secondes. The max is 127 periods of 15 secondes.
  if (cmdId == 0x02  && msgLen == 3) {
      if ( sxMsg[3] < 1 || sxMsg[3] > 0X7F ) return SX_ERROR_BAD_VALUE;
      EEPROM_Params.intelliThruDelayPeriod = sxMsg[3];
      return SX_ERROR_NO_ERROR;
  }
  else

  // Set routing
  //   0     1     2   3  4  5 6 7
  // (len) (id) (cmd) 01 01  0 1 2 3 4 5 6 7 8 9 A B C D E F
  if (cmdId == 0x03 && msgLen >= 4 && msgLen <= SERIAL_INTERFACE_COUNT+4 ) {
      uint8_t jackIn = sxMsg[3];
      uint8_t attachedSlot = sxMsg[4];

      if ( jackIn >= SERIAL_INTERFACE_COUNT) return SX_ERROR_BAD_PORT;
      if ( attachedSlot > MIDI_TRANS_PIPELINE_SLOT_SIZE  ) return SX_ERROR_BAD_SLOT;

      EEPROM_Params.midiRoutingRulesIntelliThru[jackIn].attachedSlot = attachedSlot;

      // If not jack out : disable Intellithru for this port
      if (msgLen == 4) {
        EEPROM_Params.intelliThruJackInMsk &= ~(1 << jackIn);
        return SX_ERROR_NO_ERROR;
      }
      // Set midi in jack msk
      EEPROM_Params.intelliThruJackInMsk |= (1 << jackIn);

      // Set target Jacks out msk
      uint16_t msk = 0;
      for ( uint8_t i = 5 ; i <= msgLen  ; i++) {
          if ( sxMsg[i] < SERIAL_INTERFACE_COUNT )
                msk |= 	1 << sxMsg[i] ;
          else return SX_ERROR_BAD_PORT;
      }
      EEPROM_Params.midiRoutingRulesIntelliThru[jackIn].jackOutTargetsMsk = msk;

      // reset globals for a real time update
      intelliThruDelayMillis = EEPROM_Params.intelliThruDelayPeriod * 15000;
      return SX_ERROR_NO_ERROR;
  }
  return SX_ERROR_ANY;
}

///////////////////////////////////////////////////////////////////////////////
// USB/Serial Midi midi routing rules
// ---------------------------------------------------------------------------
// Commands are :
//  00 Reset to default midi routing
//  01 Set routing +
//      arg1 - in port type : <0x00 usb cable | 0x01 jack serial>
//      arg2 - in port id : id for cable or jack serial (0-F)
//      arg3 - out port type = <0x00 usb cable in | 0x01 jack serial out>
//      arg4 - out port id targets : <port 0 1 2...n> 16 max (0-F)
//  02  Set Slot
//      arg1 - port type : : <0x00 usb cable | 0x01 jack serial>
//      arg2 - port id : id for cable or jack serial (0-F)
//      arg3 - slot number 0-8 => 0 is no slot
// EOX = F7
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnMidiRoutingSettings(uint8_t source,uint8_t *sxMsg) {
  uint8_t msgLen = sxMsg[0];
  uint8_t cmdId  = sxMsg[2];

  // reset to default routing
  if (cmdId == 0x00  && msgLen == 2) {
      ResetMidiRoutingRules(ROUTING_RESET_MIDIUSB);
      return SX_ERROR_NO_ERROR;
  }
  else

  // Set Routing targets - (len) (fnid) (cmd) 00 0F 01  00 01 02 03
  if (cmdId == 0x01 && msgLen >= 6 ) {

    uint8_t inPortType  = sxMsg[3];
    uint8_t inPort      = sxMsg[4];
    uint8_t outPortType = sxMsg[5];

    if (inPortType == PORT_TYPE_CABLE ) {
      if ( inPort >= USBCABLE_INTERFACE_MAX ) return SX_ERROR_BAD_PORT;
    }
    else
    if (outPortType == PORT_TYPE_CABLE ) {
      if (msgLen > (USBCABLE_INTERFACE_MAX + 5) ) return SX_ERROR_BAD_MSG_SIZE;
    }
    else
    if (inPortType == PORT_TYPE_JACK ) {
      if ( inPort >= SERIAL_INTERFACE_COUNT ) return SX_ERROR_BAD_PORT;
    }
    else
    if (outPortType == PORT_TYPE_JACK  ) {
      if ( msgLen > (SERIAL_INTERFACE_COUNT + 5) ) return SX_ERROR_BAD_MSG_SIZE;
    }
    else return SX_ERROR_ANY;

    // Compute mask from the port list
    uint16_t msk = 0;
    for ( uint8_t i = 6 ; i <= msgLen  ; i++) {
        if ( (outPortType == PORT_TYPE_CABLE && sxMsg[i] < USBCABLE_INTERFACE_MAX) ||
             (outPortType == PORT_TYPE_JACK  && sxMsg[i] < SERIAL_INTERFACE_COUNT) ) {
               msk |= 	1 << sxMsg[i] ;
        }
    }// for

    // Set masks
    if (inPortType == PORT_TYPE_CABLE ) {
          if (outPortType == PORT_TYPE_CABLE) EEPROM_Params.midiRoutingRulesCable[inPort].cableInTargetsMsk = msk;
          else EEPROM_Params.midiRoutingRulesCable[inPort].jackOutTargetsMsk = msk;
    }
    else
    // Jack
    if (outPortType == PORT_TYPE_CABLE) EEPROM_Params.midiRoutingRulesSerial[inPort].cableInTargetsMsk = msk;
    else EEPROM_Params.midiRoutingRulesSerial[inPort].jackOutTargetsMsk = msk;

    return SX_ERROR_NO_ERROR;
  }
  else

  // Set Slot  - (len) (fnid) (cmd) 00 0F 01
  if (cmdId == 0x02 && msgLen == 5 ) {
    uint8_t portType = sxMsg[3];
    uint8_t port     = sxMsg[4];
    uint8_t attachedSlot = sxMsg[5];

    if ( attachedSlot > MIDI_TRANS_PIPELINE_SLOT_SIZE  ) return SX_ERROR_BAD_SLOT;
    // Cable
    if (portType == PORT_TYPE_CABLE ) {
        if ( port  >= USBCABLE_INTERFACE_MAX) return SX_ERROR_BAD_PORT;
        EEPROM_Params.midiRoutingRulesCable[port].attachedSlot = attachedSlot;
    }  else
    // Jack Serial
    if (portType == PORT_TYPE_JACK) {
      if ( port >= SERIAL_INTERFACE_COUNT) return SX_ERROR_BAD_PORT;
        EEPROM_Params.midiRoutingRulesSerial[port].attachedSlot = attachedSlot;
    } else return SX_ERROR_ANY;

    return SX_ERROR_NO_ERROR;
  }

  return SX_ERROR_ANY;

}

///////////////////////////////////////////////////////////////////////////////
// Bus mode management
// ---------------------------------------------------------------------------
// Commands are :
//  00 Toggle bus mode <00 = off | 01 = ON>
//  01 Set device id <deviceid 04-08>
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnBusModeSettings(uint8_t source,uint8_t *sxMsg) {
  uint8_t msgLen = sxMsg[0];
  uint8_t cmdId  = sxMsg[2];

  // Toogle Bus mode
  if (cmdId == 0x00 && msgLen == 3 )  {

      if ( sxMsg[3] == 1  && EEPROM_Params.I2C_BusModeState == B_DISABLED)
              EEPROM_Params.I2C_BusModeState = B_ENABLED;
      else if ( sxMsg[3] == 0 && EEPROM_Params.I2C_BusModeState == B_ENABLED )
              EEPROM_Params.I2C_BusModeState = B_DISABLED;
      else return SX_ERROR_ANY;
      return SX_ERROR_NO_ERROR;
  }
  else

  // Set device Id
  if (cmdId == 0x01 && msgLen == 3 )  {
      if ( sxMsg[3] > B_SLAVE_DEVICE_LAST_ADDR || sxMsg[3] < B_SLAVE_DEVICE_BASE_ADDR ) return SX_ERROR_BAD_DEVICEID;
      if ( sxMsg[3] != EEPROM_Params.I2C_DeviceId ) {
        EEPROM_Params.I2C_DeviceId = sxMsg[3];
        return SX_ERROR_NO_ERROR;
      }
  }
  return SX_ERROR_ANY;
}

///////////////////////////////////////////////////////////////////////////////
// Midi transformation pipelines
// ---------------------------------------------------------------------------
// Commands are :
//  00 slot operation  <00 = Copy>   <source slot number 1-8> <dest slot number 1-8>
//                     <01 = Clear>  <Slot number 1-8  0x7F = ALL SLOTS>
//                     <02 = Attach> <port type : 0 cable | 1 jack serial | 2 Ithru>
//                                        <port # 0-F> <Slot number 0-8  0=>detach>
//
//  01 pipe operations <00 = Add pipe>            <slot number 1-8>
//                                                        <FN id> <par1> <par2> <par3> <par4>
//                     <01 = Insert before>       <slot number 1-8> <pipe index 0-n>
//                                                        <FN id> <par1> <par2> <par3> <par4>
//                     <02 = Clear Pipe by Index> <slot number 1-8> <pipe index 0-n>
//                     <03 = Clear pipe by pipe Id>  <slot number 1-8> <pipe Id>
//                     <04 = ByPass pipe by index>  <slot number> <pipe index 0-n> <byPass:0=no. 1=yes>
///////////////////////////////////////////////////////////////////////////////
uint8_t SysExInternal_fnPipelinesSettings(uint8_t source,uint8_t *sxMsg) {
  uint8_t msgLen    = sxMsg[0];
  uint8_t cmdId     = sxMsg[2];
  uint8_t cmdSubId  = sxMsg[3];

  // SLOTS OPERATIONS
  if (cmdId == 0x00 ) {
    // Copy slot
    if (cmdSubId == 0x00  && msgLen == 5)
      return TransPacketPipeline_CopySlot(sxMsg[4],sxMsg[5]) ? SX_ERROR_NO_ERROR : SX_ERROR_ANY ;
    else    // Clear slot <Slot number 1-8> <0x7F = ALL SLOTS>
    if (cmdSubId == 0x01  && msgLen == 4)
      return TransPacketPipeline_ClearSlot(sxMsg[4]) ? SX_ERROR_NO_ERROR : SX_ERROR_ANY;
    else  // Attach/Detach port to slot
    if (cmdSubId == 0x02  && msgLen == 6)
      return TransPacketPipeline_AttachPort(sxMsg[4],sxMsg[5],sxMsg[6]) ? SX_ERROR_NO_ERROR : SX_ERROR_ANY;
    else return SX_ERROR_ANY;
  }
  else

  // PIPE OPERATIONS
  if (cmdId == 0x01 ) {
    //  11 01  <00 = Add pipe>  <slot number 01-08> <FN id> <par1> <par2> <par3> <par4>
    if (cmdSubId == 0x00  && msgLen == 9) {
        midiTransPipe_t p;
        p.pId = sxMsg[5];  p.byPass = 0;
        p.par1 = sxMsg[6]; p.par2 = sxMsg[7];
        p.par3 = sxMsg[8]; p.par4 = sxMsg[9];
        return TransPacketPipe_AddToSlot(sxMsg[4],&p) ? SX_ERROR_NO_ERROR : SX_ERROR_ANY;
    }
    else
    // 11 01 <01 = Insert before>  <slot number> <pipe index 0-n> <FN id> <par1> <par2> <par3> <par4>
    if (cmdSubId == 0x01  && msgLen == 10) {
      midiTransPipe_t p;
      p.pId = sxMsg[6];  p.byPass = 0;
      p.par1 = sxMsg[7]; p.par2 = sxMsg[8];
      p.par3 = sxMsg[9]; p.par4 = sxMsg[10];
      return TransPacketPipe_InsertToSlot(sxMsg[4],sxMsg[5],&p)  ? SX_ERROR_NO_ERROR : SX_ERROR_ANY;
    }
    else
    // Clear pipe by index
    if (cmdSubId == 0x02  && msgLen == 5 )
      return TransPacketPipe_ClearSlotIndexPid(sxMsg[4],true,sxMsg[5])  ? SX_ERROR_NO_ERROR : SX_ERROR_ANY;
    else
    // Clear pipe first pId
    if (cmdSubId == 0x03  && msgLen == 5 )
      return TransPacketPipe_ClearSlotIndexPid(sxMsg[4],false,sxMsg[5])  ? SX_ERROR_NO_ERROR : SX_ERROR_ANY;
    else
    // ByPass pipe by index
    if (cmdSubId == 0x04  && msgLen == 6 )
      return TransPacketPipe_ByPass(sxMsg[4],sxMsg[5],sxMsg[6]) ? SX_ERROR_NO_ERROR : SX_ERROR_ANY;
  }

  return SX_ERROR_ANY;
}

///////////////////////////////////////////////////////////////////////////////
// Send a SYSEX dump to the appropriate destination stream
// 0 : Serial USB
// 1 : Serial port 0
// 2 : USB Cable 0
///////////////////////////////////////////////////////////////////////////////
void SysexInternal_DumpToStream(uint8_t dest) {

  uint16_t l;

  // Function 0B - Change USB Product String
  l = SysexInternal_DumpConf(0x0B000000, 0, sysExInternalBuffer);
  if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
  else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
  else if ( l && dest == 2 ) USBMidi_SendSysExPacket(sysExInternalBuffer,l);

  // Function 0C - Change USB Vendor ID and Product ID
  l = SysexInternal_DumpConf(0x0C000000, 0, sysExInternalBuffer);
  if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
  else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
  else if ( l && dest == 2 ) USBMidi_SendSysExPacket(sysExInternalBuffer,l);

  // Function 0E - Intellithru midi routing rules - 02 Timeout
  l = SysexInternal_DumpConf(0x0E020000, 0, sysExInternalBuffer);
  if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
  else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
  else if ( l && dest == 2 ) USBMidi_SendSysExPacket(sysExInternalBuffer,l);

  for ( uint8_t j=0; j != 16 ; j++) {
      // Function 0E - Intellithru midi routing rules - 03 Routing rules
      l = SysexInternal_DumpConf(0x0E030000, j, sysExInternalBuffer);
      if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
      else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
      else if ( l && dest == 2 ) USBMidi_SendSysExPacket(sysExInternalBuffer,l);
  }

  for ( uint8_t j=0; j != 16 ; j++) {
      // Function 0F - USB/Serial Midi midi routing rules - 02 slot for Cable
      l = SysexInternal_DumpConf(0x0F020000, j, sysExInternalBuffer);
      if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
      else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
      else if ( l && dest == 2 ) USBMidi_SendSysExPacket(sysExInternalBuffer,l);
  }

  for ( uint8_t j=0; j != 16 ; j++) {
      // Function 0F - USB/Serial Midi midi routing rules - 02 slot for Serial
      l = SysexInternal_DumpConf(0x0F020100, j, sysExInternalBuffer);
      if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
      else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
      else if ( l && dest == 2 ) USBMidi_SendSysExPacket(sysExInternalBuffer,l);
  }
  for ( uint8_t j=0; j != 16 ; j++) {
      // Function 0F - USB/Serial Midi midi routing rules - Cable to Cable
      l = SysexInternal_DumpConf(0x0F010000, j, sysExInternalBuffer);
      if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
      else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
      else if ( l && dest == 2 ) USBMidi_SendSysExPacket(sysExInternalBuffer,l);
  }

  for ( uint8_t j=0; j != 16 ; j++) {
      // Function 0F - USB/Serial Midi midi routing rules - Cable to Serial
      l = SysexInternal_DumpConf(0x0F010001, j, sysExInternalBuffer);
      if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
      else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
      else if ( l && dest == 2 ) USBMidi_SendSysExPacket(sysExInternalBuffer,l);
  }

  for ( uint8_t j=0; j != 16 ; j++) {
      // Function 0F - USB/Serial Midi midi routing rules - Serial to Cable
      l = SysexInternal_DumpConf(0x0F010100, j, sysExInternalBuffer);
      if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
      else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
      else if ( l && dest == 2 ) USBMidi_SendSysExPacket(sysExInternalBuffer,l);
   }

   for ( uint8_t j=0; j != 16 ; j++) {
      // Function 0F - USB/Serial Midi midi routing rules - Serial to Serial
      l = SysexInternal_DumpConf(0x0F010101, j, sysExInternalBuffer);
      if ( l && dest == 0 ) {ShowBufferHexDump(sysExInternalBuffer,l,0);Serial.println();}
      else if ( l && dest == 1 ) serialHw[0]->write(sysExInternalBuffer,l);
      else if ( l && dest == 2 ) USBMidi_SendSysExPacket(sysExInternalBuffer,l);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Return current configuration as a SYSEX buffer
///////////////////////////////////////////////////////////////////////////////
uint8_t SysexInternal_DumpConf(uint32_t fnId, uint8_t port,uint8_t *buff) {

  uint8_t src;
  uint8_t dest = 0 ;
  uint16_t msk ;
  uint8_t i;
  uint8_t c;
  uint8_t *buff2 = buff;

  memcpy(buff2,sysExInternalHeader,sizeof(sysExInternalHeader));
  buff2+=sizeof(sysExInternalHeader);
  *buff2 = fnId >> 24;

  switch (fnId) {

    // Function 0B - Change USB Product String
    case 0x0B000000:
          strcpy((char*)++buff2,(char*)EEPROM_Params.productString);
          buff2+=strlen((char*)EEPROM_Params.productString)-1;
          break;

    // Function 0C - Change USB Vendor ID and Product ID
    case 0x0C000000:
          *(++buff2) = EEPROM_Params.vendorID >> 12;
          *(++buff2) = (EEPROM_Params.vendorID & 0x0F00) >> 8;
          *(++buff2) = (EEPROM_Params.vendorID & 0x00F0) >> 4;
          *(++buff2) = (EEPROM_Params.vendorID & 0x000F) ;
          *(++buff2) = EEPROM_Params.productID >> 12;
          *(++buff2) = (EEPROM_Params.productID & 0x0F00) >> 8;
          *(++buff2) = (EEPROM_Params.productID & 0x00F0) >> 4;
          *(++buff2) = (EEPROM_Params.productID & 0x000F) ;
          break;

    // Function 0E - Intellithru midi routing rules
    // 02 Timeout
    case 0x0E020000:
          *(++buff2) = 0X02;
          *(++buff2) = EEPROM_Params.intelliThruDelayPeriod;
          break;

     // Function 0E - Intellithru midi routing rules
     // 03 Routing rules
     case 0x0E030000:
          *(++buff2) = 0X03;
          *(++buff2) = port;
          *(++buff2) = EEPROM_Params.midiRoutingRulesIntelliThru[port].attachedSlot;
          c = 0;
          for ( i=0; i != 16 ; i++) {
     						if ( EEPROM_Params.midiRoutingRulesIntelliThru[port].jackOutTargetsMsk & ( 1 << i) ) {
                      *(++buff2) = i;
                      c++;
                }
     		  }
          if (c == 0 ) return 0;
          break;

     // Function 0F - USB/Serial Midi midi routing rules
     // 02 Slot
     case 0x0F020000: // Cable
     case 0x0F020100: // Serial
         src  = (fnId & 0x0000FF00) >> 8;
         if (src > 1  ) return 0;
         *(++buff2) = 0X02;
         *(++buff2) = src;
         *(++buff2) = port;
         if (src) {
            *(++buff2) = EEPROM_Params.midiRoutingRulesSerial[port].attachedSlot;
         }
         else {
            *(++buff2) = EEPROM_Params.midiRoutingRulesCable[port].attachedSlot;
         }

         break;

     // Function 0F - USB/Serial Midi midi routing rules
     // 01 Routing rules
     case 0x0F010000: // Cable to Cable
     case 0x0F010001: // Cable to Serial
     case 0x0F010100: // Serial to Cable
     case 0x0F010101: // Serial to Serial
     src  = (fnId & 0x0000FF00) >> 8;
     dest = (fnId & 0x000000FF) ;
     if (src > 1 || dest > 1 ) return 0;
     *(++buff2) = 0X01;
     *(++buff2) = src;
     *(++buff2) = port;
     *(++buff2) = dest;

     if (src ) {
       msk = dest ? EEPROM_Params.midiRoutingRulesSerial[port].jackOutTargetsMsk : EEPROM_Params.midiRoutingRulesSerial[port].cableInTargetsMsk;
     } else {
       msk = dest ? EEPROM_Params.midiRoutingRulesCable[port].jackOutTargetsMsk : EEPROM_Params.midiRoutingRulesCable[port].cableInTargetsMsk;
     } ;

     c = 0;
     for ( i = 0 ; i != 16  ; i++) {
         if ( msk & ( 1 << i) ) {
           *(++buff2) = i;
           c++;
         }
     }
     if (c == 0 ) return 0;
     break;
  }
  *(++buff2) = 0xF7;
  return buff2-buff+1;
}
