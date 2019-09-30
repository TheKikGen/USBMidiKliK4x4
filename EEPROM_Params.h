/*
  USB MidiKliK 4X4 - USB MIDI 4 IN X 4 OUT firmware
  Based on the MIDITECH / MIDIPLUS 4X4 harware.
  Copyright (C) 2017/2018 by The KikGen labs.

  EEPROM PARAMETERS

  ------------------------   CAUTION  ----------------------------------
  THIS NOT A COPY OR A HACK OF ANY EXISTING MIDITECH/MIDIPLUS FIRMWARE.
  THAT FIRMWARE WAS ENTIRELY CREATED FROM A WHITE PAGE, WITHOUT
  DISASSEMBLING ANY SOFTWARE FROM MIDITECH/MIDIPLUS.

  UPLOADING THIS FIRMWARE TO YOUR MIDIPLUS/MIDITECH 4X4 USB MIDI
  INTERFACE  WILL PROBABLY CANCEL YOUR WARRANTY.

  IT WILL NOT BE POSSIBLE ANYMORE TO UPGRADE THE MODIFIED INTERFACE
  WITH THE MIDITECH/MIDIPLUS TOOLS AND PROCEDURES. NO ROLLBACK.

  THE AUTHOR DISCLAIM ANY DAMAGES RESULTING OF MODIFYING YOUR INTERFACE.
  YOU DO IT AT YOUR OWN RISKS.
  ---------------------------------------------------------------------

  This file is part of the USBMIDIKLIK-4x4 distribution
  https://github.com/TheKikGen/USBMidiKliK4x4
  Copyright (c) 2018 TheKikGen Labs team.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, version 3.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef _EEPROM_PARAMS_H_
#define _EEPROM_PARAMS_H_
#pragma once



// EEPROM parameters
// The signature is used to check if EEPROM is correctly initialized
// The parameters version is the version of the current structure.
// Changing the version will cause a new initialization in CheckEEPROM();.
// The following structure start at the first address of the EEPROM

#define EE_SIGNATURE "MDK"
#define EE_PRMVER 20

// Boot modes
enum nextBootMode {
    bootModeMidi   = 0,
    bootModeConfigMenu = 2,
};

typedef struct {
        uint8_t         signature[3];
        uint16_t        size;
        uint8_t         majorVersion;
        uint8_t         minorVersion;
        uint8_t         prmVersion;
        uint8_t         TimestampedVersion[14];
        uint8_t         nextBootMode;

        // I2C device when not master
        uint8_t         I2C_DeviceId;

        // I2C BUS Mode state
        uint8_t         I2C_BusModeState;

        // Incoming events routing rules
        // Storage space is set to the max i.e. INTERFACE_MAX for all
        // To allow dynamic change of bus mode.

        midiRoutingRule_t midiRoutingRulesCable[USBCABLE_INTERFACE_MAX];
        midiRoutingRule_t midiRoutingRulesSerial[B_SERIAL_INTERFACE_MAX];

        // IntelliThru
        midiRoutingRuleJack_t midiRoutingRulesIntelliThru[B_SERIAL_INTERFACE_MAX];
        uint16_t          intelliThruJackInMsk;
        uint8_t           intelliThruDelayPeriod; // 1 to 255 periods of 15s.

        uint16_t        vendorID;
        uint16_t        productID;
        uint8_t         productString[USB_MIDI_PRODUCT_STRING_SIZE+1]; // defined in usb_midi_devce.h
} __packed EEPROM_Params_t;


#endif
