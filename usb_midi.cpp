/*
  USB MidiKliK 4X4 - USB MIDI 4 IN X 4 OUT firmware
  Based on the MIDITECH / MIDIPLUS 4X4 harware.
  Copyright (C) 2017/2018 by The KikGen labs.

  USB MIDI LIBRARY adapted by TheKikGenLab from USB LeafLabs LLC. USB API :
  Perry Hung, Magnus Lundin,Donald Delmar Davis, Suspect Devices.

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

#include "hardware_config.h"
#include "usb_midi.h"
#include <string.h>
#include <stdint.h>
#include <libmaple/nvic.h>
#include "usb_midi_device.h"
#include <libmaple/usb.h>

#include <wirish.h>

// --------------------------------------------------------------------------------------
// USB MIDI Class
// --------------------------------------------------------------------------------------
// This class was adapted and CLEANED from the USBMidi library
// It can work for any device, but was optimized for the MIDI 4X4 board from Miditech
// based on a STM32F103RC.

// MIDI USB packet lenght
const uint8_t USBMidi::CINToLenTable[] =
{
  0, // 0X00 Miscellaneous function codes. Reserved for future extensions.
  0, // 0X01 Cable events.Reserved for future expansion.
  2, // 0x02 Two-byte System Common messages like  MTC, SongSelect, etc.
  3, // 0x03 Three-byte System Common messages like SPP, etc.
  3, // 0x04 SysEx starts or continues
  1, // 0x05 Single-byte System Common Message or SysEx ends with following single byte.
  2, // 0x06 SysEx ends with following two bytes.
  3, // 0x07 SysEx ends withfollowing three bytes.
  3, // 0x08 Note-off
  3, // 0x09 Note-on
  3, // 0x0A Poly-KeyPress
  3, // 0x0B Control Change
  2, // 0x0C Program Change
  2, // 0x0D Channel Pressure
  3, // 0x0E PitchBend Change
  1  // 0x0F Single Byte
};
// Constructor
USBMidi::USBMidi(void) {

}

// BEGIN - Call that function in SETUP
void USBMidi::begin() {

#ifdef HAS_MIDITECH_HARDWARE
		// Reset the USB interface on the MIDITECH 4x4 board.
    // The MIDI 4X4 has a DISC command, but the level logic is inverted
    // Then configure USB and Endpoints callbacks
    usb_midi_enable(PIN_MAP[PA8].gpio_device, PIN_MAP[PA8].gpio_bit,1);
#else
    usb_midi_enable(NULL, 0,0);
#endif

}

void USBMidi::end(void) {

#ifdef HAS_MIDITECH_HARDWARE
    usb_midi_disable(PIN_MAP[PA8].gpio_device, PIN_MAP[PA8].gpio_bit,0);
#else
    usb_midi_disable(NULL,0,0);
#endif

}

void USBMidi::writePacket(const uint32_t  *pk) {
    this->writePackets(pk, 1);
}

void USBMidi::writePackets(const void *buf, uint32_t len) {
    if (!this->isConnected() || !buf) {
        return;
    }

    uint32_t txed = 0;
    uint32_t old_txed = 0;
    uint32_t start = millis();

    uint32_t sent = 0;

    while (txed < len && (millis() - start < USB_MIDI_TIMEOUT)) {
        sent = usb_midi_tx((const uint32*)buf + txed, len - txed);
        txed += sent;
        if (old_txed != txed) {
            start = millis();
        }
        old_txed = txed;
    }


    if (sent == MIDI_STREAM_EPSIZE) {
        while (usb_midi_is_transmitting() != 0) {
        }
        /* flush out to avoid having the pc wait for more data */
        usb_midi_tx(NULL, 0);
    }
}

uint32_t USBMidi::available(void) {
    return usb_midi_data_available();
}

bool USBMidi::isTransmitting(void) {
   return usb_midi_is_transmitting();
}

uint32_t USBMidi::readPackets(const void *buf, uint32_t len) {
    if (!buf) {
        return 0;
    }

    uint32_t rxed = 0;
    while (rxed < len) {
        rxed += usb_midi_rx((uint32*)buf + rxed, len - rxed);
    }

    return rxed;
}

/* Blocks forever until 1 byte is received */
uint32_t USBMidi::peekPacket() {
      uint32_t p=0;
      usb_midi_peek(&p,1);
      return p;
}

void USBMidi::markPacketRead() {
    usb_midi_mark_read(1) ;
}

/* Blocks forever until 1 byte is received */
uint32_t USBMidi::readPacket() {
    uint32_t p=0;
    usb_midi_rx(&p,1);
    return p;
}

uint8_t USBMidi::pending(void) {
    return usb_midi_get_pending();
}

uint8_t USBMidi::isConnected(void) {
    return usb_is_connected(USBLIB) && usb_is_configured(USBLIB);
}


