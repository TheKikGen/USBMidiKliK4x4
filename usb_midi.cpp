/*
  -----------------------------------------------------------------------------
  USBMIDIKLIK 4X4 - USB Midi advanced firmware for STM32F1 platform.
  Copyright (C) 2019 by The KikGen labs.
  LICENCE CREATIVE COMMONS - Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)

  This file is part of the USBMIDIKLIK-4x4 distribution
  https://github.com/TheKikGen/USBMidiKliK4x4
  Copyright (c) 2019 TheKikGen Labs team.
  -----------------------------------------------------------------------------
  USB MIDI LIBRARY adapted by TheKikGenLab from USB LeafLabs LLC. USB API :
  Perry Hung, Magnus Lundin,Donald Delmar Davis, Suspect Devices.
  GPL Licence.
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
