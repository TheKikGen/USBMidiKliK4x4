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
#ifndef _USB_MIDI_DEVICE_H_
#define _USB_MIDI_DEVICE_H_
#pragma once


#include <libmaple/usb.h>
#include <libmaple/nvic.h>
#include <libmaple/delay.h>
#include <libmaple/libmaple_types.h>
#include <libmaple/gpio.h>

/* Private headers */
#include "usb_lib_globals.h"
#include "usb_reg_map.h"

/* usb_lib headers */
#include "usb_type.h"
#include "usb_core.h"
#include "usb_def.h"

// EEPROM parameters
//#include "EEPROM_Params.h"

#ifdef __cplusplus
extern "C" {
#endif

// --------------------------------------------------------------------------------------
// USB MIDI API Functions prototypes
// --------------------------------------------------------------------------------------

void usb_midi_set_vid_pid(uint16_t vid, uint16_t pid);
void usb_midi_set_product_string(char stringDescriptor[]);

void usb_midi_enable(gpio_dev *disc_dev, uint8_t disc_bit, uint8_t level);
void usb_midi_disable(gpio_dev *disc_dev, uint8_t disc_bit, uint8_t level);

void usb_midi_putc(char ch);
uint32_t usb_midi_tx(const uint32* buf, uint32_t len);
uint32_t usb_midi_rx(uint32* buf, uint32_t len);
uint32_t usb_midi_peek(uint32* buf, uint32_t len);
uint32_t usb_midi_mark_read(uint32_t n_copied) ;

uint32_t usb_midi_data_available(void); /* in RX buffer */
uint16_t usb_midi_get_pending(void);
uint8_t usb_midi_is_transmitting(void);

// --------------------------------------------------------------------------------------
// GLOBAL USB CONFIGURATION
// --------------------------------------------------------------------------------------
#define USB_MIDI_TIMEOUT 50

// --------------------------------------------------------------------------------------
// DESCRIPTOR IDS
// --------------------------------------------------------------------------------------

#define USB_MIDI_VENDORID            0x2912
#define USB_MIDI_PRODUCTID           0x1970
#define USB_MIDI_PRODUCT_STRING      "USB MIDIKliK 4x4"
#define USB_MIDI_PRODUCT_SERIAL      "07DA0908"

// String buffer Size in the descriptor without tailing zero.
// The real buffer size is USB_MIDI_PRODUCT_STRING_SIZE*2 +2
#define USB_MIDI_PRODUCT_STRING_SIZE 30

// --------------------------------------------------------------------------------------
// DESCRIPTORS TYPES
// --------------------------------------------------------------------------------------

#define USB_DESCRIPTOR_TYPE_CS_INTERFACE  0x24
#define USB_DESCRIPTOR_TYPE_CS_ENDPOINT   0x25

#define USB_DEVICE_CLASS_UNDEFINED        0x00
#define USB_DEVICE_CLASS_CDC              0x02
#define USB_DEVICE_SUBCLASS_UNDEFINED     0x00

#define USB_INTERFACE_CLASS_AUDIO         0x01
#define USB_INTERFACE_SUBCLASS_UNDEFINED  0x00
#define USB_INTERFACE_AUDIOCONTROL        0x01
#define USB_INTERFACE_AUDIOSTREAMING      0x02
#define USB_INTERFACE_MIDISTREAMING       0x03

/* MIDI Streaming class specific interfaces */
#define MIDI_IN_JACK                      0x02
#define MIDI_OUT_JACK                     0x03

#define MIDI_JACK_EMBEDDED                0x01
#define MIDI_JACK_EXTERNAL                0x02

//#define MAX_POWER (100 >> 1)
#define USB_MIDI_MAX_POWER (100 >> 1)

// --------------------------------------------------------------------------------------
// ENDPOINTS
// --------------------------------------------------------------------------------------

#define USB_MIDI_NUM_ENDPTS            0x04

// buffer table base address
#define USB_MIDI_BTABLE_ADDRESS      0x0000

// Every USB device must provide at least one control endpoint at address 0 called the
// default endpoint or Endpoint0. This endpoint is bidirectional.
// that is, the host can send data to the endpoint and receive data from it within one transfer.
// The purpose of a control transfer is to enable the host to obtain device information,
// configure the device, or perform control operations that are unique to the device.
// Control Endpoint

#define USB_MIDI_MAX_PACKET_SIZE          0x10  /* 64B, maximum for USB FS Devices */

#define USB_MIDI_CTRL_ENDP       USB_EP0
#define USB_MIDI_CTRL_RX_ADDR    0x40
#define USB_MIDI_CTRL_TX_ADDR    0x80

// MIDI data endpoints are used for transferring data. They are unidirectional,
// has a type (control, interrupt, bulk, isochronous) and other properties.
// All those properties are described in an endpoint descriptor.
// The direction of an endpoint is based on the host. Thus, IN always refers
// to transfers to the host from a device and OUT always refers to transfers
// from the host to a device.

#define MIDI_STREAM_EPSIZE       0x10

#define MIDI_STREAM_IN_ENDP      USB_EP1
#define MIDI_STREAM_IN_EPADDR    0xC0

#define MIDI_STREAM_OUT_ENDP     USB_EP2
#define MIDI_STREAM_OUT_EPADDR   0x100

// --------------------------------------------------------------------------------------
// MIDI DEVICE DESCRIPTOR STRUCTURES
// --------------------------------------------------------------------------------------

#define AC_CS_INTERFACE_DESCRIPTOR_SIZE(DataSize) (8 + DataSize)
#define AC_CS_INTERFACE_DESCRIPTOR(DataSize)        \
 struct {                                           \
      uint8_t  bLength;                               \
      uint8_t  bDescriptorType;                       \
      uint8_t  SubType;                               \
      uint16_t bcdADC;                                \
      uint16_t wTotalLength;                          \
      uint8_t  bInCollection;                         \
      uint8_t  baInterfaceNr[DataSize];               \
  } __packed

typedef struct {
      uint8_t  bLength;
      uint8_t  bDescriptorType;
      uint8_t  SubType;
      uint16_t bcdADC;
      uint16_t wTotalLength;
  } __packed MS_CS_INTERFACE_DESCRIPTOR;

typedef struct {
      uint8_t  bLength;
      uint8_t  bDescriptorType;
      uint8_t  SubType;
      uint8_t  bJackType;
      uint8_t  bJackId;
      uint8_t  iJack;
  } __packed MIDI_IN_JACK_DESCRIPTOR;

typedef struct  {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
    uint8_t  bRefresh;
    uint8_t  bSynchAddress;
} __packed MIDI_USB_DESCRIPTOR_ENDPOINT;

#define MIDI_OUT_JACK_DESCRIPTOR_SIZE(DataSize) (7 + 2*DataSize)
#define MIDI_OUT_JACK_DESCRIPTOR(DataSize)        \
 struct {                                           \
      uint8_t  bLength;                               \
      uint8_t  bDescriptorType;                       \
      uint8_t  SubType;                               \
      uint8_t  bJackType;                             \
      uint8_t  bJackId;                               \
      uint8_t  bNrInputPins;                          \
      uint8_t  baSourceId[DataSize];                  \
      uint8_t  baSourcePin[DataSize];                 \
      uint8_t  iJack;                                 \
  } __packed


#define MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(DataSize) (4 + DataSize)
#define MS_CS_BULK_ENDPOINT_DESCRIPTOR(DataSize)    \
 struct {                                           \
      uint8_t  bLength;                               \
      uint8_t  bDescriptorType;                       \
      uint8_t  SubType;                               \
      uint8_t  bNumEmbMIDIJack;                       \
      uint8_t  baAssocJackID[DataSize];               \
  } __packed


#ifdef __cplusplus
}
#endif

#endif
