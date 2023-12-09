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
//#include "EE_Prm.h"

#ifdef __cplusplus
extern "C" {
#endif

// --------------------------------------------------------------------------------------
// USB MIDI API Functions prototypes
// --------------------------------------------------------------------------------------
void usb_midi_init_descriptor_config ( uint8_t nbPorts) ;
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
// MIDI PORTS
// --------------------------------------------------------------------------------------
#define USB_MIDI_IO_PORT_NUM  16

// --------------------------------------------------------------------------------------
// DESCRIPTOR IDS
// --------------------------------------------------------------------------------------

#define USB_MIDI_VENDORID            0x2912
#define USB_MIDI_PRODUCTID           0x1970
#define USB_MIDI_PRODUCT_STRING      "MidiKlik 4x"

// String buffer max Size in the descriptor without tailing zero.
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
