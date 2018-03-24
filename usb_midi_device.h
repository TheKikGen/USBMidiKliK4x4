/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs LLC.
 * Copyright (c) 2013 Magnus Lundin.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file libmaple/include/libmaple/usb_midi.h
 * @brief USB MIDI support
 *
 * IMPORTANT: this API is unstable, and may change without notice.
 */

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

#ifdef __cplusplus 
extern "C" { 
#endif

// USB MIDI PACKET
typedef union {
	uint8  byte[4];
	uint32 data;
} USB_MIDI_Event_Packet;

/*
 * USB MIDI Requests
 */

#define LEAFLABS_ID_VENDOR                0x1EAF
#define MAPLE_ID_PRODUCT                  0x0014
#define DEFAULT_MIDI_CHANNEL    0x0A
#define DEFAULT_MIDI_DEVICE     0x0A
#define DEFAULT_MIDI_CABLE      0x00

// eventually all of this should be in a place for settings which can be written to flash.
extern volatile uint8 myMidiChannel;
extern volatile uint8 myMidiDevice;
extern volatile uint8 myMidiCable;
extern volatile uint8 myMidiID[];



/* 0x7D = ED/FREE next two DIGITS MUST BE LESS THAN 0x7f */


/*
 * Descriptors, etc.
 */
#define USB_DESCRIPTOR_TYPE_CS_INTERFACE     0x24
#define USB_DESCRIPTOR_TYPE_CS_ENDPOINT      0x25


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

#define MAX_POWER (100 >> 1)

#define AC_CS_INTERFACE_DESCRIPTOR_SIZE(DataSize) (8 + DataSize)

#define AC_CS_INTERFACE_DESCRIPTOR(DataSize)        \
 struct {                                           \
      uint8  bLength;                               \
      uint8  bDescriptorType;                       \
      uint8  SubType;                               \
      uint16 bcdADC;                                \
      uint16 wTotalLength;                          \
      uint8  bInCollection;                         \
      uint8  baInterfaceNr[DataSize];               \
  } __packed

typedef struct {
      uint8  bLength;
      uint8  bDescriptorType;
      uint8  SubType;
      uint16 bcdADC;
      uint16 wTotalLength;
  } __packed MS_CS_INTERFACE_DESCRIPTOR;

typedef struct {
      uint8  bLength;
      uint8  bDescriptorType;
      uint8  SubType;
      uint8  bJackType;
      uint8  bJackId;
      uint8  iJack;
  } __packed MIDI_IN_JACK_DESCRIPTOR;

#define MIDI_OUT_JACK_DESCRIPTOR_SIZE(DataSize) (7 + 2*DataSize)
#define MIDI_OUT_JACK_DESCRIPTOR(DataSize)        \
 struct {                                           \
      uint8  bLength;                               \
      uint8  bDescriptorType;                       \
      uint8  SubType;                               \
      uint8  bJackType;                             \
      uint8  bJackId;                               \
      uint8  bNrInputPins;                          \
      uint8  baSourceId[DataSize];                  \
      uint8  baSourcePin[DataSize];                 \
      uint8  iJack;                                 \
  } __packed


#define MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(DataSize) (4 + DataSize)
#define MS_CS_BULK_ENDPOINT_DESCRIPTOR(DataSize)    \
 struct {                                           \
      uint8  bLength;                               \
      uint8  bDescriptorType;                       \
      uint8  SubType;                               \
      uint8  bNumEmbMIDIJack;                       \
      uint8  baAssocJackID[DataSize];               \
  } __packed

/*
 * Endpoint configuration
 */

#define USB_MIDI_CTRL_ENDP            0
#define USB_MIDI_CTRL_RX_ADDR         0x40
#define USB_MIDI_CTRL_TX_ADDR         0x80
#define USB_MIDI_CTRL_EPSIZE          0x40

#define USB_MIDI_TX_ENDP              1
#define USB_MIDI_TX_ADDR              0xC0
#define USB_MIDI_TX_EPSIZE            0x40

#define USB_MIDI_RX_ENDP              2
#define USB_MIDI_RX_ADDR              0x100
#define USB_MIDI_RX_EPSIZE            0x40

#ifndef __cplusplus


#define USB_MIDI_DECLARE_DEV_DESC(vid, pid)                           \
  {                                                                     \
      .bLength            = sizeof(usb_descriptor_device),              \
      .bDescriptorType    = USB_DESCRIPTOR_TYPE_DEVICE,                 \
      .bcdUSB             = 0x0110,                                     \
      .bDeviceClass       = USB_DEVICE_CLASS_UNDEFINED,                 \
      .bDeviceSubClass    = USB_DEVICE_SUBCLASS_UNDEFINED,              \
      .bDeviceProtocol    = 0x00,                                       \
      .bMaxPacketSize0    = 0x40,                                       \
      .idVendor           = vid,                                        \
      .idProduct          = pid,                                        \
      .bcdDevice          = 0x0200,                                     \
      .iManufacturer      = 0x01,                                       \
      .iProduct           = 0x02,                                       \
      .iSerialNumber      = 0x00,                                       \
      .bNumConfigurations = 0x01,                                       \
 }
#endif

/*
 * Sysex Stuff.
 */

#define SYSEX_BUFFER_LENGTH 256


 /*
 * MIDI interface
 */

    void usb_midi_enable(gpio_dev*, uint8);
    void usb_midi_disable(gpio_dev*, uint8);

    void usb_midi_putc(char ch);
    uint32 usb_midi_tx(const uint32* buf, uint32 len);
    uint32 usb_midi_rx(uint32* buf, uint32 len);
    uint32 usb_midi_peek(uint32* buf, uint32 len);

    uint32 usb_midi_data_available(void); /* in RX buffer */
    uint16 usb_midi_get_pending(void);
    uint8 usb_midi_is_transmitting(void);

    void sendThroughSysex(char *printbuffer, int bufferlength);

#ifdef __cplusplus
}
#endif




