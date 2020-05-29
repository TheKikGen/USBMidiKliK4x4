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

// ---------------------------------------------------------------
// Full assembled USB Descriptor FOR MIDI 4X4 DEVICE
// ---------------------------------------------------------------

#include "usb_midi_device.h"

// ---------------------------------------------------------------
// DEVICE DESCRIPTOR
// ---------------------------------------------------------------
static usb_descriptor_device usbMIDIDescriptor_Device = {
      .bLength            = sizeof(usb_descriptor_device),
      .bDescriptorType    = USB_DESCRIPTOR_TYPE_DEVICE,
      .bcdUSB             = 0x0110,
      .bDeviceClass       = USB_DEVICE_CLASS_UNDEFINED,
      .bDeviceSubClass    = USB_DEVICE_SUBCLASS_UNDEFINED,
      .bDeviceProtocol    = 0x00,
      .bMaxPacketSize0    = USB_MIDI_MAX_PACKET_SIZE,
      .idVendor           = USB_MIDI_VENDORID,
      .idProduct          = USB_MIDI_PRODUCTID,
      .bcdDevice          = 0x0100,
      .iManufacturer      = 0x01,
      .iProduct           = 0x02,
      .iSerialNumber      = 0x03,
      .bNumConfigurations = 0x01,
 };

// ---------------------------------------------------------------
// CONFIGURATION DESCRIPTOR
// ---------------------------------------------------------------


typedef struct {
    usb_descriptor_config_header       Config_Header;
    /* Control Interface */
    usb_descriptor_interface           AC_Interface;
    AC_CS_INTERFACE_DESCRIPTOR(1)      AC_CS_Interface;
    /* Control Interface */
    usb_descriptor_interface           MS_Interface;
    MS_CS_INTERFACE_DESCRIPTOR         MS_CS_Interface;

    // MIDI IN DESCRIPTORS - 16 PAIRS MAX
    // 4 PORTS IS THE MINIMUM.

    // Embedded

    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_1;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_2;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_3;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_4;

    #if USB_MIDI_IO_PORT_NUM >= 8
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_5;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_6;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_7;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_8;

    #if USB_MIDI_IO_PORT_NUM >= 12
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_9;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_A;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_B;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_C;

    #if USB_MIDI_IO_PORT_NUM >= 16
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_D;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_E;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_F;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_10;

    #endif
    #endif
    #endif

    // External
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_11;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_12;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_13;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_14;

    #if USB_MIDI_IO_PORT_NUM >= 8
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_15;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_16;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_17;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_18;

    #if USB_MIDI_IO_PORT_NUM >= 12
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_19;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_1A;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_1B;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_1C;

    #if USB_MIDI_IO_PORT_NUM >= 16
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_1D;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_1E;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_1F;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_20;

    #endif
    #endif
    #endif

    // MIDI OUT DESCRIPTORS - 16 PAIRS MAX
    // Embedded

    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_21;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_22;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_23;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_24;

    #if USB_MIDI_IO_PORT_NUM >= 8
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_25;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_26;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_27;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_28;

    #if USB_MIDI_IO_PORT_NUM >= 12
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_29;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_2A;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_2B;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_2C;

    #if USB_MIDI_IO_PORT_NUM >= 16
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_2D;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_2E;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_2F;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_30;

    #endif
    #endif
    #endif


    // External
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_31;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_32;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_33;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_34;

    #if USB_MIDI_IO_PORT_NUM >= 8
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_35;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_36;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_37;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_38;

    #if USB_MIDI_IO_PORT_NUM >= 12
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_39;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_3A;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_3B;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_3C;

    #if USB_MIDI_IO_PORT_NUM >= 16
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_3D;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_3E;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_3F;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_40;

    #endif
    #endif
    #endif



    MIDI_USB_DESCRIPTOR_ENDPOINT       DataOutEndpoint;
    MS_CS_BULK_ENDPOINT_DESCRIPTOR(USB_MIDI_IO_PORT_NUM)  MS_CS_DataOutEndpoint;
    MIDI_USB_DESCRIPTOR_ENDPOINT       DataInEndpoint;
    MS_CS_BULK_ENDPOINT_DESCRIPTOR(USB_MIDI_IO_PORT_NUM)  MS_CS_DataInEndpoint;
} __packed usb_midi_descriptor_config;

static const usb_midi_descriptor_config usbMIDIDescriptor_Config = {
    .Config_Header = {
        .bLength              = sizeof(usb_descriptor_config_header),
        .bDescriptorType      = USB_DESCRIPTOR_TYPE_CONFIGURATION,
        .wTotalLength         = sizeof(usb_midi_descriptor_config),
        .bNumInterfaces       = 0x02,
        .bConfigurationValue  = 0x01,
        .iConfiguration       = 0x00,
        .bmAttributes         = 0xa0 , // (Bus Powered)  Remote Wakeup
        // .bmAttributes         = (USB_CONFIG_ATTR_BUSPOWERED |
        //                           USB_CONFIG_ATTR_SELF_POWERED),
        .bMaxPower            = USB_MIDI_MAX_POWER,
    },

    .AC_Interface = {
        .bLength            = sizeof(usb_descriptor_interface),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_INTERFACE,
        .bInterfaceNumber   = 0x00,
        .bAlternateSetting  = 0x00,
        .bNumEndpoints      = 0x00,
        .bInterfaceClass    = USB_INTERFACE_CLASS_AUDIO,
        .bInterfaceSubClass = USB_INTERFACE_AUDIOCONTROL,
        .bInterfaceProtocol = 0x00,
        .iInterface         = 0x00,
    },

    .AC_CS_Interface = {
        .bLength            = AC_CS_INTERFACE_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x01,
        .bcdADC             = 0x0100,
        .wTotalLength       = AC_CS_INTERFACE_DESCRIPTOR_SIZE(1),
        .bInCollection      = 0x01,
        .baInterfaceNr      = {0x01},
    },

    .MS_Interface = {
        .bLength            = sizeof(usb_descriptor_interface),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_INTERFACE,
        .bInterfaceNumber   = 0x01,
        .bAlternateSetting  = 0x00,
        .bNumEndpoints      = 0x02,
        .bInterfaceClass    = USB_INTERFACE_CLASS_AUDIO,
        .bInterfaceSubClass = USB_INTERFACE_MIDISTREAMING,
        .bInterfaceProtocol = 0x00,
        .iInterface         = 0x04, // Midi
    },

    .MS_CS_Interface = {
        .bLength            = sizeof(MS_CS_INTERFACE_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x01,
        .bcdADC             = 0x0100,
        .wTotalLength       = sizeof(MS_CS_INTERFACE_DESCRIPTOR)
                              +USB_MIDI_IO_PORT_NUM*2*sizeof(MIDI_IN_JACK_DESCRIPTOR)
                              +USB_MIDI_IO_PORT_NUM*2*MIDI_OUT_JACK_DESCRIPTOR_SIZE(1)
                              +sizeof(MIDI_USB_DESCRIPTOR_ENDPOINT)
                              +MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(USB_MIDI_IO_PORT_NUM)
                              +sizeof(MIDI_USB_DESCRIPTOR_ENDPOINT)
                              +MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(USB_MIDI_IO_PORT_NUM),
    },

    // Start of MIDI JACK Descriptor =======================================

    // MIDI IN JACK - EMBEDDED - 16 Descriptors -----------------------------
    .MIDI_IN_JACK_1 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x01,
        .iJack              = 0x05,  // MIDI OUT 1
    },
    .MIDI_IN_JACK_2 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x02,
        .iJack              = 0x05,  // MIDI OUT 1
    },
    .MIDI_IN_JACK_3 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x03,
        .iJack              = 0x05,  // MIDI OUT 1
    },
    .MIDI_IN_JACK_4 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x04,
        .iJack              = 0x05,  // MIDI OUT 1
    },

   #if USB_MIDI_IO_PORT_NUM >= 8

    .MIDI_IN_JACK_5 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x05,
        .iJack              = 0x05,  // MIDI OUT 1
    },
    .MIDI_IN_JACK_6 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x06,
        .iJack              = 0x05,  // MIDI OUT 1
    },
    .MIDI_IN_JACK_7 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x07,
        .iJack              = 0x05,  // MIDI OUT 1
    },
    .MIDI_IN_JACK_8 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x08,
        .iJack              = 0x05,  // MIDI OUT 1
    },

    #if USB_MIDI_IO_PORT_NUM >= 12

    .MIDI_IN_JACK_9 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x09,
        .iJack              = 0x05,  // MIDI OUT 1
    },
    .MIDI_IN_JACK_A = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x0A,
        .iJack              = 0x05,  // MIDI OUT 1
    },
    .MIDI_IN_JACK_B = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x0B,
        .iJack              = 0x05,  // MIDI OUT 1
    },
    .MIDI_IN_JACK_C = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x0C,
        .iJack              = 0x05,  // MIDI OUT 1
    },

    #if USB_MIDI_IO_PORT_NUM >= 16
    .MIDI_IN_JACK_D = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x0D,
        .iJack              = 0x05,  // MIDI OUT 1
    },
    .MIDI_IN_JACK_E = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x0E,
        .iJack              = 0x05,  // MIDI OUT 1
    },
    .MIDI_IN_JACK_F = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x0F,
        .iJack              = 0x05,  // MIDI OUT 1
    },
    .MIDI_IN_JACK_10 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x10,
        .iJack              = 0x05,  // MIDI OUT 1
    },
   #endif
   #endif
   #endif

    // MIDI IN JACK - EXTERNAL - 16 Descriptors -----------------------------

    .MIDI_IN_JACK_11 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x11,
        .iJack              = 0x00,
    },
    .MIDI_IN_JACK_12 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x12,
        .iJack              = 0x00,
    },
    .MIDI_IN_JACK_13 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x13,
        .iJack              = 0x00,
    },
    .MIDI_IN_JACK_14 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x14,
        .iJack              = 0x00,
    },
  #if USB_MIDI_IO_PORT_NUM >= 8

    .MIDI_IN_JACK_15 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x15,
        .iJack              = 0x00,
    },
    .MIDI_IN_JACK_16 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x16,
        .iJack              = 0x00,
    },
    .MIDI_IN_JACK_17 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x17,
        .iJack              = 0x00,
    },
    .MIDI_IN_JACK_18 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x18,
        .iJack              = 0x00,
    },

    #if USB_MIDI_IO_PORT_NUM >= 12

    .MIDI_IN_JACK_19 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x19,
        .iJack              = 0x00,
    },
    .MIDI_IN_JACK_1A = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x1A,
        .iJack              = 0x00,
    },
    .MIDI_IN_JACK_1B = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x1B,
        .iJack              = 0x00,
    },
    .MIDI_IN_JACK_1C = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x1C,
        .iJack              = 0x00,
    },

    #if USB_MIDI_IO_PORT_NUM >= 16
    .MIDI_IN_JACK_1D = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x1D,
        .iJack              = 0x00,
    },
    .MIDI_IN_JACK_1E = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x1E,
        .iJack              = 0x00,
    },
    .MIDI_IN_JACK_1F = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x1F,
        .iJack              = 0x00,
    },
    .MIDI_IN_JACK_20 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x20,
        .iJack              = 0x00,
    },
    #endif
    #endif
    #endif

    // MIDI OUT JACK - EMBEDDED - 16 Descriptors -----------------------------

    .MIDI_OUT_JACK_21 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x21,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x11}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },
    .MIDI_OUT_JACK_22 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x22,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x12}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },
    .MIDI_OUT_JACK_23 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x23,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x13}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },
    .MIDI_OUT_JACK_24 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x24,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x14}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },

    #if USB_MIDI_IO_PORT_NUM >= 8

    .MIDI_OUT_JACK_25 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x25,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x15}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },
    .MIDI_OUT_JACK_26 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x26,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x16}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },
    .MIDI_OUT_JACK_27 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x27,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x17}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },
    .MIDI_OUT_JACK_28 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x28,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x18}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },

    #if USB_MIDI_IO_PORT_NUM >= 12

    .MIDI_OUT_JACK_29 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x29,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x19}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },
    .MIDI_OUT_JACK_2A = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x2A,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x1A}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },
    .MIDI_OUT_JACK_2B = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x2B,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x1B}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },
    .MIDI_OUT_JACK_2C = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x2C,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x1C}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },

    #if USB_MIDI_IO_PORT_NUM >= 16
    .MIDI_OUT_JACK_2D = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x2D,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x1D}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },
    .MIDI_OUT_JACK_2E = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x2E,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x1E}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },
    .MIDI_OUT_JACK_2F = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x2F,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x1F}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },
    .MIDI_OUT_JACK_30 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x30,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x20}, // IN External
        .baSourcePin        = {0x01},
        .iJack              = 0x06, // MIDI IN 1
    },
    #endif
    #endif
    #endif


    // MIDI OUT JACK - EXTERNAL - 16 Descriptors -----------------------------

    .MIDI_OUT_JACK_31 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x31,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x01}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },
    .MIDI_OUT_JACK_32 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x32,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x02}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },
    .MIDI_OUT_JACK_33 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x33,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x03}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },
    .MIDI_OUT_JACK_34 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x34,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x04}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },

    #if USB_MIDI_IO_PORT_NUM >= 8

    .MIDI_OUT_JACK_35 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x35,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x05}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },
    .MIDI_OUT_JACK_36 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x36,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x06}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },
    .MIDI_OUT_JACK_37 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x37,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x07}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },
    .MIDI_OUT_JACK_38 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x38,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x08}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },

    #if USB_MIDI_IO_PORT_NUM >= 12

    .MIDI_OUT_JACK_39 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x39,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x09}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },
    .MIDI_OUT_JACK_3A = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x3A,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x0A}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },
    .MIDI_OUT_JACK_3B = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x3B,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x0B}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },
    .MIDI_OUT_JACK_3C = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x3C,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x0C}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },

    #if USB_MIDI_IO_PORT_NUM >= 16
    .MIDI_OUT_JACK_3D = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x3D,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x0D}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },
    .MIDI_OUT_JACK_3E = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x3E,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x0E}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },
    .MIDI_OUT_JACK_3F = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x3F,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x0F}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },
    .MIDI_OUT_JACK_40 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x40,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x10}, // IN Embedded
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },

      #endif
      #endif
      #endif

    // End of MIDI JACK Descriptor =======================================

    .DataOutEndpoint = {
        .bLength            = sizeof(MIDI_USB_DESCRIPTOR_ENDPOINT),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress   = (USB_DESCRIPTOR_ENDPOINT_OUT |
                             MIDI_STREAM_OUT_ENDP),
        .bmAttributes       = USB_EP_TYPE_BULK,
        .wMaxPacketSize     = MIDI_STREAM_EPSIZE,
        .bInterval          = 0x00,
        .bRefresh           = 0x00,
        .bSynchAddress      = 0x00,
    },

    .MS_CS_DataOutEndpoint = {
      .bLength              = MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(USB_MIDI_IO_PORT_NUM),
      .bDescriptorType      = USB_DESCRIPTOR_TYPE_CS_ENDPOINT,
      .SubType              = 0x01,
      // MIDI IN EMBEDDED
      .bNumEmbMIDIJack      = USB_MIDI_IO_PORT_NUM,
      .baAssocJackID        = {
        0x01,0X02,0X03,0X04,

        #if USB_MIDI_IO_PORT_NUM >= 8
        0X05,0X06,0X07,0X08,
        #if USB_MIDI_IO_PORT_NUM >= 12
        0X09,0X0A,0X0B,0X0C,
        #if USB_MIDI_IO_PORT_NUM >= 16
        0X0D,0X0E,0X0F,0X10

        #endif
        #endif
        #endif

      },
  },

    .DataInEndpoint = {
        .bLength          = sizeof(MIDI_USB_DESCRIPTOR_ENDPOINT),
        .bDescriptorType  = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress = (USB_DESCRIPTOR_ENDPOINT_IN | MIDI_STREAM_IN_ENDP),
        .bmAttributes     = USB_EP_TYPE_BULK,
        .wMaxPacketSize   = MIDI_STREAM_EPSIZE,
        .bInterval        = 0x00,
        .bRefresh         = 0x00,
        .bSynchAddress    = 0x00,
    },

    .MS_CS_DataInEndpoint = {
      .bLength              = MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(USB_MIDI_IO_PORT_NUM),
      .bDescriptorType      = USB_DESCRIPTOR_TYPE_CS_ENDPOINT,
      .SubType              = 0x01,
      // MIDI OUT EMBEDDED
      .bNumEmbMIDIJack      = USB_MIDI_IO_PORT_NUM,
      .baAssocJackID        = {
        0x21,0X22,0X23,0X24,

        #if USB_MIDI_IO_PORT_NUM >= 8
        0X25,0X26,0X27,0X28,
        #if USB_MIDI_IO_PORT_NUM >= 12
        0X29,0X2A,0X2B,0X2C,
        #if USB_MIDI_IO_PORT_NUM >= 16
        0X2D,0X2E,0X2F,0X30

        #endif
        #endif
        #endif

      },
  },

};
// --------------------------------------------------------------------------------------
//  String Descriptors:
// --------------------------------------------------------------------------------------

/*  we may choose to specify any or none of the following string
  identifiers:
  iManufacturer:    LeafLabs
  iProduct:         Maple
  iSerialNumber:    NONE
  iConfiguration:   NONE
  iInterface(CCI):  NONE
  iInterface(DCI):  NONE
*/

/* Unicode language identifier: 0x0409 is US English */
static const usb_descriptor_string usbMIDIDescriptor_LangID = {
    .bLength         = USB_DESCRIPTOR_STRING_LEN(1),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString         = {0x09, 0x04},
};

static const usb_descriptor_string usbMIDIDescriptor_iManufacturer = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(14),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    // TheKikGen Labs
    .bString = {'T', 0, 'h', 0, 'e', 0, 'K', 0,'i', 0, 'K', 0, 'G', 0, 'e', 0, 'n', 0, ' ', 0, 'L', 0, 'a', 0, 'b', 0, 's', 0},
};


// We reserve room to change the product string later but the lenght is manually adjusted to 18.
static usb_descriptor_string usbMIDIDescriptor_iProduct = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(18),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString =  { 'U', 0, 'S', 0, 'B', 0, ' ', 0, 'M', 0, 'i', 0, 'd', 0, 'i', 0, 'K', 0, 'l', 0,  // 10
                  'i', 0, 'K', 0, ' ', 0, '4', 0, 'X', 0, '4', 0, '*', 0, '*', 0,  0 , 0 , 0 , 0,  //  20
                   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 30
                   0,  0, // USB_MIDI_PRODUCT_STRING_SIZE = 30
                }
};

static const usb_descriptor_string usbMIDIDescriptor_iSerial = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(8),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    // 07DA0908
    .bString = {'0', 0, '7', 0, 'D', 0, 'A', 0, '0', 0, '9', 0, '0', 0, '8', 0},
};

static const usb_descriptor_string usbMIDIDescriptor_iInterface = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(4),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'M', 0, 'i', 0, 'd', 0, 'i', 0},
};

// Midi Out 1-n
static const usb_descriptor_string usbMIDIDescriptor_iJackOut = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(8),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'M', 0, 'i', 0, 'd', 0, 'i', 0, ' ', 0, 'O', 0, 'u', 0, 't', 0},
};


// Midi In 1-n
static const usb_descriptor_string usbMIDIDescriptor_iJackIn = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(7),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'M', 0, 'i', 0, 'd', 0, 'i', 0, ' ', 0, 'I', 0, 'n', 0},
};



static ONE_DESCRIPTOR usbMidiDevice_Descriptor = {
    (uint8*)&usbMIDIDescriptor_Device,
    sizeof(usb_descriptor_device)
};

static ONE_DESCRIPTOR usbMidiConfig_Descriptor = {
    (uint8*)&usbMIDIDescriptor_Config,
    sizeof(usb_midi_descriptor_config)
};

#define USB_MIDI_N_STRING_DESCRIPTORS 7
static ONE_DESCRIPTOR usbMIDIString_Descriptor[USB_MIDI_N_STRING_DESCRIPTORS] = {
    {(uint8*)&usbMIDIDescriptor_LangID,       USB_DESCRIPTOR_STRING_LEN(1) },
    {(uint8*)&usbMIDIDescriptor_iManufacturer,USB_DESCRIPTOR_STRING_LEN(14)},
    {(uint8*)&usbMIDIDescriptor_iProduct,     USB_DESCRIPTOR_STRING_LEN(18)},
    {(uint8*)&usbMIDIDescriptor_iSerial,      USB_DESCRIPTOR_STRING_LEN(8) },
    {(uint8*)&usbMIDIDescriptor_iInterface,   USB_DESCRIPTOR_STRING_LEN(4) },
    {(uint8*)&usbMIDIDescriptor_iJackIn,     USB_DESCRIPTOR_STRING_LEN(9) },
    {(uint8*)&usbMIDIDescriptor_iJackOut,    USB_DESCRIPTOR_STRING_LEN(10) },
};
