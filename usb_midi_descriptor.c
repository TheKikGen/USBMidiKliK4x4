/*
  USB MidiKliK 4X4 - USB MIDI 4 IN X 4 OUT firmware
  Based on the MIDITECH / MIDIPLUS 4X4 harware.
  Copyright (C) 2017/2018 by The KikGen labs.

  DEVICE DESCRIPTOR

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

// ---------------------------------------------------------------
// Full assembled USB Descriptor FOR MIDI 4X4 DEVICE
// ---------------------------------------------------------------
#pragma once
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
      .bMaxPacketSize0    = MAX_PACKET_SIZE,
      .idVendor           = USB_VENDORID, 
      .idProduct          = USB_PRODUCTID, 
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
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_1;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_2;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_5;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_6;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_9;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_A;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_D;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_E;

    
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_3;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_4;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_7;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_8;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_B;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_C;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_F;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_10;

    MIDI_USB_DESCRIPTOR_ENDPOINT       DataOutEndpoint;
    MS_CS_BULK_ENDPOINT_DESCRIPTOR(4)  MS_CS_DataOutEndpoint;
    MIDI_USB_DESCRIPTOR_ENDPOINT       DataInEndpoint;
    MS_CS_BULK_ENDPOINT_DESCRIPTOR(4)  MS_CS_DataInEndpoint;
} __packed usb_descriptor_config;

#define MAX_POWER (100 >> 1)
static const usb_descriptor_config usbMIDIDescriptor_Config = {
    .Config_Header = {
        .bLength              = sizeof(usb_descriptor_config_header),
        .bDescriptorType      = USB_DESCRIPTOR_TYPE_CONFIGURATION,
        .wTotalLength         = sizeof(usb_descriptor_config),
        .bNumInterfaces       = 0x02,
        .bConfigurationValue  = 0x01,
        .iConfiguration       = 0x00,
        .bmAttributes         = (USB_CONFIG_ATTR_BUSPOWERED |
                                 USB_CONFIG_ATTR_SELF_POWERED),
        .bMaxPower            = MAX_POWER,
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
                              +8*sizeof(MIDI_IN_JACK_DESCRIPTOR)                              
                              +8*MIDI_OUT_JACK_DESCRIPTOR_SIZE(1)
                              +sizeof(MIDI_USB_DESCRIPTOR_ENDPOINT)
                              +MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(4)
                              +sizeof(MIDI_USB_DESCRIPTOR_ENDPOINT)
                              +MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(4),
    },

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
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x02,
        .iJack              = 0x00,
    },

    .MIDI_IN_JACK_5 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x05,
        .iJack              = 0x06, // MIDI OUT 2
    },

    .MIDI_IN_JACK_6 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x06,
        .iJack              = 0x00,
    },

    .MIDI_IN_JACK_9 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x09,
        .iJack              = 0x07, // MIDI OUT 3
    },
           
    .MIDI_IN_JACK_A = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x0A,
        .iJack              = 0x00,
    },

    .MIDI_IN_JACK_D = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x0D,
        .iJack              = 0x08, // MIDI OUT 4
    },

    .MIDI_IN_JACK_E = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x0E,
        .iJack              = 0x00,
    },

    .MIDI_OUT_JACK_3 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x03,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x02},
        .baSourcePin        = {0x01},
        .iJack              = 0x09, // MIDI IN 1
    },

    .MIDI_OUT_JACK_4 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x04,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x01},
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },

    .MIDI_OUT_JACK_7 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x07,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x06},
        .baSourcePin        = {0x01},
        .iJack              = 0x0A, // MIDI IN 2
    },

    .MIDI_OUT_JACK_8 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x08,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x05},
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },

    .MIDI_OUT_JACK_B = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x0B,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x0A},
        .baSourcePin        = {0x01},
        .iJack              = 0x0B, // MIDI IN 3
    },

    .MIDI_OUT_JACK_C = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x0C,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x09},
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },

    .MIDI_OUT_JACK_F = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x0F,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x0E},
        .baSourcePin        = {0x01},
        .iJack              = 0x0C, // MIDI IN 4
    },

    .MIDI_OUT_JACK_10 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x10,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x0D},
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },

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
      .bLength              = MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(4),
      .bDescriptorType      = USB_DESCRIPTOR_TYPE_CS_ENDPOINT,
      .SubType              = 0x01,
      .bNumEmbMIDIJack      = 0x04,
      .baAssocJackID        = {0x01,0x05,0X09,0X0D},
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
      .bLength              = MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(4),
      .bDescriptorType      = USB_DESCRIPTOR_TYPE_CS_ENDPOINT,
      .SubType              = 0x01,
      .bNumEmbMIDIJack      = 0x04,
      .baAssocJackID        = {0x03,0X07,0X0B,0x0F},
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


static usb_descriptor_string usbMIDIDescriptor_iProduct = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(MIDI_PRODUCT_STRING_SIZE+1), // Defined in EEPROM_Params.h   
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'U',0,'S',0,'B',0,' ',0,'M',0,'i',0,'d',0,'i',0,'K',0,'l',0,'i',\ 
                 0,'K',0,' ',0,'4',0,'X',0,'4',0, 0,0,  0,0,  0,0,  0,0,  0,0,  0,0,\
                   0,0,  0,0,  0,0,  0,0,  0,0,  0,0,  0,0,  0,0,  }   ,
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

// Midi Out 1
static const usb_descriptor_string usbMIDIDescriptor_iJackOut1 = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(10),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'M', 0, 'i', 0, 'd', 0, 'i', 0, ' ', 0, 'O', 0, 'u', 0, 't', 0, ' ', 0, '1', 0},
};

static const usb_descriptor_string usbMIDIDescriptor_iJackOut2 = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(10),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'M', 0, 'i', 0, 'd', 0, 'i', 0, ' ', 0, 'O', 0, 'u', 0, 't', 0, ' ', 0, '2', 0},
};
static const usb_descriptor_string usbMIDIDescriptor_iJackOut3 = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(10),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'M', 0, 'i', 0, 'd', 0, 'i', 0, ' ', 0, 'O', 0, 'u', 0, 't', 0, ' ', 0, '3', 0},
};
static const usb_descriptor_string usbMIDIDescriptor_iJackOut4 = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(10),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'M', 0, 'i', 0, 'd', 0, 'i', 0, ' ', 0, 'O', 0, 'u', 0, 't', 0, ' ', 0, '4', 0},
};

// Midi In 1
static const usb_descriptor_string usbMIDIDescriptor_iJackIn1 = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(9),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'M', 0, 'i', 0, 'd', 0, 'i', 0, ' ', 0, 'I', 0, 'n', 0, ' ', 0, '1', 0},
};

static const usb_descriptor_string usbMIDIDescriptor_iJackIn2 = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(9),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'M', 0, 'i', 0, 'd', 0, 'i', 0, ' ', 0, 'I', 0, 'n', 0, ' ', 0, '2', 0},
};

static const usb_descriptor_string usbMIDIDescriptor_iJackIn3 = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(9),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'M', 0, 'i', 0, 'd', 0, 'i', 0, ' ', 0, 'I', 0, 'n', 0, ' ', 0, '3', 0},
};

static const usb_descriptor_string usbMIDIDescriptor_iJackIn4 = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(9),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'M', 0, 'i', 0, 'd', 0, 'i', 0, ' ', 0, 'I', 0, 'n', 0, ' ', 0, '4', 0},
};


static ONE_DESCRIPTOR usbMidiDevice_Descriptor = {
    (uint8*)&usbMIDIDescriptor_Device,
    sizeof(usb_descriptor_device)
};

static ONE_DESCRIPTOR usbMidiConfig_Descriptor = {
    (uint8*)&usbMIDIDescriptor_Config,
    sizeof(usb_descriptor_config)
};

#define N_STRING_DESCRIPTORS 13
static ONE_DESCRIPTOR String_Descriptor[N_STRING_DESCRIPTORS] = {
    {(uint8*)&usbMIDIDescriptor_LangID,       USB_DESCRIPTOR_STRING_LEN(1) },
    {(uint8*)&usbMIDIDescriptor_iManufacturer,USB_DESCRIPTOR_STRING_LEN(14)},
    {(uint8*)&usbMIDIDescriptor_iProduct,     USB_DESCRIPTOR_STRING_LEN(MIDI_PRODUCT_STRING_SIZE+1)},
    {(uint8*)&usbMIDIDescriptor_iSerial,      USB_DESCRIPTOR_STRING_LEN(8) },
    {(uint8*)&usbMIDIDescriptor_iInterface,   USB_DESCRIPTOR_STRING_LEN(4) },  
    {(uint8*)&usbMIDIDescriptor_iJackIn1,     USB_DESCRIPTOR_STRING_LEN(9) },
    {(uint8*)&usbMIDIDescriptor_iJackIn2,     USB_DESCRIPTOR_STRING_LEN(9) },
    {(uint8*)&usbMIDIDescriptor_iJackIn3,     USB_DESCRIPTOR_STRING_LEN(9) },
    {(uint8*)&usbMIDIDescriptor_iJackIn4,     USB_DESCRIPTOR_STRING_LEN(9) },
    {(uint8*)&usbMIDIDescriptor_iJackOut1,    USB_DESCRIPTOR_STRING_LEN(10) },
    {(uint8*)&usbMIDIDescriptor_iJackOut2,    USB_DESCRIPTOR_STRING_LEN(10) },
    {(uint8*)&usbMIDIDescriptor_iJackOut3,    USB_DESCRIPTOR_STRING_LEN(10) },
    {(uint8*)&usbMIDIDescriptor_iJackOut4,    USB_DESCRIPTOR_STRING_LEN(10) },
};

