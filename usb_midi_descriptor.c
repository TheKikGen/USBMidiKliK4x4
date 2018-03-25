<<<<<<< HEAD
// ---------------------------------------------------------------
// Full assembled USB Descriptor FOR MIDI 4X4 DEVICE
// ---------------------------------------------------------------

#include "usb_midi_device.h"
=======
/*
 * Descriptors
 */

/* FIXME move to Wirish */

#include "usb_midi_device.h"

// ---------------------------------------------------------------
// Full assembled USB Descriptor 
// ---------------------------------------------------------------

>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d

// ---------------------------------------------------------------
// DEVICE DESCRIPTOR
// ---------------------------------------------------------------
static const usb_descriptor_device usbMIDIDescriptor_Device =
    USB_MIDI_DECLARE_DEV_DESC(LEAFLABS_ID_VENDOR, MAPLE_ID_PRODUCT);
/*
static const usb_descriptor_device usbMIDIDescriptor_Device = {
      .bLength            = sizeof(usb_descriptor_device), 
      .bDescriptorType    = 0x01, // Device
      .bcdUSB             = 0x0110,
      .bDeviceClass       = 0x00,
      .bDeviceSubClass    = 0x00,
      .bDeviceProtocol    = 0x00,
      .bMaxPacketSize0    = 0x10, // 16 bytes
      .idVendor           = 0x2912, 
      .idProduct          = 0x1970, 
      .bcdDevice          = 0x0101,
      .iManufacturer      = 0x01,  
      .iProduct           = 0x02,
      .iSerialNumber      = 0x00,
      .bNumConfigurations = 0x01,
};      
    
*/

<<<<<<< HEAD
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
=======
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
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_3;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_4;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_5;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_6;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_7;
    MIDI_IN_JACK_DESCRIPTOR            MIDI_IN_JACK_8;

    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_1;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_2;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_3;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_4;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_5;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_6;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_7;
    MIDI_OUT_JACK_DESCRIPTOR(1)        MIDI_OUT_JACK_8;
            
    usb_descriptor_endpoint            DataOutEndpoint;
    MS_CS_BULK_ENDPOINT_DESCRIPTOR(4)  MS_CS_DataOutEndpoint;
    usb_descriptor_endpoint            DataInEndpoint;
    MS_CS_BULK_ENDPOINT_DESCRIPTOR(4)  MS_CS_DataInEndpoint;
} __packed usb_descriptor_config;


// ---------------------------------------------------------------
// Configuration Descriptor 1 Bus Powered, 100 mA
// ---------------------------------------------------------------

static const usb_descriptor_config usbMIDIDescriptor_Config = {

    // Configuration descriptor header 
/* 
0  bLength 1 09h 
1 bDescriptorType 1 02h Configuration
2 wTotalLength  2 00C5h 
4 bNumInterfaces  1 02h 
5 bConfigurationValue 1 01h 
6 iConfiguration  1 00h 
7 bmAttributes  1 A0h Bus Powered, Remote Wakeup
4..0: Reserved    ...00000  
5: Remote Wakeup    ..1.....  Yes
6: Self Powered   .0......  No, Bus Powered
7: Reserved (set to one)
(bus-powered for 1.0)   1.......  
8 bMaxPower 1 32h 100 mA
*/

    .Config_Header = {
        .bLength              = sizeof(usb_descriptor_config_header),
        .bDescriptorType      = USB_DESCRIPTOR_TYPE_CONFIGURATION,  
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
        .wTotalLength         = sizeof(usb_descriptor_config),
        .bNumInterfaces       = 0x02,
        .bConfigurationValue  = 0x01,
        .iConfiguration       = 0x00,
<<<<<<< HEAD
        .bmAttributes         = (USB_CONFIG_ATTR_BUSPOWERED |
                                 USB_CONFIG_ATTR_SELF_POWERED),
        .bMaxPower            = MAX_POWER,
    },

=======
        .bmAttributes         = 0xA0, // Bus Powered,  Remote wakeup
        .bMaxPower            = 0x32, // 100 mA 
    },

    // Interface Descriptor 0/0
/*
Interface Descriptor 0/0 Audio, 0 Endpoints
Offset  Field Size  Value Description
0 bLength 1 09h 
1 bDescriptorType 1 04h Interface
2 bInterfaceNumber  1 00h 
3 bAlternateSetting 1 00h 
4 bNumEndpoints 1 00h 
5 bInterfaceClass 1 01h Audio
6 bInterfaceSubClass  1 01h Audio Control
7 bInterfaceProtocol  1 00h 
8 iInterface  1 00h 
*/
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
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

<<<<<<< HEAD
=======
    // Audio Control Interface Header Descriptor
/*
Audio Control Interface Header Descriptor
Offset  Field Size  Value Description
0 bLength 1 09h 
1 bDescriptorType 1 24h Audio Control Interface Header
Subtype 01 
bcdADC 0001 
Total Length 09 
INCollection 0001 
baInterfaceN 01  

*/
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
    .AC_CS_Interface = {
        .bLength            = AC_CS_INTERFACE_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x01,
        .bcdADC             = 0x0100,
        .wTotalLength       = AC_CS_INTERFACE_DESCRIPTOR_SIZE(1),
        .bInCollection      = 0x01,
        .baInterfaceNr      = {0x01},
    },

<<<<<<< HEAD
=======
    // Interface Descriptor 1/0 Audio, 2 Endpoints
/*
Interface Descriptor 1/0 Audio, 2 Endpoints
Offset  Field Size  Value Description
0 bLength 1 09h 
1 bDescriptorType 1 04h Interface
2 bInterfaceNumber  1 01h 
3 bAlternateSetting 1 00h 
4 bNumEndpoints 1 02h 
5 bInterfaceClass 1 01h Audio
6 bInterfaceSubClass  1 03h MIDI Streaming
7 bInterfaceProtocol  1 00h 
8 iInterface  1 00h 
*/

>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
    .MS_Interface = {
        .bLength            = sizeof(usb_descriptor_interface),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_INTERFACE,
        .bInterfaceNumber   = 0x01,
        .bAlternateSetting  = 0x00,
        .bNumEndpoints      = 0x02,
        .bInterfaceClass    = USB_INTERFACE_CLASS_AUDIO,
        .bInterfaceSubClass = USB_INTERFACE_MIDISTREAMING,
        .bInterfaceProtocol = 0x00,
<<<<<<< HEAD
        .iInterface         = 0x04,
    },

=======
        .iInterface         = 0x00,
    },

/*
MIDI Streaming Interface Header Descriptor
Offset  Field Size  Value Description
0 bLength 1 07h 
1 bDescriptorType 1 24h MIDI Streaming Interface Header
2   5 01 00 01 9B 00  
*/
    // MIDI Streaming Interface Header Descriptor    
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
    .MS_CS_Interface = {
        .bLength            = sizeof(MS_CS_INTERFACE_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x01,
        .bcdADC             = 0x0100,
        .wTotalLength       = sizeof(MS_CS_INTERFACE_DESCRIPTOR)
                              
<<<<<<< HEAD
                              +sizeof(MIDI_IN_JACK_DESCRIPTOR)
                              +sizeof(MIDI_IN_JACK_DESCRIPTOR)
                              +sizeof(MIDI_IN_JACK_DESCRIPTOR)
                              +sizeof(MIDI_IN_JACK_DESCRIPTOR)
                              +sizeof(MIDI_IN_JACK_DESCRIPTOR)
                              +sizeof(MIDI_IN_JACK_DESCRIPTOR)
                              +sizeof(MIDI_IN_JACK_DESCRIPTOR)
                              +sizeof(MIDI_IN_JACK_DESCRIPTOR)
                              
                              +MIDI_OUT_JACK_DESCRIPTOR_SIZE(1)
                              +MIDI_OUT_JACK_DESCRIPTOR_SIZE(1)
                              +MIDI_OUT_JACK_DESCRIPTOR_SIZE(1)
                              +MIDI_OUT_JACK_DESCRIPTOR_SIZE(1)
                              +MIDI_OUT_JACK_DESCRIPTOR_SIZE(1)
                              +MIDI_OUT_JACK_DESCRIPTOR_SIZE(1)
                              +MIDI_OUT_JACK_DESCRIPTOR_SIZE(1)
                              +MIDI_OUT_JACK_DESCRIPTOR_SIZE(1)

                              +sizeof(MIDI_USB_DESCRIPTOR_ENDPOINT)
                              +MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(4)
                              +sizeof(MIDI_USB_DESCRIPTOR_ENDPOINT)
                              +MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(4)                             
                                 /* 0x41-4 */,
    },

//  Emb IN Jack 0x02   ---- Ext OUT Jack 0x01 
//  
/*
MIDIStreaming Interface Descriptor:
        bLength                 6
        bDescriptorType        36
        bDescriptorSubtype      2 (MIDI_IN_JACK)
        bJackType               1 Embedded
        bJackID                 1
        iJack                   4 Midi Out 1

*/
    .MIDI_IN_JACK_1 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x01,
        .iJack              = 0x00,  // MIDI OUT 1
    },

/*

MIDIStreaming Interface Descriptor:
        bLength                 6
        bDescriptorType        36
        bDescriptorSubtype      2 (MIDI_IN_JACK)
        bJackType               2 External
        bJackID                 2
        iJack                   0 
*/

    .MIDI_IN_JACK_2 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
=======
                              + 8*sizeof(MIDI_IN_JACK_DESCRIPTOR)
                              + 8*MIDI_OUT_JACK_DESCRIPTOR_SIZE(1)
                              
                              +sizeof(usb_descriptor_endpoint)
                              +MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(4)
                              +sizeof(usb_descriptor_endpoint)
                              +MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(4)
                                 /* 0x41-4 */,
    },

    // MIDI IN JACK DESCRIPTORS

    .MIDI_IN_JACK_1 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x02,
        .bJackType          = 0x01,
        .bJackId            = 0x01,
        .iJack              = 0x04,
    },

    .MIDI_IN_JACK_2 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x02,
        .bJackType          = 0x02,
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
        .bJackId            = 0x02,
        .iJack              = 0x00,
    },

<<<<<<< HEAD
/*
MIDIStreaming Interface Descriptor:
        bLength                 6
        bDescriptorType        36
        bDescriptorSubtype      2 (MIDI_IN_JACK)
        bJackType               1 Embedded
        bJackID                 5
        iJack                   6 Midi Out 2
*/
    .MIDI_IN_JACK_5 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x05,
        .iJack              = 0x00, // MIDI OUT 2
    },

/*
MIDIStreaming Interface Descriptor:
        bLength                 6
        bDescriptorType        36
        bDescriptorSubtype      2 (MIDI_IN_JACK)
        bJackType               2 External
        bJackID                 6
        iJack                   0 
*/
    .MIDI_IN_JACK_6 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
=======
    .MIDI_IN_JACK_3 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x02,
        .bJackType          = 0x01,
        .bJackId            = 0x05,
        .iJack              = 0x06,
    },

    .MIDI_IN_JACK_4 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x02,
        .bJackType          = 0x02,
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
        .bJackId            = 0x06,
        .iJack              = 0x00,
    },

<<<<<<< HEAD
    
/*

MIDIStreaming Interface Descriptor:
        bLength                 6
        bDescriptorType        36
        bDescriptorSubtype      2 (MIDI_IN_JACK)
        bJackType               1 Embedded
        bJackID                 9
        iJack                   8 Midi Out 3
*/

    .MIDI_IN_JACK_9 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x09,
        .iJack              = 0x00, // MIDI OUT 3
    },

/*
MIDIStreaming Interface Descriptor:
        bLength                 6
        bDescriptorType        36
        bDescriptorSubtype      2 (MIDI_IN_JACK)
        bJackType               2 External
        bJackID                10
        iJack                   0 
*/
           
    .MIDI_IN_JACK_A = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x0A,
        .iJack              = 0x00,
    },

/*
MIDIStreaming Interface Descriptor:
        bLength                 6
        bDescriptorType        36
        bDescriptorSubtype      2 (MIDI_IN_JACK)
        bJackType               1 Embedded
        bJackID                13
        iJack                  10 Midi Out 4
*/

    .MIDI_IN_JACK_D = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
        .bJackId            = 0x0D,
        .iJack              = 0x00, // MIDI OUT 4
    },
/*
MIDIStreaming Interface Descriptor:
        bLength                 6
        bDescriptorType        36
        bDescriptorSubtype      2 (MIDI_IN_JACK)
        bJackType               2 External
        bJackID                14
        iJack                   0
*/

    .MIDI_IN_JACK_E = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_IN_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
        .bJackId            = 0x0E,
        .iJack              = 0x00,
    },


        

/*   
MIDIStreaming Interface Descriptor:
        bLength                 9
        bDescriptorType        36
        bDescriptorSubtype      3 (MIDI_OUT_JACK)
        bJackType               1 Embedded
        bJackID                 3
        bNrInputPins            1
        baSourceID( 0)          2
        BaSourcePin( 0)         1
        iJack                   5 Midi In 1
*/

    .MIDI_OUT_JACK_3 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
=======
    .MIDI_IN_JACK_5 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x02,
        .bJackType          = 0x01,
        .bJackId            = 0x09,
        .iJack              = 0x08,
    },

    .MIDI_IN_JACK_6 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x02,
        .bJackType          = 0x02,
        .bJackId            = 0x0A,
        .iJack              = 0x00,
    },
    
    .MIDI_IN_JACK_7 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x02,
        .bJackType          = 0x01,
        .bJackId            = 0x0D,
        .iJack              = 0x0A,
    },

    .MIDI_IN_JACK_8 = {
        .bLength            = sizeof(MIDI_IN_JACK_DESCRIPTOR),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x02,
        .bJackType          = 0x02,
        .bJackId            = 0x0E,
        .iJack              = 0x00,
    },
               
    // MIDI OUT JACK DESCRIPTORS
    
    .MIDI_OUT_JACK_1 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x03,
        .bJackType          = 0x01,
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
        .bJackId            = 0x03,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x02},
        .baSourcePin        = {0x01},
<<<<<<< HEAD
        .iJack              = 0x00, // MIDI IN 1
    },

/*
MIDIStreaming Interface Descriptor:
        bLength                 9
        bDescriptorType        36
        bDescriptorSubtype      3 (MIDI_OUT_JACK)
        bJackType               2 External
        bJackID                 4
        bNrInputPins            1
        baSourceID( 0)          1
        BaSourcePin( 0)         1
        iJack                   0 
*/

    .MIDI_OUT_JACK_4 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
=======
        .iJack              = 0x05,
    },

    .MIDI_OUT_JACK_2 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x03,
        .bJackType          = 0x02,
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
        .bJackId            = 0x04,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x01},
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },

<<<<<<< HEAD
/*
MIDIStreaming Interface Descriptor:
        bLength                 9
        bDescriptorType        36
        bDescriptorSubtype      3 (MIDI_OUT_JACK)
        bJackType               1 Embedded
        bJackID                 7
        bNrInputPins            1
        baSourceID( 0)          6
        BaSourcePin( 0)         1
        iJack                   7 Midi In 2
*/

    .MIDI_OUT_JACK_7 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
=======

    .MIDI_OUT_JACK_3 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x03,
        .bJackType          = 0x01,
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
        .bJackId            = 0x07,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x06},
        .baSourcePin        = {0x01},
<<<<<<< HEAD
        .iJack              = 0x00, // MIDI IN 2
    },

/*
MIDIStreaming Interface Descriptor:
        bLength                 9
        bDescriptorType        36
        bDescriptorSubtype      3 (MIDI_OUT_JACK)
        bJackType               2 External
        bJackID                 8
        bNrInputPins            1
        baSourceID( 0)          5
        BaSourcePin( 0)         1
        iJack                   0 
*/
    .MIDI_OUT_JACK_8 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
=======
        .iJack              = 0x07,
    },

    .MIDI_OUT_JACK_4 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x03,
        .bJackType          = 0x02,
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
        .bJackId            = 0x08,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x05},
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },

<<<<<<< HEAD
/*
MIDIStreaming Interface Descriptor:
        bLength                 9
        bDescriptorType        36
        bDescriptorSubtype      3 (MIDI_OUT_JACK)
        bJackType               1 Embedded
        bJackID                11
        bNrInputPins            1
        baSourceID( 0)         10
        BaSourcePin( 0)         1
        iJack                   9 Midi In 3
*/

    .MIDI_OUT_JACK_B = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
=======
    .MIDI_OUT_JACK_5 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x03,
        .bJackType          = 0x01,
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
        .bJackId            = 0x0B,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x0A},
        .baSourcePin        = {0x01},
<<<<<<< HEAD
        .iJack              = 0x00, // MIDI IN 3
    },


/*
MIDIStreaming Interface Descriptor:
        bLength                 9
        bDescriptorType        36
        bDescriptorSubtype      3 (MIDI_OUT_JACK)
        bJackType               2 External
        bJackID                12
        bNrInputPins            1
        baSourceID( 0)          9
        BaSourcePin( 0)         1
        iJack                   0 
*/

    .MIDI_OUT_JACK_C = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
=======
        .iJack              = 0x09,
    },

    .MIDI_OUT_JACK_6 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x03,
        .bJackType          = 0x02,
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
        .bJackId            = 0x0C,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x09},
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },

<<<<<<< HEAD
/*
MIDIStreaming Interface Descriptor:
        bLength                 9
        bDescriptorType        36
        bDescriptorSubtype      3 (MIDI_OUT_JACK)
        bJackType               1 Embedded
        bJackID                15
        bNrInputPins            1
        baSourceID( 0)         14
        BaSourcePin( 0)         1
        iJack                  11 Midi In 4
*/
    .MIDI_OUT_JACK_F = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EMBEDDED,
=======
    .MIDI_OUT_JACK_7 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x03,
        .bJackType          = 0x01,
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
        .bJackId            = 0x0F,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x0E},
        .baSourcePin        = {0x01},
<<<<<<< HEAD
        .iJack              = 0x00, // MIDI IN 4
    },

/*
MIDIStreaming Interface Descriptor:
        bLength                 9
        bDescriptorType        36
        bDescriptorSubtype      3 (MIDI_OUT_JACK)
        bJackType               2 External
        bJackID                16
        bNrInputPins            1
        baSourceID( 0)         13
        BaSourcePin( 0)         1
        iJack                   0
*/
    .MIDI_OUT_JACK_10 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = MIDI_OUT_JACK,
        .bJackType          = MIDI_JACK_EXTERNAL,
=======
        .iJack              = 0x0B,
    },

    .MIDI_OUT_JACK_8 = {
        .bLength            = MIDI_OUT_JACK_DESCRIPTOR_SIZE(1),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .SubType            = 0x03,
        .bJackType          = 0x02,
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
        .bJackId            = 0x10,
        .bNrInputPins       = 0x01,
        .baSourceId         = {0x0D},
        .baSourcePin        = {0x01},
        .iJack              = 0x00,
    },


<<<<<<< HEAD
/*
Endpoint Descriptor:
        bLength                 9
        bDescriptorType         5
        bEndpointAddress     0x01  EP 1 OUT
        bmAttributes            2
          Transfer Type            Bulk
          Synch Type               None
          Usage Type               Data
        wMaxPacketSize     0x0010  1x 16 bytes
        bInterval               0
        bRefresh                0
        bSynchAddress           0

MIDIStreaming Endpoint Descriptor:
          bLength                 8
          bDescriptorType        37
          bDescriptorSubtype      1 (GENERAL)
          bNumEmbMIDIJack         4
          baAssocJackID( 0)       1
          baAssocJackID( 1)       5
          baAssocJackID( 2)       9
          baAssocJackID( 3)      13
*/


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
=======
    // OUT Endpoint Descriptor
   
    .DataOutEndpoint = {
        .bLength            = sizeof(usb_descriptor_endpoint),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress   = 0x01,
        .bmAttributes       = USB_EP_TYPE_BULK,
        .wMaxPacketSize     = 0x0010, // 16 Bytes
        .bInterval          = 0x00,
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
    },

    .MS_CS_DataOutEndpoint = {
      .bLength              = MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(4),
      .bDescriptorType      = USB_DESCRIPTOR_TYPE_CS_ENDPOINT,
      .SubType              = 0x01,
      .bNumEmbMIDIJack      = 0x04,
<<<<<<< HEAD
      .baAssocJackID        = {0x01,0x05,0X09,0X0D},
  },

/*
Endpoint Descriptor:
        bLength                 9
        bDescriptorType         5
        bEndpointAddress     0x81  EP 1 IN
        bmAttributes            2
          Transfer Type            Bulk
          Synch Type               None
          Usage Type               Data
        wMaxPacketSize     0x0010  1x 16 bytes
        bInterval               0
        bRefresh                0
        bSynchAddress           0

MIDIStreaming Endpoint Descriptor:
          bLength                 8
          bDescriptorType        37
          bDescriptorSubtype      1 (GENERAL)
          bNumEmbMIDIJack         4
          baAssocJackID( 0)       3
          baAssocJackID( 1)       7
          baAssocJackID( 2)      11
          baAssocJackID( 3)      15
*/

    .DataInEndpoint = {
        .bLength          = sizeof(MIDI_USB_DESCRIPTOR_ENDPOINT),
        .bDescriptorType  = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress = (USB_DESCRIPTOR_ENDPOINT_IN | MIDI_STREAM_IN_ENDP),
        .bmAttributes     = USB_EP_TYPE_BULK,
        .wMaxPacketSize   = MIDI_STREAM_EPSIZE,
        .bInterval        = 0x00,
        .bRefresh         = 0x00,
        .bSynchAddress    = 0x00,
=======
      .baAssocJackID        = {0x01,0x05,0x09,0x0D},
	  },
    
    // IN Endpoint Descriptor

    .DataInEndpoint = {
        .bLength          = sizeof(usb_descriptor_endpoint),
        .bDescriptorType  = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress = 0x81,
        .bmAttributes     = USB_EP_TYPE_BULK,
        .wMaxPacketSize   = 0x0010, // 16 Bytes 
        .bInterval        = 0x00,
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
    },

    .MS_CS_DataInEndpoint = {
      .bLength              = MS_CS_BULK_ENDPOINT_DESCRIPTOR_SIZE(4),
      .bDescriptorType      = USB_DESCRIPTOR_TYPE_CS_ENDPOINT,
      .SubType              = 0x01,
      .bNumEmbMIDIJack      = 0x04,
<<<<<<< HEAD
      .baAssocJackID        = {0x03,0X07,0X0B,0x0F},
  },

};
// --------------------------------------------------------------------------------------
//  String Descriptors:
// --------------------------------------------------------------------------------------

/*  we may choose to specify any or none of the following string
=======
      .baAssocJackID        = {0x03,0x07,0x0B,0x0F},
	},

};

/*
  String Descriptors:
  we may choose to specify any or none of the following string
>>>>>>> e883d68b241c460204d25a35e46247e8f6e66d2d
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
    .bLength = USB_DESCRIPTOR_STRING_LEN(8),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'M', 0, 'a', 0, 'p', 0, 'l', 0,
                'e', 0, ' ', 0, ' ', 0, ' ', 0},
};

static const usb_descriptor_string usbMIDIDescriptor_iProduct = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(10),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'M', 0, 'I', 0, 'D', 0, 'I', 0, '4', 0, 'X', 0, '4', 0, ' ', 0, 'M', 0, 'O', 0},
  //  .bString = {'D', 0, 'i', 0, 'r', 0, 'o', 0, ' ', 0, 'S', 0, 'y', 0, 'n', 0, 't', 0, 'h', 0},
};

static const usb_descriptor_string usbMIDIDescriptor_iInterface = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(4),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'M', 0, 'I', 0, 'D', 0, 'I', 0},
};

static const usb_descriptor_string usbMIDIDescriptor_iJack1 = {
    .bLength = USB_DESCRIPTOR_STRING_LEN(5),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
    .bString = {'J', 0, 'a', 0, 'c', 0, 'k', 0, '1', 0},
};


static ONE_DESCRIPTOR usbMidiDevice_Descriptor = {
    (uint8*)&usbMIDIDescriptor_Device,
    sizeof(usb_descriptor_device)
};

static ONE_DESCRIPTOR usbMidiConfig_Descriptor = {
    (uint8*)&usbMIDIDescriptor_Config,
    sizeof(usb_descriptor_config)
};

#define N_STRING_DESCRIPTORS 3
static ONE_DESCRIPTOR String_Descriptor[N_STRING_DESCRIPTORS] = {
    {(uint8*)&usbMIDIDescriptor_LangID,       USB_DESCRIPTOR_STRING_LEN(1)},
    {(uint8*)&usbMIDIDescriptor_iManufacturer,USB_DESCRIPTOR_STRING_LEN(8)},
    {(uint8*)&usbMIDIDescriptor_iProduct,     USB_DESCRIPTOR_STRING_LEN(10)},
    //{(uint8*)&usbMIDIDescriptor_iInterface,     USB_DESCRIPTOR_STRING_LEN(4)},
    
    //{(uint8*)&usbMIDIDescriptor_iJack1,     USB_DESCRIPTOR_STRING_LEN(5)}
};
