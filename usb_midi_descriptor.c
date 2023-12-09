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
// Full assembled USB Descriptor FOR MIDIKLIK 4X4 DEVICE
// ---------------------------------------------------------------

#include "hardware_config.h"
#include "usb_midi_device.h"

 // ---------------------------------------------------------------------------
 //  String Descriptors:
 // ----------------------------------------------------------------------------

 /* Remember that bLengh is the full length of the descriptor including
    blength and bDescriptorType (2 bytes).  So the blength of a string of n bytes
    will be 2 + n * 2. USB_DESCRIPTOR_STRING_LEN macro does that for us.
 */

 /* Unicode language identifier: 0x0409 is US English */
 static const usb_descriptor_string usbMIDIDescriptor_LangID = {
     .bLength         = USB_DESCRIPTOR_STRING_LEN(1),
     .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
     .bString         = {0x09, 0x04},
 };

 #define STRING_IMANUFACTURER_LEN 14
 #define STRING_IMANUFACTURER_ID 1

 static const usb_descriptor_string usbMIDIDescriptor_iManufacturer = {
     .bLength = USB_DESCRIPTOR_STRING_LEN(STRING_IMANUFACTURER_LEN),
     .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
     // TheKikGen Labs
     .bString = {'T', 0, 'h', 0, 'e', 0, 'K', 0,'i', 0, 'K', 0, 'G', 0, 'e', 0, 'n', 0, ' ', 0, 'L', 0, 'a', 0, 'b', 0, 's', 0},
 };


 // We reserve 30 bytes room to change the product string later but the default
 // length is manually adjusted to 16.
 #define STRING_IPRODUCT_LEN 11
 #define STRING_IPRODUCT_ID 2

 static usb_descriptor_string usbMIDIDescriptor_iProduct = {
     .bLength = USB_DESCRIPTOR_STRING_LEN(STRING_IPRODUCT_LEN),
     .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
     // USB_MIDI_PRODUCT_STRING_SIZE = 30
     .bString =  { 'M', 0, 'i', 0, 'd', 0, 'i', 0, 'K', 0, 'l', 0, 'i', 0, 'K', 0, ' ', 0 ,'4', 0,  // 10
                   'x', 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 , 0 , 0 , 0,  // 20
                    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 , 0,  // 30
                 }
 };


 #define STRING_IINTERFACE_LEN 4
 #define STRING_IINTERFACE_ID 3

 static const usb_descriptor_string usbMIDIDescriptor_iInterface = {
     .bLength = USB_DESCRIPTOR_STRING_LEN(STRING_IINTERFACE_LEN),
     .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
     .bString = {'M', 0, 'i', 0, 'd', 0, 'i', 0},
 };

 // Midi IN 1-n
 #define STRING_IJACK_IN_LEN 9
 #define STRING_IJACK_IN_ID 4

 static const usb_descriptor_string usbMIDIDescriptor_iJack_IN = {
     .bLength = USB_DESCRIPTOR_STRING_LEN(STRING_IJACK_IN_LEN),
     .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
     .bString = {'U', 0, 'M', 0, 'K', 0, ' ', 0, '4', 0, 'X', 0, ' ', 0, 'I', 0, 'N', 0 },
 };


 // Midi OUT 1-n
 #define STRING_IJACK_OUT_LEN 10
 #define STRING_IJACK_OUT_ID 5

 static const usb_descriptor_string usbMIDIDescriptor_iJack_OUT = {
     .bLength = USB_DESCRIPTOR_STRING_LEN(STRING_IJACK_OUT_LEN),
     .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
     .bString = {'U', 0, 'M', 0, 'K', 0, ' ', 0, '4', 0, 'X', 0, ' ', 0, 'O', 0, 'U', 0, 'T', 0 },
 };

 // Assemble string descriptors array
 static ONE_DESCRIPTOR usbMIDIString_Descriptor[] = {
     {(uint8*)&usbMIDIDescriptor_LangID,       USB_DESCRIPTOR_STRING_LEN(1) },
     {(uint8*)&usbMIDIDescriptor_iManufacturer,USB_DESCRIPTOR_STRING_LEN(STRING_IMANUFACTURER_LEN)},
     {(uint8*)&usbMIDIDescriptor_iProduct,     USB_DESCRIPTOR_STRING_LEN(STRING_IPRODUCT_LEN)},// R/W
     {(uint8*)&usbMIDIDescriptor_iInterface,   USB_DESCRIPTOR_STRING_LEN(STRING_IINTERFACE_LEN) },
     {(uint8*)&usbMIDIDescriptor_iJack_IN,     USB_DESCRIPTOR_STRING_LEN(STRING_IJACK_IN_LEN) },
     {(uint8*)&usbMIDIDescriptor_iJack_OUT,    USB_DESCRIPTOR_STRING_LEN(STRING_IJACK_OUT_LEN) },
 };

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
       .idVendor           = USB_MIDI_VENDORID,  // R/W
       .idProduct          = USB_MIDI_PRODUCTID, // R/W
       // Fill the firmware version from build number
       .bcdDevice          = BCD_VERSION,
       .iManufacturer      = STRING_IMANUFACTURER_ID,
       .iProduct           = STRING_IPRODUCT_ID,
       .iSerialNumber      = 0x00,
       .bNumConfigurations = 0x01,
  };

 // ---------------------------------------------------------------
 // CONFIGURATION DESCRIPTOR
 // ---------------------------------------------------------------

 // Initialized in usb_midi_init_descriptor_config. Global.
 static ONE_DESCRIPTOR usbMidiConfig_Descriptor;
 
 static const ONE_DESCRIPTOR usbMidiDevice_Descriptor = {
     (uint8*)&usbMIDIDescriptor_Device,
     sizeof(usb_descriptor_device)
 };

 