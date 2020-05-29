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
#ifndef USB_MIDI_DEVICE_H
#define USB_MIDI_DEVICE_H
#include <string.h>
#include "hardware_config.h"
#include "usb_midi_device.h"

#include <libmaple/usb.h>
#include <libmaple/nvic.h>
#include <libmaple/delay.h>

/* Private headers */
#include "usb_lib_globals.h"
#include "usb_reg_map.h"

/* usb_lib headers */
#include "usb_type.h"
#include "usb_core.h"
#include "usb_def.h"

#include "usb_midi_descriptor.c"

static void   usb_midi_DataTxCb(void);
static void   usb_midi_DataRxCb(void);
static void   usb_midi_Init(void);
static void   usb_midi_Reset(void);
static RESULT usb_midi_DataSetup(uint8_t request);
static RESULT usb_midi_NoDataSetup(uint8_t request);
static RESULT usb_midi_GetInterfaceSetting(uint8_t interface, uint8_t alt_setting);
static uint8* usb_midi_GetDeviceDescriptor(uint16_t length);
static uint8* usb_midi_GetConfigDescriptor(uint16_t length);
static uint8* usb_midi_GetStringDescriptor(uint16_t length);
static void   usb_midi_SetConfiguration(void);
static void   usb_midi_SetDeviceAddress(void);

/*
 * Etc.
 */

/* I/O state */

/* Received data */
static volatile uint32_t midiBufferRx[MIDI_STREAM_EPSIZE/4];
/* Read index into midiBufferRx */
static volatile uint32_t rx_offset = 0;
/* Transmit data */
static volatile uint32_t midiBufferTx[MIDI_STREAM_EPSIZE/4];
/* Write index into midiBufferTx */
static volatile uint32_t tx_offset = 0;
/* Number of bytes left to transmit */
static volatile uint32_t n_unsent_packets = 0;
/* Are we currently sending an IN packet? */
static volatile uint8_t transmitting = 0;
/* Number of unread bytes */
static volatile uint32_t n_unread_packets = 0;


// --------------------------------------------------------------------------------------
// ENDPOINTS CALLBACKS TABLES
// --------------------------------------------------------------------------------------
void (*usb_midi_ep_int_in[7])(void) =
    {usb_midi_DataTxCb,
     NOP_Process,
     NOP_Process,
     NOP_Process,
     NOP_Process,
     NOP_Process,
     NOP_Process};

void (*usb_midi_ep_int_out[7])(void) =
    {NOP_Process,
     usb_midi_DataRxCb,
     NOP_Process,
     NOP_Process,
     NOP_Process,
     NOP_Process,
     NOP_Process};

// --------------------------------------------------------------------------------------
// Globals required by usb_lib.
// These override core USB functionality which was declared __weak.
// --------------------------------------------------------------------------------------

//
// DEVICE Device_Table = {
//     .Total_Endpoint      = USB_MIDI_NUM_ENDPTS,
//     .Total_Configuration = 1
// };
//
//
// DEVICE_PROP Device_Property = {
//     .Init                        = usb_midi_Init,
//     .Reset                       = usb_midi_Reset,
//     .Process_Status_IN           = NOP_Process,
//     .Process_Status_OUT          = NOP_Process,
//     .Class_Data_Setup            = usb_midi_DataSetup,
//     .Class_NoData_Setup          = usb_midi_NoDataSetup,
//     .Class_Get_Interface_Setting = usb_midi_GetInterfaceSetting,
//     .GetDeviceDescriptor         = usb_midi_GetDeviceDescriptor,
//     .GetConfigDescriptor         = usb_midi_GetConfigDescriptor,
//     .GetStringDescriptor         = usb_midi_GetStringDescriptor,
//     .RxEP_buffer                 = NULL,
//     .MaxPacketSize               = USB_MIDI_MAX_PACKET_SIZE
// };
//
// USER_STANDARD_REQUESTS User_Standard_Requests = {
//     .User_GetConfiguration   = NOP_Process,
//     .User_SetConfiguration   = usb_midi_SetConfiguration,
//     .User_GetInterface       = NOP_Process,
//     .User_SetInterface       = NOP_Process,
//     .User_GetStatus          = NOP_Process,
//     .User_ClearFeature       = NOP_Process,
//     .User_SetEndPointFeature = NOP_Process,
//     .User_SetDeviceFeature   = NOP_Process,
//     .User_SetDeviceAddress   = usb_midi_SetDeviceAddress
// };


// --------------------------------------------------------------------------------------
// DEVICE DESCRIPTOR MANIPULATION
// --------------------------------------------------------------------------------------

void usb_midi_set_vid_pid(uint16_t vid, uint16_t pid) {
  usbMIDIDescriptor_Device.idVendor           = vid;
  usbMIDIDescriptor_Device.idProduct          = pid;

}

void usb_midi_set_product_string(char stringDescriptor[]) {

  // Check the true string descriptor size allocated by manual declaration
  // in usb_midi_descriptor.c. It is critical !!!

  // fill the existing string descriptor with 0
  memset(&usbMIDIDescriptor_iProduct.bString,0, (USB_MIDI_PRODUCT_STRING_SIZE)*2+2);

  // Copy string to the descriptor. The string must be zero ending !!!
  uint8_t i = 0;
  while ( stringDescriptor[i] != 0 ) {
    // The string is wide characters type.  2 bytes / char.
    usbMIDIDescriptor_iProduct.bString[i*2] = stringDescriptor[i];
    if (++i >= USB_MIDI_PRODUCT_STRING_SIZE ) break;
  }

  // Adjust the length
  usbMIDIDescriptor_iProduct.bLength = i*2+2;
  usbMIDIString_Descriptor[usbMIDIDescriptor_Device.iProduct].Descriptor_Size = i*2+2;
}

// --------------------------------------------------------------------------------------
// ENABLE / DISABLE / POWERDOWN   MIDI DEVICE
// --------------------------------------------------------------------------------------
void usb_midi_enable(gpio_dev *disc_dev, uint8_t disc_bit, uint8_t level) {
    /* Present ourselves to the host. Writing 0 to "disc" pin must
     * pull USB_DP pin up while leaving USB_DM pulled down by the
     * transceiver. See USB 2.0 spec, section 7.1.7.3.
     *
     * FT : The function was modified to support a new "level" parameter,
     * to set the 0 or 1 regarding the logic level used by the DISC pin.
     *
     */

      // USB MIDI Device setup.  We dont redeclare Device_Table

      Device_Table.Total_Endpoint      = USB_MIDI_NUM_ENDPTS;
      Device_Table.Total_Configuration = 1;

      Device_Property.Init                        = usb_midi_Init;
      Device_Property.Reset                       = usb_midi_Reset;
      Device_Property.Process_Status_IN           = NOP_Process;
      Device_Property.Process_Status_OUT          = NOP_Process;
      Device_Property.Class_Data_Setup            = usb_midi_DataSetup;
      Device_Property.Class_NoData_Setup          = usb_midi_NoDataSetup;
      Device_Property.Class_Get_Interface_Setting = usb_midi_GetInterfaceSetting;
      Device_Property.GetDeviceDescriptor         = usb_midi_GetDeviceDescriptor;
      Device_Property.GetConfigDescriptor         = usb_midi_GetConfigDescriptor;
      Device_Property.GetStringDescriptor         = usb_midi_GetStringDescriptor;
      Device_Property.RxEP_buffer                 = NULL;
      Device_Property.MaxPacketSize               = USB_MIDI_MAX_PACKET_SIZE;

      User_Standard_Requests.User_GetConfiguration   = NOP_Process;
      User_Standard_Requests.User_SetConfiguration   = usb_midi_SetConfiguration;
      User_Standard_Requests.User_GetInterface       = NOP_Process;
      User_Standard_Requests.User_SetInterface       = NOP_Process;
      User_Standard_Requests.User_GetStatus          = NOP_Process;
      User_Standard_Requests.User_ClearFeature       = NOP_Process;
      User_Standard_Requests.User_SetEndPointFeature = NOP_Process;
      User_Standard_Requests.User_SetDeviceFeature   = NOP_Process;
      User_Standard_Requests.User_SetDeviceAddress   = usb_midi_SetDeviceAddress;

      if (disc_dev != NULL) {
         gpio_set_mode(disc_dev, disc_bit, GPIO_OUTPUT_PP);
         gpio_write_bit(disc_dev, disc_bit, level);
     }

      /* Initialize the USB peripheral. */
      usb_init_usblib(USBLIB, usb_midi_ep_int_in, usb_midi_ep_int_out);
}

void usb_midi_disable(gpio_dev *disc_dev, uint8_t disc_bit, uint8_t level) {
    /* Turn off the interrupt and signal disconnect (see e.g. USB 2.0
     * spec, section 7.1.7.3).
     *
     * FT : The function was modified to support a new "level" parameter,
     * to set the 0 or 1 regarding the logic level used by the DISC pin.
     */

    nvic_irq_disable(NVIC_USB_LP_CAN_RX0);
    if (disc_dev != NULL) {
        gpio_write_bit(disc_dev, disc_bit, level);
    }
    usb_power_off();
}


// --------------------------------------------------------------------------------------
// USB BUFFERS I/O
// --------------------------------------------------------------------------------------
/* TODO these could use some improvement; they're fairly
 * straightforward ports of the analogous ST code.  The PMA blit
 * routines in particular are obvious targets for performance
 * measurement and tuning. */
static void usb_copy_to_pma(const uint8_t *buf, uint16_t len, uint16_t pma_offset) {
    uint16_t *dst = (uint16*)usb_pma_ptr(pma_offset);
    uint16_t n = len >> 1;
    uint16_t i;
    for (i = 0; i < n; i++) {
        *dst = (uint16)(*buf) | *(buf + 1) << 8;
        buf += 2;
        dst += 2;
    }
    if (len & 1) {
        *dst = *buf;
    }
}

static void usb_copy_from_pma(uint8_t *buf, uint16_t len, uint16_t pma_offset) {
    uint32_t *src = (uint32*)usb_pma_ptr(pma_offset);
    uint16_t *dst = (uint16*)buf;
    uint16_t n = len >> 1;
    uint16_t i;
    for (i = 0; i < n; i++) {
        *dst++ = *src++;
    }
    if (len & 1) {
        *dst = *src & 0xFF;
    }
}


// --------------------------------------------------------------------------------------
// USB TX / RX / PEEK
// --------------------------------------------------------------------------------------

/* This function is non-blocking.
 *
 * It copies data from a usercode buffer into the USB peripheral TX
 * buffer, and returns the number of bytes copied. */
uint32_t usb_midi_tx(const uint32* buf, uint32_t packets) {
    uint32_t bytes=packets*4;
    /* Last transmission hasn't finished, so abort. */
    if (usb_midi_is_transmitting()) {
		/* Copy to TxBuffer */

        return 0;  /* return len */
    }

    /* We can only put USB_MIDI_TX_EPSIZE bytes in the buffer. */
    if (bytes > MIDI_STREAM_EPSIZE) {
        bytes = MIDI_STREAM_EPSIZE;
        packets=bytes/4;
    }

    /* Queue bytes for sending. */
    if (packets) {
        usb_copy_to_pma((uint8_t *)buf, bytes, MIDI_STREAM_IN_EPADDR);
    }
    // We still need to wait for the interrupt, even if we're sending
    // zero bytes. (Sending zero-size packets is useful for flushing
    // host-side buffers.)
    usb_set_ep_tx_count(MIDI_STREAM_IN_ENDP, bytes);
    n_unsent_packets = packets;
    transmitting = 1;
    usb_set_ep_tx_stat(MIDI_STREAM_IN_ENDP, USB_EP_STAT_TX_VALID);

    return packets;
}

/* Nonblocking byte receive.
 *
 * Copies up to len bytes from our private data buffer (*NOT* the PMA)
 * into buf and deq's the FIFO. */
uint32_t usb_midi_rx(uint32* buf, uint32_t packets) {
    /* Copy bytes to buffer. */
    uint32_t n_copied = usb_midi_peek(buf, packets);

    usb_midi_mark_read(n_copied);

    // /* Mark bytes as read. */
    // n_unread_packets -= n_copied;
    // rx_offset += n_copied;
    //
    // /* If all bytes have been read, re-enable the RX endpoint, which
    //  * was set to NAK when the current batch of bytes was received. */
    // if (n_unread_packets == 0) {
    //     usb_set_ep_rx_count(MIDI_STREAM_OUT_ENDP, MIDI_STREAM_EPSIZE);
    //     usb_set_ep_rx_stat(MIDI_STREAM_OUT_ENDP, USB_EP_STAT_RX_VALID);
    //     rx_offset = 0;
    // }

    return n_copied;
}

/* Nonblocking byte lookahead.
 *
 * Looks at unread bytes without marking them as read. */
uint32_t usb_midi_peek(uint32* buf, uint32_t packets) {
    uint16_t i;
    if (packets > n_unread_packets) {
        packets = n_unread_packets;
    }

    for (i = 0; i < packets; i++) {
        buf[i] = midiBufferRx[i + rx_offset];
    }

    return packets;
}

/* Nonblocking byte receive.
 * Mark n packets as read  when they have been peeked
 * Warning : this call must only follow  a peek !!
 * Use readPacket instead if you need to read and mark
 */

uint32_t usb_midi_mark_read(uint32_t n_copied) {
    /* Mark bytes as read. */
    n_unread_packets -= n_copied;
    rx_offset += n_copied;

    /* If all bytes have been read, re-enable the RX endpoint, which
     * was set to NAK when the current batch of bytes was received. */
    if (n_unread_packets == 0) {
        usb_set_ep_rx_count(MIDI_STREAM_OUT_ENDP, MIDI_STREAM_EPSIZE);
        usb_set_ep_rx_stat(MIDI_STREAM_OUT_ENDP, USB_EP_STAT_RX_VALID);
        rx_offset = 0;
    }

    return n_copied;
}


// --------------------------------------------------------------------------------------
// USB MIDI STATE
// --------------------------------------------------------------------------------------

uint32_t usb_midi_data_available(void) {
    return n_unread_packets;
}

uint8_t usb_midi_is_transmitting(void) {
    return transmitting;
}

uint16_t usb_midi_get_pending(void) {
    return n_unsent_packets;
}

// --------------------------------------------------------------------------------------
// ENDPOINTS CALLBACKS
// --------------------------------------------------------------------------------------

static void usb_midi_DataTxCb(void) {
    n_unsent_packets = 0;
    transmitting = 0;
}

static void usb_midi_DataRxCb(void) {
    usb_set_ep_rx_stat(MIDI_STREAM_OUT_ENDP, USB_EP_STAT_RX_NAK);
    n_unread_packets = usb_get_ep_rx_count(MIDI_STREAM_OUT_ENDP) / 4;
    /* This copy won't overwrite unread bytes, since we've set the RX
     * endpoint to NAK, and will only set it to VALID when all bytes
     * have been read. */

    usb_copy_from_pma((uint8*)midiBufferRx, n_unread_packets * 4,
                      MIDI_STREAM_OUT_EPADDR);

    if (n_unread_packets == 0) {
        usb_set_ep_rx_count(MIDI_STREAM_OUT_ENDP, MIDI_STREAM_EPSIZE);
        usb_set_ep_rx_stat(MIDI_STREAM_OUT_ENDP, USB_EP_STAT_RX_VALID);
        rx_offset = 0;
    }

}

// --------------------------------------------------------------------------------------
// USB User functions
// --------------------------------------------------------------------------------------

/* NOTE: Nothing specific to this device class in this function, move to usb_lib */
static void usb_midi_Init(void) {
    pInformation->Current_Configuration = 0;

    USB_BASE->CNTR = USB_CNTR_FRES;

    USBLIB->irq_mask = 0;
    USB_BASE->CNTR = USBLIB->irq_mask;
    USB_BASE->ISTR = 0;
    USBLIB->irq_mask = USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
    USB_BASE->CNTR = USBLIB->irq_mask;

    USB_BASE->ISTR = 0;
    USBLIB->irq_mask = USB_ISR_MSK;
    USB_BASE->CNTR = USBLIB->irq_mask;

    nvic_irq_enable(NVIC_USB_LP_CAN_RX0);
    USBLIB->state = USB_UNCONNECTED;
}


static void usb_midi_Reset(void) {
    pInformation->Current_Configuration = 0;

    /* current feature is current bmAttributes */
    pInformation->Current_Feature = (USB_CONFIG_ATTR_BUSPOWERED |
                                     USB_CONFIG_ATTR_SELF_POWERED);

    USB_BASE->BTABLE = USB_MIDI_BTABLE_ADDRESS;

    // Setup control endpoint  */
    usb_set_ep_type     (USB_MIDI_CTRL_ENDP, USB_EP_EP_TYPE_CONTROL);
    usb_set_ep_tx_stat  (USB_MIDI_CTRL_ENDP, USB_EP_STAT_TX_STALL  );
    usb_set_ep_rx_addr  (USB_MIDI_CTRL_ENDP, USB_MIDI_CTRL_RX_ADDR );
    usb_set_ep_tx_addr  (USB_MIDI_CTRL_ENDP, USB_MIDI_CTRL_TX_ADDR );

    usb_clear_status_out(USB_MIDI_CTRL_ENDP                        );
    usb_set_ep_rx_count (USB_MIDI_CTRL_ENDP, pProperty->MaxPacketSize);
    usb_set_ep_rx_stat  (USB_MIDI_CTRL_ENDP, USB_EP_STAT_RX_VALID);

    /* TODO figure out differences in style between RX/TX EP setup */

   /* set up data endpoint OUT (RX) */
    usb_set_ep_type       (MIDI_STREAM_OUT_ENDP, USB_EP_EP_TYPE_BULK   );
    usb_set_ep_rx_addr    (MIDI_STREAM_OUT_ENDP, MIDI_STREAM_OUT_EPADDR);
    usb_set_ep_rx_count   (MIDI_STREAM_OUT_ENDP, MIDI_STREAM_EPSIZE    );
    usb_set_ep_rx_stat    (MIDI_STREAM_OUT_ENDP, USB_EP_STAT_RX_VALID  );

    /* set up data endpoint IN (TX)  */
    usb_set_ep_type       (MIDI_STREAM_IN_ENDP, USB_EP_EP_TYPE_BULK   );
    usb_set_ep_tx_addr    (MIDI_STREAM_IN_ENDP, MIDI_STREAM_IN_EPADDR );
    usb_set_ep_tx_stat    (MIDI_STREAM_IN_ENDP, USB_EP_STAT_TX_NAK    );
    usb_set_ep_rx_stat    (MIDI_STREAM_IN_ENDP, USB_EP_STAT_RX_DISABLED);

    USBLIB->state = USB_ATTACHED;
    SetDeviceAddress(0);

    /* Reset the RX/TX state */
    n_unread_packets = 0;
    n_unsent_packets = 0;
    rx_offset = 0;
}

static RESULT usb_midi_DataSetup(uint8_t request) {
    uint8* (*CopyRoutine)(uint16) = 0;

    // if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
    //
    // }

    if (CopyRoutine == NULL) {
        return USB_UNSUPPORT;
    }

    pInformation->Ctrl_Info.CopyData = CopyRoutine;
    pInformation->Ctrl_Info.Usb_wOffset = 0;
    (*CopyRoutine)(0);
    return USB_SUCCESS;
}

static RESULT usb_midi_NoDataSetup(uint8_t request) {
    RESULT ret = USB_UNSUPPORT;

    // if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
    // }
    return ret;
}

static RESULT usb_midi_GetInterfaceSetting(uint8_t interface, uint8_t alt_setting) {
    if (alt_setting > 0) {
        return USB_UNSUPPORT;
    } else if (interface > 1) {
        return USB_UNSUPPORT;
    }

    return USB_SUCCESS;
}

static uint8* usb_midi_GetDeviceDescriptor(uint16_t length) {
    return Standard_GetDescriptorData(length, &usbMidiDevice_Descriptor);
}

static uint8* usb_midi_GetConfigDescriptor(uint16_t length) {
    return Standard_GetDescriptorData(length, &usbMidiConfig_Descriptor);
}

static uint8* usb_midi_GetStringDescriptor(uint16_t length) {
    uint8_t wValue0 = pInformation->USBwValue0;

    if (wValue0 > USB_MIDI_N_STRING_DESCRIPTORS) {
        return NULL;
    }
    return Standard_GetDescriptorData(length, &usbMIDIString_Descriptor[wValue0]);
}

static void usb_midi_SetConfiguration(void) {
    if (pInformation->Current_Configuration != 0) {
        USBLIB->state = USB_CONFIGURED;
    }
}

static void usb_midi_SetDeviceAddress(void) {
    USBLIB->state = USB_ADDRESSED;
}
#endif
