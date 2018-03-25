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
 * @file libmaple/usb/stm32f1/usb_midi_device.c
 * @brief USB MIDI.
 *
 * FIXME: this works on the STM32F1 USB peripherals, and probably no
 * place else. Nonportable bits really need to be factored out, and
 * the result made cleaner.
 */

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


static void midiDataTxCb(void);
static void midiDataRxCb(void);

static void usbInit(void);
static void usbReset(void);
static RESULT usbDataSetup(uint8 request);
static RESULT usbNoDataSetup(uint8 request);
static RESULT usbGetInterfaceSetting(uint8 interface, uint8 alt_setting);
static uint8* usbGetDeviceDescriptor(uint16 length);
static uint8* usbGetConfigDescriptor(uint16 length);
static uint8* usbGetStringDescriptor(uint16 length);
static void usbSetConfiguration(void);
static void usbSetDeviceAddress(void);

/*
 * Etc.
 */

/* I/O state */

/* Received data */
static volatile uint32 midiBufferRx[MIDI_STREAM_EPSIZE/4];
/* Read index into midiBufferRx */
static volatile uint32 rx_offset = 0;
/* Transmit data */
static volatile uint32 midiBufferTx[MIDI_STREAM_EPSIZE/4];
/* Write index into midiBufferTx */
static volatile uint32 tx_offset = 0;
/* Number of bytes left to transmit */
static volatile uint32 n_unsent_packets = 0;
/* Are we currently sending an IN packet? */
static volatile uint8 transmitting = 0;
/* Number of unread bytes */
static volatile uint32 n_unread_packets = 0;


// --------------------------------------------------------------------------------------
// ENDPOINTS CALLBACKS TABLE
// --------------------------------------------------------------------------------------
void (*ep_int_in[7])(void) =
    {midiDataTxCb,midiDataTxCb,midiDataTxCb,
     NOP_Process,          
     NOP_Process,
     NOP_Process,
     NOP_Process};

void (*ep_int_out[7])(void) =
    {midiDataRxCb,midiDataRxCb,midiDataRxCb,
     NOP_Process,
     NOP_Process,
     NOP_Process,         
     NOP_Process};

// --------------------------------------------------------------------------------------
// Globals required by usb_lib.
// These override core USB functionality which was declared __weak.
// --------------------------------------------------------------------------------------


DEVICE Device_Table = {
    .Total_Endpoint      = NUM_ENDPTS,
    .Total_Configuration = 1
};


DEVICE_PROP Device_Property = {
    .Init                        = usbInit,
    .Reset                       = usbReset,
    .Process_Status_IN           = NOP_Process,
    .Process_Status_OUT          = NOP_Process,
    .Class_Data_Setup            = usbDataSetup,
    .Class_NoData_Setup          = usbNoDataSetup,
    .Class_Get_Interface_Setting = usbGetInterfaceSetting,
    .GetDeviceDescriptor         = usbGetDeviceDescriptor,
    .GetConfigDescriptor         = usbGetConfigDescriptor,
    .GetStringDescriptor         = usbGetStringDescriptor,
    .RxEP_buffer                 = NULL,
    .MaxPacketSize               = MAX_PACKET_SIZE
};

USER_STANDARD_REQUESTS User_Standard_Requests = {
    .User_GetConfiguration   = NOP_Process,
    .User_SetConfiguration   = usbSetConfiguration,
    .User_GetInterface       = NOP_Process,
    .User_SetInterface       = NOP_Process,
    .User_GetStatus          = NOP_Process,
    .User_ClearFeature       = NOP_Process,
    .User_SetEndPointFeature = NOP_Process,
    .User_SetDeviceFeature   = NOP_Process,
    .User_SetDeviceAddress   = usbSetDeviceAddress
};

// --------------------------------------------------------------------------------------
// ENABLE / DISABLE / POWERDOWN   MIDI DEVICE
// --------------------------------------------------------------------------------------
void usb_midi_enable(gpio_dev *disc_dev, uint8 disc_bit, uint8 level) {
    /* Present ourselves to the host. Writing 0 to "disc" pin must
     * pull USB_DP pin up while leaving USB_DM pulled down by the
     * transceiver. See USB 2.0 spec, section 7.1.7.3.           
     * 
     * FT : The function was modified to support a new "level" parameter,
     * to set the 0 or 1 regarding the logic level used by the DISC pin.
     * 
     */

     if (disc_dev != NULL) {
        gpio_set_mode(disc_dev, disc_bit, GPIO_OUTPUT_PP);
        gpio_write_bit(disc_dev, disc_bit, level);
    }

    /* Initialize the USB peripheral. */
    usb_init_usblib(USBLIB, ep_int_in, ep_int_out);
}

void usb_power_down() {
    USB_BASE->CNTR = USB_CNTR_FRES;
    USB_BASE->ISTR = 0;
    USB_BASE->CNTR = USB_CNTR_FRES + USB_CNTR_PDWN;
}

void usb_midi_disable(gpio_dev *disc_dev, uint8 disc_bit, uint8 level) {
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
    usb_power_down();
}


// --------------------------------------------------------------------------------------
// USB BUFFERS I/O
// --------------------------------------------------------------------------------------
/* TODO these could use some improvement; they're fairly
 * straightforward ports of the analogous ST code.  The PMA blit
 * routines in particular are obvious targets for performance
 * measurement and tuning. */
static void usb_copy_to_pma(const uint8 *buf, uint16 len, uint16 pma_offset) {
    uint16 *dst = (uint16*)usb_pma_ptr(pma_offset);
    uint16 n = len >> 1;
    uint16 i;
    for (i = 0; i < n; i++) {
        *dst = (uint16)(*buf) | *(buf + 1) << 8;
        buf += 2;
        dst += 2;
    }
    if (len & 1) {
        *dst = *buf;
    }
}

static void usb_copy_from_pma(uint8 *buf, uint16 len, uint16 pma_offset) {
    uint32 *src = (uint32*)usb_pma_ptr(pma_offset);
    uint16 *dst = (uint16*)buf;
    uint16 n = len >> 1;
    uint16 i;
    for (i = 0; i < n; i++) {
        *dst++ = *src++;
    }
    if (len & 1) {
        *dst = *src & 0xFF;
    }
}

//void usb_midi_putc(char ch) {
//    while (!usb_midi_tx((uint8*)&ch, 1))
//        ;
//}

// --------------------------------------------------------------------------------------
// USB TX / RX / PEEK
// --------------------------------------------------------------------------------------

/* This function is non-blocking.
 *
 * It copies data from a usercode buffer into the USB peripheral TX
 * buffer, and returns the number of bytes copied. */
uint32 usb_midi_tx(const uint32* buf, uint32 packets) {
    uint32 bytes=packets*4;
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
        usb_copy_to_pma((uint8 *)buf, bytes, MIDI_STREAM_IN_EPADDR);
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
uint32 usb_midi_rx(uint32* buf, uint32 packets) {
    /* Copy bytes to buffer. */
    uint32 n_copied = usb_midi_peek(buf, packets);

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

/* Nonblocking byte lookahead.
 *
 * Looks at unread bytes without marking them as read. */
uint32 usb_midi_peek(uint32* buf, uint32 packets) {
    int i;
    if (packets > n_unread_packets) {
        packets = n_unread_packets;
    }

    for (i = 0; i < packets; i++) {
        buf[i] = midiBufferRx[i + rx_offset];
    }

    return packets;
}

// --------------------------------------------------------------------------------------
// USB MIDI STATE 
// --------------------------------------------------------------------------------------

uint32 usb_midi_data_available(void) {
    return n_unread_packets;
}

uint8 usb_midi_is_transmitting(void) {
    return transmitting;
}

uint16 usb_midi_get_pending(void) {
    return n_unsent_packets;
}

// --------------------------------------------------------------------------------------
// ENDPOINTS CALLBACKS
// --------------------------------------------------------------------------------------

static void midiDataTxCb(void) {
    n_unsent_packets = 0;
    transmitting = 0;
}

static void midiDataRxCb(void) {
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

/* NOTE: Nothing specific to this device class in this function, move to usb_lib */
static void usbInit(void) {
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


static void usbReset(void) {
    pInformation->Current_Configuration = 0;

    /* current feature is current bmAttributes */
    pInformation->Current_Feature = (USB_CONFIG_ATTR_BUSPOWERED |
                                     USB_CONFIG_ATTR_SELF_POWERED);

    USB_BASE->BTABLE = BTABLE_ADDRESS;

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

static RESULT usbDataSetup(uint8 request) {
    uint8* (*CopyRoutine)(uint16) = 0;

    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {

    }

    if (CopyRoutine == NULL) {
        return USB_UNSUPPORT;
    }

    pInformation->Ctrl_Info.CopyData = CopyRoutine;
    pInformation->Ctrl_Info.Usb_wOffset = 0;
    (*CopyRoutine)(0);
    return USB_SUCCESS;
}

static RESULT usbNoDataSetup(uint8 request) {
    RESULT ret = USB_UNSUPPORT;

    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
    }
    return ret;
}

static RESULT usbGetInterfaceSetting(uint8 interface, uint8 alt_setting) {
    if (alt_setting > 0) {
        return USB_UNSUPPORT;
    } else if (interface > 1) {
        return USB_UNSUPPORT;
    }

    return USB_SUCCESS;
}

static uint8* usbGetDeviceDescriptor(uint16 length) {
    return Standard_GetDescriptorData(length, &usbMidiDevice_Descriptor);
}

static uint8* usbGetConfigDescriptor(uint16 length) {
    return Standard_GetDescriptorData(length, &usbMidiConfig_Descriptor);
}

static uint8* usbGetStringDescriptor(uint16 length) {
    uint8 wValue0 = pInformation->USBwValue0;

    if (wValue0 > N_STRING_DESCRIPTORS) {
        return NULL;
    }
    return Standard_GetDescriptorData(length, &String_Descriptor[wValue0]);
}

static void usbSetConfiguration(void) {
    if (pInformation->Current_Configuration != 0) {
        USBLIB->state = USB_CONFIGURED;
    }
}

static void usbSetDeviceAddress(void) {
    USBLIB->state = USB_ADDRESSED;
}
