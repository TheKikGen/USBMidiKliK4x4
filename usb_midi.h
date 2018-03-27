/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
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
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
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



#include "usb_midi_device.h"

/**
 * @brief Wirish USB MIDI port (MidiUSB).
 */

#ifndef _WIRISH_USB_MIDI_H_
#define _WIRISH_USB_MIDI_H_

#define USB_MIDI
#define USB_HARDWARE

#include <Print.h>
#include <boards.h>


class USBMidi {
private:

public:
    // Constructor
    USBMidi();

    void begin();
    void end();
    uint32 available(void);
    uint32 readPackets(void *buf, uint32 len);
    uint32 readPacket(void);
    void writePacket(uint32);
    void writePackets(const void*, uint32);
    uint8 isConnected();
    uint8 pending();
 
 };

#endif
