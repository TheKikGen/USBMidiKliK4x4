/******************************************************************************
 * The MIT License
 *
 * Adapted by TheKikGenLab from USB LeafLabs LLC. USB API :
 * Perry Hung, Magnus Lundin,
 * Donald Delmar Davis, Suspect Devices.
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
 *
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with Tymm's Arduino Midi Library.  If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *
 *****************************************************************************/

#include "usb_midi.h"

#include <string.h>
#include <stdint.h>
#include <libmaple/nvic.h>
#include "usb_midi_device.h"
#include <libmaple/usb.h>

#include <wirish.h>

extern USBMidi MidiUSB;

// --------------------------------------------------------------------------------------
// USB MIDI Class
// --------------------------------------------------------------------------------------
// This class was adapted and CLEANED from the USBMidi library
// It can work for any device, but was optimized for the MIDI 4X4 board from Miditech
// based on a STM32F103RC.

// Constructor
USBMidi::USBMidi(void) {

}

// BEGIN - Call that function in SETUP
void USBMidi::begin() {

		// Reset the USB interface on the MIDITECH 4x4 board.
    // The MIDI 4X4 has a DISC command, but the level logic is inverted
    // Then configure USB and Endpoints callbacks
    usb_midi_enable(PIN_MAP[PA8].gpio_device, PIN_MAP[PA8].gpio_bit,1);

}

void USBMidi::end(void) {

    usb_midi_disable(PIN_MAP[PA8].gpio_device, PIN_MAP[PA8].gpio_bit,0);

}

void USBMidi::writePacket(uint32 p) {
    this->writePackets(&p, 1);
}

void USBMidi::writePackets(const void *buf, uint32 len) {
    if (!this->isConnected() || !buf) {
        return;
    }

    uint32 txed = 0;
    uint32 old_txed = 0;
    uint32 start = millis();

    uint32 sent = 0;

    while (txed < len && (millis() - start < USB_TIMEOUT)) {
        sent = usb_midi_tx((const uint32*)buf + txed, len - txed);
        txed += sent;
        if (old_txed != txed) {
            start = millis();
        }
        old_txed = txed;
    }


    if (sent == MIDI_STREAM_EPSIZE) {
        while (usb_midi_is_transmitting() != 0) {
        }
        /* flush out to avoid having the pc wait for more data */
        usb_midi_tx(NULL, 0);
    }
}

uint32 USBMidi::available(void) {
    return usb_midi_data_available();
}

uint32 USBMidi::readPackets(void *buf, uint32 len) {
    if (!buf) {
        return 0;
    }

    uint32 rxed = 0;
    while (rxed < len) {
        rxed += usb_midi_rx((uint32*)buf + rxed, len - rxed);
    }

    return rxed;
}

/* Blocks forever until 1 byte is received */
uint32 USBMidi::readPacket(void) {
    uint32 p;
    this->readPackets(&p, 1);
    return p;
}

uint8 USBMidi::pending(void) {
    return usb_midi_get_pending();
}

uint8 USBMidi::isConnected(void) {
    return usb_is_connected(USBLIB) && usb_is_configured(USBLIB);
}
