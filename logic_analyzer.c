/*
 * Copyright (c) 2019 Maciej Suminski <orson@orson.net.pl>
 *
 * This source code is free software; you can redistribute it
 * and/or modify it in source code form under the terms of the GNU
 * General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include "logic_analyzer.h"
#include "io_capture.h"
#include "lcd.h"
#include "udi_cdc.h"
#include "command_handlers.h"
#include "sump.h"
#include <string.h>

// Samples buffer
#define MAX_LA_BUF_SIZE     128
static uint8_t la_buffer[MAX_LA_BUF_SIZE];
static uint32_t la_acq_size;
static uint8_t la_chan_enabled;
static la_target_t la_target = LA_NONE;

// Acquisition finished handler
static void la_acq_finished(void* param);

void la_init(void) {
    la_acq_size = MAX_LA_BUF_SIZE;
    la_chan_enabled = 0xFF;
    ioc_set_clock(F1MHZ);
    ioc_set_handler(la_acq_finished, la_buffer);
}


void la_trigger(void) {
    /*memset(la_buffer, 0x00, sizeof(la_buffer));*/
    while(ioc_busy());
    ioc_fetch(la_buffer, la_acq_size);
}


void la_set_target(la_target_t target) {
    la_target = target;

    if (target == LA_LCD) {
        // there is no point acquiring more samples than
        // what can be displayed on the LCD
        la_acq_size = LCD_WIDTH;
    }
}


static void la_display_acq(void) {
    // double buffering
    uint8_t lcd_page[LCD_WIDTH], lcd_page2[LCD_WIDTH];
    uint8_t chan_mask;

    // Display the acquisition on the LCD
    for(int chan = 0; chan < LA_CHANNELS; ++chan) {
        uint8_t* buf_ptr = la_buffer;
        chan_mask = (1 << chan);

        if ((la_chan_enabled & chan_mask) == 0)
            continue;   // channel disabled

        #define CURRENT_PAGE ((chan & 0x01) ? lcd_page : lcd_page2)
        uint8_t* page_ptr = CURRENT_PAGE;

        for(int x = 0; x < LCD_WIDTH; ++x) {
            *page_ptr = (*buf_ptr & chan_mask) ? 0x02 : 0x80;
            ++page_ptr;
            ++buf_ptr;
        }

        while(SSD1306_isBusy());
        SSD1306_drawPage(chan, CURRENT_PAGE);
    }
}


static void la_usb_send_acq(void) {
    udi_cdc_write_buf(la_buffer, la_acq_size);
}


static void la_acq_finished(void* param) {
    (void) param;
    //uint8_t* data = (uint8_t*)(param);

    switch (la_target) {
        case LA_LCD: la_display_acq(); break;
        case LA_USB: la_usb_send_acq(); break;
        case LA_NONE: break;    // mute warnings
    }
}


// Finds the closest available clock frequency
// basing a 100 MHz prescaler (default OLS clock frequency)
static clock_freq_t la_get_clock(int prescaler_100M) {
    struct {
        clock_freq_t clock_freq;
        int max_prescaler;
    } freqs[] = {
        { F50MHZ,   1 },
        { F32MHZ,   2 },
        { F25MHZ,   3 },
        { F20MHZ,   4 },
        { F16MHZ,   5 },
        { F12_5MHZ, 7 },
        { F10MHZ,   10 },
        { F8MHZ,    13 },
        { F6MHZ,    17 },
        { F5MHZ,    21 },
        { F4MHZ,    27 },
        { F3MHZ,    39 },
        { F2MHZ,    65 },
        { F1MHZ,    132 },
        { F500KHZ,  265 },
        { F250KHZ,  532 },
    };

    for(unsigned int i = 0; i < sizeof(freqs); ++i) {
        if (freqs[i].max_prescaler < prescaler_100M)
            continue;

        return freqs[i].clock_freq;
    }

    // the slowest available sampling frequency
    return F125KHZ;
}


/* ID command response */
static const uint8_t SUMP_ID_RESP[] = "1ALS";
/* Device metadata */
static const uint8_t SUMP_METADATA_RESP[] =
//  token  value
    "\x01" "KiCon-Badge\x00"   // device name
    "\x20" "\x00\x00\x00\x08"  // number of channels
    "\x21" "\x00\x00\x00\xFF"  // sample memory available [bytes]
    "\x23" "\x02\xfa\xf0\x80"  // maximum sampling rate [Hz] = 50 MHz
    "\x24" "\x00\x00\x00\x00"  // protocol version
    ;


int cmd_sump(const uint8_t* cmd, unsigned int len)
{
    if (len == 1) {
        switch (cmd[0]) {
            case METADATA:
                cmd_response(SUMP_METADATA_RESP, sizeof(SUMP_METADATA_RESP) - 1);
                break;

            case RUN:
                la_trigger();
                break;

            case ID:
                cmd_response(SUMP_ID_RESP, sizeof(SUMP_ID_RESP) - 1);
                break;

            case XON: break;    // TODO
            case XOFF: break;   // TODO
            case RESET: break;
            default: return 0;   // unknown 1-byte command, perhaps incomplete
        }

        return 1;   // default handles unknown command

    } else if (len >= 5) {
        unsigned int arg = (cmd[4] << 3) | (cmd[3] << 2) | (cmd[2] << 1) | cmd[1];

        switch (cmd[0]) {
            case SET_TRG_MASK: break;
            case SET_TRG_VAL: break;
            case SET_TRG_CFG: break;

            case SET_DIV:
                ioc_set_clock(la_get_clock(arg));
                break;

            case SET_FLAGS: break;
        }

        return 1;   // clear the buffer anyway, there are no longer commands
    }

    return 0;
}
