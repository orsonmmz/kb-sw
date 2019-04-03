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
#include "buttons.h"
#include "udi_cdc.h"
#include "commands.h"
#include "command_handlers.h"
#include "sump.h"
#include "apps_list.h"
#include "settings_list.h"
#include <string.h>
#include <limits.h>

// Samples buffer
#define LA_BUFFER_SIZE     65536
static uint8_t la_buffer[LA_BUFFER_SIZE];
static const uint8_t *la_buffer_end = &la_buffer[LA_BUFFER_SIZE - 1];

// Pointer to the acquired buffer
static uint8_t *la_acq_start;
// Number of the acquired samples
static uint32_t la_acq_size;

static uint8_t la_chan_enabled;
static la_target_t la_target = LA_NONE;

// Acquisition settings
static uint8_t la_trigger_mask = 0;
static uint8_t la_trigger_val = 0;
static uint32_t la_read_cnt = 0;
static uint32_t la_delay_cnt = 0;
static uint32_t la_sump_active = 0;

// Acquisition finished handler
static void la_acq_finished(void* param);

static volatile enum { IDLE, RUNNING, ACQUIRED } la_state = IDLE;

// Fixes the hardware channel order
// (see the connection between the logic probes pin header and the input buffer)
#define LA_FIX_ORDER(val) ((val & 0x0f) \
            | (val & 0x80) >> 3 \
            | (val & 0x40) >> 1 \
            | (val & 0x20) << 1 \
            | (val & 0x10) << 3)

static void la_fix_channels(uint32_t offset, uint32_t size) {
    uint8_t* buf_ptr = &la_buffer[offset];

    for(unsigned int i = 0; i < size; ++i) {
        *buf_ptr = LA_FIX_ORDER(*buf_ptr);

        // Buffer wrapping
        if (++buf_ptr > la_buffer_end) {
            buf_ptr = la_buffer;
        }
    }
}


// Searches the acquisition buffer for a sample matching the configured
// trigger. Will return the sample index or UINT_MAX if nothing found.
// This function works with samples which do not have the order fixed
// (see LA_FIX_ORDER macro)
static uint32_t la_find_trigger_unfixed(const uint8_t *buf, uint32_t size) {
    if (la_trigger_mask == 0) {
        return 0;
    }

    const uint8_t mask = LA_FIX_ORDER(la_trigger_mask);
    const uint8_t val = LA_FIX_ORDER(la_trigger_val);
    const uint8_t *buf_ptr = buf;

    for (uint32_t i = 0; i < size; ++i) {
        if ((*buf_ptr & mask) == val) {
            return i;
        }

        ++buf_ptr;
    }

    return UINT_MAX;    // trigger not matched
}


void la_init(void) {
    la_acq_size = LA_BUFFER_SIZE;
    la_acq_start = la_buffer;
    la_chan_enabled = 0xFF;
    ioc_set_clock(F1MHZ);
    ioc_set_handler(la_acq_finished, la_acq_start);
}


void la_trigger(void) {
    while(ioc_busy());
    la_state = RUNNING;
    ioc_fetch(la_acq_start, la_acq_size);
}


void la_set_target(la_target_t target) {
    la_target = target;

    if (target == LA_LCD) {
        // there is no point acquiring more samples than
        // what can be displayed on the LCD
        la_acq_size = LCD_WIDTH;
    }
}


void la_set_trigger(uint8_t trigger_mask, uint8_t trigger_val) {
    la_trigger_mask = trigger_mask;
    la_trigger_val = trigger_val;
}


static void la_display_acq(uint32_t offset) {
    // double buffering
    uint8_t lcd_page[LCD_WIDTH], lcd_page2[LCD_WIDTH];
    uint8_t chan_mask;

    // Display the acquisition on the LCD
    for(int chan = 0; chan < LA_CHANNELS; ++chan) {
        uint8_t* buf_ptr = &la_buffer[offset];
        chan_mask = (1 << chan);

        if ((la_chan_enabled & chan_mask) == 0)
            continue;   // channel disabled

        #define CURRENT_PAGE ((chan & 0x01) ? lcd_page : lcd_page2)
        uint8_t* page_ptr = CURRENT_PAGE;

        for(int x = 0; x < LCD_WIDTH; ++x) {
            *page_ptr = (*buf_ptr & chan_mask) ? 0x02 : 0x80;
            ++page_ptr;

            // Buffer wrapping
            if (++buf_ptr > la_buffer_end) {
                buf_ptr = la_buffer;
            }
        }

        while(SSD1306_isBusy());
        SSD1306_drawPage(chan, CURRENT_PAGE);
    }
}


static void la_acq_finished(void* param) {
    (void) param;
    la_state = ACQUIRED;
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


static uint32_t power2_ceil(uint32_t x) {
    if (x <= 1)
        return 1;

    int power = 2;
    --x;

    while (x >>= 1)
        power <<= 1;

    return power;
}


/* ID command response */
static const uint8_t SUMP_ID_RESP[] = "1ALS";
/* Device metadata */
static const uint8_t SUMP_METADATA_RESP[] =
//  token  value
    "\x01" "KiCon-Badge\x00"   // device name
    "\x20" "\x00\x00\x00\x08"  // number of channels
    "\x21" "\x00\x01\x00\x00"  // sample memory available [bytes] = 65536
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
                la_acq_start = la_buffer;
                la_acq_size = min(power2_ceil(la_read_cnt), LA_BUFFER_SIZE / 2);
                la_sump_active = 1;
                la_trigger();
                break;

            case ID:
                cmd_response(SUMP_ID_RESP, sizeof(SUMP_ID_RESP) - 1);
                break;

            case XON: break;
            case XOFF: break;
            case RESET: break;
            default: return 0;   // unknown 1-byte command, perhaps incomplete
        }

        return 1;   // default handles unknown commands

    } else if (len >= 5) {
        // change the endianess
        unsigned int arg = (cmd[4] << 24) | (cmd[3] << 16) | (cmd[2] << 8) | cmd[1];

        switch (cmd[0]) {
            case SET_TRG_MASK:
                la_trigger_mask = (uint8_t)(arg & 0xff);
                break;

            case SET_TRG_VAL:
                la_trigger_val = (uint8_t)(arg & 0xff);
                break;

            case SET_TRG_CFG:
                break;

            case SET_DIV:
                ioc_set_clock(la_get_clock(arg));
                break;

            case SET_READ_DLY_CNT:
                la_read_cnt = (uint16_t)((arg & 0xffff) + 1) * 4;
                la_delay_cnt = (uint16_t)((arg >> 16) + 1) * 4;
                la_acq_size = la_read_cnt;
                break;

            case SET_DELAY_COUNT:
                la_delay_cnt = arg;
                break;

            case SET_READ_COUNT:
                la_read_cnt = arg;
                la_acq_size = la_read_cnt;
                break;

            // none of the flags has any meaning in this implementation
            //case SET_FLAGS: break;
        }

        // clear the buffer anyway, commands cannot be longer than 5 bytes
        return 1;
    }

    return 0;
}


static void la_usb_send(uint32_t offset, uint32_t size) {
    if (offset + size <= LA_BUFFER_SIZE) {
        udi_cdc_write_buf(&la_buffer[offset], size);
    } else {
        unsigned int firstChunk = LA_BUFFER_SIZE - offset;
        unsigned int secondChunk = size - firstChunk;
        udi_cdc_write_buf(&la_buffer[offset], firstChunk);
        udi_cdc_write_buf(la_buffer, secondChunk);
    }
}


void app_la_usb_func(void) {
    const uint8_t *resp;
    unsigned int resp_len;
    int processed;

    la_trigger_mask = 0;
    la_trigger_val = 0;

    la_set_target(LA_USB);
    cmd_set_mode(CMD_SUMP);

    while(SSD1306_isBusy());
    SSD1306_clearBufferFull();
    SSD1306_setString(5, 3, "Logic Analyzer (USB)", 20, WHITE);
    SSD1306_drawBufferDMA();

    uint32_t trig_offset = UINT_MAX;
    uint32_t trig_samples = 0;

    while(btn_state() != BUT_LEFT) {
        /* process commands */
        if (udi_cdc_is_rx_ready()) {
            cmd_new_data(udi_cdc_getc());
            processed = cmd_try_execute();

            if (processed) {
                cmd_get_resp(&resp, &resp_len);

                if(resp && resp_len > 0) {
                    udi_cdc_write_buf(resp, resp_len);
                    cmd_resp_processed();
                }
            }
        }


        if (la_state == ACQUIRED) {
            /* acquisition has finished */
            la_state = IDLE;

            if (!la_sump_active)
                return;

            if (trig_offset == UINT_MAX) {
                /* store the current acquisition pointer */
                uint8_t *last_acq = la_acq_start;

                /* trigger has not been found yet,so start the next acquisition */
                la_acq_start += la_acq_size;

                if (&la_acq_start[la_acq_size] > la_buffer_end)
                    la_acq_start = la_buffer;

                la_trigger();


                trig_offset = la_find_trigger_unfixed(last_acq, la_acq_size);

                if (trig_offset != UINT_MAX) {
                    // TODO decrement the offset by 8, so the triggering edge is visible
                    /* trigger has just been found */
                    trig_offset += (last_acq - la_buffer);
                    trig_samples += (la_acq_size - trig_offset);
                }
            } else if (trig_samples < la_read_cnt) {
                /* trigger has been found, but still gathering samples */
                trig_samples += la_acq_size;
            }


            if (trig_offset != UINT_MAX && trig_samples >= la_read_cnt) {
                /* got all the samples, draw the results */
                la_fix_channels(trig_offset, la_read_cnt);
                la_usb_send(trig_offset, la_read_cnt);
                trig_offset = UINT_MAX;
                trig_samples = 0;
            }
        }
    }

    while(btn_state());    /* wait for the button release */
    la_state = IDLE;
}


void app_la_lcd_func(void) {
    la_set_target(LA_LCD);

    // configure the logic analyzer
    switch (menu_la_lcd_sampling_freq.val) {
        case 0: ioc_set_clock(F50MHZ); break;
        case 1: ioc_set_clock(F20MHZ); break;
        case 2: ioc_set_clock(F10MHZ); break;
        case 3: ioc_set_clock(F5MHZ); break;
        case 4: ioc_set_clock(F2MHZ); break;
        case 5: ioc_set_clock(F1MHZ); break;
        case 6: ioc_set_clock(F500KHZ); break;
    }

    if (menu_la_lcd_trigger_input.val == 0) {
        /* no trigger input -> free-running mode */
        la_trigger_mask = 0;
    } else {
        la_trigger_mask = (1 << (menu_la_lcd_trigger_input.val - 1));
    }

    la_trigger_val = menu_la_lcd_trigger_level.val ? la_trigger_mask : 0;
    la_acq_start = la_buffer;
    la_acq_size = 2 * LCD_WIDTH;    /* always the same acquisition size */
    la_read_cnt = LCD_WIDTH;        /* number of requested samples */

    uint32_t trig_offset = UINT_MAX;
    uint32_t trig_samples = 0;

    /* start acquisition */
    la_trigger();

    while(btn_state() != BUT_LEFT) {
        if (la_state != ACQUIRED)
            continue;

        /* acquisition has finished */
        la_state = IDLE;

        /* store the current acquisition pointer */
        uint8_t *last_acq = la_acq_start;

        /* start the next acquisition and process the acquired data in the meantime */
        la_acq_start += la_acq_size;

        if (&la_acq_start[la_acq_size] > la_buffer_end)
            la_acq_start = la_buffer;

        la_trigger();


        if (trig_offset == UINT_MAX) {
            /* trigger has not been found yet */
            trig_offset = la_find_trigger_unfixed(last_acq, la_acq_size);

            if (trig_offset != UINT_MAX) {
                // TODO decrement the offset by 8, so the triggering edge is visible
                /* trigger has just been found */
                trig_offset += (last_acq - la_buffer);
                trig_samples += (la_acq_size - trig_offset);
            }
        } else if (trig_samples < la_read_cnt) {
            /* trigger has been found, but still gathering samples */
            trig_samples += la_acq_size;
        }


        if (trig_offset != UINT_MAX && trig_samples >= la_read_cnt) {
            /* got all the samples, draw the results */
            la_fix_channels(trig_offset, la_read_cnt);
            la_display_acq(trig_offset);
            trig_offset = UINT_MAX;
            trig_samples = 0;
            la_sump_active = 0;
        }
    }

    while(btn_state());    /* wait for the button release */
    la_state = IDLE;
}
