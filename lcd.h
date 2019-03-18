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

#ifndef LCD_H
#define LCD_H

#include <stdint.h>

#define LCD_WIDTH           128
#define LCD_HEIGHT          64
#define LCD_PAGE_SIZE       8
#define LCD_PAGES           LCD_HEIGHT/LCD_PAGE_SIZE

#define SSD1306_address     0x3C //0x3D
#define SSD1306_DC          6        //0=data 1=command
#define SSD1306_CO          7         //continuation bit
#define SSD1306_data        0x40
#define SSD1306_cmd         0x00

#define SSD1306_Offset      0x00

#define BLACK               0 ///< Draw 'off' pixels
#define WHITE               1 ///< Draw 'on' pixels
#define INVERSE             2 ///< Invert pixels

typedef enum
{
    DATA = SSD1306_data,
    CMD = SSD1306_cmd
} control_byte;

void SSD1306_init(void);

void SSD1306_setOrientation(uint8_t orientation);
void SSD1306_swap(uint8_t *a, uint8_t *b);
void SSD1306_orientCoordinates(uint8_t *x1, uint8_t *y1);
void SSD1306_clear(void);

void SSD1306_setPixel(uint8_t x, uint8_t y, uint8_t color);
void SSD1306_setLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
void SSD1306_setBuffer(uint8_t x, uint8_t pageIndex, uint8_t *buffer, int size);
void SSD1306_setString(uint8_t x, uint8_t pageIndex, const char *string,
        int size, uint8_t color);
void SSD1306_drawBitmap(uint8_t x0, uint8_t y0, const uint8_t *bitmap,
        uint8_t width, uint8_t height);

void SSD1306_drawPage(uint8_t pageIndex, const uint8_t *pageBuffer);
void SSD1306_drawBuffer(void);
void SSD1306_drawPageDMA(uint8_t pageIndex, const uint8_t *pageBuffer);
void SSD1306_drawBufferDMA(void);

int SSD1306_isBusy(void);

#endif /* LCD_H */
