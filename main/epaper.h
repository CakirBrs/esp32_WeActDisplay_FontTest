#ifndef EPAPER_H
#define EPAPER_H

#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "fontStruct.h"

// Pin Definitions
#define PIN_MOSI    23
#define PIN_MISO    -1
#define PIN_CLK     18
#define PIN_CS      5
#define PIN_DC      0
#define PIN_RST     2
#define PIN_BUSY    15

// Screen Definitions
#define DISPLAY_WIDTH   250
#define DISPLAY_HEIGHT  122

void epaper_init(void);
void epaper_writeBufferToDisplay(void);
void epaper_update(void);
void epaper_clear(void);
void epaper_deep_sleep(void);
void epaper_draw_blackBitmap(const unsigned char IMAGE[]);
void epaper_draw_redBitmap(const unsigned char IMAGE[]);
void epaper_draw_blackAndRedBitmaps(const unsigned char IMAGE_BLACK[], const unsigned char IMAGE_RED[]);
//void epaper_draw_partial_blackAndRedBitmaps(const unsigned char* black_bitmap, const unsigned char* red_bitmap, int x, int y, int width, int height);
void epaper_draw_partial_blackAndRedBitmaps(const unsigned char* black_bitmap, const unsigned char* red_bitmap, int x, int y, int width, int height, int bitmap_width, int bitmap_height);
void epaper_draw_partial_blackAndRedBitmapsEnhanced(const unsigned char* black_bitmap, const unsigned char* red_bitmap, int x, int y, int width, int height, int bitmap_width, int bitmap_height);
void epaper_draw_partial_blackAndRedBitmaps2(const unsigned char* black_bitmap, const unsigned char* red_bitmap, int x, int y, int width, int height);
void epaper_wait_seconds(uint32_t seconds);
void epaper_wait_milliseconds(uint32_t milliseconds);
void epaper_write_to_bw_buffer(const unsigned char IMAGE_BLACK[],int x_imageSize, int y_imageSize, int x_coordinate, int y_coordinate);
void epaper_red_buffer_clear(void);
void epaper_bw_buffer_clear(void);
void epaperCalced(const unsigned char* black_bitmap);
void place_image_into_buffer(int x, int y,const unsigned char* black_bitmap);


void draw_Char(int letter, int x, int y);
void draw_Char2(int letter, int x, int y);
void draw_Char3(int letter, int x, int y);

void draw_char_withFont(uint8_t letter, int x, int y, const GFXfont *font);
void draw_word_withFont(const char* word, int x, int y, const GFXfont *font);

void draw_word(const char* word, int x, int y);


void draw_line_horizontal(int x1, int x2);
void draw_line_vertical(int y1, int y2);

#endif