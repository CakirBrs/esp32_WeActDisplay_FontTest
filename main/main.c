#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "nvs_flash.h"

#include "epaper.h"

#include "FreeSerif9pt7b.h"
#include "FreeMono12pt7b.h"

#include "FreeMono9pt7b2.h"
#include "FreeSansBold12pt7b.h"

////#include "robotobold12pt7b.h"

static const char *TAG = "DFont";

void app_main(void)
{
    epaper_init();
    //epaper_clear();
    //epaper_wait_milliseconds(1000);

    int letter = 'E';
    //draw_Char(letter, 10, 100);
    //draw_Char2(letter, 20, 100);
    //draw_Char3(letter, 40, 100);
    //draw_char_withFont(letter, 70, 100, &FreeSerif9pt7b);
    //draw_word("HELLO my name is Baris", 10, 10);
    draw_word("Baris Cakir", 10, 10);
    //draw_word("34; Deggree Celcius;; ", 10, 40);
    //draw_word(" `_;\".$!'", 10, 60);

    draw_word_withFont("Baris Cakir", 10, 60, &FreeMono9pt7b2);
    ////draw_word_withFont("Baris Cakir", 10, 50, &FreeSerif9pt7b);
    draw_word_withFont("Baris Cakir", 10, 100, &FreeMono12pt7b);
    draw_word_withFont("Baris Cakir", 10, 40, &FreeSansBold12pt7b);
    
    /////draw_word_withFont("Baris Cakir", 10, 20, &Robotobold12pt7b);

    //draw_line_horizontal(10, 100);
    //draw_line_vertical(10, 100);
    epaper_clear();

    epaper_deep_sleep();

}
