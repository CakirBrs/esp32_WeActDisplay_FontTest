#include "epaper.h"
#include <inttypes.h>
#include "FreeMono9pt7b.h"
#include "FreeMonoBold12pt7b.h"
#include "Robotobold12pt7b.h"

static const char *TAG = "SSD1680";
static spi_device_handle_t spi;


int bw_bufsize;         // size of the black and white buffer
int red_bufsize;        // size of the red buffer

uint8_t *bw_buf;        // the pointer to the black and white buffer if using on-chip ram
uint8_t *red_buf;       // the pointer to the red buffer if using on-chip ram

uint8_t *char_buf;  

static void spi_cmd(const uint8_t cmd) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    t.user = (void*)0;
    gpio_set_level(PIN_DC, 0);
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static void spi_data(const uint8_t data) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &data;
    t.user = (void*)1;
    gpio_set_level(PIN_DC, 1);
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static void wait_busy(void) {
    while(gpio_get_level(PIN_BUSY) == 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void epaper_init(void){

    // GPIO yapılandırması
    gpio_set_direction(PIN_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_BUSY, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BUSY, GPIO_PULLUP_ONLY);
    
    // SPI yapılandırması
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = (DISPLAY_WIDTH * DISPLAY_HEIGHT / 8) + 8
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4*1000*1000,
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 7,
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 0));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi));

    bw_buf = (uint8_t *)malloc(DISPLAY_WIDTH * (DISPLAY_HEIGHT / 8 + 1) * 8 / 8);
    red_buf = (uint8_t *)malloc(DISPLAY_WIDTH * (DISPLAY_HEIGHT / 8 + 1) * 8 / 8);
    bw_bufsize = DISPLAY_WIDTH * (DISPLAY_HEIGHT / 8 + 1) * 8 / 8;
    red_bufsize = bw_bufsize;
    ESP_LOGI(TAG, "Buffer sizes - BW: %d, Red: %d", bw_bufsize, red_bufsize);  // Added logging
    memset(bw_buf, 0xFF, bw_bufsize);
    //memset(bw_buf, 0x00, bw_bufsize);
    memset(red_buf, 0x00, red_bufsize);


    char_buf = (uint8_t *)malloc(15 * (15 / 8 + 1) * 8 / 8);
    memset(char_buf, 0x00, 15 * (15 / 8 + 1) * 8 / 8);


    // Reset ekran
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    wait_busy();

    spi_cmd(0x12);  // Software reset
    vTaskDelay(pdMS_TO_TICKS(100));
    wait_busy();


    spi_cmd(0x01); //Driver output control
    spi_data(0xF9);
    spi_data(0x00);
    spi_data(0x00);
    
    
    spi_cmd(0x11); //data entry mode
    spi_data(0x01);

    spi_cmd(0x44); //set Ram-X address start/end position
    spi_data(0x00);
    spi_data(0x0F); //0x0F-->(15+1)*8=128

    spi_cmd(0x45);  //set Ram-Y address start/end position
    spi_data(0xF9); //0xF9-->(249+1)=250
    spi_data(0x00);
    spi_data(0x00);
    spi_data(0x00);
    


    spi_cmd(0x3C); //BorderWavefrom
    spi_data(0x05);

    spi_cmd(0x18); //Read built-in temperature sensor
    spi_data(0x80);

    spi_cmd(0x21); //  Display update control
    spi_data(0x00);
    spi_data(0x80);

    spi_cmd(0x4E); // set RAM x address count to 0;
    spi_data(0x00);
    spi_cmd(0x4F); // set RAM y address count to 0X199;
    spi_data(0xF9);
    spi_data(0x00);
    wait_busy();
    
}

void epaper_update(void){
    spi_cmd(0x22);
    spi_data(0xF7);
    spi_cmd(0x20);
    vTaskDelay(pdMS_TO_TICKS(100));
    wait_busy();
    ESP_LOGI(TAG, "Epaper Display Updated");
}
void epaper_clear(void){
    
    spi_cmd(0x4E); // set RAM x address count to 0;
    spi_data(0x00);
    spi_cmd(0x4F); // set RAM y address count to 0X199;
    spi_data(0xF9);
    spi_data(0x00);
    wait_busy();


    spi_cmd(0x24); //write RAM for black(0)/white (1)

    for (uint16_t i = 0; i < bw_bufsize; i++)
    {
        spi_data(bw_buf[i]);
    }

    spi_cmd(0x26); //write RAM for red(1)/white (0)

    for (uint16_t i = 0; i < red_bufsize; i++)
    {
        spi_data(red_buf[i]);
    }

    epaper_update();
    
}
void epaper_deep_sleep(void){
    spi_cmd(0x10);
    spi_data(0x01); // Deep sleep mode 1
}
void epaper_draw_blackBitmap(const unsigned char IMAGE[]){
    spi_cmd(0x4E); // set RAM x address count to 0;
    spi_data(0x00);
    spi_cmd(0x4F); // set RAM y address count to 0X199;
    spi_data(0xF9);
    spi_data(0x00);

    spi_cmd(0x24); //write RAM for black(0)/white (1)

    for (uint16_t i = 0; i < bw_bufsize; i++)
    {
        spi_data(~IMAGE[i]);
    }
}
void epaper_draw_redBitmap(const unsigned char IMAGE[]){
    spi_cmd(0x4E); // set RAM x address count to 0;
    spi_data(0x00);
    spi_cmd(0x4F); // set RAM y address count to 0X199;
    spi_data(0xF9);
    spi_data(0x00);

    spi_cmd(0x26); //write RAM for red(1)/white (0)

    for (uint16_t i = 0; i < red_bufsize; i++)
    {
        spi_data(IMAGE[i]);
    }
}
void epaper_draw_blackAndRedBitmaps(const unsigned char IMAGE_BLACK[], const unsigned char IMAGE_RED[]){
    
    spi_cmd(0x4E); // set RAM x address count to 0;
    spi_data(0x00);
    spi_cmd(0x4F); // set RAM y address count to 0X199;
    spi_data(0xF9);
    spi_data(0x00);
    wait_busy();


    spi_cmd(0x24); //write RAM for black(0)/white (1)

    for (uint16_t i = 0; i < bw_bufsize; i++)
    {
        spi_data(~IMAGE_BLACK[i]);
    }

    spi_cmd(0x26); //write RAM for red(1)/white (0)

    for (uint16_t i = 0; i < red_bufsize; i++)
    {
        spi_data(IMAGE_RED[i]);
    }
}

void epaper_draw_partial_blackAndRedBitmaps3(const unsigned char* black_bitmap, const unsigned char* red_bitmap, int x, int y, int width, int height) {
    for(int h = 0; h < height; h++) {
        for(int w = 0; w < width; w++) {
            int pixel_index = (y + h) * DISPLAY_WIDTH + (x + w);
            /*
            burda bitmap deki değeri byte olarak okumam lazım, her bitin byte da kaşılık geldiği yeri okuyup sonra yazarken birer birer yazabilirim. Yada bitsel olarak kaydırıp kalanı sonraki bite yazdırmalı bi şeyler yapabilirim. Ama ilki daha mantıklı gibi her srıadaki biti oku sonra onu bufferdaki ilgli bite yaz sonra ikisinde de bir bit ilerle eğer byte bitmişse o zaman sonraki bytein ilk bitine gel
            */
            bw_buf[pixel_index] = ~black_bitmap[pixel_index];
            red_buf[pixel_index] = red_bitmap[pixel_index];
        }
    }
}

void epaper_draw_partial_blackAndRedBitmaps2(const unsigned char* black_bitmap, const unsigned char* red_bitmap, int x, int y, int width, int height) {
    // Write RAM for black/white
    spi_cmd(0x24);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            int pixel_index = (y + i) * DISPLAY_WIDTH + (x + j);
            spi_data(~black_bitmap[pixel_index]);
        }
    }

    // Write RAM for red/white
    spi_cmd(0x26);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            int pixel_index = (y + i) * DISPLAY_WIDTH + (x + j);
            spi_data(red_bitmap[pixel_index]);
        }
    }
}

void epaper_wait_seconds(uint32_t seconds) {
    vTaskDelay(pdMS_TO_TICKS(seconds * 1000));
    ESP_LOGI(TAG, "Waited %d seconds", (int)seconds);
}

void epaper_wait_milliseconds(uint32_t milliseconds) {
    vTaskDelay(pdMS_TO_TICKS(milliseconds));
    ESP_LOGI(TAG, "Waited %d miliseconds", (int)milliseconds);
}

void epaper_bw_buffer_clear(void){
    memset(bw_buf, 0xFF, bw_bufsize);
}
void epaper_red_buffer_clear(void){
    
    memset(red_buf, 0x00, red_bufsize);
}
void epaper_write_to_bw_buffer(const unsigned char IMAGE_BLACK[],int x_imageSize, int y_imageSize, int x_coordinate, int y_coordinate){
    for(int i=0;i<x_imageSize;i++){
        bw_buf[255-i]=IMAGE_BLACK[255-i]; //çalışmıyo
    }
}

void epaper_draw_partial_blackAndRedBitmaps5(const unsigned char* black_bitmap, const unsigned char* red_bitmap, int x, int y, int width, int height, int bitmap_width, int bitmap_height) {
    for(int h = 0; h < height; h++) {
        for(int w = 0; w < width; w++) {
            int display_pixel_index = (y + h) * DISPLAY_WIDTH + (x + w);
            int bitmap_pixel_index = h * bitmap_width + w;

            if (h < bitmap_height && w < bitmap_width) {
                // Update the buffer with the bitmap data
                bw_buf[display_pixel_index / 8] &= ~(1 << (7 - (display_pixel_index % 8))); // Clear the bit
                bw_buf[display_pixel_index / 8] |= ((black_bitmap[bitmap_pixel_index / 8] >> (7 - (bitmap_pixel_index % 8))) & 1) << (7 - (display_pixel_index % 8)); // Set the bit

                red_buf[display_pixel_index / 8] &= ~(1 << (7 - (display_pixel_index % 8))); // Clear the bit
                red_buf[display_pixel_index / 8] |= ((red_bitmap[bitmap_pixel_index / 8] >> (7 - (bitmap_pixel_index % 8))) & 1) << (7 - (display_pixel_index % 8)); // Set the bit
            }
        }
    }
}

void epaper_draw_partial_blackAndRedBitmapsww(const unsigned char* black_bitmap, const unsigned char* red_bitmap, int x, int y, int width, int height, int bitmap_width, int bitmap_height) {
    for(int genislik=0 ; genislik < 60 ; genislik++) {
        for(int yukseklik=0 ; yukseklik < (60/8+1) ; yukseklik++){

            for(int byteici=0 ; byteici < 8 ; byteici++){

                //int display_pixel_index = yukseklik + genislik * 16 + byteici;
                //int bitmap_pixel_index = (yukseklik + genislik * 8) + byteici;
                //bw_buf[display_pixel_index] = ~black_bitmap[bitmap_pixel_index];

                //bw_buf[yukseklik+genislik*16] &= ~(1 << (7 - (byteici % 8))); // Clear the bit
                //bw_buf[yukseklik+genislik*16] |= ((black_bitmap[yukseklik + genislik * 8] >> (7 - (byteici % 8))) & 1) << (7 - (byteici % 8)); // Set the bit

                bw_buf[yukseklik+(y/8)+((genislik+250-bitmap_width-x)*16)] &= ~(1 << (7 - (byteici % 8))); // Clear the bit
                bw_buf[yukseklik+(y/8)+((genislik+250-bitmap_width-x)*16)] |= ((~black_bitmap[yukseklik + genislik * 8] >> (7 - (byteici % 8))) & 1) << (7 - (byteici % 8)); // Set the bit
                

                red_buf[yukseklik+(y/8)+((genislik+250-bitmap_width-x)*16)] &= ~(1 << (7 - (byteici % 8))); // Clear the bit
                red_buf[yukseklik+(y/8)+((genislik+250-bitmap_width-x)*16)] |= ((red_bitmap[yukseklik + genislik * 8] >> (7 - (byteici % 8))) & 1) << (7 - (byteici % 8)); // Set the bit

            }
            //int display_pixel_index = yukseklik + genislik * 16;
            //int bitmap_pixel_index = (yukseklik + genislik * 8) ;
            

            // Update the buffer with the bitmap data
            //bw_buf[yukseklik+genislik*(DISPLAY_HEIGHT/8+1)] &= ~(1 << (7 - (yukseklik % 8))); // Clear the bit
            //bw_buf[yukseklik+genislik*16] |= ((black_bitmap[yukseklik + genislik * 8] >> (7 - (yukseklik % 8))) & 1) << (7 - (yukseklik % 8)); // Set the bit

            //red_buf[display_pixel_index / 8] &= ~(1 << (7 - (display_pixel_index % 8))); // Clear the bit
            //red_buf[display_pixel_index / 8] |= ((red_bitmap[bitmap_pixel_index / 8] >> (7 - (bitmap_pixel_index % 8))) & 1) << (7 - (display_pixel_index % 8)); // Set the bit

        }
    }
}

void epaper_draw_partial_blackAndRedBitmaps(const unsigned char* black_bitmap, const unsigned char* red_bitmap, int x, int y, int width, int height, int bitmap_width, int bitmap_height) {
// Image'in buffer sınırlarını aşmamasını kontrol et
    if (x < 0 || y < 0 || x + width > DISPLAY_WIDTH || y + height > DISPLAY_HEIGHT) {
        return; // Geçersiz konum, işlemi iptal et
    }

    // Image'i buffer'a yerleştir
    for (int h = 0; h < height; h++) { // Image'in her satırı
        for (int w = 0; w < width; w++) { // Image'in her sütunu
            // Image'deki piksel değeri (0 veya 1)
            uint8_t pixel = (black_bitmap[h * (bitmap_width / 8) + w / 8] >> (7 - (w % 8))) & 0x01;

            // Buffer'daki ilgili byte ve bit pozisyonu
            int buffer_col = x + w; // Buffer'da sütun
            int buffer_row = (y + h) / 8; // Buffer'da byte satırı
            int buffer_bit = (y + h) % 8; // Buffer'da bit pozisyonu

            // Buffer'daki ilgili byte'ı al
            uint8_t *buffer_byte = &bw_buf[buffer_col * DISPLAY_WIDTH + buffer_row];

            // Pikseli buffer'a yerleştir
            if (pixel) {
                *buffer_byte |= (1 << buffer_bit); // Bit'i 1 yap
            } else {
                *buffer_byte &= ~(1 << buffer_bit); // Bit'i 0 yap
            }
        }
    }
}

void epaper_draw_partial_blackAndRedBitmapsEnhanced(const unsigned char* black_bitmap, const unsigned char* red_bitmap, int x, int y, int width, int height, int bitmap_width, int bitmap_height) {
// Image'in buffer sınırlarını aşmamasını kontrol et
    if (x < 0 || y < 0 || x + width > DISPLAY_WIDTH || y + height > DISPLAY_HEIGHT) {
        return; // Geçersiz konum, işlemi iptal et
    }
    //int i = 0;
    // Image'i buffer'a yerleştir
    for (int w = 0; w < width; w++) { // Image'in her satırı
        for (int h = 0; h < height; h++) { // Image'in her sütunu
            // Image'deki piksel değeri (0 veya 1)
            //uint8_t pixel = (black_bitmap[(h + (bitmap_width / 8+1) * (w / 8))/8] >> (7 - (h % 8))) & 0x01;
            uint8_t pixel = (black_bitmap[w * 8 + h / 8] >> (7 - (h % 8))) & 0x01;
            uint8_t pixelR = (red_bitmap[w * 8 + h / 8] >> (7 - (h % 8))) & 0x01;
            

            // Buffer'daki ilgili byte ve bit pozisyonu
            int buffer_col = x + w; // Buffer'da sütun
            int buffer_row = (y + h) / 8; // Buffer'da byte satırı
            int buffer_bit = (y + h) % 8; // Buffer'da bit pozisyonu

            // Buffer'daki ilgili byte'ı al
            uint8_t *buffer_byte = &bw_buf[(buffer_col * (DISPLAY_HEIGHT / 8+1)) + buffer_row];
            uint8_t *buffer_byteR = &red_buf[(buffer_col * (DISPLAY_HEIGHT / 8+1)) + buffer_row];

            //if(i<300){
            //    ESP_LOGI(TAG, "buffer_col: %d buffer_row: %d buffer_bit: %d buffer_byte: %04x pixel: %d pixel: %d h: %d w: %d px:%d", buffer_col, buffer_row, buffer_bit, *buffer_byte, pixel, pixel, h, w, (w * 8 + h / 8));
            //    i++;
            //}

            // Pikseli buffer'a yerleştir
            if (!pixel) {
                *buffer_byte |= (1 << (7-buffer_bit)); // Bit'i 1 yap
            } else {
                *buffer_byte &= ~(1 << (7-buffer_bit)); // Bit'i 0 yap
            }

            if (pixelR) {
                *buffer_byteR |= (1 << (7-buffer_bit)); // Bit'i 1 yap
            } else {
                *buffer_byteR &= ~(1 << (7-buffer_bit)); // Bit'i 0 yap
            }
        }
    }
}

void epaperCalced(const unsigned char* black_bitmap){
    int i = 0;
    for(int genislik=0 ; genislik < 60 ; genislik++){ // max 250
        for(int yukseklik=0 ; yukseklik < (60/8+1) ; yukseklik++){ //max 122 ama 15.2 byte

            int pixel_index = yukseklik + genislik * (DISPLAY_HEIGHT/8+1);
            //bw_buf[pixel_index % 8] = black_bitmap[pixel_index % 8];
            bw_buf[yukseklik+genislik*16] = ~black_bitmap[yukseklik + genislik * 8];
            ESP_LOGI(TAG, "pixel_index: %d yükseklik: %d genislik: %d px:%d" , pixel_index, yukseklik, genislik, (yukseklik + genislik * (60/8+1)));
            i++;
        }
        
    }
    ESP_LOGI(TAG, "ii: %d ", i);
}

void epaperCalced2(const unsigned char* black_bitmap){
    int i = 0;
    for(int genislik=0 ; genislik < 60 ; genislik++){ // max 250
        for(int yukseklik=0 ; yukseklik < 60 ; yukseklik++){ //max 122 ama 15.2 byte -> (60/8+1)

            int display_pixel_index = yukseklik + genislik * DISPLAY_HEIGHT;
            int bitmap_pixel_index = yukseklik + genislik * 60;
            
            bw_buf[display_pixel_index / 8] |= ((black_bitmap[bitmap_pixel_index / 8] >> (7 - (bitmap_pixel_index % 8))) & 1) << (7 - (display_pixel_index % 8)); // Set the bit
            //if(display_pixel_index<1000){
            //    ESP_LOGI(TAG, "display_pixel_index: %d bitmap_pixel_index: %d display byte: %d bitmap byte: %d genislik: %d yükseklik: %d", display_pixel_index, bitmap_pixel_index,display_pixel_index / 8,bitmap_pixel_index / 8, genislik, yukseklik);
            //    vTaskDelay(pdMS_TO_TICKS(10));        
            //}
            
        }
        
    }
    ESP_LOGI(TAG, "ii: %d ", i);
}
#define IMAGE_WIDTH 60
#define IMAGE_HEIGHT 60


// Image'i buffer'a yerleştirme fonksiyonu
void place_image_into_buffer(int x, int y,const unsigned char* black_bitmap) {
// Image'in buffer sınırlarını aşmamasını kontrol et
    if (x < 0 || y < 0 || x + IMAGE_WIDTH > DISPLAY_WIDTH || y + IMAGE_HEIGHT > DISPLAY_HEIGHT) {
        return; // Geçersiz konum, işlemi iptal et
    }

    // Image'i buffer'a yerleştir
    for (int i = 0; i < IMAGE_HEIGHT; i++) { // Image'in her satırı
        for (int j = 0; j < IMAGE_WIDTH; j++) { // Image'in her sütunu
            // Image'deki piksel değeri (0 veya 1)
            uint8_t pixel = (black_bitmap[i * (IMAGE_WIDTH / 8) + j / 8] >> (7 - (j % 8))) & 0x01;

            // Buffer'daki ilgili byte ve bit pozisyonu
            int buffer_col = x + j; // Buffer'da sütun
            int buffer_row = (y + i) / 8; // Buffer'da byte satırı
            int buffer_bit = (y + i) % 8; // Buffer'da bit pozisyonu

            // Buffer'daki ilgili byte'ı al
            uint8_t *buffer_byte = &bw_buf[buffer_row * DISPLAY_WIDTH + buffer_col];

            // Pikseli buffer'a yerleştir
            if (pixel) {
                *buffer_byte |= (1 << buffer_bit); // Bit'i 1 yap
            } else {
                *buffer_byte &= ~(1 << buffer_bit); // Bit'i 0 yap
            }
        }
    }
}

void drawCH(){

    for (int w = 0; w < 15; w++) { // Image'in her satırı
        for (int h = 0; h < 15; h++) { // Image'in her sütunu
            // Image'deki piksel değeri (0 veya 1)
            //uint8_t pixel = (black_bitmap[(h + (bitmap_width / 8+1) * (w / 8))/8] >> (7 - (h % 8))) & 0x01;
            uint8_t pixel = (char_buf[w * 8 + h / 8] >> (7 - (h % 8))) & 0x01; //char_buf
            

            // Buffer'daki ilgili byte ve bit pozisyonu
            int buffer_col =  w; // Buffer'da sütun
            int buffer_row = ( h) / 8; // Buffer'da byte satırı
            int buffer_bit = ( h) % 8; // Buffer'da bit pozisyonu

            // Buffer'daki ilgili byte'ı al
            uint8_t *buffer_byte = &bw_buf[(buffer_col * (DISPLAY_HEIGHT / 8+1)) + buffer_row];

            //if(i<300){
            //    ESP_LOGI(TAG, "buffer_col: %d buffer_row: %d buffer_bit: %d buffer_byte: %04x pixel: %d pixel: %d h: %d w: %d px:%d", buffer_col, buffer_row, buffer_bit, *buffer_byte, pixel, pixel, h, w, (w * 8 + h / 8));
            //    i++;
            //}

            // Pikseli buffer'a yerleştir
            if (!pixel) {
                *buffer_byte |= (1 << (7-buffer_bit)); // Bit'i 1 yap
            } else {
                *buffer_byte &= ~(1 << (7-buffer_bit)); // Bit'i 0 yap
            }
        }
    }

}

void testFont(int w, int h, uint8_t pixel, int x, int y){
    // Buffer'daki ilgili byte ve bit pozisyonu
    int buffer_col = x + w; // Buffer'da sütun
    int buffer_row = ( y + h ) / 8; // Buffer'da byte satırı
    int buffer_bit = ( y + h ) % 8; // Buffer'da bit pozisyonu
    uint8_t *buffer_byte = &bw_buf[(buffer_col * (DISPLAY_HEIGHT / 8+1)) + buffer_row];
    if (!pixel) {
        *buffer_byte |= (1 << (7-buffer_bit)); // Bit'i 1 yap
    } else {
        *buffer_byte &= ~(1 << (7-buffer_bit)); // Bit'i 0 yap
    }
}


void draw_line_horizontal(int x1, int x2){
    for(int i = x1; i < x2; i++){
        testFont(i, 70, 1, 0, 0);
    }
    
    //testFont(x1, 100, 1, x2, 100);

}

void draw_line_vertical(int y1, int y2){
    for(int i = y1; i < y2; i++){
        testFont(100, i, 1, 0, 0);
    }
    
    //testFont(x1, 100, 1, x2, 100);

}

int counter2 = 0;
int counterHeight2 = 0;
int counterByte2 = 0;
int wC = 0;
int hC = 0;

void printBinary2(unsigned char value, int length, int height, int x, int y){
    for (int i = 7; i >= 0; i--) {
        if(counterHeight2==height){
            continue;
        }
        
        if(wC==length){
            wC = 0;
            hC++;
        }
        testFont(wC, hC, (value >> i) & 1, x, y);
        wC++;

        
        if((value >> i) & 1){
            printf("█ ");
        }else{
            printf("  ");
        }
        counter2++;
        if(counter2==length){
            printf("\n");
            counterHeight2++;
            counter2 = 0;
        }
    }
}



void draw_Char(int letter, int x, int y){

    ESP_LOGI(TAG, "FreeMono9pt7bBitmaps: %d", sizeof(FreeMono9pt7bBitmaps));
    ESP_LOGI(TAG, "test: %d", FreeMono9pt7bGlyphs[letter - FreeMono9pt7b.first].xAdvance);

    int widthOfLetter = FreeMono9pt7bGlyphs[letter - FreeMono9pt7b.first].width;
    int heightOfLetter = FreeMono9pt7bGlyphs[letter - FreeMono9pt7b.first].height;

    ESP_LOGI(TAG, "Char %c: \nBitmapOffset: %d \nWidth: %d \nHeight: %d \nxAdvence: %d \nxOffset: %d \nyOffset: %d \nyAdvance: %d \nFirstByte: %.4x", 
                                                                                                                        letter,
                                                                                                                        FreeMono9pt7bGlyphs[letter - FreeMono9pt7b.first].bitmapOffset,
                                                                                                                        widthOfLetter,
                                                                                                                        heightOfLetter,
                                                                                                                        FreeMono9pt7bGlyphs[letter - FreeMono9pt7b.first].xAdvance,
                                                                                                                        FreeMono9pt7bGlyphs[letter - FreeMono9pt7b.first].xOffset,
                                                                                                                        FreeMono9pt7bGlyphs[letter - FreeMono9pt7b.first].yOffset,
                                                                                                                        FreeMono9pt7b.yAdvance,
                                                                                                                        FreeMono9pt7bBitmaps[FreeMono9pt7bGlyphs[letter - FreeMono9pt7b.first].bitmapOffset]);
    
    //for(int i=0; i<(widthOfLetter * heightOfLetter / 8 +1);i++){
    //    printBinary(FreeMono9pt7bBitmaps[FreeMono9pt7bGlyphs[letter - FreeMono9pt7b.first].bitmapOffset + i],widthOfLetter, heightOfLetter);
    //}


    for(int i=0; i<(widthOfLetter * heightOfLetter / 8 +1);i++){
        printBinary2(FreeMono9pt7bBitmaps[FreeMono9pt7bGlyphs[letter - FreeMono9pt7b.first].bitmapOffset + i],widthOfLetter, heightOfLetter, x, y);
        //test(width, height, (FreeMono9pt7bBitmaps[FreeMono9pt7bGlyphs[letter - FreeMono9pt7b.first].bitmapOffset + i] >> (7 - (width % 8))) & 0x01);
        //width++;
        //if(width==widthOfLetter){
        //    width = 0;
        //    height++;
        //}
    }
    counter2 = 0;
    counterHeight2 = 0;
    counterByte2 = 0;
    wC = 0;
    hC = 0;

    //for(int i=0; i<(widthOfLetter * heightOfLetter / 8 +1);i++){
    //    ESP_LOGI(TAG, "Bitmaps %d: %.4x", i, FreeMono9pt7bBitmaps[FreeMono9pt7bGlyphs[letter - FreeMono9pt7b.first].bitmapOffset + i]);
    //}

}



void draw_Char2(int letter, int x, int y){

    ESP_LOGI(TAG, "FreeMonoBold12pt7bBitmaps: %d", sizeof(FreeMonoBold12pt7bBitmaps));
    ESP_LOGI(TAG, "test: %d", FreeMonoBold12pt7bGlyphs[letter - FreeMonoBold12pt7b.first].xAdvance);

    int widthOfLetter = FreeMonoBold12pt7bGlyphs[letter - FreeMonoBold12pt7b.first].width;
    int heightOfLetter = FreeMonoBold12pt7bGlyphs[letter - FreeMonoBold12pt7b.first].height;

    ESP_LOGI(TAG, "Char %c: \nBitmapOffset: %d \nWidth: %d \nHeight: %d \nxAdvence: %d \nxOffset: %d \nyOffset: %d \nyAdvance: %d \nFirstByte: %.4x", 
                                                                                                                        letter,
                                                                                                                        FreeMonoBold12pt7bGlyphs[letter - FreeMonoBold12pt7b.first].bitmapOffset,
                                                                                                                        widthOfLetter,
                                                                                                                        heightOfLetter,
                                                                                                                        FreeMonoBold12pt7bGlyphs[letter - FreeMonoBold12pt7b.first].xAdvance,
                                                                                                                        FreeMonoBold12pt7bGlyphs[letter - FreeMonoBold12pt7b.first].xOffset,
                                                                                                                        FreeMonoBold12pt7bGlyphs[letter - FreeMonoBold12pt7b.first].yOffset,
                                                                                                                        FreeMonoBold12pt7b.yAdvance,
                                                                                                                        FreeMonoBold12pt7bBitmaps[FreeMonoBold12pt7bGlyphs[letter - FreeMonoBold12pt7b.first].bitmapOffset]);

    for(int i=0; i<(widthOfLetter * heightOfLetter / 8 +1);i++){
        printBinary2(FreeMonoBold12pt7bBitmaps[FreeMonoBold12pt7bGlyphs[letter - FreeMonoBold12pt7b.first].bitmapOffset + i],widthOfLetter, heightOfLetter, x, y);
    }
    counter2 = 0;
    counterHeight2 = 0;
    counterByte2 = 0;
    wC = 0;
    hC = 0;

}


void draw_Char3(int letter, int x, int y){

    ESP_LOGI(TAG, "Robotobold12pt7bBitmaps: %d", sizeof(Robotobold12pt7bBitmaps));
    ESP_LOGI(TAG, "test: %d", Robotobold12pt7bGlyphs[letter - Robotobold12pt7b.first].xAdvance);

    int widthOfLetter = Robotobold12pt7bGlyphs[letter - Robotobold12pt7b.first].width;
    int heightOfLetter = Robotobold12pt7bGlyphs[letter - Robotobold12pt7b.first].height;

    ESP_LOGI(TAG, "Char %c: \nBitmapOffset: %d \nWidth: %d \nHeight: %d \nxAdvence: %d \nxOffset: %d \nyOffset: %d \nyAdvance: %d \nFirstByte: %.4x", 
                                                                                                                        letter,
                                                                                                                        Robotobold12pt7bGlyphs[letter - Robotobold12pt7b.first].bitmapOffset,
                                                                                                                        widthOfLetter,
                                                                                                                        heightOfLetter,
                                                                                                                        Robotobold12pt7bGlyphs[letter - Robotobold12pt7b.first].xAdvance,
                                                                                                                        Robotobold12pt7bGlyphs[letter - Robotobold12pt7b.first].xOffset,
                                                                                                                        Robotobold12pt7bGlyphs[letter - Robotobold12pt7b.first].yOffset,
                                                                                                                        Robotobold12pt7b.yAdvance,
                                                                                                                        Robotobold12pt7bBitmaps[Robotobold12pt7bGlyphs[letter - Robotobold12pt7b.first].bitmapOffset]);

    for(int i=0; i<(widthOfLetter * heightOfLetter / 8 +1);i++){
        printBinary2(Robotobold12pt7bBitmaps[Robotobold12pt7bGlyphs[letter - Robotobold12pt7b.first].bitmapOffset + i],widthOfLetter, heightOfLetter, x, y);
    }
    counter2 = 0;
    counterHeight2 = 0;
    counterByte2 = 0;
    wC = 0;
    hC = 0;

}

void draw_char_withFont(uint8_t letter, int x, int y, const GFXfont *font){
    int widthOfLetter = font->glyph[letter - font->first].width;
    int heightOfLetter = font->glyph[letter - font->first].height;

    ESP_LOGI(TAG, "Char %c: \nBitmapOffset: %d \nWidth: %d \nHeight: %d \nxAdvance: %d \nxOffset: %d \nyOffset: %d \nyAdvance: %d \nFirstByte: %.4x", 
             letter,
             font->glyph[letter - font->first].bitmapOffset,
             widthOfLetter,
             heightOfLetter,
             font->glyph[letter - font->first].xAdvance,
             font->glyph[letter - font->first].xOffset,
             font->glyph[letter - font->first].yOffset,
             font->yAdvance,
             font->bitmap[font->glyph[letter - font->first].bitmapOffset]);

    for(int i = 0; i < (widthOfLetter * heightOfLetter / 8 + 1); i++) {
        printBinary2(font->bitmap[font->glyph[letter - font->first].bitmapOffset + i], widthOfLetter, heightOfLetter, x, y);
    }
    counter2 = 0;
    counterHeight2 = 0;
    counterByte2 = 0;
    wC = 0;
    hC = 0;
}


void draw_word(const char* word, int x, int y){
    uint8_t letter = word[0];
    uint8_t spaceOfLetter = FreeMono9pt7bGlyphs[letter - FreeMono9pt7b.first].xAdvance;
    ESP_LOGI(TAG, "Word: %s", word);
    for(int i = 0; i < strlen(word); i++){
        int yYeni = y + FreeMono9pt7bGlyphs[word[i] - FreeMono9pt7b.first].yOffset;
        draw_Char(word[i], x, yYeni);
        x += FreeMono9pt7bGlyphs[word[i] - FreeMono9pt7b.first].xAdvance;
        
    }
}



void draw_word_withFont(const char* word, int x, int y, const GFXfont *font){ 
    uint8_t letter = word[0];
    ESP_LOGI(TAG, "Word: %s", word);
    
    for(int i = 0; i < strlen(word); i++){
        int yYeni = y + font->glyph[word[i] - font->first].yOffset;
        draw_char_withFont(word[i], x, yYeni, font);
        x += font->glyph[word[i] - font->first].xAdvance;
        ESP_LOGI(TAG, "Harf: %c Y ekseni: %d Satır Yüksekliği: %d Y Offseti: %d Harf Yüksekliği: %d yYeni: %d",
                        word[i],
                                        y,
                                                    font->yAdvance,
                                                                    font->glyph[word[i] - font->first].yOffset,
                                                                                        font->glyph[word[i] - font->first].height,
                                                                                                        yYeni);    
    }
}