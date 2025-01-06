#ifndef FONTSTRUCT_H
#define FONTSTRUCT_H


#include <stdint.h>


// Glif yapısı
typedef struct {
    uint16_t bitmapOffset; // Bitmap verisine ofset
    uint8_t width;         // Glif genişliği
    uint8_t height;        // Glif yüksekliği
    uint8_t xAdvance;      // İmlecin ilerleme miktarı
    int8_t xOffset;        // X ofseti
    int8_t yOffset;        // Y ofseti
} GFXglyph;

// Font yapısı
typedef struct {
    const uint8_t *bitmap;      // Bitmap verisi
    const GFXglyph *glyph;      // Glif verileri
    uint8_t first;              // İlk karakter (ASCII)
    uint8_t last;               // Son karakter (ASCII)
    uint8_t yAdvance;           // Satır yüksekliği
} GFXfont;
#endif