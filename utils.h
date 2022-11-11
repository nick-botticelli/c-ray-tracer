#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef uint8_t PXCHANNEL;

typedef struct Pixel {
    PXCHANNEL r;
    PXCHANNEL g;
    PXCHANNEL b;
} Pixel;

typedef float PXNCHANNEL;

// Normalized Pixel data
typedef struct PixelN {
    PXNCHANNEL r;
    PXNCHANNEL g;
    PXNCHANNEL b;
} PixelN;

void checkError(bool error, const char *errorFormat, ...);
