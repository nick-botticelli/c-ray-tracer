#pragma once

#include "utils.h"

typedef struct PPM {
    unsigned int format, maxColorVal, width, height;
    Pixel *imageData;
} PPM;

/**
 Returns a PPM struct representing the image data read from inputFilename where inputFilename is
 the PPM file to read the image data from.
 */
PPM readImage(const char *inputFilename);

/**
 Write a PPM image to a file where ppm is the input image data to write, newFmt is the output PPM
 format type, and outputFilename is the path to the created output file to write to.
 
 ppm.imageData must be freed by the caller.
 */
void writeImage(PPM ppm, int newFmt, const char *outputFilename);
