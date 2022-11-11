#include "ppmrw.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "utils.h"

void skipWhitespace(FILE *file) {
    char curChar = getc(file);

    while (isspace(curChar)) {
        curChar = getc(file);
    }

    ungetc(curChar, file);
}

void skipGarbage(FILE *file) {
    long int lastIndex;
    char curChar;

    // Loop until no more whitespace or comments are skipped
    do {
        lastIndex = ftell(file);
        curChar = getc(file);

        // Skip whitespace
        while (isspace(curChar)) {
            curChar = getc(file);
        }

        // Skip comments
        while (curChar == '#') {
            while (curChar != '\n') {
                curChar = getc(file);
            }
        }

        ungetc(curChar, file);
    }
    while (ftell(file) != lastIndex);
}

PPM readImage(const char *inputFilename) {
    FILE *inputFile = fopen(inputFilename, "r");
    char magicBuf[3];
    PPM ppm;
    
    checkError(!inputFile, "Error: There was an error opening the input file %s!\n",
               inputFilename);

    // Get and check magic
    fgets(magicBuf, 3, inputFile);

    checkError(magicBuf[0] != 'P' && (magicBuf[1] != '3' || magicBuf[1] != '6'),
               "There was an error reading the PPM format from %s!\n", inputFilename);

    ppm.format = atoi(&magicBuf[1]);
    skipGarbage(inputFile);

    // Read width
    checkError(fscanf(inputFile, "%u", &ppm.width) != 1, "Error: Could not get width of %s!\n",
                      inputFilename);
    skipGarbage(inputFile);

    // Read height
    checkError(fscanf(inputFile, "%u", &ppm.height) != 1, "Error: Could not get height of %s!\n",
                      inputFilename);
    skipGarbage(inputFile);

    // Read max color value
    checkError(fscanf(inputFile, "%u", &ppm.maxColorVal) != 1,
                      "Error: Could not get the max color value of %s!\n", inputFilename);

    // Check if maxColorVal is 255
    checkError(ppm.maxColorVal != 255,
               "Error: Max color value of %u is not 255 (image is not 8-bits per channel)!\n",
               ppm.maxColorVal);
    skipGarbage(inputFile);

    ppm.imageData = malloc(ppm.width * ppm.height * sizeof(Pixel));
    checkError(!ppm.imageData, "Error: Image is too large!\n");

    // Read image data
    if (ppm.format == 3) {
        unsigned int index = 0;

        skipWhitespace(inputFile);

        while (feof(inputFile) == 0) {
            Pixel curPixel;
            int curRed, curGreen, curBlue;
            
            // Read red, green, and blue channels
            checkError(fscanf(inputFile, "%u", &curRed) != 1,
                       "Could not read red channel data!\n", inputFilename);
            checkError(curRed > ppm.maxColorVal, "Error: Invalid red channel in file!\n");
            
            checkError(fscanf(inputFile, "%u", &curGreen) != 1,
                       "Could not read green channel data!\n", inputFilename);
            checkError(curGreen > ppm.maxColorVal, "Error: Invalid green channel in file!\n");
            
            checkError(fscanf(inputFile, "%u", &curBlue) != 1,
                       "Could not read blue channel data!\n", inputFilename);
            checkError(curBlue > ppm.maxColorVal, "Error: Invalid blue channel in file!\n");
            
            // Create pixel
            curPixel.r = curRed;
            curPixel.g = curGreen;
            curPixel.b = curBlue;
            
            // Set the pixel in the output
            ppm.imageData[index] = curPixel;

            skipWhitespace(inputFile);

            index++;
        }
    }
    else if (ppm.format == 6) {
        size_t numRead = fread(ppm.imageData, sizeof(Pixel),
                               ppm.width * ppm.height, inputFile);
        checkError(numRead != ppm.width * ppm.height,
                   "Error: Could not read full image data! Corrupted file?\n");
    }
    else {
        checkError(false, "Error: Input PPM format is not 3 or 6!");
    }

    fclose(inputFile);

    return ppm;
}

void writeImage(PPM ppm, int newFmt, const char *outputFilename) {
    FILE *outputFile = fopen(outputFilename, "w");

    fprintf(outputFile,
            "P%u\n"
            "%u %u\n"
            "%u\n",
            newFmt, ppm.width, ppm.height, ppm.maxColorVal);

    if (newFmt == 3) {
        // Write ascii image data
        for (unsigned int index = 0; index < ppm.width * ppm.height; index++) {
            Pixel curPixel = ppm.imageData[index];
            
            fprintf(outputFile, "%u\n%u\n%u\n", curPixel.r, curPixel.g, curPixel.b);
        }
    }
    else if (newFmt == 6) { // format = 6
        // Write binary image data
        fwrite(ppm.imageData, sizeof(Pixel), ppm.width * ppm.height, outputFile);
    }
    else {
        checkError(false, "Error: Output PPM format is not 3 or 6!");
    }

    fclose(outputFile);
}
