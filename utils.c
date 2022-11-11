#include "utils.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

void checkError(bool error, const char *errorFormat, ...) {
    va_list args;
    va_start(args, errorFormat);

    if (error) {
        vfprintf(stderr, errorFormat, args);
        exit(EXIT_FAILURE);
    }

    va_end(args);
}
