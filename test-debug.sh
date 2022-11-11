#!/bin/bash

#
# Test debug builds using Clang sanitizers
#

PROGRAM="./raytrace 100 100 input.json output.ppm"

make clean
CC="clang" make asan
ASAN_OPTIONS=detect_leaks=1 LSAN_OPTIONS=suppressions=lsan.supp ${PROGRAM}

if [[ "$OSTYPE" == "linux-gnu"* ]]
then
    make clean
    CC="clang" make msan
    ${PROGRAM}
fi

make clean
CC="clang" make tsan
${PROGRAM}
