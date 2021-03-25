#!/bin/bash

if [[ $# -eq 0 ]]; then
    printf 'Usage: <BUILD_TYPE> [C++ COMPILER]\n'
    exit
fi

if [[ $# -ge 1 ]]; then
    BUILD_TYPE="$1"
fi
if [[ $# -ge 2 ]]; then
    COMPILER="$2"
fi

WARNINGS="-Wall -Wpedantic -Wextra -Werror"
FLAGS="-march=native"

cmake -E remove_directory build
cmake -E make_directory build

cmake -E env CXXFLAGS="$FLAGS $WARNINGS" \
    cmake -B build -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DCMAKE_CXX_COMPILER="$COMPILER"

make -B -C build -j $(nproc)
