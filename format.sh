#!/bin/bash

SCRIPT_DIR="$(dirname $0)"

function format {
    clang-format-9 -style=file -i "$1"
}

for d in src include test; do
    FULL_DIR="$SCRIPT_DIR/$d"
    for f in "$FULL_DIR"/*.hpp "$FULL_DIR"/*.h "$FULL_DIR"/*.cpp; do
        if [[ -f "$f" && $(basename "$f") != 3rd* ]]; then
            (printf 'Formatting %s\n' "$f"; format "$f") &
        fi
    done
done
