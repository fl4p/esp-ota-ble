#!/bin/sh
# Host-native test suite. No hardware, no flash.
#
# tamp is compiled in from a real checkout when one can be found, so the Tamp transform is tested
# against streams produced by the actual Python compressor rather than against a mock of itself.
# Point TAMP_DIR at a brianpugh/tamp checkout to override the search. esp_delta_ota is always the
# passthrough fake in test/host-stub -- see esp_delta_ota.h for what that does and does not cover.
set -e
cd "$(dirname "$0")/.."
out="${TMPDIR:-/tmp}/ota_ble-test"

if [ -z "$TAMP_DIR" ]; then
    for d in ../fugu-mppt-firmware/managed_components/brianpugh__tamp \
             ../fugu-mppt-firmware/components/tamp/tamp/_c_src; do
        [ -f "$d/tamp/decompressor.c" ] && TAMP_DIR="$d" && break
    done
fi

tamp_srcs=""
tamp_inc=""
if [ -n "$TAMP_DIR" ] && [ -f "$TAMP_DIR/tamp/decompressor.c" ]; then
    tamp_inc="-I $TAMP_DIR"
    tamp_srcs="$TAMP_DIR/tamp/decompressor.c $TAMP_DIR/tamp/common.c $TAMP_DIR/tamp/compressor.c"
    echo "tamp: $TAMP_DIR"
else
    echo "tamp: not found -- Tamp transform tests will be skipped (set TAMP_DIR to enable)"
fi

${CXX:-clang++} -std=gnu++17 -Wall -Wextra -g -fsanitize=address,undefined \
    -I test/host-stub -I src $tamp_inc \
    -x c++ $tamp_srcs \
    -x c++ \
    -o "$out" \
    test/host-stub/ota_ble-test.cpp test/host-stub/fake_ota.cpp test/host-stub/sha256.cpp \
    test/host-stub/fake_delta_ota.cpp \
    src/ota_ble.cpp src/ota_xform.cpp
exec "$out"
