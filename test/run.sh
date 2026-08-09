#!/bin/sh
# Host-native test suite. No hardware, no flash.
set -e
cd "$(dirname "$0")/.."
out="${TMPDIR:-/tmp}/ota_ble-test"
${CXX:-clang++} -std=gnu++17 -Wall -Wextra -g -fsanitize=address,undefined \
    -I test/host-stub -I src \
    -o "$out" \
    test/host-stub/ota_ble-test.cpp test/host-stub/fake_ota.cpp test/host-stub/sha256.cpp \
    src/ota_ble.cpp
exec "$out"
