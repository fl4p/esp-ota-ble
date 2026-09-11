#pragma once
// Private bench override, applied after the prebuilt SDK defaults, before
// every NimBLE-Arduino translation unit. Runtime `buffers` reports the result.
#include <sdkconfig.h>
#undef CONFIG_BT_NIMBLE_MSYS_1_BLOCK_COUNT
#define CONFIG_BT_NIMBLE_MSYS_1_BLOCK_COUNT 50
#undef CONFIG_BT_NIMBLE_MSYS1_BLOCK_COUNT
#define CONFIG_BT_NIMBLE_MSYS1_BLOCK_COUNT 50
#ifdef PROTO_BLE_WIDE_POOLS
#undef CONFIG_BT_NIMBLE_MSYS_2_BLOCK_COUNT
#define CONFIG_BT_NIMBLE_MSYS_2_BLOCK_COUNT 96
#undef CONFIG_BT_NIMBLE_TRANSPORT_ACL_FROM_LL_COUNT
#define CONFIG_BT_NIMBLE_TRANSPORT_ACL_FROM_LL_COUNT 96
#endif
#ifdef PROTO_BLE_CE_MODE
// Passed into the controller's initialization structure by NimBLEDevice.cpp.
// 0=original, 1=HCI CE length, 2=Espressif method (esp_bt.h field contract).
#undef CONFIG_BT_CTRL_CE_LENGTH_TYPE_EFF
#define CONFIG_BT_CTRL_CE_LENGTH_TYPE_EFF PROTO_BLE_CE_MODE
#endif
