#pragma once

#include <stdbool.h>

// The receiver withdraws skip-identical-sectors when this is true; fake_ota.cpp drives it from
// g_fake.encryptionEnabled so a test can exercise the fallback without an encrypted board.
bool esp_flash_encryption_enabled(void);
