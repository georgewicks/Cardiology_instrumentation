/**
 * @file logic_analyzer_sd_capture.h
 * @brief In-app capture of the MicroSD SPI bus using ok-home/logic_analyzer.
 *
 * The capture is configured entirely from menuconfig (Logic Analyzer
 * Configuration): channel GPIOs, sample rate, sample count, trigger, and
 * PSRAM usage.  When CONFIG_ANALYZER_USE_PORT_ASSIGN is not set, the helper
 * logs a warning and does nothing, so it is safe to call unconditionally.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void la_sd_capture_start(void);

#ifdef __cplusplus
}
#endif
