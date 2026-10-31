/**
 * @file logic_analyzer_sd_capture.c
 * @brief Minimal in-app capture of the MicroSD SPI bus for self-diagnostics.
 *
 * Uses the ok-home/logic_analyzer component in its in-app (self-diagnostic)
 * mode: the SD SPI pins keep their normal SPI function, and the analyzer
 * merely enables an extra input path from those pins into the LCD_CAM
 * peripheral.  It does NOT reconfigure the SD pins.
 *
 * Channel packing (8-bit mode) is LSB-first: bit0 = ch0, bit1 = ch1, ...
 * The default menuconfig assignment used by this project is:
 *   ch0 = CLK, ch1 = MOSI, ch2 = MISO, ch3 = CS
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "logic_analyzer_hal.h"

static const char *TAG = "LA_SD";

/* Number of leading sample bytes dumped to the console as hex. */
#define LA_DUMP_PREFIX_BYTES 128

static void la_capture_done(uint8_t *sample_buf, int samples, int sample_rate, int channels)
{
    if (sample_buf == NULL || samples <= 0) {
        ESP_LOGW(TAG, "capture finished with no data (timeout or empty)");
        return;
    }

    uint32_t clk_edges = 0;
    uint32_t mosi_edges = 0;
    uint32_t miso_edges = 0;
    uint32_t cs_asserts = 0;
    uint32_t cs_low_samples = 0;

    uint8_t prev = sample_buf[0];
    for (int i = 1; i < samples; i++) {
        uint8_t cur = sample_buf[i];
        uint8_t changed = cur ^ prev;

        if (changed & 0x01) {           /* ch0: CLK  */
            clk_edges++;
        }
        if (changed & 0x02) {           /* ch1: MOSI */
            mosi_edges++;
        }
        if (changed & 0x04) {           /* ch2: MISO */
            miso_edges++;
        }
        if ((cur & 0x08) == 0) {        /* ch3: CS (active low) */
            cs_low_samples++;
            if (changed & 0x08) {
                cs_asserts++;           /* high -> low transition */
            }
        }
        prev = cur;
    }

    ESP_LOGI(TAG, "--- SD SPI capture summary ---");
    ESP_LOGI(TAG, "samples=%d rate=%" PRId32 " Hz channels=%d (window=%.1f ms)",
             samples, (int32_t)sample_rate, channels,
             1000.0 * (double)samples / (double)sample_rate);
    ESP_LOGI(TAG, "CLK edges=%" PRIu32 "  MOSI edges=%" PRIu32 "  MISO edges=%" PRIu32
             "  CS asserts=%" PRIu32 " (CS low=%" PRIu32 " samples)",
             clk_edges, mosi_edges, miso_edges, cs_asserts, cs_low_samples);

    if (clk_edges == 0) {
        ESP_LOGW(TAG, ">> CLK never toggled: SPI clock not reaching the analyzer");
    }
    if (miso_edges == 0) {
        ESP_LOGW(TAG, ">> MISO never toggled: card is not driving data "
                      "(check MISO wiring, pull-up, or the card)");
    }
    if (cs_asserts == 0) {
        ESP_LOGW(TAG, ">> CS never asserted: card was never selected");
    }

    ESP_LOGI(TAG, "hex dump (first %d sample bytes):", LA_DUMP_PREFIX_BYTES);
    int dump = (samples < LA_DUMP_PREFIX_BYTES) ? samples : LA_DUMP_PREFIX_BYTES;
    for (int i = 0; i < dump; i++) {
        printf("%02X", sample_buf[i]);
        if ((i & 31) == 31) {
            printf("\n");
        }
    }
    if ((dump & 31) != 0) {
        printf("\n");
    }
}

void la_sd_capture_start(void)
{
#if defined(CONFIG_ANALYZER_USE_PORT_ASSIGN) && CONFIG_ANALYZER_USE_PORT_ASSIGN
    logic_analyzer_config_t cfg = {
        .pin = {
            CONFIG_ANALYZER_CHAN_0,  CONFIG_ANALYZER_CHAN_1,
            CONFIG_ANALYZER_CHAN_2,  CONFIG_ANALYZER_CHAN_3,
            CONFIG_ANALYZER_CHAN_4,  CONFIG_ANALYZER_CHAN_5,
            CONFIG_ANALYZER_CHAN_6,  CONFIG_ANALYZER_CHAN_7,
            CONFIG_ANALYZER_CHAN_8,  CONFIG_ANALYZER_CHAN_9,
            CONFIG_ANALYZER_CHAN_10, CONFIG_ANALYZER_CHAN_11,
            CONFIG_ANALYZER_CHAN_12, CONFIG_ANALYZER_CHAN_13,
            CONFIG_ANALYZER_CHAN_14, CONFIG_ANALYZER_CHAN_15,
        },
        .pin_trigger       = CONFIG_ANALYZER_TRIG_PIN,
        .trigger_edge      = CONFIG_ANALYZER_TRIG_EDGE,
        .number_of_samples = CONFIG_ANALYZER_SAMPLES_COUNT,
        .sample_rate       = CONFIG_ANALYZER_SAMPLE_RATE,
        .number_channels   = CONFIG_ANALYZER_CHANNELS,
        .samples_to_psram  = CONFIG_ANALYZER_PSRAM,
        .meashure_timeout  = (CONFIG_ANALYZER_TIMEOUT >= 0)
                                 ? (CONFIG_ANALYZER_TIMEOUT * 100)
                                 : portMAX_DELAY,
        .logic_analyzer_cb = la_capture_done,
    };

    ESP_LOGI(TAG, "starting capture: ch0=%d ch1=%d ch2=%d ch3=%d, "
             "%d samples @ %" PRId32 " Hz (%d ch)",
             cfg.pin[0], cfg.pin[1], cfg.pin[2], cfg.pin[3],
             cfg.number_of_samples, (int32_t)cfg.sample_rate,
             cfg.number_channels);

    esp_err_t err = start_logic_analyzer(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start_logic_analyzer failed: 0x%x (%s)",
                 err, esp_err_to_name(err));
    }
#else
    ESP_LOGW(TAG, "logic analyzer not enabled (CONFIG_ANALYZER_USE_PORT_ASSIGN=n); "
             "skipping SD SPI capture");
#endif
}
