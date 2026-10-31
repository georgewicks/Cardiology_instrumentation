# In-app logic analyzer (SD SPI self-diagnostics)

This project can capture its **own** MicroSD SPI bus (MOSI / MISO / CLK / CS)
with no external hardware, using the
[ok-home/logic_analyzer](https://github.com/ok-home/logic_analyzer) ESP-IDF
component. The ESP32-S3 samples the pins through its `LCD_CAM` peripheral and
prints a decoded summary over the console.

> **Status (important):**
> - The component is vendored under [`components/logic_analyzer/`](../components/logic_analyzer/).
> - It is **disabled by default** (`CONFIG_ANALYZER_USE_PORT_ASSIGN=n`) so it
>   costs nothing at run time in normal builds.
> - It has been ported to *compile* under ESP-IDF v6.0.2, but the capture path
>   currently returns no data under v6 (`capture finished with no data`) — the
>   component was written for IDF 5.x and its LCD_CAM/GDMA runtime setup needs a
>   v6 pass. Enable it only while actively debugging, and treat the capture as
>   work-in-progress.

## How it works

- **In-app (self-diagnostic) mode** taps the SD pins as *extra inputs* into the
  `LCD_CAM` peripheral. It does **not** reconfigure or disturb the SPI driver.
- One free GPIO (`PCLK`, GPIO 38) is used as an internal sample-clock loopback
  (the ESP32 drives the clock out and reads it back on the same pin).
- A small harness in
  [`main/logic_analyzer_sd_capture.c`](../main/logic_analyzer_sd_capture.c)
  starts a capture just before the SD mount and, on completion, prints edge
  counts for each signal plus a hex dump.

## Wiring — nothing to solder

| Channel | Signal | GPIO | Notes |
|---|---|---|---|
| ch0 | CLK  | 18 | tapped in parallel, no change |
| ch1 | MOSI | 17 | tapped in parallel, no change |
| ch2 | MISO | 15 | tapped in parallel, no change |
| ch3 | CS   | 16 | tapped in parallel, no change |
| PCLK | —    | 38 | **leave unconnected** (internal loopback) |

No pull-up/pull-down resistors are needed for the analyzer itself. (The SD bus
still wants its own 10 kΩ pull-ups on MISO/MOSI/CS — that is about SD
reliability, not the analyzer.)

## Enabling it

1. `idf.py menuconfig` → **Logic Analyzer Configuration**.
2. Set:

   | Symbol | Value |
   |---|---|
   | `CONFIG_ANALYZER_USE_PORT_ASSIGN` | `y` |
   | `CONFIG_ANALYZER_CHANNELS` | `8` |
   | `CONFIG_ANALYZER_CHAN_0` | `18` (CLK) |
   | `CONFIG_ANALYZER_CHAN_1` | `17` (MOSI) |
   | `CONFIG_ANALYZER_CHAN_2` | `15` (MISO) |
   | `CONFIG_ANALYZER_CHAN_3` | `16` (CS) |
   | `CONFIG_ANALYZER_PCLK_PIN` | `38` |
   | `CONFIG_ANALYZER_SAMPLE_RATE` | `2000000` |
   | `CONFIG_ANALYZER_SAMPLES_COUNT` | `40000` |
   | `CONFIG_ANALYZER_TIMEOUT` | `10` |
   | `CONFIG_ANALYZER_PSRAM` | `0` |

3. `idf.py build flash monitor`.

The capture runs once at boot, before `InitSD_SPI_debug()`.

## Reading the output

Look for the `LA_SD` lines:

```
I (5048) LA_SD: starting capture: ch0=18 ch1=17 ch2=15 ch3=16, 40000 samples @ 2000000 Hz
I (...) LA_SD: CLK edges=...  MOSI edges=...  MISO edges=...  CS asserts=...
```

- `CLK edges = 0` → SPI clock not reaching the analyzer.
- `MISO edges = 0` → card not driving data (wiring / pull-up / card).
- `CS asserts = 0` → card never selected.

The hex dump (first 128 sample bytes) can be converted to a binary file and
imported into Sigrok **PulseView** as 8-bit digital for protocol-level SPI
decoding. (The component also ships optional WS/SUMP/CLI front-ends that can be
enabled in menuconfig if you want full PulseView integration.)

## Disabling it

Set `CONFIG_ANALYZER_USE_PORT_ASSIGN=n` in menuconfig (or delete the line from
`sdkconfig`). The harness and the call site in `app_main` are guarded by that
symbol, so they compile to nothing; only the ~5 KB core component remains
linked.

## Files

- [`components/logic_analyzer/`](../components/logic_analyzer/) — vendored component (IDF v6 compile port applied)
- [`main/logic_analyzer_sd_capture.c`](../main/logic_analyzer_sd_capture.c) — capture harness
- [`main/include/logic_analyzer_sd_capture.h`](../main/include/logic_analyzer_sd_capture.h) — harness API
- [`sdkconfig.defaults`](../sdkconfig.defaults) — disabled-by-default toggle + PCLK pin
