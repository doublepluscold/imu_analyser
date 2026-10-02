#pragma once
// Master (ESP32-C3 Mini) settings. Output is USB-CDC only: binary datagrams (format:
// imuview/netproto.py) plus text lines that start with "# " (diagnostics); the laptop program
// separates them.

// Must be the same on master and all slaves.
#define WIFI_CHANNEL 1

// ---- Pipeline ----
#define RX_QUEUE_LEN   64   // ESP-NOW callback -> output task; full -> oldest dropped

// ---- IDs ----
// BOOT = GPIO9 on C3. It is a strapping pin: held LOW at reset it enters the USB bootloader, so the
// firmware never runs. Therefore the table is cleared by holding BOOT for RESET_HOLD_MS WHILE the
// firmware is running (not at power-up). Console command "reset_ids" does the same.
#define RESET_BUTTON_PIN 9
#define RESET_HOLD_MS    5000
#define FULL_LOG_EVERY_MS 10000

// A slave whose status packet did not arrive for this long is reported as lost.
#define HEARTBEAT_LOST_MS 3500

// Status LED. Differs between C3 Mini boards, check yours:
//   LED_PIN       GPIO number, or -1 if the board has no LED
//   LED_IS_RGB    1 = WS2812 (neopixelWrite), 0 = plain LED
//   LED_ACTIVE_LOW  plain LED only: 1 = LOW turns it on
//   board                  LED_PIN  LED_IS_RGB  LED_ACTIVE_LOW
//   ESP32-C3 SuperMini        8        0           1   (plain LED; GPIO8 is a strapping pin, keep it unloaded)
//   Lolin/WeAct C3 (WS2812)   7        1           0
#define LED_PIN 8
#define LED_IS_RGB 0
#define LED_ACTIVE_LOW 1
#define LED_ON_MS 40
