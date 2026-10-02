#pragma once
// Slave (ESP32-C3 Mini) settings. Same firmware on every slave, no per-board ID.

// Must be the same on master and all slaves. 1, 6, 11 do not overlap.
#define WIFI_CHANNEL 1

// Unicast (ACK + MAC retries) needs the master MAC, printed by the master at boot.
// Leave it commented out to send to broadcast FF:FF:FF:FF:FF:FF (simplest first run).
// #define MASTER_MAC {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

// 1 = stats on USB-CDC (Serial). Safe: it is a different port from the module UART (Serial1).
// Normally set by the PlatformIO env (esp32c3 / esp32c3_debug).
#ifndef DEBUG_LOG
#define DEBUG_LOG 0
#endif

// 1 = RADIO SELF-TEST: the slave ignores the UART and sends a synthetic MTData2 frame 20 times a
// second. Set by the PlatformIO env esp32c3_beacon.
#ifndef TEST_BEACON
#define TEST_BEACON 0
#endif
#define BEACON_PERIOD_MS 50

// Status packet to the master once a second (see common/heartbeat.h).
#define HEARTBEAT_MS 1000

// Module UART (Serial1): RX only, TX is never configured (-1), nothing is written to the module.
// ESP32-C3 pins are 3.3 V only, NOT 5 V tolerant: the converter logic level must be 3.3 V.
// Do not use strapping pins 2, 8, 9 (GPIO20 is the default U0RXD pin; UART0 itself is unused
// because Serial is USB-CDC).
#define RX_PIN 20
#define UART_BAUD 115200
#define UART_RX_BUFFER 1024

// Frame queue between UART parser and radio. Full -> oldest frame is dropped.
#define TX_QUEUE_FRAMES 12

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
#define LED_ON_MS 15

// TX power in dBm (ESP32-C3: 2..20). Converted to 0.25 dBm units for esp_wifi_set_max_tx_power.
// USB-C supply is enough for 10 dBm; lower it if the heartbeat shows restarts.
#define TX_POWER_DBM 10.0f
