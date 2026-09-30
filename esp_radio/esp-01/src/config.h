#pragma once
// Slave (ESP-01) settings. Same firmware on every slave, no per-board ID.

// Must be the same on master and all slaves. 1, 6, 11 do not overlap.
#define WIFI_CHANNEL 1

// Unicast (ACK + MAC retries) needs the master MAC, printed by the master at boot.
// Leave it commented out to send to broadcast FF:FF:FF:FF:FF:FF (simplest first run).
// #define MASTER_MAC {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

// 1 = stats on UART TX (bench only, TX must NOT go to the IMU module RX). 0 = UART silent.
// Normally set by the PlatformIO env (esp01 / esp01_debug).
#ifndef DEBUG_LOG
#define DEBUG_LOG 0
#endif

#define UART_BAUD 115200
#define UART_RX_BUFFER 1024

// Frame queue between UART parser and radio. Full -> oldest frame is dropped.
#define TX_QUEUE_FRAMES 12

#define LED_PIN 2            // GPIO2, active low
#define LED_ON_MS 15

// TX power in dBm, 0..20.5. Lower = smaller current peaks (helps weak 3.3 V supply,
// brownout resets). 20.5 = SDK default. Range drops with power.
#define TX_POWER_DBM 1.0f
