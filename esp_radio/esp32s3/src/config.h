#pragma once
// Master (ESP32-S3) settings.

// Must be the same on master and all slaves.
#define WIFI_CHANNEL 1

// ---- Ethernet: W5500 on SPI. Pins of Waveshare ESP32-S3-ETH family (wiki). ----
#define ETH_PHY_ADDR_W5500 1
#define ETH_SPI_HOST   SPI2_HOST
#define ETH_PIN_SCK    13
#define ETH_PIN_MISO   12
#define ETH_PIN_MOSI   11
#define ETH_PIN_CS     14
#define ETH_PIN_INT    10
#define ETH_PIN_RST    9

// ---- Network ----
#define MASTER_IP      192, 168, 50, 2
#define MASTER_GATEWAY 192, 168, 50, 1
#define MASTER_NETMASK 255, 255, 255, 0
#define LAPTOP_IP      192, 168, 50, 1
#define BROADCAST_IP   192, 168, 50, 255
#define UDP_PORT       5005
// 1 = send to BROADCAST_IP (laptop address not needed), 0 = send to LAPTOP_IP.
#define UDP_USE_BROADCAST 0

// ---- Pipeline ----
#define RX_QUEUE_LEN   64   // ESP-NOW callback -> UDP task; full -> oldest dropped
#define UDP_TASK_STACK 6144

// ---- IDs ----
#define RESET_BUTTON_PIN 0        // BOOT
#define RESET_HOLD_MS    5000     // hold at power-up to clear the ID table
#define FULL_LOG_EVERY_MS 10000

#define LED_PIN 21                // WS2812 RGB LED on the Waveshare board (neopixelWrite)
