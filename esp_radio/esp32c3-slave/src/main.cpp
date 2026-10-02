// Slave (ESP32-C3): reads MTData2 from the IMU module UART (RX only), sends every valid
// frame as one ESP-NOW packet, plus a small status packet (heartbeat) once a second.
// Never writes to the UART wired to the module (Serial1 TX = -1).
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_system.h>
#include <esp_wifi.h>

#include "config.h"
#include "heartbeat.h"
#include "mtdata2_framer.h"
#include "test_frame.h"

#ifdef MASTER_MAC
static uint8_t dest_mac[6] = MASTER_MAC;
static const bool kUnicast = true;
#else
static uint8_t dest_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const bool kUnicast = false;
#endif

struct QueuedFrame {
    uint8_t len;
    uint8_t data[Mtdata2Framer::kMaxFrame];
};

static QueuedFrame queue_[TX_QUEUE_FRAMES];
static size_t q_head = 0;  // oldest
static size_t q_count = 0;

static Mtdata2Framer framer;

static uint32_t bytes_read = 0, frames_sent = 0, send_errors = 0, dropped = 0;
static volatile uint32_t cb_fail = 0;  // radio said "not delivered" (unicast: no ACK)

// Send state. The ESP-NOW callback runs in the Wi-Fi task; keep it to flags and counters.
static volatile bool send_busy = false;
static uint32_t send_started_ms = 0;
static uint32_t led_off_at = 0;
static uint32_t last_uart_ms = 0;  // when the last UART byte arrived

static uint8_t currentChannel() {
    uint8_t ch = 0;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&ch, &second);
    return ch;
}

// Send callback signature changed between Arduino-ESP32 3.x releases (mac vs wifi_tx_info_t).
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void onSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
    (void)info;
#else
static void onSent(const uint8_t *mac, esp_now_send_status_t status) {
    (void)mac;
#endif
    if (status != ESP_NOW_SEND_SUCCESS) cb_fail++;
    send_busy = false;
}

static void enqueue(const uint8_t *data, size_t len) {
    if (q_count == TX_QUEUE_FRAMES) {  // full: drop oldest
        q_head = (q_head + 1) % TX_QUEUE_FRAMES;
        q_count--;
        dropped++;
    }
    QueuedFrame &f = queue_[(q_head + q_count) % TX_QUEUE_FRAMES];
    f.len = (uint8_t)len;
    memcpy(f.data, data, len);
    q_count++;
}

static void ledSet(bool on) {
#if LED_PIN >= 0
#if LED_IS_RGB
    neopixelWrite(LED_PIN, 0, on ? 24 : 0, 0);
#else
    digitalWrite(LED_PIN, (on != (LED_ACTIVE_LOW != 0)) ? HIGH : LOW);
#endif
#else
    (void)on;
#endif
}

static void ledPulse(uint32_t ms) {
    ledSet(true);
    led_off_at = millis() + ms;
}

static void trySend() {
    // Do not wait for the callback forever: if it never comes, move on.
    if (send_busy && millis() - send_started_ms > 50) send_busy = false;
    if (send_busy || q_count == 0) return;

    QueuedFrame &f = queue_[q_head];
    const bool is_hb = is_heartbeat(f.data, f.len);
    send_busy = true;
    send_started_ms = millis();
    esp_err_t r = esp_now_send(dest_mac, f.data, f.len);
    q_head = (q_head + 1) % TX_QUEUE_FRAMES;
    q_count--;
    if (r != ESP_OK) {
        send_busy = false;
        send_errors++;
        return;
    }
    if (!is_hb) {  // only real (or beacon) frames count and blink; the heartbeat is housekeeping
        frames_sent++;
        ledPulse(LED_ON_MS);
    }
}

// Status packet: what the slave sees. The master prints a diagnosis from it.
static void queueHeartbeat() {
    const Mtdata2Framer::Stats &s = framer.stats();
    Heartbeat h;
    h.uptime_ms = millis();
    h.uart_bytes = bytes_read;
    h.frames = s.frames;
    h.bad_checksum = s.bad_checksum;
    h.too_long = s.too_long;
    h.sent = frames_sent;
    h.send_errors = send_errors;
    h.nack = cb_fail;
    h.dropped = dropped;
    h.channel = currentChannel();
    h.reset_reason = (uint8_t)esp_reset_reason();
    h.flags = (TEST_BEACON ? kHbFlagBeacon : 0) | (kUnicast ? kHbFlagUnicast : 0);
    uint8_t buf[kHeartbeatSize];
    size_t n = encode_heartbeat(h, buf, sizeof(buf));
    if (n) enqueue(buf, n);
}

#if DEBUG_LOG
static void printStats() {
    const Mtdata2Framer::Stats &s = framer.stats();
    Serial.printf("bytes=%lu frames=%lu bad_cs=%lu too_long=%lu sent=%lu send_err=%lu cb_fail=%lu dropped=%lu ch=%u\n",
                  (unsigned long)bytes_read, (unsigned long)s.frames, (unsigned long)s.bad_checksum,
                  (unsigned long)s.too_long, (unsigned long)frames_sent, (unsigned long)send_errors,
                  (unsigned long)cb_fail, (unsigned long)dropped, (unsigned)currentChannel());
}
#endif

void setup() {
#if LED_PIN >= 0 && !LED_IS_RGB
    pinMode(LED_PIN, OUTPUT);
#endif
    ledSet(false);

#if DEBUG_LOG
    Serial.begin(115200);  // USB-CDC, not the module UART
#endif

    // RX only: TX = -1, so nothing can ever leave toward the module.
    Serial1.setRxBufferSize(UART_RX_BUFFER);
    Serial1.begin(UART_BAUD, SERIAL_8N1, RX_PIN, -1);

    // Radio only for ESP-NOW: station mode, no access point, nothing stored in flash, never
    // connects to a router (that would move the channel and silently break ESP-NOW).
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_max_tx_power((int8_t)(TX_POWER_DBM * 4));
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
#if DEBUG_LOG
        Serial.println("esp_now_init failed");
#endif
        delay(1000);
        ESP.restart();
    }
    esp_now_register_send_cb(onSent);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, dest_mac, 6);
    peer.channel = WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
#if DEBUG_LOG
    Serial.printf("slave up, mac=%s ch=%u (asked %d) %s%s\n", WiFi.macAddress().c_str(),
                  (unsigned)currentChannel(), WIFI_CHANNEL, kUnicast ? "unicast" : "broadcast",
                  TEST_BEACON ? " BEACON" : "");
#endif
}

void loop() {
    // Take everything the UART has; no delay().
    int n = Serial1.available();
    if (n > 0) last_uart_ms = millis();
    while (n-- > 0) {
        int b = Serial1.read();
        if (b < 0) break;
        bytes_read++;
        if (framer.feed((uint8_t)b)) {
            do {
                if (!TEST_BEACON) enqueue(framer.frame(), framer.frame_len());
            } while (framer.poll());
        }
    }

#if TEST_BEACON
    // Radio self-test: synthetic frames, whatever the UART does (it is still counted above).
    static uint32_t last_beacon = 0, beacon_n = 0;
    if (millis() - last_beacon >= BEACON_PERIOD_MS) {
        last_beacon = millis();
        uint8_t f[kTestFrameSize];
        size_t len = build_test_frame(beacon_n++, f, sizeof(f));
        if (len) enqueue(f, len);
    }
#endif

    static uint32_t last_hb = 0;
    if (millis() - last_hb >= HEARTBEAT_MS) {
        last_hb = millis();
        queueHeartbeat();
    }

    trySend();

    // LED language: one short blink per frame sent. One blink per second WITHOUT frames in between
    // = the chip is alive but the UART has been silent for 2 s (check wiring / tap point).
    static uint32_t last_idle_blink = 0;
    if (millis() - last_uart_ms > 2000 && millis() - last_idle_blink >= 1000) {
        last_idle_blink = millis();
        ledPulse(60);
    }
    if (led_off_at && (int32_t)(millis() - led_off_at) >= 0) {
        ledSet(false);
        led_off_at = 0;
    }

#if DEBUG_LOG
    static uint32_t last_print = 0;
    if (millis() - last_print >= 1000) {
        last_print = millis();
        printStats();
    }
#endif
}
