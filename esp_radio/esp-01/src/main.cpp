// Slave: reads MTData2 from the IMU module UART (RX only), sends every valid
// frame as one ESP-NOW packet. Never writes to the UART wired to the module.
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

#include "config.h"
#include "mtdata2_framer.h"

extern "C" {
#include "user_interface.h"
}

#ifdef MASTER_MAC
static uint8_t dest_mac[6] = MASTER_MAC;
static const uint8_t dest_role = ESP_NOW_ROLE_SLAVE;
#else
static uint8_t dest_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t dest_role = ESP_NOW_ROLE_SLAVE;
#endif

struct QueuedFrame {
    uint8_t len;
    uint8_t data[Mtdata2Framer::kMaxFrame];
};

static QueuedFrame queue_[TX_QUEUE_FRAMES];
static size_t q_head = 0;  // oldest
static size_t q_count = 0;

static Mtdata2Framer framer;

static uint32_t bytes_read = 0, frames_sent = 0, send_errors = 0, dropped = 0, send_failed_cb = 0;

// Send state. ESP-NOW callback runs in the SDK task; keep it to flags.
static volatile bool send_busy = false;
static volatile bool send_cb_fail = false;
static uint32_t send_started_ms = 0;
static uint32_t led_off_at = 0;

static void onSent(uint8_t *mac, uint8_t status) {
    (void)mac;
    if (status != 0) send_cb_fail = true;
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

static void trySend() {
    if (send_cb_fail) {
        send_cb_fail = false;
        send_failed_cb++;
    }
    // Do not wait for the callback forever: if it never comes, move on.
    if (send_busy && millis() - send_started_ms > 50) send_busy = false;
    if (send_busy || q_count == 0) return;

    QueuedFrame &f = queue_[q_head];
    send_busy = true;
    send_started_ms = millis();
    int r = esp_now_send(dest_mac, f.data, f.len);
    q_head = (q_head + 1) % TX_QUEUE_FRAMES;
    q_count--;
    if (r != 0) {
        send_busy = false;
        send_errors++;
        return;
    }
    frames_sent++;
    digitalWrite(LED_PIN, LOW);  // LED on
    led_off_at = millis() + LED_ON_MS;
}

#if DEBUG_LOG
static void printStats() {
    const Mtdata2Framer::Stats &s = framer.stats();
    Serial.printf("bytes=%lu frames=%lu bad_cs=%lu too_long=%lu sent=%lu send_err=%lu cb_fail=%lu dropped=%lu\n",
                  (unsigned long)bytes_read, (unsigned long)s.frames, (unsigned long)s.bad_checksum,
                  (unsigned long)s.too_long, (unsigned long)frames_sent, (unsigned long)send_errors,
                  (unsigned long)send_failed_cb, (unsigned long)dropped);
}
#endif

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  // LED off

    // RX only: SERIAL_RX_ONLY leaves the TX pin unconfigured, so nothing can leave
    // toward the module. With DEBUG_LOG the TX pin is used for stats (bench only).
    Serial.setRxBufferSize(UART_RX_BUFFER);
#if DEBUG_LOG
    Serial.begin(UART_BAUD, SERIAL_8N1, SERIAL_FULL);
#else
    Serial.begin(UART_BAUD, SERIAL_8N1, SERIAL_RX_ONLY);
#endif
    Serial.setDebugOutput(false);

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setOutputPower(TX_POWER_DBM);
    wifi_set_channel(WIFI_CHANNEL);

    if (esp_now_init() != 0) {
#if DEBUG_LOG
        Serial.println("esp_now_init failed");
#endif
        delay(1000);
        ESP.restart();
    }
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_register_send_cb(onSent);
    esp_now_add_peer(dest_mac, dest_role, WIFI_CHANNEL, NULL, 0);
#if DEBUG_LOG
    Serial.printf("slave up, mac=%s ch=%d\n", WiFi.macAddress().c_str(), WIFI_CHANNEL);
#endif
}

void loop() {
    // Take everything the UART has; no delay().
    int n = Serial.available();
    while (n-- > 0) {
        int b = Serial.read();
        if (b < 0) break;
        bytes_read++;
        if (framer.feed((uint8_t)b)) {
            do {
                enqueue(framer.frame(), framer.frame_len());
            } while (framer.poll());
        }
    }

    trySend();

    if (led_off_at && (int32_t)(millis() - led_off_at) >= 0) {
        digitalWrite(LED_PIN, HIGH);
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
