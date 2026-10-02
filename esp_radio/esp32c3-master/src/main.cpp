// Master (ESP32-C3): ESP-NOW (from slaves) -> laptop over USB-CDC serial.
// Binary datagrams plus text lines that start with "# ".
// The receive callback only queues packets; a separate task validates, assigns module_id and sends.
// Slaves also send a status packet (heartbeat) once a second; the master prints it with a diagnosis,
// so one USB cable is enough to see where a chain breaks.
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <stdarg.h>

#include "config.h"
#include "datagram.h"
#include "heartbeat.h"
#include "id_table.h"
#include "mtdata2_framer.h"

struct RxPacket {
    uint8_t mac[6];
    int8_t rssi;
    uint8_t len;
    uint8_t data[Mtdata2Framer::kMaxFrame];
};

struct ModuleStats {
    uint32_t rx = 0, bad = 0, sent = 0, dropped = 0;
    int8_t rssi = 0;
};

static QueueHandle_t rx_queue;
static Preferences prefs;
static SemaphoreHandle_t io_mu;  // one writer at a time: a text line must never land inside a datagram

// Touched only by out_task (loop() reads counters for printing, torn reads are harmless).
static IdTable id_table;
static uint16_t seq_[IdTable::kMaxModules + 1];
static ModuleStats stats_[IdTable::kMaxModules + 1];
static Heartbeat hb_last_[IdTable::kMaxModules + 1];
static bool hb_have_[IdTable::kMaxModules + 1];
static SlaveDiag diag_[IdTable::kMaxModules + 1];
static volatile uint32_t hb_ms_[IdTable::kMaxModules + 1];  // millis() of the last heartbeat
static volatile uint32_t bad_unknown = 0;       // bad packets from a MAC without an id
static volatile uint32_t table_full_drops = 0;
static volatile uint32_t queue_drops = 0;       // written by callback
static volatile uint32_t out_errors = 0;        // USB buffer full
static volatile bool reset_requested = false;

// ---------------- text lines ("# " + text) ----------------

// Whole write or nothing: a half line / half datagram would break the framing on the laptop.
static bool writeAll(const uint8_t *buf, size_t n) {
    bool ok = false;
    xSemaphoreTake(io_mu, portMAX_DELAY);
    if ((size_t)Serial.availableForWrite() >= n) ok = Serial.write(buf, n) == n;
    xSemaphoreGive(io_mu);
    return ok;
}

static void logf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void logf(const char *fmt, ...) {
    char line[360];
    int n = snprintf(line, sizeof line, "# ");
    va_list ap;
    va_start(ap, fmt);
    int m = vsnprintf(line + n, sizeof line - n - 1, fmt, ap);
    va_end(ap);
    if (m < 0) return;
    n += m;
    if (n > (int)sizeof line - 2) n = (int)sizeof line - 2;
    line[n++] = '\n';
    writeAll((const uint8_t *)line, n);
}

// ---------------- NVS ----------------

static void saveTable() {
    IdTable::Entry e[IdTable::kMaxModules];
    for (size_t i = 0; i < id_table.count(); i++) e[i] = id_table.entry(i);
    prefs.putBytes("ids", e, id_table.count() * sizeof(IdTable::Entry));
    id_table.clear_dirty();
}

static void loadTable() {
    IdTable::Entry e[IdTable::kMaxModules];
    size_t bytes = prefs.getBytesLength("ids");
    if (bytes == 0 || bytes % sizeof(IdTable::Entry) != 0 || bytes > sizeof(e)) return;
    prefs.getBytes("ids", e, bytes);
    if (!id_table.load(e, bytes / sizeof(IdTable::Entry))) {
        logf("saved ID table is invalid, ignoring it");
    }
}

static void resetTable() {
    prefs.remove("ids");
    id_table.reset();
    memset(seq_, 0, sizeof(seq_));
    for (ModuleStats &s : stats_) s = ModuleStats();
    memset(hb_have_, 0, sizeof(hb_have_));
}

// ---------------- ESP-NOW (Wi-Fi task) ----------------

// Receive callback signature differs between Arduino-ESP32 releases (mac vs esp_now_recv_info_t).
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    const uint8_t *src = info->src_addr;
    int8_t rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
#else
static void onReceive(const uint8_t *src, const uint8_t *data, int len) {
    int8_t rssi = 0;
#endif
    if (len <= 0 || len > (int)Mtdata2Framer::kMaxFrame) return;
    RxPacket p;
    memcpy(p.mac, src, 6);
    p.rssi = rssi;
    p.len = (uint8_t)len;
    memcpy(p.data, data, len);
    if (xQueueSend(rx_queue, &p, 0) != pdTRUE) {  // full: drop the oldest, keep the new
        RxPacket old;
        xQueueReceive(rx_queue, &old, 0);
        queue_drops++;
        xQueueSend(rx_queue, &p, 0);
    }
}

// ---------------- output task ----------------

static void sendDatagram(uint8_t id, const uint8_t *frame, size_t len) {
    uint8_t out[kDatagramHeader + Mtdata2Framer::kMaxFrame];
    size_t n = encode_datagram(id, seq_[id], frame, len, out, sizeof(out));
    seq_[id]++;  // uint16_t wraps 65535 -> 0
    bool ok = n != 0 && writeAll(out, n);
    if (!ok) {
        out_errors++;
        stats_[id].dropped++;
        return;
    }
    stats_[id].sent++;
}

static void handleHeartbeat(const RxPacket &p) {
    Heartbeat h;
    if (!decode_heartbeat(p.data, p.len, h)) {
        bad_unknown++;
        return;
    }
    uint8_t id = id_table.lookup_or_assign(p.mac);  // a slave with a silent UART still gets its id
    if (id == 0) {
        table_full_drops++;
        return;
    }
    if (id_table.dirty()) saveTable();
    stats_[id].rssi = p.rssi;
    diag_[id] = diagnose(h, hb_have_[id] ? &hb_last_[id] : nullptr);
    hb_last_[id] = h;
    hb_have_[id] = true;
    hb_ms_[id] = millis();
}

static void outTask(void *) {
    RxPacket p;
    for (;;) {
        if (reset_requested) {
            resetTable();
            reset_requested = false;
        }
        if (xQueueReceive(rx_queue, &p, pdMS_TO_TICKS(100)) != pdTRUE) continue;

        if (is_heartbeat(p.data, p.len)) {
            handleHeartbeat(p);
            continue;
        }
        if (!mtdata2_is_single_frame(p.data, p.len)) {
            uint8_t id = id_table.find(p.mac);
            if (id) stats_[id].bad++;
            else bad_unknown++;
            continue;
        }
        uint8_t id = id_table.lookup_or_assign(p.mac);
        if (id == 0) {
            table_full_drops++;
            continue;
        }
        if (id_table.dirty()) saveTable();

        stats_[id].rx++;
        stats_[id].rssi = p.rssi;
        sendDatagram(id, p.data, p.len);
    }
}

// ---------------- setup / loop ----------------

static void ledSet(bool on) {
#if LED_PIN >= 0
#if LED_IS_RGB
    neopixelWrite(LED_PIN, 0, on ? 16 : 0, 0);
#else
    digitalWrite(LED_PIN, (on != (LED_ACTIVE_LOW != 0)) ? HIGH : LOW);
#endif
#else
    (void)on;
#endif
}

// BOOT (GPIO9) is only sampled by the ROM at reset; while the firmware runs it is a normal input.
static void pollResetButton(uint32_t now) {
    static uint32_t down_since = 0;
    static bool fired = false;
    if (digitalRead(RESET_BUTTON_PIN) != LOW) {
        down_since = 0;
        fired = false;
        return;
    }
    if (!down_since) down_since = now ? now : 1;
    if (!fired && now - down_since >= RESET_HOLD_MS) {
        fired = true;
        reset_requested = true;
        logf("BOOT held %d s: ID table reset requested", RESET_HOLD_MS / 1000);
    }
}

void setup() {
    io_mu = xSemaphoreCreateMutex();
    Serial.setTxBufferSize(2048);  // call before begin(): room for bursts from several slaves
    Serial.setTxTimeoutMs(0);      // never block when nobody listens: drop instead
    Serial.begin(115200);
    delay(1000);  // let USB CDC attach

    prefs.begin("imumaster", false);
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
#if LED_PIN >= 0 && !LED_IS_RGB
    pinMode(LED_PIN, OUTPUT);
#endif
    ledSet(false);
    loadTable();

    // Wi-Fi only for ESP-NOW: STA, fixed channel, no AP.
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect();
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    rx_queue = xQueueCreate(RX_QUEUE_LEN, sizeof(RxPacket));
    if (esp_now_init() != ESP_OK) {
        logf("esp_now_init failed, restarting");
        delay(1000);
        ESP.restart();
    }
    esp_now_register_recv_cb(onReceive);
    xTaskCreate(outTask, "out", 6144, nullptr, 5, nullptr);

    logf("master up: wifi mac=%s ch=%d out=usb ids_restored=%u", WiFi.macAddress().c_str(), WIFI_CHANNEL,
         (unsigned)id_table.count());
}

static void printStats() {
    logf("link out=usb slaves=%u queue=%u qdrop=%lu out_err=%lu bad_unknown=%lu", (unsigned)id_table.count(),
         (unsigned)uxQueueMessagesWaiting(rx_queue), (unsigned long)queue_drops, (unsigned long)out_errors,
         (unsigned long)bad_unknown);
    for (size_t i = 0; i < id_table.count(); i++) {
        const IdTable::Entry &e = id_table.entry(i);
        const ModuleStats &s = stats_[e.id];
        char hb[170];
        if (hb_have_[e.id] && millis() - hb_ms_[e.id] < HEARTBEAT_LOST_MS) {
            const Heartbeat &h = hb_last_[e.id];
            snprintf(hb, sizeof hb,
                     "hb=ok up=%lu uart=%lu frames=%lu badcs=%lu tx=%lu err=%lu nack=%lu qdrop=%lu ch=%u rst=%u diag=%s",
                     (unsigned long)(h.uptime_ms / 1000), (unsigned long)h.uart_bytes, (unsigned long)h.frames,
                     (unsigned long)h.bad_checksum, (unsigned long)h.sent, (unsigned long)h.send_errors,
                     (unsigned long)h.nack, (unsigned long)h.dropped, (unsigned)h.channel,
                     (unsigned)h.reset_reason, diag_name(diag_[e.id]));
        } else {
            snprintf(hb, sizeof hb, hb_have_[e.id] ? "hb=lost" : "hb=none");
        }
        logf("id=%u mac=%02X:%02X:%02X:%02X:%02X:%02X rx=%lu bad=%lu sent=%lu drop=%lu rssi=%d %s", e.id,
             e.mac[0], e.mac[1], e.mac[2], e.mac[3], e.mac[4], e.mac[5], (unsigned long)s.rx,
             (unsigned long)s.bad, (unsigned long)s.sent, (unsigned long)s.dropped, s.rssi, hb);
    }
    if (id_table.count() == 0) {
        logf("no slave heard yet: is it powered, on channel %d, and (esp32c3_beacon test) sending?", WIFI_CHANNEL);
    }
}

void loop() {
    static uint32_t last_print = 0, last_full_log = 0, last_total = 0, led_off_at = 0;
    static String cmd;
    uint32_t now = millis();

    // Console: "reset_ids" + Enter. (The laptop program never sends anything.)
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            cmd.trim();
            if (cmd == "reset_ids") {
                reset_requested = true;
                logf("ID table reset requested");
            } else if (cmd.length()) {
                logf("unknown command (known: reset_ids)");
            }
            cmd = "";
        } else if (cmd.length() < 32) {
            cmd += c;
        }
    }

    if (now - last_print >= 1000) {
        last_print = now;
        printStats();
    }

    if (now - last_full_log >= FULL_LOG_EVERY_MS) {
        last_full_log = now;
        static uint32_t last_full_drops = 0;
        uint32_t d = table_full_drops;
        if (d != last_full_drops) {
            logf("ID table full (%u modules): %lu packets from new MACs dropped (hold BOOT 5 s while running to clear)",
                 (unsigned)IdTable::kMaxModules, (unsigned long)(d - last_full_drops));
            last_full_drops = d;
        }
    }

    // LED: blink rate follows traffic (toggle whenever 20 more datagrams went out).
    uint32_t total = 0;
    for (size_t i = 1; i <= IdTable::kMaxModules; i++) total += stats_[i].sent;
    if (total - last_total >= 20) {
        last_total = total;
        ledSet(true);
        led_off_at = now + LED_ON_MS;
    }
    if (led_off_at && (int32_t)(now - led_off_at) >= 0) {
        ledSet(false);
        led_off_at = 0;
    }
    pollResetButton(now);
    delay(2);
}
