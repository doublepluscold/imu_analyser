// Master: ESP-NOW (from slaves) -> UDP over W5500 Ethernet (to laptop).
// Receive callback only queues packets; a separate task validates, assigns
// module_id and sends UDP. Only loop() writes to Serial.
#include <Arduino.h>
#include <ETH.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <lwip/sockets.h>

#include "config.h"
#include "datagram.h"
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

// Touched only by udp_task (loop() reads counters for printing, torn reads are harmless).
static IdTable id_table;
static uint16_t seq_[IdTable::kMaxModules + 1];
static ModuleStats stats_[IdTable::kMaxModules + 1];
static volatile uint32_t bad_unknown = 0;       // bad packets from a MAC without an id
static volatile uint32_t table_full_drops = 0;
static volatile uint32_t queue_drops = 0;       // written by callback
static volatile uint32_t udp_errors = 0;
static volatile bool reset_requested = false;

static volatile bool eth_link = false;
static volatile bool eth_ip = false;
static int udp_sock = -1;
static struct sockaddr_in dest_addr;

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
        Serial.println("saved ID table is invalid, ignoring it");
    }
}

static void resetTable() {
    prefs.remove("ids");
    id_table.reset();
    memset(seq_, 0, sizeof(seq_));
    memset(stats_, 0, sizeof(stats_));
}

// ---------------- ESP-NOW (Wi-Fi task) ----------------

static void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len <= 0 || len > (int)Mtdata2Framer::kMaxFrame) return;
    RxPacket p;
    memcpy(p.mac, info->src_addr, 6);
    p.rssi = info->rx_ctrl->rssi;
    p.len = (uint8_t)len;
    memcpy(p.data, data, len);
    if (xQueueSend(rx_queue, &p, 0) != pdTRUE) {  // full: drop the oldest, keep the new
        RxPacket old;
        xQueueReceive(rx_queue, &old, 0);
        queue_drops++;
        xQueueSend(rx_queue, &p, 0);
    }
}

// ---------------- UDP task ----------------

static void sendDatagram(uint8_t id, const uint8_t *frame, size_t len) {
    uint8_t out[kDatagramHeader + Mtdata2Framer::kMaxFrame];
    size_t n = encode_datagram(id, seq_[id], frame, len, out, sizeof(out));
    seq_[id]++;  // uint16_t wraps 65535 -> 0
    if (n == 0 || !eth_ip || sendto(udp_sock, out, n, 0, (sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        udp_errors++;
        stats_[id].dropped++;
        return;
    }
    stats_[id].sent++;
}

static void udpTask(void *) {
    RxPacket p;
    for (;;) {
        if (reset_requested) {
            resetTable();
            reset_requested = false;
        }
        if (xQueueReceive(rx_queue, &p, pdMS_TO_TICKS(100)) != pdTRUE) continue;

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

// ---------------- Ethernet ----------------

static void onNetEvent(arduino_event_id_t event, arduino_event_info_t) {
    switch (event) {
        case ARDUINO_EVENT_ETH_CONNECTED: eth_link = true; break;
        case ARDUINO_EVENT_ETH_GOT_IP: eth_ip = true; break;
        case ARDUINO_EVENT_ETH_LOST_IP: eth_ip = false; break;
        case ARDUINO_EVENT_ETH_DISCONNECTED:
        case ARDUINO_EVENT_ETH_STOP: eth_link = eth_ip = false; break;
        default: break;
    }
}

// ---------------- setup / loop ----------------

static void checkResetButton() {
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
    if (digitalRead(RESET_BUTTON_PIN) != LOW) return;
    uint32_t t0 = millis();
    while (digitalRead(RESET_BUTTON_PIN) == LOW) {
        if (millis() - t0 >= RESET_HOLD_MS) {
            resetTable();
            Serial.println("BOOT held: ID table cleared");
            while (digitalRead(RESET_BUTTON_PIN) == LOW) delay(10);
            return;
        }
        delay(10);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);  // let USB CDC attach

    prefs.begin("imumaster", false);
    checkResetButton();
    loadTable();

    // Ethernet (static IP, no DHCP).
    Network.onEvent(onNetEvent);
    ETH.begin(ETH_PHY_W5500, ETH_PHY_ADDR_W5500, ETH_PIN_CS, ETH_PIN_INT, ETH_PIN_RST, ETH_SPI_HOST,
              ETH_PIN_SCK, ETH_PIN_MISO, ETH_PIN_MOSI);
    ETH.config(IPAddress(MASTER_IP), IPAddress(MASTER_GATEWAY), IPAddress(MASTER_NETMASK));

    // UDP socket, send only.
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int yes = 1;
    setsockopt(udp_sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);
#if UDP_USE_BROADCAST
    dest_addr.sin_addr.s_addr = (uint32_t)IPAddress(BROADCAST_IP);
#else
    dest_addr.sin_addr.s_addr = (uint32_t)IPAddress(LAPTOP_IP);
#endif

    // Wi-Fi only for ESP-NOW: STA, fixed channel, no AP.
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect();
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    rx_queue = xQueueCreate(RX_QUEUE_LEN, sizeof(RxPacket));
    if (esp_now_init() != ESP_OK) {
        Serial.println("esp_now_init failed, restarting");
        delay(1000);
        ESP.restart();
    }
    esp_now_register_recv_cb(onReceive);
    xTaskCreate(udpTask, "udp", UDP_TASK_STACK, nullptr, 5, nullptr);

    Serial.printf("master up: wifi mac=%s ch=%d, ids restored=%u, dest=%s:%d\n", WiFi.macAddress().c_str(),
                  WIFI_CHANNEL, (unsigned)id_table.count(), UDP_USE_BROADCAST ? "broadcast" : "laptop", UDP_PORT);
}

static void printStats() {
    Serial.printf("eth link=%d ip=%s queue=%u queue_drops=%lu udp_err=%lu bad_unknown=%lu\n", (int)eth_link,
                  eth_ip ? ETH.localIP().toString().c_str() : "-", (unsigned)uxQueueMessagesWaiting(rx_queue),
                  (unsigned long)queue_drops, (unsigned long)udp_errors, (unsigned long)bad_unknown);
    for (size_t i = 0; i < id_table.count(); i++) {
        const IdTable::Entry &e = id_table.entry(i);
        const ModuleStats &s = stats_[e.id];
        Serial.printf("id=%u mac=%02X:%02X:%02X:%02X:%02X:%02X rx=%lu bad=%lu sent=%lu dropped=%lu rssi=%d\n", e.id,
                      e.mac[0], e.mac[1], e.mac[2], e.mac[3], e.mac[4], e.mac[5], (unsigned long)s.rx,
                      (unsigned long)s.bad, (unsigned long)s.sent, (unsigned long)s.dropped, s.rssi);
    }
}

void loop() {
    static uint32_t last_print = 0, last_full_log = 0, last_total = 0, led_off_at = 0;
    static String cmd;
    uint32_t now = millis();

    // Console: "reset_ids" + Enter.
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            cmd.trim();
            if (cmd == "reset_ids") {
                reset_requested = true;
                Serial.println("ID table reset requested");
            } else if (cmd.length()) {
                Serial.println("unknown command (known: reset_ids)");
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
            Serial.printf("ID table full (%u modules): %lu packets from new MACs dropped\n",
                          (unsigned)IdTable::kMaxModules, (unsigned long)(d - last_full_drops));
            last_full_drops = d;
        }
    }

    // LED: blink rate follows traffic (toggle whenever 20 more datagrams went out).
    uint32_t total = 0;
    for (size_t i = 1; i <= IdTable::kMaxModules; i++) total += stats_[i].sent;
    if (total - last_total >= 20) {
        last_total = total;
        neopixelWrite(LED_PIN, 0, 16, 0);
        led_off_at = now + 40;
    }
    if (led_off_at && (int32_t)(now - led_off_at) >= 0) {
        neopixelWrite(LED_PIN, 0, 0, 0);
        led_off_at = 0;
    }
    delay(2);
}
