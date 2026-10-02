#pragma once
// Slave status packet ("heartbeat"): every slave sends one per second over ESP-NOW, on the same radio
// path as the MTData2 frames. It lets the master (and through it the laptop) see WHERE the chain
// breaks: radio, UART input, or the slave itself (restarts).
//
// The packet starts with 'I' 'V' 'S'. An MTData2 frame starts with 0xFA, so the two can never be
// mixed up. Little-endian, 44 bytes. Plain C++, no Arduino/ESP-IDF dependencies.
#include <stddef.h>
#include <stdint.h>

constexpr size_t kHeartbeatSize = 44;
constexpr uint8_t kHbVersion = 1;
constexpr uint8_t kHbFlagBeacon = 0x01;   // slave runs TEST_BEACON: synthetic frames, UART not forwarded
constexpr uint8_t kHbFlagUnicast = 0x02;  // slave sends to a fixed master MAC (MAC-level ACK), not broadcast

//  offset size field
//  0      3    'I' 'V' 'S'
//  3      1    version = 1
//  4      4    uptime_ms
//  8      4    uart_bytes     bytes read from the UART since boot
//  12     4    frames         valid MTData2 frames found in them
//  16     4    bad_checksum   complete frames with a wrong checksum (dropped)
//  20     4    too_long       frames longer than one ESP-NOW packet (skipped)
//  24     4    sent           frames handed to the radio
//  28     4    send_errors    esp_now_send() refused
//  32     4    nack           radio reported "not delivered" (unicast: no ACK)
//  36     4    dropped        frames thrown away because the send queue was full
//  40     1    channel        Wi-Fi channel the slave actually runs on
//  41     1    reset_reason   ESP8266 reset reason code (0 = power on)
//  42     1    flags          kHbFlag*
//  43     1    reserved = 0
struct Heartbeat {
    uint32_t uptime_ms = 0;
    uint32_t uart_bytes = 0;
    uint32_t frames = 0;
    uint32_t bad_checksum = 0;
    uint32_t too_long = 0;
    uint32_t sent = 0;
    uint32_t send_errors = 0;
    uint32_t nack = 0;
    uint32_t dropped = 0;
    uint8_t channel = 0;
    uint8_t reset_reason = 0;
    uint8_t flags = 0;
};

// What the numbers say about the last second. Codes are ASCII on purpose (the laptop translates).
enum class SlaveDiag : uint8_t {
    Ok = 0,
    Restarted,      // uptime went backwards: slave rebooted (weak 3.3 V supply is the usual cause)
    Beacon,         // radio self-test mode, UART is not forwarded
    UartSilent,     // no byte arrived on the UART: wiring, tap point, module off
    NoValidFrames,  // bytes arrive but no valid MTData2 frame: baud, levels, inverted signal
    NoAck,          // unicast only: the master does not acknowledge (wrong MAC, channel, range)
    QueueDrops,     // radio cannot keep up, frames are lost in the slave
};

bool is_heartbeat(const uint8_t *buf, size_t len);
size_t encode_heartbeat(const Heartbeat &h, uint8_t *out, size_t cap);  // 0 if cap is too small
bool decode_heartbeat(const uint8_t *buf, size_t len, Heartbeat &h);    // false if not a valid heartbeat

// `prev` is the heartbeat before this one (nullptr for the first): rates are judged on the
// difference, so a link that was bad at boot but works now reads Ok.
SlaveDiag diagnose(const Heartbeat &now, const Heartbeat *prev);
const char *diag_name(SlaveDiag d);  // "OK", "RESTARTED", "BEACON", "UART_SILENT", ...
