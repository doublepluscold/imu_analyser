#include "heartbeat.h"

#include <string.h>

static void put32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

bool is_heartbeat(const uint8_t *buf, size_t len) {
    return len == kHeartbeatSize && buf[0] == 'I' && buf[1] == 'V' && buf[2] == 'S';
}

size_t encode_heartbeat(const Heartbeat &h, uint8_t *out, size_t cap) {
    if (cap < kHeartbeatSize) return 0;
    out[0] = 'I';
    out[1] = 'V';
    out[2] = 'S';
    out[3] = kHbVersion;
    put32(out + 4, h.uptime_ms);
    put32(out + 8, h.uart_bytes);
    put32(out + 12, h.frames);
    put32(out + 16, h.bad_checksum);
    put32(out + 20, h.too_long);
    put32(out + 24, h.sent);
    put32(out + 28, h.send_errors);
    put32(out + 32, h.nack);
    put32(out + 36, h.dropped);
    out[40] = h.channel;
    out[41] = h.reset_reason;
    out[42] = h.flags;
    out[43] = 0;
    return kHeartbeatSize;
}

bool decode_heartbeat(const uint8_t *buf, size_t len, Heartbeat &h) {
    if (!is_heartbeat(buf, len) || buf[3] != kHbVersion) return false;
    h.uptime_ms = get32(buf + 4);
    h.uart_bytes = get32(buf + 8);
    h.frames = get32(buf + 12);
    h.bad_checksum = get32(buf + 16);
    h.too_long = get32(buf + 20);
    h.sent = get32(buf + 24);
    h.send_errors = get32(buf + 28);
    h.nack = get32(buf + 32);
    h.dropped = get32(buf + 36);
    h.channel = buf[40];
    h.reset_reason = buf[41];
    h.flags = buf[42];
    return true;
}

SlaveDiag diagnose(const Heartbeat &now, const Heartbeat *prev) {
    if (prev && now.uptime_ms < prev->uptime_ms) return SlaveDiag::Restarted;
    if (now.flags & kHbFlagBeacon) return SlaveDiag::Beacon;
    // Unsigned subtraction: correct even if a counter wrapped.
    const uint32_t bytes = prev ? now.uart_bytes - prev->uart_bytes : now.uart_bytes;
    const uint32_t frames = prev ? now.frames - prev->frames : now.frames;
    const uint32_t sent = prev ? now.sent - prev->sent : now.sent;
    const uint32_t nack = prev ? now.nack - prev->nack : now.nack;
    const uint32_t dropped = prev ? now.dropped - prev->dropped : now.dropped;
    if (bytes == 0) return SlaveDiag::UartSilent;
    if (frames == 0) return SlaveDiag::NoValidFrames;
    if ((now.flags & kHbFlagUnicast) && sent > 0 && nack * 2 > sent) return SlaveDiag::NoAck;
    if (dropped > 0) return SlaveDiag::QueueDrops;
    return SlaveDiag::Ok;
}

const char *diag_name(SlaveDiag d) {
    switch (d) {
        case SlaveDiag::Ok: return "OK";
        case SlaveDiag::Restarted: return "RESTARTED";
        case SlaveDiag::Beacon: return "BEACON";
        case SlaveDiag::UartSilent: return "UART_SILENT";
        case SlaveDiag::NoValidFrames: return "NO_VALID_FRAMES";
        case SlaveDiag::NoAck: return "NO_ACK";
        case SlaveDiag::QueueDrops: return "QUEUE_DROPS";
    }
    return "?";
}
