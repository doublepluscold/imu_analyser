#pragma once
// UDP datagram master -> laptop, see netproto.py in imuview. Little-endian.
//   'I' 'V' | version=1 | module_id | seq u16 | len u16 | len bytes MTData2
#include <stddef.h>
#include <stdint.h>

constexpr size_t kDatagramHeader = 8;
constexpr size_t kDatagramMaxPayload = 1400;

// Writes the datagram to out. Returns its size, or 0 if payload is too long
// or out_cap is too small.
size_t encode_datagram(uint8_t module_id, uint16_t seq, const uint8_t *payload, size_t len,
                       uint8_t *out, size_t out_cap);
