#include "datagram.h"

#include <string.h>

size_t encode_datagram(uint8_t module_id, uint16_t seq, const uint8_t *payload, size_t len,
                       uint8_t *out, size_t out_cap) {
    if (len > kDatagramMaxPayload || out_cap < kDatagramHeader + len) return 0;
    out[0] = 'I';
    out[1] = 'V';
    out[2] = 1;
    out[3] = module_id;
    out[4] = (uint8_t)(seq & 0xFF);
    out[5] = (uint8_t)(seq >> 8);
    out[6] = (uint8_t)(len & 0xFF);
    out[7] = (uint8_t)(len >> 8);
    if (len) memcpy(out + kDatagramHeader, payload, len);
    return kDatagramHeader + len;
}
