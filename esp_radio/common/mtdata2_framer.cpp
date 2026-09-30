#include "mtdata2_framer.h"

#include <string.h>

void Mtdata2Framer::drop(size_t n) {
    if (n > n_) n = n_;
    memmove(buf_, buf_ + n, n_ - n);
    n_ -= n;
}

bool Mtdata2Framer::feed(uint8_t b) {
    // buf_ never holds more than kMaxFrame bytes: poll() only waits for a frame
    // whose declared total length fits, and drops the rest.
    if (n_ < kMaxFrame) buf_[n_++] = b;
    return poll();
}

bool Mtdata2Framer::poll() {
    while (n_ > 0) {
        // Find the sync pattern FA FF 36; a mismatch at any position shifts by 1 byte.
        if (buf_[0] != 0xFA || (n_ >= 2 && buf_[1] != 0xFF) || (n_ >= 3 && buf_[2] != 0x36)) {
            drop(1);
            stats_.skipped_bytes++;
            continue;
        }
        if (n_ < 4) return false;

        size_t hdr = 4;
        size_t len = buf_[3];
        if (len == 0xFF) {  // extended length: 2 bytes big-endian
            if (n_ < 6) return false;
            len = ((size_t)buf_[4] << 8) | buf_[5];
            hdr = 6;
        }
        size_t total = hdr + len + 1;
        if (total > kMaxFrame) {
            stats_.too_long++;
            drop(1);
            continue;
        }
        if (n_ < total) return false;

        uint8_t sum = 0;  // BID .. CS inclusive
        for (size_t i = 1; i < total; i++) sum += buf_[i];
        if (sum != 0) {
            stats_.bad_checksum++;
            drop(1);  // resync: maybe a real frame starts inside this one
            continue;
        }

        memcpy(out_, buf_, total);
        out_len_ = total;
        drop(total);
        stats_.frames++;
        return true;
    }
    return false;
}

bool mtdata2_is_single_frame(const uint8_t *buf, size_t len) {
    if (len < 5 || len > Mtdata2Framer::kMaxFrame) return false;
    if (buf[0] != 0xFA || buf[1] != 0xFF || buf[2] != 0x36) return false;
    size_t hdr = 4, plen = buf[3];
    if (plen == 0xFF) {
        if (len < 7) return false;
        plen = ((size_t)buf[4] << 8) | buf[5];
        hdr = 6;
    }
    if (hdr + plen + 1 != len) return false;
    uint8_t sum = 0;
    for (size_t i = 1; i < len; i++) sum += buf[i];
    return sum == 0;
}
