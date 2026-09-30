#pragma once
// Xsens MTData2 frame assembler: FA FF 36 LEN payload CS.
// Plain C++, no Arduino/ESP-IDF dependencies.
#include <stddef.h>
#include <stdint.h>

class Mtdata2Framer {
public:
    // Longest frame we keep (ESP-NOW payload limit). Longer frames are skipped and counted.
    static constexpr size_t kMaxFrame = 250;

    struct Stats {
        uint32_t frames = 0;        // valid frames produced
        uint32_t bad_checksum = 0;  // complete frames with wrong checksum (dropped)
        uint32_t too_long = 0;      // frames declaring more than kMaxFrame bytes (skipped)
        uint32_t skipped_bytes = 0; // bytes thrown away while looking for FA FF 36
    };

    // Feed one byte. Returns true if a valid frame is now available in frame().
    // After true, call poll() until it returns false: a resync can leave
    // another complete frame already buffered.
    bool feed(uint8_t b);
    bool poll();

    // Valid until the next feed()/poll().
    const uint8_t *frame() const { return out_; }
    size_t frame_len() const { return out_len_; }
    const Stats &stats() const { return stats_; }

private:
    void drop(size_t n);

    uint8_t buf_[kMaxFrame];
    size_t n_ = 0;
    uint8_t out_[kMaxFrame];
    size_t out_len_ = 0;
    Stats stats_;
};

// True if buf is exactly one valid MTData2 frame (sync, length, checksum), nothing before or after.
bool mtdata2_is_single_frame(const uint8_t *buf, size_t len);
