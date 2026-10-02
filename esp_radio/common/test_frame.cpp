#include "test_frame.h"

#include <math.h>
#include <string.h>

static void put_f32(uint8_t *p, float v) {  // MTData2 floats are big-endian
    uint32_t u;
    memcpy(&u, &v, 4);
    p[0] = (uint8_t)(u >> 24);
    p[1] = (uint8_t)(u >> 16);
    p[2] = (uint8_t)(u >> 8);
    p[3] = (uint8_t)u;
}

static size_t put_block(uint8_t *p, uint16_t xdi, const float *v, size_t nf) {
    p[0] = (uint8_t)(xdi >> 8);
    p[1] = (uint8_t)xdi;
    p[2] = (uint8_t)(nf * 4);
    for (size_t i = 0; i < nf; i++) put_f32(p + 3 + 4 * i, v[i]);
    return 3 + 4 * nf;
}

size_t build_test_frame(uint32_t n, uint8_t *out, size_t cap) {
    if (cap < kTestFrameSize) return 0;
    const float t = (float)n / 20.0f;  // seconds
    float yaw = fmodf(18.0f * t, 360.0f) - 180.0f;
    const float euler[3] = {20.0f * sinf(0.8f * t), 15.0f * sinf(0.5f * t + 1.0f), yaw};
    const float acc[3] = {0.0f, 0.0f, 9.81f};
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    const float latlon[2] = {0.0f, 0.0f};
    const float alt = 0.0f;
    const float vel[3] = {0.0f, 0.0f, 0.0f};
    const float x2060 = 0.0f;

    out[0] = 0xFA;
    out[1] = 0xFF;
    out[2] = 0x36;
    out[3] = 0x59;
    size_t i = 4;
    i += put_block(out + i, 0x2030, euler, 3);
    i += put_block(out + i, 0x4020, acc, 3);
    i += put_block(out + i, 0x8020, gyro, 3);
    out[i++] = 0xE0;  // StatusByte: filter valid, no GNSS fix (bit 2 stays 0)
    out[i++] = 0x10;
    out[i++] = 0x01;
    out[i++] = 0x02;
    i += put_block(out + i, 0x5040, latlon, 2);
    i += put_block(out + i, 0x5020, &alt, 1);
    i += put_block(out + i, 0xD010, vel, 3);
    i += put_block(out + i, 0x2060, &x2060, 1);
    uint8_t sum = 0;  // checksum: BID .. last payload byte, negated
    for (size_t k = 1; k < i; k++) sum += out[k];
    out[i++] = (uint8_t)(-sum);
    return i;  // 94
}
