#pragma once
// A synthetic but valid MTData2 IMU frame for the radio self-test (slave TEST_BEACON mode).
// Same block layout as the real module (94 bytes, LEN = 0x59): Euler, Acc, Gyro, Status, LatLon,
// AltitudeEllipsoid, Velocity, 0x2060. The model slowly rolls, pitches and turns, so a GUI that
// receives it shows a moving model without any IMU. Plain C++, no Arduino/ESP-IDF dependencies.
#include <stddef.h>
#include <stdint.h>

constexpr size_t kTestFrameSize = 94;

// n = frame counter (the motion follows it at 20 frames/s). Returns 94, or 0 if cap is too small.
size_t build_test_frame(uint32_t n, uint8_t *out, size_t cap);
