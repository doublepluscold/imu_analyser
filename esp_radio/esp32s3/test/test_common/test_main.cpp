#include <string.h>
#include <unity.h>

#include <vector>

#include "datagram.h"
#include "id_table.h"
#include "mtdata2_framer.h"

void setUp() {}
void tearDown() {}

// Builds FA FF 36 LEN payload CS. Payload bytes follow the same formula as the
// vectors generated with imuview.netproto: (i*7+3) & 0xFF.
static std::vector<uint8_t> make_frame(size_t n, bool good = true) {
    std::vector<uint8_t> f = {0xFA, 0xFF, 0x36};
    if (n < 0xFF) {
        f.push_back((uint8_t)n);
    } else {
        f.push_back(0xFF);
        f.push_back((uint8_t)(n >> 8));
        f.push_back((uint8_t)n);
    }
    for (size_t i = 0; i < n; i++) f.push_back((uint8_t)(i * 7 + 3));
    uint8_t sum = 0;
    for (size_t i = 1; i < f.size(); i++) sum += f[i];
    f.push_back((uint8_t)(-sum + (good ? 0 : 1)));
    return f;
}

// Feeds bytes, collects every frame the framer yields.
static std::vector<std::vector<uint8_t>> run(Mtdata2Framer &fr, const std::vector<uint8_t> &in) {
    std::vector<std::vector<uint8_t>> out;
    for (uint8_t b : in) {
        if (fr.feed(b)) {
            do {
                out.emplace_back(fr.frame(), fr.frame() + fr.frame_len());
            } while (fr.poll());
        }
    }
    return out;
}

static void append(std::vector<uint8_t> &a, const std::vector<uint8_t> &b) {
    a.insert(a.end(), b.begin(), b.end());
}

// ---------------- framer ----------------

void test_framer_valid_frame() {
    Mtdata2Framer fr;
    auto f = make_frame(89);
    TEST_ASSERT_EQUAL(94, f.size());
    auto out = run(fr, f);
    TEST_ASSERT_EQUAL(1, out.size());
    TEST_ASSERT_TRUE(out[0] == f);
    TEST_ASSERT_EQUAL(1, fr.stats().frames);
}

void test_framer_bad_checksum_resync() {
    Mtdata2Framer fr;
    std::vector<uint8_t> in = make_frame(89, false);
    auto good = make_frame(55);
    append(in, good);
    auto out = run(fr, in);
    TEST_ASSERT_EQUAL(1, out.size());
    TEST_ASSERT_TRUE(out[0] == good);
    TEST_ASSERT_EQUAL(1, fr.stats().bad_checksum);
    TEST_ASSERT_EQUAL(1, fr.stats().frames);
}

void test_framer_bad_frame_with_wrong_len_hides_good_frame() {
    // Bad frame declares a long LEN that swallows the next good frame: the good
    // one must still be found after the bad one fails its checksum.
    Mtdata2Framer fr;
    std::vector<uint8_t> in = {0xFA, 0xFF, 0x36, 60, 1, 2, 3};
    auto good = make_frame(10);
    append(in, good);
    for (int i = 0; i < 80; i++) in.push_back(0x11);  // enough bytes to complete the bogus frame
    auto out = run(fr, in);
    TEST_ASSERT_EQUAL(1, out.size());
    TEST_ASSERT_TRUE(out[0] == good);
}

void test_framer_arbitrary_chunks() {
    auto a = make_frame(89), b = make_frame(55);
    std::vector<uint8_t> in;
    append(in, a);
    append(in, b);
    Mtdata2Framer fr;
    std::vector<std::vector<uint8_t>> out;
    // byte-at-a-time is what the UART loop does; chunking is irrelevant to feed(), but
    // split the stream at every possible point to be sure.
    for (size_t cut = 0; cut <= in.size(); cut++) {
        Mtdata2Framer f2;
        std::vector<uint8_t> p1(in.begin(), in.begin() + cut), p2(in.begin() + cut, in.end());
        auto o1 = run(f2, p1);
        auto o2 = run(f2, p2);
        TEST_ASSERT_EQUAL(2, o1.size() + o2.size());
    }
    out = run(fr, in);
    TEST_ASSERT_EQUAL(2, out.size());
    TEST_ASSERT_TRUE(out[0] == a);
    TEST_ASSERT_TRUE(out[1] == b);
}

void test_framer_two_frames_back_to_back() {
    Mtdata2Framer fr;
    auto a = make_frame(89), b = make_frame(55);
    std::vector<uint8_t> in = a;
    append(in, b);
    auto out = run(fr, in);
    TEST_ASSERT_EQUAL(2, out.size());
    TEST_ASSERT_TRUE(out[0] == a);
    TEST_ASSERT_TRUE(out[1] == b);
}

void test_framer_garbage_between_frames() {
    Mtdata2Framer fr;
    auto a = make_frame(89), b = make_frame(55);
    std::vector<uint8_t> in = {0x00, 0xFA, 0x12, 0xFA, 0xFF, 0x00, 0x55};
    append(in, a);
    for (uint8_t g : {0xFA, 0xFF, 0x13, 0x00, 0xFA}) in.push_back(g);
    append(in, b);
    auto out = run(fr, in);
    TEST_ASSERT_EQUAL(2, out.size());
    TEST_ASSERT_TRUE(out[0] == a);
    TEST_ASSERT_TRUE(out[1] == b);
    TEST_ASSERT_TRUE(fr.stats().skipped_bytes > 0);
}

void test_framer_extended_len() {
    Mtdata2Framer fr;
    auto f = make_frame(200);  // 6 + 200 + 1 = 207 bytes, fits
    std::vector<uint8_t> ext = {0xFA, 0xFF, 0x36, 0xFF, 0x00, 0x05, 1, 2, 3, 4, 5};
    uint8_t sum = 0;
    for (size_t i = 1; i < ext.size(); i++) sum += ext[i];
    ext.push_back((uint8_t)-sum);
    auto out = run(fr, ext);
    TEST_ASSERT_EQUAL(1, out.size());
    TEST_ASSERT_TRUE(out[0] == ext);
    (void)f;
}

void test_framer_frame_longer_than_buffer() {
    Mtdata2Framer fr;
    auto big = make_frame(400);  // extended, 407 bytes > kMaxFrame
    auto good = make_frame(89);
    std::vector<uint8_t> in = big;
    append(in, good);
    auto out = run(fr, in);
    TEST_ASSERT_EQUAL(1, out.size());
    TEST_ASSERT_TRUE(out[0] == good);
    TEST_ASSERT_TRUE(fr.stats().too_long >= 1);
}

void test_single_frame_check() {
    auto f = make_frame(89);
    TEST_ASSERT_TRUE(mtdata2_is_single_frame(f.data(), f.size()));
    TEST_ASSERT_FALSE(mtdata2_is_single_frame(f.data(), f.size() - 1));
    auto bad = make_frame(89, false);
    TEST_ASSERT_FALSE(mtdata2_is_single_frame(bad.data(), bad.size()));
    auto two = f;
    two.insert(two.end(), f.begin(), f.end());
    TEST_ASSERT_FALSE(mtdata2_is_single_frame(two.data(), two.size()));
    auto ext = make_frame(200);
    TEST_ASSERT_TRUE(mtdata2_is_single_frame(ext.data(), ext.size()));
}

// ---------------- datagram ----------------

static std::vector<uint8_t> pattern(size_t n) {
    std::vector<uint8_t> p(n);
    for (size_t i = 0; i < n; i++) p[i] = (uint8_t)(i * 7 + 3);
    return p;
}

void test_datagram_spec_vector() {
    const uint8_t payload[] = {0xFA, 0xFF, 0x36};
    const uint8_t want[] = {0x49, 0x56, 0x01, 0x03, 0x01, 0x00, 0x03, 0x00, 0xFA, 0xFF, 0x36};
    uint8_t out[32];
    size_t n = encode_datagram(3, 1, payload, 3, out, sizeof(out));
    TEST_ASSERT_EQUAL(sizeof(want), n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, out, sizeof(want));
}

// Vectors produced by imuview.netproto.encode_datagram (payload = make_frame formula).
void test_datagram_python_vectors() {
    uint8_t out[1500];

    auto f94 = make_frame(89);
    size_t n = encode_datagram(1, 65535, f94.data(), f94.size(), out, sizeof(out));
    const uint8_t h2[] = {0x49, 0x56, 0x01, 0x01, 0xFF, 0xFF, 0x5E, 0x00};
    TEST_ASSERT_EQUAL(8 + 94, n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(h2, out, 8);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(f94.data(), out + 8, 94);
    TEST_ASSERT_EQUAL_HEX8(0x53, out[8 + 93]);  // last byte of the Python payload

    auto f60 = make_frame(55);
    n = encode_datagram(8, 0, f60.data(), f60.size(), out, sizeof(out));
    const uint8_t h3[] = {0x49, 0x56, 0x01, 0x08, 0x00, 0x00, 0x3C, 0x00};
    TEST_ASSERT_EQUAL(8 + 60, n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(h3, out, 8);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(f60.data(), out + 8, 60);
    TEST_ASSERT_EQUAL_HEX8(0x54, out[8 + 59]);

    const uint8_t want4[] = {0x49, 0x56, 0x01, 0xFF, 0x34, 0x12, 0x00, 0x00};
    n = encode_datagram(255, 0x1234, nullptr, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL(8, n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want4, out, 8);
}

void test_datagram_limits() {
    uint8_t out[8 + 1400];
    auto p = pattern(1401);
    TEST_ASSERT_EQUAL(0, encode_datagram(1, 0, p.data(), 1401, out, sizeof(out)));
    TEST_ASSERT_EQUAL(1408, encode_datagram(1, 0, p.data(), 1400, out, sizeof(out)));
    TEST_ASSERT_EQUAL(0, encode_datagram(1, 0, p.data(), 10, out, 17));  // out too small
}

void test_datagram_seq_wrap() {
    // master keeps seq as uint16_t; 65535 + 1 wraps to 0
    uint16_t seq = 65535;
    uint8_t out[16];
    uint8_t p = 0xAA;
    encode_datagram(1, seq, &p, 1, out, sizeof(out));
    TEST_ASSERT_EQUAL_HEX8(0xFF, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, out[5]);
    seq++;
    encode_datagram(1, seq, &p, 1, out, sizeof(out));
    TEST_ASSERT_EQUAL_HEX8(0x00, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[5]);
}

// ---------------- id_table ----------------

static void mac(uint8_t *m, uint8_t last) {
    const uint8_t base[6] = {0x5C, 0xCF, 0x7F, 0x00, 0x00, last};
    memcpy(m, base, 6);
}

void test_id_order_and_repeat() {
    IdTable t;
    uint8_t a[6], b[6], c[6];
    mac(a, 1);
    mac(b, 2);
    mac(c, 3);
    TEST_ASSERT_EQUAL(1, t.lookup_or_assign(a));
    TEST_ASSERT_EQUAL(2, t.lookup_or_assign(b));
    TEST_ASSERT_EQUAL(1, t.lookup_or_assign(a));
    TEST_ASSERT_EQUAL(3, t.lookup_or_assign(c));
    TEST_ASSERT_EQUAL(2, t.lookup_or_assign(b));
    TEST_ASSERT_EQUAL(3, t.count());
    uint8_t unknown[6] = {9, 9, 9, 9, 9, 9};
    TEST_ASSERT_EQUAL(0, t.find(unknown));
}

void test_id_overflow() {
    IdTable t;
    uint8_t m[6];
    for (uint8_t i = 1; i <= IdTable::kMaxModules; i++) {
        mac(m, i);
        TEST_ASSERT_EQUAL(i, t.lookup_or_assign(m));
    }
    mac(m, 100);
    TEST_ASSERT_EQUAL(0, t.lookup_or_assign(m));
    TEST_ASSERT_EQUAL(IdTable::kMaxModules, t.count());
    mac(m, 3);  // known MAC still works when full
    TEST_ASSERT_EQUAL(3, t.lookup_or_assign(m));
}

void test_id_restore() {
    IdTable t;
    uint8_t a[6], b[6], c[6];
    mac(a, 1);
    mac(b, 2);
    mac(c, 3);
    t.lookup_or_assign(a);
    t.lookup_or_assign(b);
    TEST_ASSERT_TRUE(t.dirty());

    IdTable::Entry saved[IdTable::kMaxModules];
    for (size_t i = 0; i < t.count(); i++) saved[i] = t.entry(i);

    IdTable r;
    TEST_ASSERT_TRUE(r.load(saved, t.count()));
    TEST_ASSERT_FALSE(r.dirty());
    TEST_ASSERT_EQUAL(1, r.lookup_or_assign(a));
    TEST_ASSERT_EQUAL(2, r.lookup_or_assign(b));
    TEST_ASSERT_EQUAL(3, r.lookup_or_assign(c));  // continues after restored ids
}

void test_id_restore_rejects_bad_state() {
    IdTable t;
    IdTable::Entry e[2];
    mac(e[0].mac, 1);
    mac(e[1].mac, 2);
    e[0].id = 1;
    e[1].id = 1;  // duplicate id
    TEST_ASSERT_FALSE(t.load(e, 2));
    TEST_ASSERT_EQUAL(0, t.count());
    e[1].id = 0;  // id 0 reserved
    TEST_ASSERT_FALSE(t.load(e, 2));
    e[1].id = 2;
    mac(e[1].mac, 1);  // duplicate mac
    TEST_ASSERT_FALSE(t.load(e, 2));
    e[0].id = 9;  // out of range
    mac(e[1].mac, 2);
    TEST_ASSERT_FALSE(t.load(e, 2));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_framer_valid_frame);
    RUN_TEST(test_framer_bad_checksum_resync);
    RUN_TEST(test_framer_bad_frame_with_wrong_len_hides_good_frame);
    RUN_TEST(test_framer_arbitrary_chunks);
    RUN_TEST(test_framer_two_frames_back_to_back);
    RUN_TEST(test_framer_garbage_between_frames);
    RUN_TEST(test_framer_extended_len);
    RUN_TEST(test_framer_frame_longer_than_buffer);
    RUN_TEST(test_single_frame_check);
    RUN_TEST(test_datagram_spec_vector);
    RUN_TEST(test_datagram_python_vectors);
    RUN_TEST(test_datagram_limits);
    RUN_TEST(test_datagram_seq_wrap);
    RUN_TEST(test_id_order_and_repeat);
    RUN_TEST(test_id_overflow);
    RUN_TEST(test_id_restore);
    RUN_TEST(test_id_restore_rejects_bad_state);
    return UNITY_END();
}
