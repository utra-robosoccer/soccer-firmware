/* Tests for the resynchronizing frame scanner proto_frame_scan() (protocol.h),
 * driven by host/tests/test_proto_frame.py.
 *
 * `Framer` mirrors the master's MotorMaster_ProcessUsbRx drain loop exactly (scan
 * + slide + resync counting), minus dispatch, so these scenarios pin the real
 * recovery behavior: clean back-to-back frames, byte-by-byte split delivery,
 * leading junk (the observed 14-zero-prefix bug), bad-CRC, truncated/dropped
 * frames, and wrong-version frames — each must recover on the next valid frame.
 *
 * Built: gcc -std=c11 -Wall -Wextra -Werror -I firmware/common/include ... */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "protocol.h"

typedef struct {
    uint8_t  buf[768];
    uint16_t len;
    uint8_t  in_resync;
    int      n_frames;
    uint8_t  types[128];
    uint32_t resyncs;
    uint32_t discarded;
} Framer;

/* Identical to MotorMaster_ProcessUsbRx's scan/slide/resync accounting. */
static void framer_feed(Framer *f, const uint8_t *data, uint16_t n)
{
    memcpy(f->buf + f->len, data, n);
    f->len = (uint16_t)(f->len + n);

    uint16_t off = 0;
    while ((uint16_t)(f->len - off) >= MSG_HEADER_SIZE) {
        ProtoScanResult r;
        ProtoScanStatus s = proto_frame_scan(f->buf + off, (uint16_t)(f->len - off), &r);
        if (s == PROTO_SCAN_NEED_MORE) break;
        if (s == PROTO_SCAN_RESYNC) {
            if (!f->in_resync) { f->resyncs++; f->in_resync = 1u; }
            f->discarded += r.consumed;
            off = (uint16_t)(off + r.consumed);
            continue;
        }
        f->in_resync = 0u;
        if (f->n_frames < (int)sizeof(f->types)) f->types[f->n_frames] = r.type;
        f->n_frames++;
        off = (uint16_t)(off + r.consumed);
    }
    if (off > 0u) { f->len = (uint16_t)(f->len - off); memmove(f->buf, f->buf + off, f->len); }
}

static Framer FR;
static void reset(void) { memset(&FR, 0, sizeof(FR)); }

/* Build a valid frame of `type` with `pay_len` payload bytes (pattern). */
static uint16_t make_frame(uint8_t *out, uint8_t type, uint16_t pay_len)
{
    static uint8_t pay[PROTO_MAX_PAYLOAD];
    for (uint16_t i = 0; i < pay_len; i++) pay[i] = (uint8_t)(0xA0 + (i & 0x0F));
    return proto_build(out, 1024, type, 0x1234, NODE_JETSON, NODE_MASTER,
                       0x00ABCDEF, pay_len ? pay : NULL, pay_len);
}

static int fails = 0;
#define CHECK(cond, msg) do { if (!(cond)) { fprintf(stderr, "FAIL: %s\n", msg); fails++; } } while (0)

int main(void)
{
    uint8_t fr_cmd[1024], fr_ping[64];
    uint16_t nc = make_frame(fr_cmd, MSG_ROBOT_CMD, (uint16_t)sizeof(cmd_robot_t));
    uint16_t np = make_frame(fr_ping, MSG_PING, 0);

    /* 1) two clean frames in one feed → both recovered, no resync. */
    reset();
    framer_feed(&FR, fr_cmd, nc);
    framer_feed(&FR, fr_ping, np);
    CHECK(FR.n_frames == 2 && FR.types[0] == MSG_ROBOT_CMD && FR.types[1] == MSG_PING, "clean back-to-back");
    CHECK(FR.resyncs == 0 && FR.discarded == 0, "clean: no resync");

    /* 2) a frame delivered one byte per feed → recovered exactly when complete. */
    reset();
    for (uint16_t i = 0; i < nc; i++) {
        framer_feed(&FR, &fr_cmd[i], 1);
        CHECK(FR.n_frames == (i == nc - 1 ? 1 : 0), "split: recover only when complete");
    }
    CHECK(FR.resyncs == 0, "split: no resync");

    /* 3) 14 leading zero bytes then a frame (the observed desync) → recovers,
          discards exactly 14 in one resync run. */
    reset();
    { uint8_t z[14] = {0}; framer_feed(&FR, z, 14); }
    framer_feed(&FR, fr_cmd, nc);
    CHECK(FR.n_frames == 1 && FR.types[0] == MSG_ROBOT_CMD, "leading junk: recovered");
    CHECK(FR.resyncs == 1 && FR.discarded == 14, "leading junk: 14 discarded, 1 run");

    /* 4) bad-CRC frame then a good frame → good one recovered after a resync. */
    reset();
    { uint8_t bad[1024]; memcpy(bad, fr_cmd, nc); bad[nc - 1] ^= 0xFF;  /* corrupt last byte */
      framer_feed(&FR, bad, nc); }
    framer_feed(&FR, fr_ping, np);
    CHECK(FR.n_frames >= 1 && FR.types[FR.n_frames - 1] == MSG_PING, "bad-CRC then good recovered");
    CHECK(FR.resyncs >= 1, "bad-CRC: resynced");

    /* 5) truncated frame (last 10 bytes dropped) then a good frame → recovers. */
    reset();
    framer_feed(&FR, fr_cmd, (uint16_t)(nc - 10));   /* missing tail */
    framer_feed(&FR, fr_ping, np);
    CHECK(FR.n_frames >= 1 && FR.types[FR.n_frames - 1] == MSG_PING, "truncated then good recovered");

    /* 6) wrong-version frame then a good frame → wrong version is rejected by the
          header-first check (cheap resync), good one recovered. */
    reset();
    { uint8_t wv[1024]; memcpy(wv, fr_cmd, nc); wv[12] = (uint8_t)(PROTO_VERSION + 1);
      framer_feed(&FR, wv, nc); }
    framer_feed(&FR, fr_ping, np);
    CHECK(FR.n_frames >= 1 && FR.types[FR.n_frames - 1] == MSG_PING, "wrong-version then good recovered");
    CHECK(FR.resyncs >= 1, "wrong-version: resynced");

    if (fails) { fprintf(stderr, "%d checks failed\n", fails); return 1; }
    printf("OK 6 scenarios\n");
    return 0;
}
