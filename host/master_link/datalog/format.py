"""On-disk binary log format.

File = header, then a stream of records.

Header:
    magic            4s   b"RLOG"
    log_fmt_version  u16
    proto_version    u16  (protocol.PROTO_VERSION at write time)
    wall_start_ns    u64  time.time_ns()      — wall clock at start
    mono_start_ns    u64  time.monotonic_ns() — record timestamps share this clock
    meta_len         u32
    meta             meta_len bytes of UTF-8 JSON (git, config name/hash/yaml, producer)

Record (repeated):
    kind    u8
    mono_ns u64   time.monotonic_ns() when the record was produced
    length  u32
    payload length bytes

Record kinds and their payloads:
    RX_FRAME    raw bytes of one received frame (header+payload, as read off the wire)
    TX_FRAME    raw bytes of a frame we successfully wrote
    RX_DISCARD  bytes dropped during resync / CRC failure (coalesced per read)
    EVENT       UTF-8 text: "start" | "stop" | "arm" | "disarm" | "error:<msg>" ...
    ANNOTATION  UTF-8 free text
    LOOP_TIMING packed LOOP_TIMING_FMT (see below)
"""
import struct

MAGIC = b"RLOG"
LOG_FMT_VERSION = 1

# ── record kinds ────────────────────────────────────────────────────────────
RX_FRAME    = 1
TX_FRAME    = 2
RX_DISCARD  = 3
EVENT       = 4
ANNOTATION  = 5
LOOP_TIMING = 6

KIND_NAMES = {
    RX_FRAME: "RX_FRAME", TX_FRAME: "TX_FRAME", RX_DISCARD: "RX_DISCARD",
    EVENT: "EVENT", ANNOTATION: "ANNOTATION", LOOP_TIMING: "LOOP_TIMING",
}

# ── struct layouts ──────────────────────────────────────────────────────────
_HEADER_FIXED = struct.Struct("<4sHHQQI")   # magic, fmt_ver, proto_ver, wall, mono, meta_len
_REC_HEADER   = struct.Struct("<BQI")       # kind, mono_ns, length

# LOOP_TIMING payload: seq (u64) + period/step/send/lateness (i64 ns).
LOOP_TIMING_FMT = struct.Struct("<Qqqqq")


def pack_header(proto_version: int, wall_start_ns: int, mono_start_ns: int,
                meta_bytes: bytes) -> bytes:
    return _HEADER_FIXED.pack(MAGIC, LOG_FMT_VERSION, proto_version,
                              wall_start_ns, mono_start_ns, len(meta_bytes)) + meta_bytes


def pack_record(kind: int, mono_ns: int, payload: bytes) -> bytes:
    return _REC_HEADER.pack(kind, mono_ns, len(payload)) + payload
