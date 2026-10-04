"""BinaryLogReader — iterate records out of a binary session log.

    hdr = read_header(open(path, "rb"))
    for rec in BinaryLogReader(path):
        rec.kind, rec.ts_ns, rec.payload
"""
import json
from dataclasses import dataclass

from . import format as fmt


@dataclass(frozen=True)
class Record:
    kind: int
    ts_ns: int          # time.monotonic_ns() at production
    payload: bytes


def read_header(f) -> dict:
    """Read and validate the header from an open binary file positioned at 0.
    Leaves the file positioned at the first record. Returns a dict with fixed
    fields plus the parsed JSON metadata under 'meta'."""
    fixed = f.read(fmt._HEADER_FIXED.size)
    if len(fixed) < fmt._HEADER_FIXED.size:
        raise ValueError("log too short for header")
    magic, log_ver, proto_ver, wall_ns, mono_ns, meta_len = fmt._HEADER_FIXED.unpack(fixed)
    if magic != fmt.MAGIC:
        raise ValueError(f"bad magic {magic!r} (not a RLOG file)")
    meta_bytes = f.read(meta_len)
    if len(meta_bytes) < meta_len:
        raise ValueError("log truncated in header metadata")
    return {
        "log_fmt_version": log_ver,
        "proto_version": proto_ver,
        "wall_start_ns": wall_ns,
        "mono_start_ns": mono_ns,
        "meta": json.loads(meta_bytes.decode("utf-8")) if meta_len else {},
    }


class BinaryLogReader:
    def __init__(self, path: str):
        self.path = path
        with open(path, "rb") as f:
            self.header = read_header(f)

    def __iter__(self):
        with open(self.path, "rb") as f:
            read_header(f)  # advance past header
            hsz = fmt._REC_HEADER.size
            while True:
                rh = f.read(hsz)
                if not rh:
                    return
                if len(rh) < hsz:
                    raise ValueError("log truncated in record header")
                kind, ts_ns, length = fmt._REC_HEADER.unpack(rh)
                payload = f.read(length)
                if len(payload) < length:
                    raise ValueError("log truncated in record payload")
                yield Record(kind, ts_ns, payload)
