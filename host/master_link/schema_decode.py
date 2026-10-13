"""Decode wire frames using a protocol.yaml embedded in a session log.

Every binary log written since v10 carries the FULL schema text (and its
SCHEMA_HASH) in the RLOG header metadata, so a log stays decodable after the
wire protocol moves on. ``SchemaDecoder`` builds, from that embedded text, the
exact decode surface ``host/analysis/convert_log.py`` needs — the same names
the live ``master_link.protocol`` module provides — so the converter can run
off either interchangeably:

    HDR_FMT / HDR_SIZE / PROTO_VERSION / MSG_* / DRIVE_REQ_NAMES
    parse_robot_tele()  (motors as attribute objects, scaled + named fields)
    parse_robot_cmd() / parse_master_status() / parse_slave_status()

Only layout, enums, flags and scales come from the schema; framing (CRC,
header discipline) is the stable hand-written contract. Missing fields or
enums in an older schema simply yield blank CSV cells — the decoder never
guesses.
"""
from __future__ import annotations

import struct
from types import SimpleNamespace

import yaml

# Wire scalar types: name -> struct format char (little-endian).
_TYPES = {"u8": "B", "u16": "H", "u32": "I", "i16": "h"}

# Display-name prefixes, mirroring master_link.protocol's stripped dicts.
_ENUM_PREFIX = {
    "MsgType": "MSG_", "RobotState": "ROBOT_", "DriveReq": "DRV_REQ_",
    "DriveState": "DRV_", "DriveFault": "DRVF_", "ChainLink": "CHAIN_LINK_",
    "NodeId": "NODE_",
}

# RS vendor Type-2 run modes — vendor constants, never in the schema.
_MOTOR_MODE_NAMES = {0: "RESET", 1: "CAL", 2: "NORMAL"}


class SchemaDecoder:
    def __init__(self, yaml_text: str):
        s = yaml.safe_load(yaml_text)
        self.PROTO_VERSION = int(s["meta"]["proto_version"])
        self._consts = {c["name"]: int(c["value"]) for c in s.get("constants", [])}
        self._scales = {sc["name"]: float(sc["value"]) for sc in s.get("scales", [])}

        # Enums: constant per value name + {value: stripped-name} display dicts.
        self._enum_names: dict[str, dict[int, str]] = {}
        for e in s.get("enums", []):
            pre = _ENUM_PREFIX.get(e["name"], "")
            names = {}
            for v in e["values"]:
                setattr(self, v["name"], int(v["value"]))
                names[int(v["value"])] = (v["name"][len(pre):]
                                          if v["name"].startswith(pre) else v["name"])
            self._enum_names[e["name"]] = names
        self.DRIVE_REQ_NAMES = self._enum_names.get("DriveReq", {})

        # Flag bits by name (for request_rejected / saturated derivations).
        self._flag_bits: dict[str, int] = {}
        for g in s.get("flags", []):
            for b in g.get("bits", []):
                if not b.get("reserved"):
                    self._flag_bits[b["name"]] = 1 << int(b["bit"])

        # Structs: ordered field tables with resolved counts + sizes.
        self._structs: dict[str, dict] = {}
        for st in s.get("structs", []):
            fields, off = [], 0
            for fld in st["fields"]:
                t = fld["type"]
                count = fld.get("count")
                if isinstance(count, str):
                    count = self._consts[count]
                if t in _TYPES:
                    size = struct.calcsize("<" + _TYPES[t]) * (count or 1)
                else:
                    size = self._structs[t]["size"] * (count or 1)
                fields.append(dict(name=fld["name"], type=t, count=count,
                                   scale=fld.get("scale"), enum=fld.get("enum"),
                                   truncate_by=fld.get("truncate_by"), offset=off))
                off += size
            self._structs[st["name"]] = dict(fields=fields, size=off)

        hdr = self._structs["MsgHeader"]
        self.HDR_SIZE = hdr["size"]
        self.HDR_FMT = "<" + "".join(_TYPES[f["type"]] for f in hdr["fields"])

        # Message-type ids convert_log dispatches on; None when the schema
        # predates one (comparisons against None never match).
        for name in ("MSG_PING", "MSG_MASTER_STATUS", "MSG_SLAVE_STATUS",
                     "MSG_ROBOT_CMD", "MSG_ROBOT_TELE"):
            if not hasattr(self, name):
                setattr(self, name, None)

    # ── generic unpack ────────────────────────────────────────────────────────
    def unpack(self, name: str, data: bytes, off: int = 0,
               truncated: bool = False) -> dict:
        """One struct → dict (struct arrays → lists of dicts). With
        ``truncated``, a truncate_by array reads only the count named by its
        earlier field (the transmitted populated prefix)."""
        out: dict = {}
        st = self._structs[name]
        for f in st["fields"]:
            t, count = f["type"], f["count"]
            if t in _TYPES:
                ch = _TYPES[t]
                fmt = "<" + ((str(count) + ch) if count else ch)
                if off + struct.calcsize(fmt) > len(data):
                    return out                     # short/older payload: stop
                vals = struct.unpack_from(fmt, data, off)
                out[f["name"]] = vals[0] if len(vals) == 1 else list(vals)
                off += struct.calcsize(fmt)
            else:
                n = count or 1
                if truncated and f["truncate_by"]:
                    n = min(int(out.get(f["truncate_by"], 0)), n)
                sz = self._structs[t]["size"]
                elems = []
                for i in range(n):
                    if off + sz > len(data):
                        break
                    elems.append(self.unpack(t, data, off))
                    off += sz
                out[f["name"]] = elems
        return out

    def _scaled(self, d: dict, struct_name: str) -> dict:
        """Add engineering-unit values for scale-annotated fields (in place)."""
        for f in self._structs[struct_name]["fields"]:
            if f["scale"] and f["name"] in d:
                d[f["name"]] = d[f["name"]] / self._scales[f["scale"]]
        return d

    def _ename(self, enum: str, value) -> str:
        return self._enum_names.get(enum, {}).get(value, f"?{value}")

    # ── the convert_log decode surface ────────────────────────────────────────
    def parse_robot_tele(self, p: bytes) -> dict:
        if "tele_robot_t" not in self._structs:
            return {}
        d = self.unpack("tele_robot_t", p, truncated=True)
        if not d:
            return {}
        rr = self._flag_bits.get("TELE_FLAG_REQUEST_REJECTED", 0)
        sat = self._flag_bits.get("TELE_FLAG_SATURATED", 0)
        for ch in d.get("chains", []):
            motors = []
            # n_motors counts the POPULATED slots (the wire array is fixed-size).
            for m in ch.get("motors", [])[:int(ch.get("n_motors", 0))]:
                self._scaled(m, "tele_motor_t")
                flags = m.get("flags", 0)
                attrs = dict(m)
                attrs.setdefault("fb_age_ms", "")
                attrs["temp"] = float(m.get("temp_c", 0))
                attrs["drive_state_name"] = self._ename("DriveState",
                                                        m.get("drive_state"))
                attrs["drive_fault_name"] = self._ename("DriveFault",
                                                        m.get("drive_fault"))
                attrs["rs_mode_name"] = _MOTOR_MODE_NAMES.get(
                    m.get("rs_mode"), f"?{m.get('rs_mode')}")
                attrs["request_rejected"] = bool(flags & rr)
                attrs["saturated"] = bool(flags & sat)
                motors.append(SimpleNamespace(**attrs))
            ch["motors"] = motors
        return d

    def parse_robot_cmd(self, p: bytes) -> dict:
        if "cmd_robot_t" not in self._structs:
            return {}
        d = self.unpack("cmd_robot_t", p)
        # Full-size on the wire; n_chains / n_motors count the populated slots.
        d["chains"] = d.get("chains", [])[:int(d.get("n_chains", 0))]
        for ch in d["chains"]:
            ch["motors"] = [self._scaled(m, "cmd_motor_t")
                            for m in ch.get("motors", [])[:int(ch.get("n_motors", 0))]]
        return d

    def parse_master_status(self, p: bytes) -> dict:
        return (self.unpack("MasterStatus", p)
                if "MasterStatus" in self._structs else {})

    def parse_slave_status(self, p: bytes) -> dict:
        return (self.unpack("SlaveStatus", p)
                if "SlaveStatus" in self._structs else {})
