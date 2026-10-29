#!/usr/bin/env python3
"""Generate wire-protocol code from firmware/common/protocol.yaml.

The schema is the single source of truth for everything on the wire (structs,
enums, flag bits, fixed-point scales, constants). Emitters:

  c       → firmware/common/include/protocol_gen.h   (packed structs, enums,
            _Static_assert on every sizeof and field offsetof)
  python  → host/master_link/_protocol_gen.py        (step 2)
  cpp     → generated/cpp/robot_wire_gen.hpp          (step 3)

Generated files are committed (the firmware build does not run Python).

SCHEMA_HASH — CRC-32 over the schema's canonical WIRE PROJECTION: the parsed
document with every `doc`, `unit` and `hex` key removed, serialized as compact
JSON (sorted object keys; list order — i.e. field order — preserved), UTF-8.
Comment/doc-only edits therefore do NOT change the hash; names, field order,
types, counts, truncate_by, enum/flag values, scales, constants and derived
expressions DO.

Dependencies: Python stdlib + PyYAML.

Usage:
    python3 scripts/gen_protocol.py            # regenerate all implemented emitters
    python3 scripts/gen_protocol.py --emit c   # one emitter
"""

import argparse
import json
import os
import re
import sys
import zlib

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCHEMA_PATH = os.path.join(REPO_ROOT, "firmware", "common", "protocol.yaml")

OUT_C = os.path.join(REPO_ROOT, "firmware", "common", "include", "protocol_gen.h")
OUT_PY = os.path.join(REPO_ROOT, "host", "master_link", "_protocol_gen.py")
OUT_CPP = os.path.join(REPO_ROOT, "generated", "cpp", "robot_wire_gen.hpp")
OUT_FIXTURES = os.path.join(REPO_ROOT, "generated", "cpp", "fixtures")

# Scalar wire types: name -> (size, C type, python struct-format char)
WIRE_TYPES = {
    "u8":  (1, "uint8_t",  "B"),
    "u16": (2, "uint16_t", "H"),
    "u32": (4, "uint32_t", "I"),
    "i16": (2, "int16_t",  "h"),
}

NAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


class SchemaError(Exception):
    pass


def fail(msg):
    raise SchemaError(msg)


# ── loading + validation ──────────────────────────────────────────────────────
# PyYAML is YAML 1.1 (no/on/off parse as booleans, unquoted values can change
# type), so every value is type-checked here instead of trusting the parse.

def _is_int(v):
    return type(v) is int          # bool is an int subclass — rejected on purpose


def _check_name(v, what):
    if not isinstance(v, str) or not NAME_RE.match(v):
        fail(f"{what}: {v!r} is not a valid identifier")
    return v


def load_schema(path=SCHEMA_PATH):
    with open(path, encoding="utf-8") as f:
        s = yaml.safe_load(f)

    for section in ("meta", "constants", "enums", "flags", "scales",
                    "structs", "derived"):
        if section not in s:
            fail(f"schema missing section {section!r}")

    if not _is_int(s["meta"].get("proto_version")):
        fail("meta.proto_version must be an integer")
    if s["meta"].get("endianness") != "little":
        fail("meta.endianness must be 'little'")

    names_seen = {}

    def claim(name, what):
        _check_name(name, what)
        if name in names_seen:
            fail(f"duplicate name {name!r} ({what}; already a {names_seen[name]})")
        names_seen[name] = what

    consts = {}
    for c in s["constants"]:
        claim(c["name"], "constant")
        if not _is_int(c["value"]):
            fail(f"constant {c['name']}: value {c['value']!r} is not an integer")
        consts[c["name"]] = c["value"]

    for e in s["enums"]:
        claim(e["name"], "enum")
        wt = e.get("wire_type")
        if wt not in WIRE_TYPES:
            fail(f"enum {e['name']}: unknown wire_type {wt!r}")
        size = WIRE_TYPES[wt][0]
        vmax = (1 << (8 * size)) - 1
        seen_vals = set()
        for v in e["values"]:
            claim(v["name"], f"enum value ({e['name']})")
            if not _is_int(v["value"]):
                fail(f"enum value {v['name']}: {v['value']!r} is not an integer")
            if not (0 <= v["value"] <= vmax):
                fail(f"enum value {v['name']} = {v['value']} does not fit {wt}")
            if v["value"] in seen_vals:
                fail(f"enum {e['name']}: duplicate value {v['value']}")
            seen_vals.add(v["value"])

    flag_groups = set()
    for g in s["flags"]:
        claim(g["name"], "flag group")
        flag_groups.add(g["name"])
        bits_seen = set()
        for b in g["bits"]:
            if not _is_int(b["bit"]) or not (0 <= b["bit"] <= 31):
                fail(f"flag group {g['name']}: bad bit {b.get('bit')!r}")
            if b["bit"] in bits_seen:
                fail(f"flag group {g['name']}: overlapping bit {b['bit']}")
            bits_seen.add(b["bit"])
            if b.get("reserved"):
                if "name" in b:
                    fail(f"flag group {g['name']} bit {b['bit']}: reserved bits "
                         f"carry no name")
            else:
                claim(b["name"], f"flag ({g['name']})")

    for sc in s["scales"]:
        claim(sc["name"], "scale")
        if type(sc["value"]) not in (int, float) or type(sc["value"]) is bool:
            fail(f"scale {sc['name']}: value {sc['value']!r} is not a number")

    enum_names = {e["name"] for e in s["enums"]}
    struct_names = []
    for st in s["structs"]:
        claim(st["name"], "struct")
        field_names = set()
        truncated = None
        for fld in st["fields"]:
            _check_name(fld["name"], f"field of {st['name']}")
            if fld["name"] in field_names:
                fail(f"struct {st['name']}: duplicate field {fld['name']!r}")
            field_names.add(fld["name"])
            if truncated is not None:
                fail(f"struct {st['name']}: field {fld['name']!r} follows the "
                     f"truncate_by array {truncated!r} — it would be silently cut "
                     f"off when the populated prefix is transmitted")
            t = fld["type"]
            if t not in WIRE_TYPES and t not in struct_names:
                fail(f"struct {st['name']}.{fld['name']}: unknown type {t!r} "
                     f"(wire types or a PREVIOUSLY defined struct)")
            if "count" in fld:
                cnt = fld["count"]
                if isinstance(cnt, str):
                    if cnt not in consts:
                        fail(f"struct {st['name']}.{fld['name']}: count {cnt!r} "
                             f"is not a defined constant")
                elif not _is_int(cnt):
                    fail(f"struct {st['name']}.{fld['name']}: count must be an "
                         f"integer or a constant name")
            if "truncate_by" in fld:
                if "count" not in fld:
                    fail(f"struct {st['name']}.{fld['name']}: truncate_by on a "
                         f"non-array field")
                if fld["truncate_by"] not in field_names:
                    fail(f"struct {st['name']}.{fld['name']}: truncate_by "
                         f"{fld['truncate_by']!r} is not an earlier field")
                truncated = fld["name"]
            if "enum" in fld and fld["enum"] not in enum_names:
                fail(f"struct {st['name']}.{fld['name']}: unknown enum "
                     f"{fld['enum']!r}")
            if "flags" in fld and fld["flags"] not in flag_groups:
                fail(f"struct {st['name']}.{fld['name']}: unknown flag group "
                     f"{fld['flags']!r}")
        struct_names.append(st["name"])

    for d in s["derived"]:
        claim(d["name"], "derived constant")
        if not isinstance(d.get("expr"), str):
            fail(f"derived {d['name']}: expr must be a string")
        if "cast" in d and d["cast"] not in WIRE_TYPES:
            fail(f"derived {d['name']}: unknown cast {d['cast']!r}")

    return s


# ── wire projection + SCHEMA_HASH ────────────────────────────────────────────

_NON_WIRE_KEYS = frozenset(("doc", "unit", "hex"))


def wire_projection(node):
    """The schema minus free-text/display keys — everything that changes wire
    bytes or their meaning, nothing that doesn't."""
    if isinstance(node, dict):
        return {k: wire_projection(v) for k, v in node.items()
                if k not in _NON_WIRE_KEYS}
    if isinstance(node, list):
        return [wire_projection(v) for v in node]
    return node


def schema_hash(schema):
    canon = json.dumps(wire_projection(schema), separators=(",", ":"),
                       sort_keys=True, ensure_ascii=False)
    return zlib.crc32(canon.encode("utf-8")) & 0xFFFFFFFF


# ── layout ───────────────────────────────────────────────────────────────────

def resolve_count(fld, consts):
    cnt = fld.get("count")
    if cnt is None:
        return None
    return consts[cnt] if isinstance(cnt, str) else cnt


def compute_layout(schema):
    """Per-struct {size, fields:[(name, offset, size, count, elem_type)]} for
    the packed little-endian layout."""
    consts = {c["name"]: c["value"] for c in schema["constants"]}
    layout = {}
    for st in schema["structs"]:
        off = 0
        fields = []
        for fld in st["fields"]:
            t = fld["type"]
            elem = WIRE_TYPES[t][0] if t in WIRE_TYPES else layout[t]["size"]
            count = resolve_count(fld, consts)
            fsize = elem * (count or 1)
            fields.append(dict(name=fld["name"], offset=off, size=fsize,
                               count=count, type=t))
            off += fsize
        layout[st["name"]] = dict(size=off, fields=fields)
    return layout


# ── derived-expression evaluator (sizeof, +, max; no eval) ───────────────────

_TOKEN_RE = re.compile(r"\s*(sizeof|max|[A-Za-z_][A-Za-z0-9_]*|\d+|[(),+])")


def eval_derived(expr, consts, layout, derived_vals):
    pos = 0
    toks = []
    while pos < len(expr):
        m = _TOKEN_RE.match(expr, pos)
        if not m:
            fail(f"derived expr {expr!r}: bad token at {expr[pos:]!r}")
        toks.append(m.group(1))
        pos = m.end()
    toks.append(None)   # end marker
    i = [0]

    def peek():
        return toks[i[0]]

    def take(expected=None):
        t = toks[i[0]]
        if expected is not None and t != expected:
            fail(f"derived expr {expr!r}: expected {expected!r}, got {t!r}")
        i[0] += 1
        return t

    def atom():
        t = take()
        if t == "sizeof":
            take("(")
            name = take()
            take(")")
            if name not in layout:
                fail(f"derived expr {expr!r}: sizeof of unknown struct {name!r}")
            return layout[name]["size"]
        if t == "max":
            take("(")
            a = expr_sum()
            take(",")
            b = expr_sum()
            take(")")
            return max(a, b)
        if t == "(":
            v = expr_sum()
            take(")")
            return v
        if t is not None and t.isdigit():
            return int(t)
        if t in consts:
            return consts[t]
        if t in derived_vals:
            return derived_vals[t]
        fail(f"derived expr {expr!r}: unknown name {t!r}")

    def expr_sum():
        v = atom()
        while peek() == "+":
            take("+")
            v += atom()
        return v

    v = expr_sum()
    if peek() is not None:
        fail(f"derived expr {expr!r}: trailing tokens")
    return v


def compute_derived(schema, layout):
    consts = {c["name"]: c["value"] for c in schema["constants"]}
    out = {}
    for d in schema["derived"]:
        out[d["name"]] = eval_derived(d["expr"], consts, layout, out)
    # Transport budgets — fail generation, don't wait for the compiler.
    if "USB_MAX_FRAME_SIZE" in out:
        if "USB_TX_SLOT_MAX" not in consts:
            fail("USB_MAX_FRAME_SIZE defined but USB_TX_SLOT_MAX is not")
        if out["USB_MAX_FRAME_SIZE"] > consts["USB_TX_SLOT_MAX"]:
            fail(f"largest USB frame ({out['USB_MAX_FRAME_SIZE']} B) exceeds "
                 f"USB_TX_SLOT_MAX ({consts['USB_TX_SLOT_MAX']} B) — grow the "
                 f"slot or shrink the frame")
    return out


# ── shared formatting helpers ────────────────────────────────────────────────

def fmt_int_c(value, hexa=False):
    if hexa:
        return f"0x{value:02X}u" if value <= 0xFF else f"0x{value:04X}u"
    return f"{value}u"


def fmt_float_c(v):
    v = float(v)
    return f"{int(v)}.0f" if v == int(v) else f"{v}f"


def wrap_comment(text, width=74):
    words = str(text).split()
    lines, cur = [], ""
    for w in words:
        if cur and len(cur) + 1 + len(w) > width:
            lines.append(cur)
            cur = w
        else:
            cur = f"{cur} {w}" if cur else w
    if cur:
        lines.append(cur)
    return lines


def block_comment(text, indent=""):
    lines = wrap_comment(text)
    if not lines:
        return []
    return ([f"{indent}/* {lines[0]}" + (" */" if len(lines) == 1 else "")]
            + [f"{indent} * {l}" for l in lines[1:-1]]
            + ([f"{indent} * {lines[-1]} */"] if len(lines) > 1 else []))


# ── C emitter ────────────────────────────────────────────────────────────────

def emit_c(schema, layout, derived, shash):
    L = []
    o = L.append
    o("/* AUTO-GENERATED — DO NOT EDIT.")
    o(" * Source:      firmware/common/protocol.yaml")
    o(" * Regenerate:  python3 scripts/gen_protocol.py")
    o(" *")
    o(f" * SCHEMA_HASH 0x{shash:08X}: CRC-32 of the schema's canonical wire")
    o(" * projection (parsed YAML minus doc/unit/hex keys, compact sorted-key")
    o(" * JSON, UTF-8) — see gen_protocol.py. Doc-only edits do not change it.")
    o(" */")
    o("#ifndef PROTOCOL_GEN_H")
    o("#define PROTOCOL_GEN_H")
    o("")
    o("#include <stdint.h>")
    o("#include <stddef.h>   /* offsetof for the layout guards */")
    o("")
    o("/* Packed-struct attribute; a wrapper may define it first. */")
    o("#ifndef PROTO_PACKED")
    o("#  ifdef __GNUC__")
    o("#    define PROTO_PACKED __attribute__((packed))")
    o("#  else")
    o("#    define PROTO_PACKED")
    o("#  endif")
    o("#endif")
    o("")
    o("/* Wire-contract version. Carried in MsgHeader.ver_flags low byte; both")
    o(" * ends drop and count any frame whose version != this. */")
    o(f"#define PROTO_VERSION {schema['meta']['proto_version']}u")
    o(f"#define PROTO_SCHEMA_HASH 0x{shash:08X}u")
    o("")

    o("/* ── wire constants ─────────────────────────────────────────────────── */")
    for c in schema["constants"]:
        L.extend(block_comment(c["doc"]) if c.get("doc") else [])
        o(f"#define {c['name']} {fmt_int_c(c['value'], c.get('hex'))}")
    o("")

    for e in schema["enums"]:
        o(f"/* ── {e['name']} ({e['wire_type']} on the wire) ── */")
        if e.get("doc"):
            L.extend(block_comment(e["doc"]))
        o("typedef enum {")
        w = max(len(v["name"]) for v in e["values"])
        for v in e["values"]:
            val = fmt_int_c(v["value"], v.get("hex"))
            line = f"    {v['name']:<{w}} = {val},"
            if v.get("doc"):
                dl = wrap_comment(v["doc"], width=60)
                line += f"  /* {dl[0]}" + (" */" if len(dl) == 1 else "")
                o(line)
                for extra in dl[1:-1]:
                    o(f"       {extra}")
                if len(dl) > 1:
                    o(f"       {dl[-1]} */")
                continue
            o(line)
        o(f"}} {e['name']};")
        o("")

    for g in schema["flags"]:
        o(f"/* ── {g['doc'] if g.get('doc') else g['name']} ── */")
        for b in g["bits"]:
            if b.get("reserved"):
                o(f"/* bit{b['bit']} reserved — {b.get('doc', '')} */")
            else:
                doc = f"  /* {b['doc']} */" if b.get("doc") else ""
                o(f"#define {b['name']} (1u << {b['bit']}u){doc}")
        o("")

    o("/* ── fixed-point scales (host mirrors these exactly) ────────────────── */")
    for sc in schema["scales"]:
        parts = [p for p in (sc.get("unit"), sc.get("doc")) if p]
        doc = f"  /* {'; '.join(parts)} */" if parts else ""
        o(f"#define {sc['name']} {fmt_float_c(sc['value'])}{doc}")
    o("")

    for st in schema["structs"]:
        if st.get("doc"):
            L.extend(block_comment(st["doc"]))
        o("typedef struct PROTO_PACKED {")
        for fld in st["fields"]:
            t = fld["type"]
            ctype = WIRE_TYPES[t][1] if t in WIRE_TYPES else t
            decl = f"{ctype} {fld['name']}"
            if "count" in fld:
                decl += f"[{fld['count']}]"
            ann = [fld.get(k) for k in ("enum", "flags", "scale")]
            parts = [a for a in ann if a]
            if fld.get("truncate_by"):
                parts.append(f"transmitted prefix sized by {fld['truncate_by']}")
            if fld.get("doc"):
                parts.append(fld["doc"])
            doc = f"  /* {'; '.join(parts)} */" if parts else ""
            o(f"    {decl + ';':<34}{doc}".rstrip())
        o(f"}} {st['name']};   /* {layout[st['name']]['size']} bytes */")
        o("")

    o("/* ── derived wire constants (computed from the schema; asserted below) ── */")
    for d in schema["derived"]:
        if d.get("doc"):
            L.extend(block_comment(d["doc"]))
        val = derived[d["name"]]
        if "cast" in d:
            ctype = WIRE_TYPES[d["cast"]][1]
            o(f"#define {d['name']} (({ctype}){val}u)   /* {d['expr']} */")
        else:
            o(f"#define {d['name']} {val}u   /* {d['expr']} */")
    o("")

    o("/* ── layout guards (catch C↔schema drift at compile time) ───────────── */")
    o("#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)")
    for st in schema["structs"]:
        lay = layout[st["name"]]
        o(f"_Static_assert(sizeof({st['name']}) == {lay['size']}u, "
          f"\"{st['name']} size\");")
        for fl in lay["fields"]:
            o(f"_Static_assert(offsetof({st['name']}, {fl['name']}) == "
              f"{fl['offset']}u, \"{st['name']}.{fl['name']} offset\");")
    for e in schema["enums"]:
        vmax_name = max(e["values"], key=lambda v: v["value"])["name"]
        bound = (1 << (8 * WIRE_TYPES[e["wire_type"]][0])) - 1
        o(f"_Static_assert({vmax_name} <= {bound}u, "
          f"\"{e['name']} fits {e['wire_type']}\");")
    if "USB_MAX_FRAME_SIZE" in derived:
        o("/* Every full USB frame must fit one master TX slot (usb_tx.c ring). */")
        o("_Static_assert(USB_MAX_FRAME_SIZE <= USB_TX_SLOT_MAX, "
          "\"USB frame fits one TX slot\");")
    o("#endif")
    o("")
    o("#endif /* PROTOCOL_GEN_H */")
    return "\n".join(L) + "\n"


# ── Python emitter ───────────────────────────────────────────────────────────

def _py_struct_macro(name):
    """cmd_motor_t → CMD_MOTOR, MsgHeader → MSG_HEADER."""
    s = name[:-2] if name.endswith("_t") else name
    return re.sub(r"(?<!^)(?=[A-Z])", "_", s).upper()


def _py_value(value, hexa=False):
    return f"0x{value:02X}" if hexa else str(value)


def emit_python(schema, layout, derived, shash):
    consts = {c["name"]: c["value"] for c in schema["constants"]}
    L = []
    o = L.append
    o('"""AUTO-GENERATED — DO NOT EDIT.')
    o("")
    o("Source:      firmware/common/protocol.yaml")
    o("Regenerate:  python3 scripts/gen_protocol.py")
    o("")
    o(f"SCHEMA_HASH 0x{shash:08X}: CRC-32 of the schema's canonical wire projection")
    o("(parsed YAML minus doc/unit/hex keys, compact sorted-key JSON, UTF-8) — see")
    o("gen_protocol.py. Doc-only edits do not change it.")
    o("")
    o("Layout data for the little-endian packed wire structs, plus every enum")
    o("value (PLAIN INTS, not IntEnum — str() of an IntEnum changed in Python")
    o('3.11, which would silently alter stringified CSV/log output), flag bit,')
    o("fixed-point scale and wire constant. master_link.protocol is the public")
    o('API and re-exports from here; consumers should not import this directly."""')
    o("import struct as _struct")
    o("")
    o(f"PROTO_VERSION = {schema['meta']['proto_version']}")
    o(f"SCHEMA_HASH = 0x{shash:08X}")
    o("")
    o("# ── wire constants ──")
    for c in schema["constants"]:
        o(f"{c['name']} = {_py_value(c['value'], c.get('hex'))}")
    o("")
    for e in schema["enums"]:
        o(f"# ── {e['name']} ({e['wire_type']} on the wire) ──")
        for v in e["values"]:
            o(f"{v['name']} = {_py_value(v['value'], v.get('hex'))}")
        pairs = ", ".join(f"{v['value']}: \"{v['name']}\"" for v in e["values"])
        o(f"{_py_struct_macro(e['name'])}_VALUE_NAMES = {{{pairs}}}")
        o("")
    for g in schema["flags"]:
        o(f"# ── {g['name']} ──")
        for b in g["bits"]:
            if b.get("reserved"):
                o(f"# bit {b['bit']} reserved — {b.get('doc', '')}")
            else:
                o(f"{b['name']} = 1 << {b['bit']}")
        o("")
    o("# ── fixed-point scales ──")
    for sc in schema["scales"]:
        o(f"{sc['name']} = {float(sc['value'])!r}")
    o("")
    o("# ── struct formats ('<' = little-endian, packed) ──")
    o("# Nested structs (a trailing struct array) get a _PREFIX format covering")
    o("# the scalar fields before the array; SZ_* is always the full wire size.")
    structs_table = {}
    for st in schema["structs"]:
        mn = _py_struct_macro(st["name"])
        lay = layout[st["name"]]
        fmt = "<"
        array = None
        tbl = []
        for fld in st["fields"]:
            t = fld["type"]
            count = resolve_count(fld, consts)
            if t in WIRE_TYPES:
                if array is not None:
                    fail(f"python emitter: {st['name']}.{fld['name']} follows a "
                         f"struct array — unsupported")
                ch = WIRE_TYPES[t][2]
                fmt += (str(count) + ch) if count else ch
                tbl.append((fld["name"], "scalar",
                            (str(count) + ch) if count else ch))
            else:
                if array is not None:
                    fail(f"python emitter: {st['name']} has more than one "
                         f"struct-typed field — unsupported")
                array = (fld["name"], t, count or 1)
                tbl.append((fld["name"], "array", t, count or 1))
        if array is None:
            o(f"FMT_{mn} = \"{fmt}\"")
        else:
            o(f"FMT_{mn}_PREFIX = \"{fmt}\"")
        o(f"SZ_{mn} = {lay['size']}")
        offs = ", ".join(f"\"{f['name']}\": {f['offset']}" for f in lay["fields"])
        o(f"OFF_{mn} = {{{offs}}}")
        structs_table[st["name"]] = tbl
    o("")
    o("# ── derived wire constants ──")
    for d in schema["derived"]:
        o(f"{d['name']} = {derived[d['name']]}   # {d['expr']}")
    o("")
    o("# ── generic pack/unpack (field tables + recursive helpers) ──")
    o("# STRUCTS[name] = [(field, 'scalar', fmt) | (field, 'array', struct, n)]")
    o("STRUCTS = {")
    for name, tbl in structs_table.items():
        rows = ", ".join(repr(r) for r in tbl)
        o(f"    \"{name}\": [{rows}],")
    o("}")
    o("")
    o("")
    o("def unpack_struct(name, data, off=0):")
    o("    \"\"\"Decode one wire struct into a dict (struct arrays → lists of dicts).")
    o("    Raw wire values only — no fixed-point decode, no truncation handling.\"\"\"")
    o("    out = {}")
    o("    for row in STRUCTS[name]:")
    o("        if row[1] == \"scalar\":")
    o("            fmt = \"<\" + row[2]")
    o("            vals = _struct.unpack_from(fmt, data, off)")
    o("            out[row[0]] = vals[0] if len(vals) == 1 else list(vals)")
    o("            off += _struct.calcsize(fmt)")
    o("        else:")
    o("            _, _, sub, n = row")
    o("            sz = SZ_BY_STRUCT[sub]")
    o("            out[row[0]] = [unpack_struct(sub, data, off + i * sz)")
    o("                           for i in range(n)]")
    o("            off += n * sz")
    o("    return out")
    o("")
    o("")
    o("def pack_struct(name, values):")
    o("    \"\"\"Inverse of unpack_struct. Missing fields pack as zero; a short")
    o("    struct-array list is zero-filled to the fixed wire length.\"\"\"")
    o("    out = bytearray()")
    o("    for row in STRUCTS[name]:")
    o("        if row[1] == \"scalar\":")
    o("            fmt = \"<\" + row[2]")
    o("            v = values.get(row[0], 0)")
    o("            out += _struct.pack(fmt, *v) if isinstance(v, (list, tuple)) \\")
    o("                else _struct.pack(fmt, v)")
    o("        else:")
    o("            _, _, sub, n = row")
    o("            elems = values.get(row[0], [])")
    o("            for i in range(n):")
    o("                out += pack_struct(sub, elems[i] if i < len(elems) else {})")
    o("    return bytes(out)")
    o("")
    o("")
    sz_pairs = ", ".join(f"\"{name}\": SZ_{_py_struct_macro(name)}"
                         for name in structs_table)
    o(f"SZ_BY_STRUCT = {{{sz_pairs}}}")
    return "\n".join(L) + "\n"


# ── C++ emitter ──────────────────────────────────────────────────────────────

CPP_TYPES = {"u8": "std::uint8_t", "u16": "std::uint16_t",
             "u32": "std::uint32_t", "i16": "std::int16_t"}


def emit_cpp(schema, layout, derived, shash):
    L = []
    o = L.append
    o("// AUTO-GENERATED — DO NOT EDIT.")
    o("// Source:      firmware/common/protocol.yaml")
    o("// Regenerate:  python3 scripts/gen_protocol.py")
    o("//")
    o(f"// SCHEMA_HASH 0x{shash:08X}: CRC-32 of the schema's canonical wire")
    o("// projection (parsed YAML minus doc/unit/hex keys, compact sorted-key")
    o("// JSON, UTF-8) — see gen_protocol.py. Doc-only edits do not change it.")
    o("//")
    o("// Header-only mirror of the soccer wire protocol for C++ consumers")
    o("// (soccer-software pins this file + generated/cpp/fixtures/). Packed")
    o("// little-endian structs; encode/decode are whole-struct memcpy, so a")
    o("// LITTLE-ENDIAN host is assumed (same assumption as the firmware side).")
    o("// Never take references or pointers to the packed members — use the")
    o("// to_bytes/from_bytes helpers and whole-field loads/stores.")
    o("#ifndef ROBOT_WIRE_GEN_HPP")
    o("#define ROBOT_WIRE_GEN_HPP")
    o("")
    o("#include <array>")
    o("#include <cstddef>   // offsetof for the layout guards")
    o("#include <cstdint>")
    o("#include <cstring>")
    o("")
    o("namespace robot_wire {")
    o("")
    o(f"constexpr std::uint8_t  PROTO_VERSION = {schema['meta']['proto_version']}u;")
    o(f"constexpr std::uint32_t PROTO_SCHEMA_HASH = 0x{shash:08X}u;")
    o("")
    o("// ── wire constants ──")
    for c in schema["constants"]:
        ctype = "std::uint32_t" if c["value"] > 0xFFFF else "std::uint16_t"
        o(f"constexpr {ctype} {c['name']} = "
          f"{fmt_int_c(c['value'], c.get('hex'))};")
    o("")
    for e in schema["enums"]:
        if e.get("doc"):
            for l in wrap_comment(e["doc"]):
                o(f"// {l}")
        o(f"enum class {e['name']} : {CPP_TYPES[e['wire_type']]} {{")
        w = max(len(v["name"]) for v in e["values"])
        for v in e["values"]:
            doc = f"  // {v['doc']}" if v.get("doc") else ""
            o(f"    {v['name']:<{w}} = {fmt_int_c(v['value'], v.get('hex'))},{doc}")
        o("};")
        o("")
    for g in schema["flags"]:
        o(f"// ── {g['doc'] if g.get('doc') else g['name']} ──")
        for b in g["bits"]:
            if b.get("reserved"):
                o(f"// bit {b['bit']} reserved — {b.get('doc', '')}")
            else:
                doc = f"  // {b['doc']}" if b.get("doc") else ""
                o(f"constexpr std::uint8_t {b['name']} = 1u << {b['bit']}u;{doc}")
        o("")
    o("// ── fixed-point scales ──")
    for sc in schema["scales"]:
        parts = [p for p in (sc.get("unit"), sc.get("doc")) if p]
        doc = f"  // {'; '.join(parts)}" if parts else ""
        o(f"constexpr float {sc['name']} = {fmt_float_c(sc['value'])};{doc}")
    o("")
    o("#pragma pack(push, 1)")
    o("")
    for st in schema["structs"]:
        if st.get("doc"):
            for l in wrap_comment(st["doc"]):
                o(f"// {l}")
        o(f"struct {st['name']} {{")
        for fld in st["fields"]:
            t = fld["type"]
            ctype = CPP_TYPES[t] if t in CPP_TYPES else t
            decl = f"{ctype} {fld['name']}"
            if "count" in fld:
                decl += f"[{fld['count']}]"
            ann = [fld.get(k) for k in ("enum", "flags", "scale")]
            parts = [a for a in ann if a]
            if fld.get("truncate_by"):
                parts.append(f"transmitted prefix sized by {fld['truncate_by']}")
            if fld.get("doc"):
                parts.append(fld["doc"])
            doc = f"  // {'; '.join(parts)}" if parts else ""
            o(f"    {decl + ';':<34}{doc}".rstrip())
        o("};")
        o("")
    o("#pragma pack(pop)")
    o("")
    o("// ── derived wire constants ──")
    for d in schema["derived"]:
        o(f"constexpr std::uint16_t {d['name']} = {derived[d['name']]}u;"
          f"  // {d['expr']}")
    o("")
    o("// ── layout guards ──")
    for st in schema["structs"]:
        lay = layout[st["name"]]
        o(f"static_assert(sizeof({st['name']}) == {lay['size']}u, "
          f"\"{st['name']} size\");")
        for fl in lay["fields"]:
            o(f"static_assert(offsetof({st['name']}, {fl['name']}) == "
              f"{fl['offset']}u, \"{st['name']}.{fl['name']} offset\");")
    o("")
    o("// ── encode/decode: whole-struct memcpy (no packed-member references) ──")
    o("template <typename T>")
    o("inline std::array<std::uint8_t, sizeof(T)> to_bytes(const T& v) {")
    o("    std::array<std::uint8_t, sizeof(T)> out{};")
    o("    std::memcpy(out.data(), &v, sizeof(T));")
    o("    return out;")
    o("}")
    o("")
    o("template <typename T>")
    o("inline T from_bytes(const std::uint8_t* buf) {")
    o("    T v{};")
    o("    std::memcpy(&v, buf, sizeof(T));")
    o("    return v;")
    o("}")
    o("")
    o("}  // namespace robot_wire")
    o("")
    o("#endif  // ROBOT_WIRE_GEN_HPP")
    return "\n".join(L) + "\n"


# ── golden fixtures (byte files the C++ test and soccer-software pin) ─────────
# RAW wire values mirroring firmware/common/test/gen_fixture.c exactly — the
# host test byte-compares these files against gen_fixture.c's output.

FIXTURES = {
    "robot_cmd.bin": ("cmd_robot_t", {
        "cycle_id": 7, "cmd_seq": 42, "n_chains": 1,
        "reserved": 0xDE,          # master test sentinel, exercised on purpose
        "chains": [{
            "chain_id": 0, "n_motors": 1,
            "motors": [{"drive_req": 1, "pos": 5000, "vel": -125, "kp": 150,
                        "kd": 100, "tau_ff": 30, "flags": 1}],
        }],
    }),
    "robot_tele.bin": ("tele_robot_t", {
        "cycle_id": 9, "master_time_us": 123456, "last_cmd_seq_rx": 42,
        "cmd_seq_active": 41, "cmd_on_time": 1000, "cmd_late": 7,
        "cmd_missing": 3, "cmd_duplicate": 2, "n_chains": 1, "robot_state": 1,
        "chains": [{
            "chain_id": 0, "n_motors": 1, "link": 1, "spi_seq_echo": 0x2A,
            "slave_time_us": 7777, "cmd_crc_errors": 3, "can_tx_errors": 1,
            "spi_tx_arm_fails": 4,
            "motors": [{"pos": 5000, "vel": -125, "tau": 30, "temp_c": 42,
                        "drive_state": 5, "drive_fault": 0, "rs_mode": 2,
                        "rs_fault_bits": 0x0A, "flags": 4, "fb_age_ms": 250,
                        "rs_fault_word": 0xDEADBEEF,
                        "last_applied_seq": 0x1234}],
        }],
    }),
}


def pack_fixture(schema, struct_name, values):
    import struct as pystruct
    consts = {c["name"]: c["value"] for c in schema["constants"]}
    by_name = {st["name"]: st for st in schema["structs"]}

    def pack(st_name, vals):
        out = bytearray()
        for fld in by_name[st_name]["fields"]:
            t = fld["type"]
            count = resolve_count(fld, consts)
            if t in WIRE_TYPES:
                ch = WIRE_TYPES[t][2]
                fmt = "<" + ((str(count) + ch) if count else ch)
                v = vals.get(fld["name"], 0)
                out += (pystruct.pack(fmt, *v) if isinstance(v, (list, tuple))
                        else pystruct.pack(fmt, v))
            else:
                elems = vals.get(fld["name"], [])
                for i in range(count or 1):
                    out += pack(t, elems[i] if i < len(elems) else {})
        return bytes(out)

    return pack(struct_name, values)


def write_fixtures(schema, layout):
    os.makedirs(OUT_FIXTURES, exist_ok=True)
    for fname, (st_name, values) in FIXTURES.items():
        data = pack_fixture(schema, st_name, values)
        if len(data) != layout[st_name]["size"]:
            fail(f"fixture {fname}: packed {len(data)} B, expected "
                 f"{layout[st_name]['size']} B")
        with open(os.path.join(OUT_FIXTURES, fname), "wb") as f:
            f.write(data)
        print(f"gen_protocol: wrote "
              f"{os.path.relpath(os.path.join(OUT_FIXTURES, fname), REPO_ROOT)} "
              f"({len(data)} B)")


# ── main ─────────────────────────────────────────────────────────────────────

EMITTERS = {
    "c": (emit_c, OUT_C),
    "python": (emit_python, OUT_PY),
    "cpp": (emit_cpp, OUT_CPP),
}


def check_outputs(schema, layout, derived, shash, emit_names):
    """Regenerate in memory and compare with the committed outputs. Returns the
    list of stale paths (empty = everything up to date)."""
    stale = []

    def compare(path, want_bytes):
        try:
            with open(path, "rb") as f:
                have = f.read()
        except FileNotFoundError:
            have = None
        if have != want_bytes:
            stale.append(os.path.relpath(path, REPO_ROOT))

    for name in emit_names:
        emit_fn, out_path = EMITTERS[name]
        compare(out_path, emit_fn(schema, layout, derived, shash).encode("utf-8"))
        if name == "cpp":
            for fname, (st_name, values) in FIXTURES.items():
                compare(os.path.join(OUT_FIXTURES, fname),
                        pack_fixture(schema, st_name, values))
    return stale


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--emit", default=",".join(EMITTERS),
                    help=f"comma list of emitters (default: all implemented: "
                         f"{','.join(EMITTERS)})")
    ap.add_argument("--check", action="store_true",
                    help="regenerate in memory and exit non-zero if any "
                         "committed output differs (drift gate; writes nothing)")
    args = ap.parse_args()

    emit_names = [n.strip() for n in args.emit.split(",")]
    for name in emit_names:
        if name not in EMITTERS:
            print(f"gen_protocol: unknown emitter {name!r} "
                  f"(have: {', '.join(EMITTERS)})", file=sys.stderr)
            return 1

    try:
        schema = load_schema()
        layout = compute_layout(schema)
        derived = compute_derived(schema, layout)
        shash = schema_hash(schema)
    except SchemaError as e:
        print(f"gen_protocol: schema error: {e}", file=sys.stderr)
        return 1

    if args.check:
        stale = check_outputs(schema, layout, derived, shash, emit_names)
        if stale:
            print("gen_protocol --check: STALE outputs (re-run "
                  "scripts/gen_protocol.py and commit):", file=sys.stderr)
            for p in stale:
                print(f"  {p}", file=sys.stderr)
            return 1
        print(f"gen_protocol --check: outputs up to date (schema 0x{shash:08X})")
        return 0

    for name in emit_names:
        emit_fn, out_path = EMITTERS[name]
        text = emit_fn(schema, layout, derived, shash)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"gen_protocol: wrote {os.path.relpath(out_path, REPO_ROOT)} "
              f"(schema 0x{shash:08X})")
        if name == "cpp":
            write_fixtures(schema, layout)
    return 0


if __name__ == "__main__":
    sys.exit(main())
