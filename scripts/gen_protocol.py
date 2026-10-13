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
    o("#endif")
    o("")
    o("#endif /* PROTOCOL_GEN_H */")
    return "\n".join(L) + "\n"


# ── main ─────────────────────────────────────────────────────────────────────

EMITTERS = {
    "c": (emit_c, OUT_C),
    # "python": step 2; "cpp": step 3
}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--emit", default=",".join(EMITTERS),
                    help=f"comma list of emitters (default: all implemented: "
                         f"{','.join(EMITTERS)})")
    args = ap.parse_args()

    try:
        schema = load_schema()
        layout = compute_layout(schema)
        derived = compute_derived(schema, layout)
        shash = schema_hash(schema)
    except SchemaError as e:
        print(f"gen_protocol: schema error: {e}", file=sys.stderr)
        return 1

    for name in args.emit.split(","):
        name = name.strip()
        if name not in EMITTERS:
            print(f"gen_protocol: unknown emitter {name!r} "
                  f"(have: {', '.join(EMITTERS)})", file=sys.stderr)
            return 1
        emit_fn, out_path = EMITTERS[name]
        text = emit_fn(schema, layout, derived, shash)
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"gen_protocol: wrote {os.path.relpath(out_path, REPO_ROOT)} "
              f"(schema 0x{shash:08X})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
