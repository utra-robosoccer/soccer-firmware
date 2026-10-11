#!/usr/bin/env python3
"""Import a robot's firmware_config.yaml artifact from the soccer-software repo.

The model/generators in soccer-software export ONE firmware config per robot —
``.../model/robots/<robot>/generated/firmware_config.yaml`` (+ a ``.sha256``
sidecar) — covering every slave. This script copies that artifact into this repo
(``configs/<setup>/firmware_config.yaml``, where <setup> is the artifact's own
``setup:`` field) after verifying its content hash, so scripts/gen_motor_config.py
can consume it directly. No per-slave files are produced.

Hash scheme (must match soccer-software model/generators/emit.py):
    content_sha256 = sha256(yaml.safe_dump(body, sort_keys=True,
                                           default_flow_style=False))
where body = {setup, rates, shared_ranges, models, slaves} (the doc WITHOUT its own
content_sha256 field and without the banner comments). Both the embedded
``content_sha256:`` field and the ``.sha256`` sidecar carry this digest.

Usage:
    python3 scripts/import_firmware_config.py                 # robot bench_2s_10m
    python3 scripts/import_firmware_config.py --robot <name>
    python3 scripts/import_firmware_config.py --software-root <path> --robot <name>
"""
import argparse
import hashlib
import os
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SOFTWARE_ROOT = os.path.expanduser("~/robosoccer/soccer-software")
_BODY_KEYS = ("setup", "rates", "shared_ranges", "models", "slaves")


def content_hash(doc: dict) -> str:
    """Recompute the artifact's content_sha256 over its body (no hash field/banner)."""
    missing = [k for k in _BODY_KEYS if k not in doc]
    if missing:
        sys.exit(f"firmware_config.yaml missing required keys: {missing}")
    body = {k: doc[k] for k in _BODY_KEYS}
    return hashlib.sha256(
        yaml.safe_dump(body, sort_keys=True, default_flow_style=False).encode()).hexdigest()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Import a firmware_config.yaml artifact.")
    ap.add_argument("--robot", default="bench_2s_10m",
                    help="robot name under model/robots/ (default bench_2s_10m)")
    ap.add_argument("--software-root", default=DEFAULT_SOFTWARE_ROOT,
                    help=f"soccer-software repo root (default {DEFAULT_SOFTWARE_ROOT})")
    ap.add_argument("--dest-root", default=os.path.join(REPO_ROOT, "configs"),
                    help="configs/ root in this repo (default configs/)")
    args = ap.parse_args(argv)

    gen_dir = os.path.join(args.software_root, "model", "robots", args.robot, "generated")
    src = os.path.join(gen_dir, "firmware_config.yaml")
    src_sha = src + ".sha256"
    if not os.path.isfile(src):
        sys.exit(f"artifact not found: {src}\n(is --software-root / --robot correct?)")

    with open(src) as fh:
        raw = fh.read()
    doc = yaml.safe_load(raw)
    setup = doc.get("setup")
    embedded = doc.get("content_sha256")
    if not setup or not embedded:
        sys.exit("firmware_config.yaml missing 'setup' or 'content_sha256'")

    # 1) recompute the content hash and compare to the embedded field
    calc = content_hash(doc)
    if calc != embedded:
        sys.exit(f"HASH MISMATCH: recomputed {calc} != embedded content_sha256 {embedded}\n"
                 f"the artifact is corrupt or the hash scheme changed — refusing to import.")

    # 2) cross-check the .sha256 sidecar (format: '<hash>  firmware_config.yaml')
    if os.path.isfile(src_sha):
        with open(src_sha) as fh:
            side = fh.read().split()[0].strip()
        if side != embedded:
            sys.exit(f"SIDECAR MISMATCH: {src_sha} has {side} != content_sha256 {embedded}")
    else:
        print(f"warning: no sidecar {os.path.basename(src_sha)} (verified embedded hash only)")

    # 3) copy into configs/<setup>/ (verbatim, so the sidecar/hash stay valid)
    dest_dir = os.path.join(args.dest_root, setup)
    os.makedirs(dest_dir, exist_ok=True)
    dest = os.path.join(dest_dir, "firmware_config.yaml")
    with open(dest, "w") as fh:
        fh.write(raw)
    if os.path.isfile(src_sha):
        with open(src_sha) as fh, open(dest + ".sha256", "w") as out:
            out.write(fh.read())

    n_slaves = len(doc["slaves"])
    n_motors = sum(len(s["motors"]) for s in doc["slaves"])
    rel = os.path.relpath(dest, REPO_ROOT)
    print(f"imported {args.robot} -> {rel}")
    print(f"  setup={setup}  slaves={n_slaves}  motors={n_motors}  content_sha256={embedded[:12]}… verified")


if __name__ == "__main__":
    main()
