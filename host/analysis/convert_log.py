#!/usr/bin/env python3
"""Convert a binary session log (.bin) into a session folder of CSVs.

    logs/2026-09-27/20-55-09_listen.bin
    logs/2026-09-27/20-55-09_listen/          (named after the .bin, next to it)
        motor_state.csv    telemetry rows (RX MOTOR_STATE)
        motor_cmd.csv      commands (TX MIT + control requests)
        status.csv         MASTER_STATUS / SLAVE_STATUS
        control_resp.csv   CONTROL_RESP
        events.csv         EVENT / ANNOTATION / wrong-version markers
        loop_timing.csv    LOOP_TIMING

Every file is always written (header-only if empty) so tools can rely on them.
The shared `host_ts` column is derived from each record's monotonic timestamp plus
the header's wall/mono start; motor_state also carries master_ts_ms where available.
Re-running overwrites the folder cleanly. Reports RX/TX / discard / wrong-version counts.

Usage:
    python3 host/analysis/convert_log.py logs/2026-09-27/20-55-09_listen.bin
"""
import argparse
import csv
import os
import shutil
import struct
import sys

from master_link import protocol as P
from master_link import motor_config_gen as mc
from master_link.datalog import BinaryLogReader
from master_link.datalog import format as LOG

_CTRL_NAMES = {P.CTRL_ARM_HOLD: "ARM_HOLD", P.CTRL_DISABLE: "DISABLE",
               P.CTRL_GOTO_ZERO: "GOTO_ZERO"}
_sz = getattr(P, "CTRL_SET_ZERO", None)
if _sz is not None:
    _CTRL_NAMES[_sz] = "SET_ZERO"

_GIDX = {(m["slave"], m["idx"]): g for g, m in enumerate(mc.MOTORS)}

MOTOR_STATE_FIELDS = ["host_ts", "master_ts_ms", "motor", "slave", "local",
                      "state", "cause", "cause_name", "motor_fault", "cmd_flags",
                      "fault_word", "fb_age", "pos", "vel", "tau", "temp",
                      "last_applied_seq"]
MOTOR_CMD_FIELDS = ["host_ts", "motor", "slave", "local", "opcode",
                    "pos", "vel", "kp", "kd", "tau_ff", "cmd_seq"]
STATUS_FIELDS = ["host_ts", "type", "robot_state", "slave_alive", "uptime_ms",
                 "link_errors", "rx_frames", "slave_id", "motors_alive",
                 "crc_errors", "cmd_crc_errors", "seq_gaps"]
CTRL_FIELDS = ["host_ts", "slave_id", "motor_idx", "cmd", "result", "new_state", "req_seq"]
EVENT_FIELDS = ["host_ts", "kind", "text"]
LOOP_FIELDS = ["host_ts", "seq", "period_ms", "step_ms", "send_ms", "lateness_ms"]


def _host_ts(hdr, ts_ns):
    return (hdr["wall_start_ns"] + (ts_ns - hdr["mono_start_ns"])) / 1e9


def _frame_header(payload):
    """(msg_type, version, ts_ms, inner_payload) or (None, ...) if too short."""
    if len(payload) < P.HDR_SIZE:
        return None, None, None, b""
    mt, seq, src, dst, ts_ms, pay_len, ver_flags, crc = struct.unpack_from(P.HDR_FMT, payload)
    inner = payload[P.HDR_SIZE:P.HDR_SIZE + pay_len]
    return mt, ver_flags & 0xFF, ts_ms, inner


def convert(path):
    reader = BinaryLogReader(path)
    hdr = reader.header
    folder = os.path.splitext(path)[0]      # logs/<date>/<time>_<policy>/
    if os.path.isdir(folder):
        shutil.rmtree(folder)               # overwrite cleanly
    os.makedirs(folder)

    files = {}
    def _writer(name, fields):
        f = open(os.path.join(folder, name), "w", newline="")
        files[name] = f
        w = csv.DictWriter(f, fieldnames=fields, restval="")
        w.writeheader()
        return w

    ms_w = _writer("motor_state.csv", MOTOR_STATE_FIELDS)
    cmd_w = _writer("motor_cmd.csv", MOTOR_CMD_FIELDS)
    status_w = _writer("status.csv", STATUS_FIELDS)
    ctrl_w = _writer("control_resp.csv", CTRL_FIELDS)
    event_w = _writer("events.csv", EVENT_FIELDS)
    loop_w = _writer("loop_timing.csv", LOOP_FIELDS)

    c = dict(rx=0, tx=0, discard_records=0, discard_bytes=0, version_frames=0,
             dropped_records=0, drop_events=0)

    for rec in reader:
        ht = _host_ts(hdr, rec.ts_ns)

        if rec.kind == LOG.RX_FRAME:
            c["rx"] += 1
            mt, ver, ts_ms, inner = _frame_header(rec.payload)
            if mt is None:
                continue
            if ver != P.PROTO_VERSION:
                c["version_frames"] += 1
                event_w.writerow({"host_ts": ht, "kind": "VERSION_MISMATCH",
                                  "text": f"version={ver} msg_type={mt}"})
                continue
            if mt == P.MSG_MOTOR_STATE:
                d = P.parse_motor_state(inner)
                if d:
                    ms_w.writerow({
                        "host_ts": ht, "master_ts_ms": ts_ms,
                        "motor": _GIDX.get((d["slave_id"], d["motor_idx"])),
                        "slave": d["slave_id"], "local": d["motor_idx"],
                        "state": d["state"], "cause": d["cause"],
                        "cause_name": P.CAUSE_NAMES.get(d["cause"], str(d["cause"])),
                        "motor_fault": d["motor_fault"], "cmd_flags": d["cmd_flags"],
                        "fault_word": d["fault_word"], "fb_age": d["fb_age"],
                        "pos": d["pos"], "vel": d["vel"], "tau": d["tau"], "temp": d["temp"],
                        "last_applied_seq": d["last_applied_seq"]})
            elif mt == P.MSG_MASTER_STATUS:
                d = P.parse_master_status(inner)
                if d:
                    status_w.writerow({"host_ts": ht, "type": "MASTER", **{
                        k: d[k] for k in ("robot_state", "slave_alive", "uptime_ms",
                                          "link_errors", "rx_frames")}})
            elif mt == P.MSG_SLAVE_STATUS:
                d = P.parse_slave_status(inner)
                if d:
                    status_w.writerow({"host_ts": ht, "type": "SLAVE", **{
                        k: d[k] for k in ("slave_id", "motors_alive", "uptime_ms",
                                          "crc_errors", "cmd_crc_errors", "seq_gaps")}})
            elif mt == P.MSG_CONTROL_RESP:
                d = P.parse_control_resp(inner)
                if d:
                    ctrl_w.writerow({"host_ts": ht, **d})

        elif rec.kind == LOG.TX_FRAME:
            c["tx"] += 1
            mt, ver, ts_ms, inner = _frame_header(rec.payload)
            if mt == P.MSG_MOTOR_CMD:
                s, l, pos, vel, kp, kd, tau, cmd_seq = struct.unpack(P.FMT_MOTOR_CMD, inner)
                cmd_w.writerow({"host_ts": ht, "motor": _GIDX.get((s, l)),
                                "slave": s, "local": l, "opcode": "MIT",
                                "pos": pos, "vel": vel, "kp": kp, "kd": kd, "tau_ff": tau,
                                "cmd_seq": cmd_seq})
            elif mt == P.MSG_CONTROL_REQ:
                s, l, cmd, _res = struct.unpack(P.FMT_CONTROL_REQ, inner)
                cmd_w.writerow({"host_ts": ht, "motor": _GIDX.get((s, l)),
                                "slave": s, "local": l,
                                "opcode": _CTRL_NAMES.get(cmd, f"CTRL_{cmd}")})

        elif rec.kind == LOG.RX_DISCARD:
            c["discard_records"] += 1
            c["discard_bytes"] += len(rec.payload)

        elif rec.kind == LOG.EVENT:
            event_w.writerow({"host_ts": ht, "kind": "EVENT",
                              "text": rec.payload.decode("utf-8", "replace")})
        elif rec.kind == LOG.ANNOTATION:
            event_w.writerow({"host_ts": ht, "kind": "ANNOTATION",
                              "text": rec.payload.decode("utf-8", "replace")})
        elif rec.kind == LOG.LOOP_TIMING:
            seq, period, step, send, late = LOG.LOOP_TIMING_FMT.unpack(rec.payload)
            loop_w.writerow({"host_ts": ht, "seq": seq, "period_ms": period / 1e6,
                             "step_ms": step / 1e6, "send_ms": send / 1e6,
                             "lateness_ms": late / 1e6})
        elif rec.kind == LOG.LOG_DROP:
            cnt, first_ns, last_ns = LOG.LOG_DROP_FMT.unpack(rec.payload)
            c["dropped_records"] += cnt
            c["drop_events"] += 1
            t0 = (hdr["wall_start_ns"] + (first_ns - hdr["mono_start_ns"])) / 1e9
            t1 = (hdr["wall_start_ns"] + (last_ns - hdr["mono_start_ns"])) / 1e9
            event_w.writerow({"host_ts": t0, "kind": "LOG_DROP",
                              "text": f"{cnt} records dropped between "
                                      f"host_ts {t0:.3f} and {t1:.3f} (writer queue full)"})

    for f in files.values():
        f.close()
    return folder, c


def main(argv=None):
    ap = argparse.ArgumentParser(description="Convert a binary session log to a CSV folder")
    ap.add_argument("logfile")
    args = ap.parse_args(argv)
    if not os.path.isfile(args.logfile):
        sys.exit(f"no such file: {args.logfile}")
    folder, c = convert(args.logfile)
    print(f"wrote {folder}/ "
          "(motor_state, motor_cmd, status, control_resp, events, loop_timing).csv")
    print(f"RX frames {c['rx']}  TX frames {c['tx']}  "
          f"discards {c['discard_records']} rec / {c['discard_bytes']} B  "
          f"wrong-version {c['version_frames']}")
    if c["dropped_records"]:
        print(f"** INCOMPLETE LOG: {c['dropped_records']} records DROPPED "
              f"in {c['drop_events']} window(s) (writer queue full) — see events.csv **")
    else:
        print("log complete: 0 records dropped")


if __name__ == "__main__":
    main()
