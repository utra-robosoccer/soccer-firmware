#!/usr/bin/env python3
"""Convert a binary session log (.bin) into a session folder of CSVs.

    logs/<date>/<time>_<policy>.bin
    logs/<date>/<time>_<policy>/              (named after the .bin, next to it)
        motor_state.csv    per-motor telemetry rows (from RX MSG_ROBOT_TELE)
        motor_cmd.csv      per-motor command rows   (from TX MSG_ROBOT_CMD)
        status.csv         MASTER_STATUS / SLAVE_STATUS / ROBOT (tele meta)
        events.csv         EVENT / ANNOTATION / wrong-version / LOG_DROP markers
        loop_timing.csv    LOOP_TIMING

Every file is always written (header-only if empty). States, causes and modes are
decoded by name. Re-running overwrites the folder cleanly.

Usage:
    python3 host/analysis/convert_log.py logs/<date>/<time>_<policy>.bin
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

_GIDX = {(m["slave"], m["idx"]): g for g, m in enumerate(mc.MOTORS)}

MOTOR_STATE_FIELDS = ["host_ts", "master_ts_ms", "motor", "slave", "local",
                      "state", "state_name", "cause", "cause_name",
                      "motor_mode", "motor_mode_name", "motor_fault", "flags",
                      "fault_word", "fb_age", "pos", "vel", "tau", "temp",
                      "last_applied_seq", "request_rejected", "to_zero_arrived",
                      "saturated"]
MOTOR_CMD_FIELDS = ["host_ts", "motor", "slave", "local", "mode", "mode_name",
                    "pos", "vel", "kp", "kd", "tau_ff", "cmd_seq", "cycle_id", "flags"]
STATUS_FIELDS = ["host_ts", "type", "robot_state", "slave_alive", "uptime_ms",
                 "link_errors", "rx_frames", "master_poll_hz", "telemetry_hz",
                 "slave_tick_hz", "host_cmd_hz", "slave_id", "motors_alive",
                 "crc_errors", "cmd_crc_errors", "seq_gaps",
                 "cycle_id", "last_cmd_seq_rx", "missed_deadlines", "n_chains"]
EVENT_FIELDS = ["host_ts", "kind", "text"]
LOOP_FIELDS = ["host_ts", "seq", "period_ms", "step_ms", "send_ms", "lateness_ms"]


def _host_ts(hdr, ts_ns):
    return (hdr["wall_start_ns"] + (ts_ns - hdr["mono_start_ns"])) / 1e9


def _frame_header(payload):
    if len(payload) < P.HDR_SIZE:
        return None, None, None, b""
    mt, seq, src, dst, ts_ms, pay_len, ver_flags, crc = struct.unpack_from(P.HDR_FMT, payload)
    inner = payload[P.HDR_SIZE:P.HDR_SIZE + pay_len]
    return mt, ver_flags & 0xFF, ts_ms, inner


def convert(path):
    reader = BinaryLogReader(path)
    hdr = reader.header
    folder = os.path.splitext(path)[0]
    if os.path.isdir(folder):
        shutil.rmtree(folder)
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
            if mt == P.MSG_ROBOT_TELE:
                d = P.parse_robot_tele(inner)
                if not d:
                    continue
                status_w.writerow({"host_ts": ht, "type": "ROBOT",
                                   "robot_state": d["robot_state"],
                                   "cycle_id": d["cycle_id"],
                                   "last_cmd_seq_rx": d["last_cmd_seq_rx"],
                                   "missed_deadlines": d["missed_deadlines"],
                                   "n_chains": d["n_chains"]})
                for ch in d["chains"]:
                    sid = ch["chain_id"]
                    for local, mo in enumerate(ch["motors"]):
                        ms_w.writerow({
                            "host_ts": ht, "master_ts_ms": ts_ms,
                            "motor": _GIDX.get((sid, local)),
                            "slave": sid, "local": local,
                            "state": mo.state, "state_name": mo.lifecycle_name,
                            "cause": mo.cause, "cause_name": mo.cause_name,
                            "motor_mode": mo.motor_mode,
                            "motor_mode_name": mo.motor_mode_name,
                            "motor_fault": mo.motor_fault, "flags": mo.flags,
                            "fault_word": mo.fault_word, "fb_age": mo.fb_age_ms,
                            "pos": mo.pos, "vel": mo.vel, "tau": mo.tau, "temp": mo.temp,
                            "last_applied_seq": mo.last_applied_seq,
                            "request_rejected": int(mo.request_rejected),
                            "to_zero_arrived": int(mo.to_zero_arrived),
                            "saturated": int(mo.saturated)})
            elif mt == P.MSG_MASTER_STATUS:
                d = P.parse_master_status(inner)
                if d:
                    status_w.writerow({"host_ts": ht, "type": "MASTER", **{
                        k: d[k] for k in ("robot_state", "slave_alive", "uptime_ms",
                                          "link_errors", "rx_frames", "master_poll_hz",
                                          "telemetry_hz", "slave_tick_hz", "host_cmd_hz")}})
            elif mt == P.MSG_SLAVE_STATUS:
                d = P.parse_slave_status(inner)
                if d:
                    status_w.writerow({"host_ts": ht, "type": "SLAVE", **{
                        k: d[k] for k in ("slave_id", "motors_alive", "uptime_ms",
                                          "crc_errors", "cmd_crc_errors", "seq_gaps")}})

        elif rec.kind == LOG.TX_FRAME:
            c["tx"] += 1
            mt, ver, ts_ms, inner = _frame_header(rec.payload)
            if mt == P.MSG_ROBOT_CMD:
                d = P.parse_robot_cmd(inner)
                if not d:
                    continue
                for ch in d["chains"]:
                    sid = ch["chain_id"]
                    for local, m in enumerate(ch["motors"]):
                        cmd_w.writerow({"host_ts": ht,
                                        "motor": _GIDX.get((sid, local)),
                                        "slave": sid, "local": local,
                                        "mode": m["mode_req"],
                                        "mode_name": P.MODE_REQ_NAMES.get(m["mode_req"],
                                                                          str(m["mode_req"])),
                                        "pos": m["pos"], "vel": m["vel"],
                                        "kp": m["kp"], "kd": m["kd"], "tau_ff": m["tau_ff"],
                                        "cmd_seq": d["cmd_seq"], "cycle_id": d["cycle_id"],
                                        "flags": m["flags"]})

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
    print(f"wrote {folder}/ (motor_state, motor_cmd, status, events, loop_timing).csv")
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
