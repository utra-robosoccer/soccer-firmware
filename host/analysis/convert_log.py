#!/usr/bin/env python3
"""Convert a binary session log (.bin) into a session folder of CSVs.

    logs/<date>/<time>_<policy>.bin
    logs/<date>/<time>_<policy>/              (named after the .bin, next to it)
        motor_state.csv    per-motor telemetry rows (from RX USB_MSG_TELE)
        motor_cmd.csv      per-motor command rows   (from TX USB_MSG_CMD)
        robot.csv          per-frame robot/status_master_t row
        slaves.csv         per-frame per-slave status_slave_t row
        imu.csv            per-frame BMI088 sample (deduped on imu.seq)
        events.csv         EVENT / ANNOTATION / LOG_DROP markers
        loop_timing.csv    LOOP_TIMING

Every file is always written (header-only if empty). States, faults and modes
are decoded by name. Re-running overwrites the folder cleanly. Only
LOG_FMT_VERSION 2 logs (the protocol_wire.h era) are supported — convert v1
logs with the pre-revamp checkout their meta's git_commit points at.

Usage:
    python3 host/analysis/convert_log.py logs/<date>/<time>_<policy>.bin
"""
import argparse
import csv
import os
import shutil
import sys

from master_link import protocol as P
from master_link import motor_config_gen as mc
from master_link.timeutil import U32Unwrapper
from master_link.datalog import BinaryLogReader
from master_link.datalog import format as LOG

_GIDX = {(m["slave"], m["idx"]): g for g, m in enumerate(mc.MOTORS)}

MOTOR_STATE_FIELDS = ["host_ts", "master_time_s", "cycle_id", "motor", "slave",
                      "local", "state", "state_name", "fault", "fault_name",
                      "flags", "ready", "request_rejected", "saturated",
                      "rs_mode", "rs_mode_name", "rs_fault_bits",
                      "rs_fault_word", "pos", "vel", "tau", "temp",
                      "time_slave_us", "last_applied_seq"]
MOTOR_CMD_FIELDS = ["host_ts", "motor", "slave", "local", "req", "req_name",
                    "slave_req", "pos", "vel", "kp", "kd", "tau_ff",
                    "cmd_seq", "cycle_id"]
ROBOT_FIELDS = ["host_ts", "cycle_id", "master_time_s", "robot_state",
                "robot_state_name", "cmd_seq_active", "n_slaves",
                "host_lost", "host_last_cmd_seq_rx", "host_on_time",
                "host_late", "host_missing", "host_duplicate",
                "usb_rx_frames", "usb_rx_errors", "usb_rx_resyncs",
                "usb_rx_discarded_bytes", "usb_tx_drops", "cycle_overruns"]
SLAVE_FIELDS = ["host_ts", "cycle_id", "slave", "alive", "state", "state_name",
                "n_motors", "time_slave_us", "spi_crc_errors", "spi_seq_gaps",
                "boot_found_mask", "boot_unexpected_id",
                "boot_zero_sta_fail_mask", "boot_version_fail_mask",
                "cmd_crc_errors", "resyncs", "tx_arm_fails", "seq_echo",
                "can_tx_errors", "can_tec", "can_rec"]
IMU_FIELDS = ["host_ts", "master_time_s", "seq", "flags", "errors",
              "ax_raw", "ay_raw", "az_raw", "gx_raw", "gy_raw", "gz_raw",
              "ax_mps2", "ay_mps2", "az_mps2", "gx_rps", "gy_rps", "gz_rps"]
EVENT_FIELDS = ["host_ts", "kind", "text"]
LOOP_FIELDS = ["host_ts", "seq", "period_ms", "step_ms", "send_ms", "lateness_ms"]


def _host_ts(hdr, ts_ns):
    return (hdr["wall_start_ns"] + (ts_ns - hdr["mono_start_ns"])) / 1e9


def convert(path):
    reader = BinaryLogReader(path)
    hdr = reader.header
    if hdr["log_fmt_version"] != LOG.LOG_FMT_VERSION:
        meta = hdr.get("meta") or {}
        sys.exit(f"log format v{hdr['log_fmt_version']} is not v{LOG.LOG_FMT_VERSION} "
                 f"(pre-revamp log?) — convert it with the checkout its meta "
                 f"points at (git_commit={meta.get('git_commit', '?')})")
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
    robot_w = _writer("robot.csv", ROBOT_FIELDS)
    slave_w = _writer("slaves.csv", SLAVE_FIELDS)
    imu_w = _writer("imu.csv", IMU_FIELDS)
    event_w = _writer("events.csv", EVENT_FIELDS)
    loop_w = _writer("loop_timing.csv", LOOP_FIELDS)

    mtu = U32Unwrapper()   # unwrap the 32-bit time_master_us across the log
    prev_imu_seq = None
    c = dict(rx=0, tx=0, discard_records=0, discard_bytes=0,
             dropped_records=0, drop_events=0)

    for rec in reader:
        ht = _host_ts(hdr, rec.ts_ns)

        if rec.kind == LOG.RX_FRAME:
            c["rx"] += 1
            if len(rec.payload) < P.HDR_SIZE:
                continue
            mt, _seq, pay_len, _crc = P._HDR.unpack_from(rec.payload, 0)
            inner = rec.payload[P.HDR_SIZE:P.HDR_SIZE + pay_len]
            if mt != P.USB_MSG_TELE:
                continue
            tm = P.parse_tele_master(inner)
            if tm is None:
                continue
            mts = mtu.update(tm.time_master_us) / 1e6
            st = tm.status
            robot_w.writerow({
                "host_ts": ht, "cycle_id": tm.cycle_id, "master_time_s": mts,
                "robot_state": tm.robot_state,
                "robot_state_name": P.ROBOT_STATE_NAMES.get(tm.robot_state,
                                                            tm.robot_state),
                "cmd_seq_active": tm.cmd_seq_active, "n_slaves": tm.n_slaves,
                "host_lost": st.host.lost,
                "host_last_cmd_seq_rx": st.host.last_cmd_seq_rx,
                "host_on_time": st.host.on_time, "host_late": st.host.late,
                "host_missing": st.host.missing,
                "host_duplicate": st.host.duplicate,
                "usb_rx_frames": st.usb.rx_frames,
                "usb_rx_errors": st.usb.rx_errors,
                "usb_rx_resyncs": st.usb.rx_resyncs,
                "usb_rx_discarded_bytes": st.usb.rx_discarded_bytes,
                "usb_tx_drops": st.usb.tx_drops,
                "cycle_overruns": st.cycle_overruns})
            if tm.imu.seq != prev_imu_seq:
                prev_imu_seq = tm.imu.seq
                ax, ay, az = tm.imu.accel
                gx, gy, gz = tm.imu.gyro
                imu_w.writerow({
                    "host_ts": ht, "master_time_s": mts, "seq": tm.imu.seq,
                    "flags": tm.imu.flags, "errors": tm.imu.errors,
                    "ax_raw": ax, "ay_raw": ay, "az_raw": az,
                    "gx_raw": gx, "gy_raw": gy, "gz_raw": gz,
                    "ax_mps2": ax * P.IMU_ACCEL_LSB_MPS2,
                    "ay_mps2": ay * P.IMU_ACCEL_LSB_MPS2,
                    "az_mps2": az * P.IMU_ACCEL_LSB_MPS2,
                    "gx_rps": gx * P.IMU_GYRO_LSB_RPS,
                    "gy_rps": gy * P.IMU_GYRO_LSB_RPS,
                    "gz_rps": gz * P.IMU_GYRO_LSB_RPS})
            for sid in range(min(tm.n_slaves, P.MAX_SLAVES)):
                sl = tm.slaves[sid]
                alive = st.spi[sid].alive
                slave_w.writerow({
                    "host_ts": ht, "cycle_id": tm.cycle_id, "slave": sid,
                    "alive": alive, "state": sl.state,
                    "state_name": P.SLAVE_STATE_NAMES.get(sl.state, sl.state),
                    "n_motors": sl.n_motors, "time_slave_us": sl.time_slave_us,
                    "spi_crc_errors": st.spi[sid].crc_errors,
                    "spi_seq_gaps": st.spi[sid].seq_gaps,
                    "boot_found_mask": sl.status.boot.found_mask,
                    "boot_unexpected_id": sl.status.boot.unexpected_id,
                    "boot_zero_sta_fail_mask": sl.status.boot.zero_sta_fail_mask,
                    "boot_version_fail_mask": sl.status.boot.version_fail_mask,
                    "cmd_crc_errors": sl.status.spi.cmd_crc_errors,
                    "resyncs": sl.status.spi.resyncs,
                    "tx_arm_fails": sl.status.spi.tx_arm_fails,
                    "seq_echo": sl.status.spi.seq_echo,
                    "can_tx_errors": sl.status.can.tx_errors,
                    "can_tec": sl.status.can.tec, "can_rec": sl.status.can.rec})
                if not alive:
                    continue
                for local in range(min(sl.n_motors, P.MAX_MOTORS_PER_SLAVE)):
                    mo = sl.motors[local]
                    ms_w.writerow({
                        "host_ts": ht, "master_time_s": mts,
                        "cycle_id": tm.cycle_id,
                        "motor": _GIDX.get((sid, local)),
                        "slave": sid, "local": local,
                        "state": mo.state,
                        "state_name": P.MOTOR_STATE_NAMES.get(mo.state, mo.state),
                        "fault": mo.fault,
                        "fault_name": P.MOTOR_FAULT_NAMES.get(mo.fault, mo.fault),
                        "flags": mo.flags,
                        "ready": int(bool(mo.flags & P.MOTOR_FLAG_READY)),
                        "request_rejected":
                            int(bool(mo.flags & P.MOTOR_FLAG_REQ_REJECTED)),
                        "saturated": int(bool(mo.flags & P.MOTOR_FLAG_SATURATED)),
                        "rs_mode": mo.rs.mode,
                        "rs_mode_name": P.RS_MODE_NAMES.get(mo.rs.mode, mo.rs.mode),
                        "rs_fault_bits": mo.rs.fault_bits,
                        "rs_fault_word": mo.rs.fault_word,
                        "pos": P.dec_i16(mo.rs.pos, P.POS_SCALE),
                        "vel": P.dec_i16(mo.rs.vel, P.VEL_SCALE),
                        "tau": P.dec_i16(mo.rs.tau, P.TAU_SCALE),
                        "temp": P.dec_i16(mo.rs.temp, P.TEMP_SCALE),
                        "time_slave_us": mo.rs.time_slave_us,
                        "last_applied_seq": mo.last_applied_seq})

        elif rec.kind == LOG.TX_FRAME:
            c["tx"] += 1
            if len(rec.payload) < P.HDR_SIZE:
                continue
            mt, _seq, pay_len, _crc = P._HDR.unpack_from(rec.payload, 0)
            inner = rec.payload[P.HDR_SIZE:P.HDR_SIZE + pay_len]
            if mt != P.USB_MSG_CMD:
                continue
            cm = P.parse_cmd_master(inner)
            if cm is None:
                continue
            n_slaves = (len(inner) - P._CMD_SLAVES_OFF) // 56
            for sid in range(min(n_slaves, P.MAX_SLAVES)):
                cs = cm.slaves[sid]
                for local in range(P.MAX_MOTORS_PER_SLAVE):
                    m = cs.motors[local]
                    cmd_w.writerow({
                        "host_ts": ht,
                        "motor": _GIDX.get((sid, local)),
                        "slave": sid, "local": local,
                        "req": m.req,
                        "req_name": P.MOTOR_REQ_NAMES.get(m.req, m.req),
                        "slave_req": cs.req,
                        "pos": P.dec_i16(m.mit.pos, P.POS_SCALE),
                        "vel": P.dec_i16(m.mit.vel, P.VEL_SCALE),
                        "kp": P.dec_u16(m.mit.kp, P.KP_SCALE),
                        "kd": P.dec_u16(m.mit.kd, P.KD_SCALE),
                        "tau_ff": P.dec_i16(m.mit.tau_ff, P.TAU_SCALE),
                        "cmd_seq": cm.cmd_seq, "cycle_id": cm.cycle_id})

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
    print(f"wrote {folder}/ (motor_state, motor_cmd, robot, slaves, imu, "
          f"events, loop_timing).csv")
    print(f"RX frames {c['rx']}  TX frames {c['tx']}  "
          f"discards {c['discard_records']} rec / {c['discard_bytes']} B")
    if c["dropped_records"]:
        print(f"** INCOMPLETE LOG: {c['dropped_records']} records DROPPED "
              f"in {c['drop_events']} window(s) (writer queue full) — see events.csv **")
    else:
        print("log complete: 0 records dropped")


if __name__ == "__main__":
    main()
