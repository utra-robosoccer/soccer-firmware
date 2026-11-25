#!/usr/bin/env python3
"""Be the Jetson by hand: a console on the master's host link, with virtual time frozen between
your commands.

    python3 jetson.py [--setup 1s_1m]        (Linux/WSL; build first: python3 robosim.py build)

Nothing happens in the firmware until you advance time (`boot`, `step`, `run`). Frames you send
land at the current virtual instant, so the same commands always give the same numbers. Framing,
CRCs and the decoder are the production ones (host/master_link/protocol.py); the firmware is the
production source built for Renode (see README.md for what is substituted).

Type `help` for the commands, `help <command>` for details.
"""
import argparse
import cmd
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import robosim  # noqa: E402

REQS = {"DISABLE": 0, "ENABLE": 1, "STOP": 2, "RESET": 3}


class Host:
    """The Jetson's side of the protocol with no console: sends ROBOT_CMD frames, keeps them coming
    at the host rate while time advances, and decodes what the master sends back. Used by the
    console below and by test_sim.py."""

    def __init__(self, robot: robosim.Robot):
        os.environ["SOCCER_SETUP"] = robot.setup         # master_link regenerates its constants on import
        sys.path.insert(0, os.path.join(robosim.REPO, "host"))
        from master_link import protocol
        self.P = protocol
        self.robot = robot
        self.buf = bytearray()
        self.last = {}             # msg type -> payload of the newest frame
        self.raws = {}             # msg type -> raw bytes (header + payload) of the newest frame
        self.held = {}             # (chain, motor index) -> cmd_motor dict the "host loop" keeps sending
        self.cmd_seq = 0
        self.last_send_ms = -1e9
        self.cycle_ms, self.host_period_ms = 5.0, 20.0   # replaced by MASTER_STATUS once it arrives

    def advance(self, ms: float):
        """Advance virtual time by `ms`; return the message types received. While commands are held,
        act like the real host loop and re-send them every host period: the master's host-death
        watchdog trips within a few periods of silence once a motor is armed."""
        got, left = [], ms
        while left > 1e-9:
            if self.held and self.robot.vt_ms - self.last_send_ms >= self.host_period_ms - 1e-9:
                self.send_held()
            chunk = min(self.cycle_ms if self.held else 250.0, left)
            self.robot.run_ms(chunk)
            left -= chunk
            got += self.drain()
        return got

    def send_held(self):
        P = self.P
        self.cmd_seq = self.cmd_seq % 65535 + 1
        chains = []
        for s in self.robot.slaves:
            motors = [self.held.get((s.index, m.idx), dict(drive_req=P.DRV_REQ_DISABLE)) for m in s.motors]
            chains.append(dict(chain_id=s.index, motors=motors))
        # Echo the newest telemetry cycle, as the real host does: the master counts a command
        # on_time only if it answers the latest cycle (firmware/common/include/cmd_mailbox.h).
        d = self.tele()
        payload = P.pack_robot_cmd(d["cycle_id"] if d else 0, self.cmd_seq, chains)
        self.robot.send(P.encode_frame(P.MSG_ROBOT_CMD, P.NODE_JETSON, P.NODE_MASTER, payload))
        self.last_send_ms = self.robot.vt_ms

    def drain(self):
        P = self.P
        self.buf += self.robot.take()
        got = []
        while True:
            raw = bytes(self.buf)
            r = P.decode_frame(self.buf)
            if r is None:
                return got
            t, seq, ts, pay, n = r
            del self.buf[:n]
            self.last[t], self.raws[t] = pay, raw[:n]
            if t == P.MSG_MASTER_STATUS:
                st = P.parse_master_status(pay)
                if st.get("master_poll_hz"):
                    self.cycle_ms = 1000.0 / st["master_poll_hz"]
                if st.get("host_cmd_hz"):
                    self.host_period_ms = 1000.0 / st["host_cmd_hz"]
            got.append(t)

    def tele(self):
        P = self.P
        return P.parse_robot_tele(self.last[P.MSG_ROBOT_TELE]) if P.MSG_ROBOT_TELE in self.last else None

    def status(self, which):
        P = self.P
        t, f = {"master": (P.MSG_MASTER_STATUS, P.parse_master_status),
                "slave": (P.MSG_SLAVE_STATUS, P.parse_slave_status)}[which]
        return f(self.last[t]) if t in self.last else None

    def motors(self):
        """{(chain, motor index): TeleMotor} from the newest telemetry."""
        d = self.tele() or {"chains": []}
        return {(c["chain_id"], i): m for c in d["chains"] for i, m in enumerate(c["motors"])}

    def targets(self, spec=None):
        """None (every motor) or 'c.m' (chain.motor index) -> list of (chain, motor index)."""
        if spec in (None, "all"):
            return [(s.index, m.idx) for s in self.robot.slaves for m in s.motors]
        c, m = (int(x) for x in spec.split("."))
        return [(c, m)]

    def can_id(self, target):
        c, m = target
        return next(x.can_id for x in self.robot.slaves[c].motors if x.idx == m)

    def hold(self, req: int, targets, pos=None):
        """Hold drive request `req` for `targets` at `pos` (default: where each motor is now)."""
        now = self.motors()
        for t in targets:
            here = now[t].pos if t in now else 0.0
            self.held[t] = dict(drive_req=req, pos=here if pos is None else pos,
                                flags=self.P.CMD_FLAG_VALID | self.P.CMD_FLAG_USE_CONFIG_GAINS)
        self.send_held()

    def boot(self, max_ms: float = 12000):
        """Run until every chain is ALIVE; return the message types received."""
        P, got = self.P, []
        while self.robot.vt_ms < max_ms:
            got += self.advance(250)
            d = self.tele()
            if d and d["chains"] and all(c["link"] == P.CHAIN_LINK_ALIVE for c in d["chains"]):
                break
        return got


class Jetson(cmd.Cmd):
    intro = "\nHand-driven Jetson. Virtual time is frozen; `help` lists commands; start with `boot`.\n"
    prompt = "jetson> "

    def __init__(self, robot: robosim.Robot):
        super().__init__()
        self.host = Host(robot)
        self.P = self.host.P
        self.robot = robot
        self.can_trace = False

    def advance(self, ms):
        got = self.host.advance(ms)
        if self.can_trace:
            self.print_can()
        return got

    def print_can(self):
        lines = []
        for s in self.robot.slaves:
            for m in s.motors:
                text = self.robot.on(s.name, f"sysbus.motor{m.can_id} TakeTrace")
                for x in text.splitlines():
                    if x.strip()[:1].isdigit():
                        t, rest = x.split(None, 1)
                        lines.append((float(t), s.name, rest))
        for t, name, rest in sorted(lines):
            print(f"  {name} {t:10.6f} {rest}")

    def tele(self):
        return self.host.tele()

    def summary(self, got):
        P = self.P
        counts = {}
        for t in got:
            counts[P.MSG_NAMES.get(t, t)] = counts.get(P.MSG_NAMES.get(t, t), 0) + 1
        print(f"[t={self.robot.vt_ms / 1000:8.3f} s] received: " +
              (", ".join(f"{k} x{v}" for k, v in counts.items()) or "nothing"))
        d = self.tele()
        if not d:
            return
        print(f"  cycle {d['cycle_id']}  robot {P.ROBOT_STATE_NAMES.get(d['robot_state'])}  "
              f"host cmds: on_time {d['cmd_on_time']} late {d['cmd_late']} missing {d['cmd_missing']}")
        flag_names = (("REJECTED", P.TELE_FLAG_REQUEST_REJECTED), ("SATURATED", P.TELE_FLAG_SATURATED),
                      ("CLAMPED_POS", P.TELE_FLAG_CLAMPED_POS), ("CLAMPED_TAU", P.TELE_FLAG_CLAMPED_TAU),
                      ("CMD_STALE", P.TELE_FLAG_CMD_STALE))
        for c in d["chains"]:
            print(f"  chain {c['chain_id']}: {P.CHAIN_LINK_NAMES.get(c['link'], c['link'])}")
            for i, m in enumerate(c["motors"]):
                flags = "|".join(n for n, b in flag_names if m.flags & b)
                print(f"    m{i}: {m.drive_state_name:<11} fault={m.drive_fault_name:<14} rs_mode={m.rs_mode_name:<6} "
                      f"pos={m.pos:+.3f} tau={m.tau:+.2f} fb_age={m.fb_age_ms}ms" + (f"  {flags}" if flags else ""))

    # ── commands ─────────────────────────────────────────────────────────────────────────────────
    def do_boot(self, arg):
        """boot -- run until every chain is ALIVE (at most 12 s of virtual time).
        The master waits ~3 s before its first cycle; each slave waits 2.5 s for the motors to
        power up, then discovers them over CAN before it starts answering SPI."""
        self.summary(self.host.boot())

    def do_step(self, arg):
        """step [N] -- advance N master cycles (default 1) and show what arrived."""
        self.summary(self.advance((int(arg) if arg.strip() else 1) * self.host.cycle_ms))

    def do_run(self, arg):
        """run MS -- advance MS milliseconds of virtual time."""
        self.summary(self.advance(float(arg)))

    def do_tele(self, arg):
        """tele -- every field of the newest ROBOT_TELE."""
        d = self.tele()
        if not d:
            return print("no ROBOT_TELE yet -- `boot` first")
        chains = d.pop("chains")
        for k, v in d.items():
            print(f"  {k:18} {v}")
        for c in chains:
            print("  chain", {k: v for k, v in c.items() if k != "motors"})
            for m in c["motors"]:
                print("    ", m)

    def do_status(self, arg):
        """status -- newest MASTER_STATUS and SLAVE_STATUS (sent every 10th cycle)."""
        for which in ("master", "slave"):
            st = self.host.status(which)
            if st:
                print(f"  {which.upper()}_STATUS: {st}")

    def do_ping(self, arg):
        """ping -- send MSG_PING (header only); the master echoes it. `step` to let it answer."""
        P = self.P
        frame = P.encode_frame(P.MSG_PING, P.NODE_JETSON, P.NODE_MASTER)
        self.robot.send(frame)
        print(f"sent PING seq={struct.unpack(P.HDR_FMT, frame[:P.HDR_SIZE])[1]}")

    def do_cmd(self, arg):
        """cmd REQ [pos] [@c.m] -- hold a ROBOT_CMD request (default: every motor).
        REQ: DISABLE | ENABLE | STOP | RESET. pos (rad) defaults to each motor's current position
        (a hold); the config's default gains are used. @c.m picks chain c, motor index m.
        The command goes out now and every host period while time advances, until `release`."""
        a = arg.split()
        if not a or a[0].upper() not in REQS:
            return print("usage: cmd DISABLE|ENABLE|STOP|RESET [pos] [@c.m]")
        target = next((x[1:] for x in a[1:] if x.startswith("@")), None)
        nums = [x for x in a[1:] if not x.startswith("@")]
        targets = self.host.targets(target)
        self.host.hold(REQS[a[0].upper()], targets, float(nums[0]) if nums else None)
        print(f"holding {a[0].upper()} for {', '.join(f'{c}.{m}' for c, m in targets)} "
              f"(cmd_seq {self.host.cmd_seq}); re-sent every {self.host.host_period_ms:.0f} ms while time advances")

    def do_release(self, arg):
        """release -- stop sending commands: the host goes silent (the host-death watchdog's cue)."""
        self.host.held.clear()
        print("host is now silent")

    def do_motor(self, arg):
        """motor [@c.m] [ACTION] -- the emulated motor itself (ground truth, not what the firmware thinks).
        ACTION: unplug | plug | fault BIT (0 undervolt, 1 driver, 2 overtemp, 3 encoder, 4 stall,
        5 uncalibrated) | clear | refuse | accept | ignore-disable | obey-disable | load NM | temp C | vbus V"""
        a = arg.split()
        target = self.host.targets(a.pop(0)[1:] if a and a[0].startswith("@") else "0.0")[0]
        slave, can_id = target[0], self.host.can_id(target)
        hooks = {"unplug": "Present false", "plug": "Present true", "clear": "ClearFaults",
                 "refuse": "RefuseEnable true", "accept": "RefuseEnable false",
                 "ignore-disable": "IgnoreDisable true", "obey-disable": "IgnoreDisable false"}
        valued = {"fault": "InjectFault {}", "load": "LoadTorque {}", "temp": "Temperature {}", "vbus": "BusVoltage {}"}
        if not a:
            print("  " + "  ".join(f"{k}={v}" for k, v in self.robot.motor(slave, can_id).items()))
            return
        if a[0] in hooks:
            action = hooks[a[0]]
        elif a[0] in valued and len(a) == 2:
            action = valued[a[0]].format(a[1])
        else:
            return print(self.do_motor.__doc__)
        self.robot.motor_do(slave, can_id, action)
        print(f"  motor{can_id} on {self.robot.slaves[slave].name}: {action}")

    def do_can(self, arg):
        """can on|off -- print every CAN frame to and from the emulated motors after each step, like
        candump: virtual time (s), motor id, > to the motor / < from it, type, raw id, data, decoded."""
        on = arg.strip().lower() != "off"
        for s in self.robot.slaves:
            for m in s.motors:
                self.robot.on(s.name, f"sysbus.motor{m.can_id} Trace {'true' if on else 'false'}")
                self.robot.on(s.name, f"sysbus.motor{m.can_id} TakeTrace")
        self.can_trace = on
        print(f"  CAN trace {'on' if on else 'off'}")

    def do_wire(self, arg):
        """wire -- the SPI bridge's own counters per slave (what the wire did, not what firmware counted)."""
        for w in self.robot.wire():
            print("  " + "  ".join(f"{k}={v}" for k, v in w.items()))

    def do_frame(self, arg):
        """frame [tele|status|slave|ping] -- the 16-byte header of the newest such frame, field by field."""
        P = self.P
        t = {"tele": P.MSG_ROBOT_TELE, "status": P.MSG_MASTER_STATUS,
             "slave": P.MSG_SLAVE_STATUS, "ping": P.MSG_PING}.get(arg.strip() or "tele")
        raw = self.host.raws.get(t)
        if not raw:
            return print("  none received yet")
        f = struct.unpack(P.HDR_FMT, raw[:P.HDR_SIZE])
        print("  bytes   ", raw[:P.HDR_SIZE].hex(" "))
        print(f"  type     {f[0]:#04x} ({P.MSG_NAMES.get(f[0], '?')})")
        print(f"  seq      {f[1]}  (sender's frame counter)")
        print(f"  src,dst  {f[2]}, {f[3]}  (node ids: 1 Jetson, 2 master)")
        print(f"  ts_ms    {f[4]}  (sender's clock when built)")
        print(f"  pay_len  {f[5]}  (frame = {P.HDR_SIZE + f[5]} B)")
        print(f"  version  {f[6] & 0xFF}  (receivers drop other protocol versions)")
        print(f"  crc16    {raw[14] | raw[15] << 8:#06x}  (CCITT over header-with-crc-zeroed + payload)")

    def do_hex(self, arg):
        """hex [tele|status|slave] -- hex dump of the newest frame's payload."""
        P = self.P
        t = {"tele": P.MSG_ROBOT_TELE, "status": P.MSG_MASTER_STATUS,
             "slave": P.MSG_SLAVE_STATUS}.get(arg.strip() or "tele")
        pay = self.host.last.get(t, b"")
        for i in range(0, len(pay), 16):
            print(f"  {i:04x}  {pay[i:i + 16].hex(' ')}")

    def do_junk(self, arg):
        """junk [N] -- send N garbage bytes (default 7); watch link_errors / rx_resyncs in `status`."""
        n = int(arg) if arg.strip() else 7
        self.robot.send(bytes([0xA5] * n))
        print(f"sent {n} x 0xA5")

    def do_mon(self, arg):
        """mon COMMAND -- pass a command straight to the Renode monitor, e.g.
        `mon mach set "slave0"; sysbus.motor1 Status` or `mon mach set "master"; cpu PC`."""
        try:
            print(self.robot.monitor(arg))
        except RuntimeError as e:
            print(e)

    def do_quit(self, arg):
        """quit -- stop Renode and exit."""
        return True
    do_EOF = do_quit

    def emptyline(self):
        pass


def main(argv=None, console=Jetson):
    ap = argparse.ArgumentParser(description="hand-driven Jetson console")
    ap.add_argument("--setup", help="configs/<setup> (default: $SOCCER_SETUP, else configs/active)")
    a = ap.parse_args(argv)
    print("starting Renode (compiling the C# peripherals takes a few seconds)...")
    with robosim.Robot(a.setup) as robot:
        console(robot).cmdloop()


if __name__ == "__main__":
    main()
