#!/usr/bin/env python3
"""Run the robot's firmware in Renode: build it, describe the robot to Renode, start it, talk to it.

    python3 robosim.py build [--setup S]   compile master + one slave ELF per slave of the setup
    python3 robosim.py resc  [--setup S]   write run/<S>/robot.resc (+ .repl files) and print the path
    python3 robosim.py serve [--setup S] [--pty PATH] [--tcp PORT]
                                           run the robot free; host link on a TCP port and a pty
    python3 robosim.py monitor [--setup S] open Renode's own monitor with the robot loaded (not started)

The setup (configs/<setup>/, the same switch the firmware build uses: --setup, else $SOCCER_SETUP,
else configs/active) decides how many slaves there are and which RobStride motors sit on each CAN
bus, so the emulated robot always matches the firmware it runs. Everything generated goes under
run/<setup>/ and build/<setup>/ (both gitignored).

As a library (jetson.py, test_sim.py):

    with Robot("1s_1m") as robot:         # builds nothing; run `build` first
        robot.run_ms(5)                    # advance virtual time (frozen otherwise)
        robot.send(frame_bytes)            # to the master's host link, at this virtual instant
        robot.take()                       # bytes the master sent since the last take()
        robot.motor(0, 1)                  # ground truth of slave 0's motor with CAN id 1

Run inside Linux/WSL. Virtual time is NOT wall time; nothing here is timing evidence (ADR-007).
"""
from __future__ import annotations

import argparse
import contextlib
import fcntl
import glob
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
CONFIGS = os.path.join(REPO, "configs")
GEN_SCRIPT = os.path.join(REPO, "scripts", "gen_motor_config.py")
GENERATED = [  # per-bench generated files the build rewrites; restored afterwards
    os.path.join(REPO, "firmware", "common", "include", "motor_config.h"),
    os.path.join(REPO, "firmware", "common", "include", "system_config.h"),
    os.path.join(REPO, "host", "master_link", "motor_config_gen.py"),
]
CS_PINS = [9, 8, 7, 6]          # master PC9..PC6 = SLAVE_CS_0..3 (firmware/master/Core/Inc/main.h)
MOTOR_WINDOW = 0x7F000000       # unused address range where motors are registered (see RobStrideMotor.cs)
MODELS = ("RS00", "RS02", "RS03", "RS05", "RS06")
_ANSI = re.compile(rb"\x1b\[[0-9;?]*[A-Za-z]|\xff[\xfb-\xfe].|\xff\xfa.*?\xff\xf0|\r")


# ── the robot as the setup describes it ──────────────────────────────────────────────────────────
@dataclass
class Motor:
    idx: int            # index within its chain (= position in the firmware's motor table)
    can_id: int
    model: str
    joint: str


@dataclass
class Slave:
    index: int          # = chain id = which master chip select (0..3)
    name: str           # "slave0" ...: the config section and the Renode machine name
    motors: list = field(default_factory=list)


def active_setup(explicit: str | None = None) -> str:
    setup = explicit or os.environ.get("SOCCER_SETUP", "").strip()
    if not setup:
        ptr = os.path.join(CONFIGS, "active")
        if os.path.isfile(ptr):
            setup = open(ptr).read().strip()
    if not setup:
        sys.exit("no setup selected: pass --setup NAME, set SOCCER_SETUP, or write configs/active")
    if not os.path.isdir(os.path.join(CONFIGS, setup)):
        sys.exit(f"setup {setup!r} has no directory configs/{setup}/")
    return setup


def topology(setup: str) -> list[Slave]:
    """Slaves and motors of a setup: configs/<setup>/firmware_config.yaml, else legacy slaveN.yaml."""
    artifact = os.path.join(CONFIGS, setup, "firmware_config.yaml")
    if os.path.isfile(artifact):
        sections = yaml.safe_load(open(artifact))["slaves"]
    else:
        files = sorted(glob.glob(os.path.join(CONFIGS, setup, "slave*.yaml")))
        if not files:
            sys.exit(f"configs/{setup}/ has neither firmware_config.yaml nor slave*.yaml")
        sections = [yaml.safe_load(open(f)) for f in files]
    slaves = []
    for i, sec in enumerate(sections):
        motors = [Motor(m["idx"], m["can_id"], m["model"], m.get("joint_name", f"motor{m['idx']}"))
                  for m in sec["motors"]]
        for m in motors:
            if m.model not in MODELS:
                sys.exit(f"{sec['slave']}: motor can_id {m.can_id} is a {m.model}; the emulator models {', '.join(MODELS)}")
        slaves.append(Slave(i, sec["slave"], sorted(motors, key=lambda m: m.idx)))
    if not 1 <= len(slaves) <= len(CS_PINS):
        sys.exit(f"setup {setup}: {len(slaves)} slaves; the master has {len(CS_PINS)} chip selects")
    return slaves


# ── build ─────────────────────────────────────────────────────────────────────────────────────────
def build(setup: str, jobs: int = os.cpu_count() or 4) -> None:
    """Compile build/<setup>/master.elf and build/<setup>/slaveN.elf.

    Each slave is compiled against its own motor_config.h, which gen_motor_config.py writes to one
    fixed path. The generated files are therefore regenerated per slave and put back as they were
    afterwards, so a sim build never changes what the CubeIDE build would pick up."""
    slaves = topology(setup)
    saved = {p: open(p, "rb").read() if os.path.isfile(p) else None for p in GENERATED}
    env = dict(os.environ, SOCCER_SETUP=setup)

    def run(*cmd):
        print("+", " ".join(cmd), flush=True)
        subprocess.run(cmd, cwd=HERE, env=env, check=True)

    try:
        run(sys.executable, GEN_SCRIPT, "--system")
        run("make", f"-j{jobs}", "master", f"SETUP={setup}")
        for s in slaves:
            run(sys.executable, GEN_SCRIPT, "--slave", s.name)
            run("make", f"-j{jobs}", "slave", f"SETUP={setup}", f"SLAVE={s.name}")
    finally:
        for p, data in saved.items():
            if data is None:
                with contextlib.suppress(FileNotFoundError):
                    os.remove(p)
            else:
                with open(p, "wb") as fh:
                    fh.write(data)
    print(f"built build/{setup}/: master.elf " + " ".join(f"{s.name}.elf" for s in slaves))


# ── describe the robot to Renode ──────────────────────────────────────────────────────────────────
def write_run_files(setup: str, hostlink: str = "buffer", tcp_port: int = 3456) -> str:
    """Write run/<setup>/robot.resc plus one .repl per machine; return the .resc path.

    hostlink: "buffer" = HostLink external (stepped, deterministic; jetson.py, test_sim.py)
              "socket" = Renode's raw server socket terminal on tcp_port (serve)."""
    slaves = topology(setup)
    run_dir = os.path.join(HERE, "run", setup)
    build_dir = os.path.join(HERE, "build", setup)
    os.makedirs(run_dir, exist_ok=True)
    for elf in ["master"] + [s.name for s in slaves]:
        if not os.path.isfile(os.path.join(build_dir, f"{elf}.elf")):
            sys.exit(f"build/{setup}/{elf}.elf is missing: run `python3 robosim.py build --setup {setup}`")
    p = lambda *parts: os.path.join(HERE, *parts)   # noqa: E731  (absolute paths: Renode needs them)
    banner = (f"GENERATED by sim/renode/robosim.py from configs/{setup}/ -- do not edit.\n"
              f"Regenerate: python3 robosim.py resc --setup {setup}")

    for s in slaves:
        lines = [f"// {banner.replace(chr(10), chr(10) + '// ')}", "//",
                 f"// {s.name}: chain {s.index}. The slave MCU plus the RobStride motors on its CAN bus.",
                 f'using "{p("platforms", "slave.repl")}"', ""]
        for k, m in enumerate(s.motors):
            lines += [f"// {m.joint}: motor index {m.idx}",
                      f"motor{m.can_id}: CAN.RobStrideMotor @ sysbus {MOTOR_WINDOW + 0x100 * k:#x}",
                      f"    canId: {m.can_id}", f'    model: "{m.model}"', ""]
        _write_atomic(os.path.join(run_dir, f"{s.name}.repl"), "\n".join(lines))

    lines = [f"// {banner.replace(chr(10), chr(10) + '// ')}", "//",
             "// The master MCU, plus the SPI wire to the slaves on its chip-select lines.",
             f'using "{p("platforms", "master.repl")}"', "",
             "spibridge: SPI.SpiSlaveBridge @ spi1",
             f'    slaves: "{",".join(s.name for s in slaves)}"', "",
             "gpioPortC:"]
    lines += [f"    {CS_PINS[s.index]} -> spibridge@{s.index}    // SLAVE_CS_{s.index} -> {s.name}" for s in slaves]
    _write_atomic(os.path.join(run_dir, "master.repl"), "\n".join(lines) + "\n")

    motors_total = sum(len(s.motors) for s in slaves)
    r = [f"# {banner.replace(chr(10), chr(10) + '# ')}", "#",
         f"# Setup {setup}: the master, {len(slaves)} slave(s), {motors_total} motor(s).",
         "# Renode runs this file: it creates one machine per MCU, gives each its board description",
         "# (.repl) and firmware (.elf), and wires the buses between them. Virtual time does not move",
         "# until something says `start` or `emulation RunFor`.", "",
         "# Custom peripherals, compiled by Renode when included (sim/renode/peripherals/).",
         f"include @{p('peripherals', 'RobStrideMotor.cs')}",
         f"include @{p('peripherals', 'SpiSlaveBridge.cs')}",
         f"include @{p('peripherals', 'HostLink.cs')}",
         f"include @{p('peripherals', 'CortexMDwt.cs')}", ""]
    for s in slaves:
        names = ", ".join(f"{m.model} id {m.can_id}" for m in s.motors)
        r += [f"# ── {s.name} (chain {s.index}; CAN bus can_{s.name}: {names}) " + "─" * 10,
              f'mach create "{s.name}"',
              f"machine LoadPlatformDescription @{os.path.join(run_dir, s.name + '.repl')}",
              f"sysbus LoadELF @{os.path.join(build_dir, s.name + '.elf')}",
              f"uart4 CreateFileBackend @{os.path.join(run_dir, s.name + '_uart4.log')} true",
              "# Model gap: this HAL sets CAN_MCR.INRQ while SLEEP (reset value 1) is still set; silicon",
              "# enters init mode, Renode's STMCAN never raises INAK and HAL_CAN_Init times out. Clear SLEEP.",
              "sysbus WriteDoubleWord 0x40006400 0x00010000",
              f'emulation CreateCANHub "can_{s.name}"',
              f"connector Connect sysbus.can1 can_{s.name}"]
        r += [f"connector Connect sysbus.motor{m.can_id} can_{s.name}" for m in s.motors]
        r.append("")
    r += ["# ── master " + "─" * 60,
          'mach create "master"',
          f"machine LoadPlatformDescription @{os.path.join(run_dir, 'master.repl')}",
          f"sysbus LoadELF @{os.path.join(build_dir, 'master.elf')}",
          f"uart4 CreateFileBackend @{os.path.join(run_dir, 'master_uart4.log')} true",
          "sysbus.spi1.spibridge SetupSlaves", "",
          "# Host link (USB CDC on the robot; USART2 in the sim firmware, see shim/master_host_uart.c)."]
    if hostlink == "socket":
        r += ["# Raw TCP (the final `false` = no telnet negotiation, which would double every 0xFF byte).",
              f'emulation CreateServerSocketTerminal {tcp_port} "hostlink" false']
    else:
        r += ["# A byte buffer the monitor fills and drains while time is paused (peripherals/HostLink.cs).",
              'emulation CreateHostLink "hostlink"']
    r += ["connector Connect sysbus.usart2 hostlink",
          "# usart2 is unpaced (frequency 0, see platforms/master.repl): silence its per-byte baud warning.",
          "logLevel 3 sysbus.usart2", ""]
    # One file per host-link kind, so a console and a `serve` of the same setup never swap scripts.
    resc = os.path.join(run_dir, "robot-serve.resc" if hostlink == "socket" else "robot.resc")
    _write_atomic(resc, "\n".join(r))
    return resc


def _write_atomic(path: str, text: str) -> None:
    """Write via a temp file and rename: several robots of one setup may start at once (parallel
    tests), and a Renode reading a half-rewritten robot.resc stops at the cut (seen: no `hostlink`)."""
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "w") as fh:
        fh.write(text)
    os.replace(tmp, path)


def renode_binary() -> str:
    for cand in (os.environ.get("RENODE"), os.path.expanduser("~/tools/renode/renode"), shutil.which("renode")):
        if cand and os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
    sys.exit("Renode not found: set RENODE=/path/to/renode (Renode 1.17+)")


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# ── a running robot ───────────────────────────────────────────────────────────────────────────────
class Robot:
    """Renode with the robot loaded, driven through its monitor socket. Time is frozen unless
    run_ms() (or start()) moves it."""

    def __init__(self, setup: str | None = None, hostlink: str = "buffer", tcp_port: int = 3456):
        self.setup = active_setup(setup)
        self.slaves = topology(self.setup)
        self.hostlink = hostlink
        self.tcp_port = tcp_port
        self.resc = write_run_files(self.setup, hostlink, tcp_port)
        self.log_path = os.path.join(HERE, "run", self.setup, "renode.log")
        # One Renode starts at a time across processes: in parallel test runs, 2 of 12 simultaneous
        # starts came up without the `hostlink` external (the C# peripherals are compiled at load).
        with open(os.path.join(HERE, "run", ".renode-start.lock"), "w") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            self.port = _free_port()
            self.proc = subprocess.Popen(
                [renode_binary(), "--disable-xwt", "--port", str(self.port), "-e", f"include @{self.resc}"],
                stdin=subprocess.DEVNULL, stdout=open(self.log_path, "wb"), stderr=subprocess.STDOUT,
                cwd=os.path.join(HERE, "run", self.setup))
            self._mon = self._connect(self.port)
            self._lock = threading.Lock()
            self._seq = 0
            self._sync()                             # script loaded, C# compiled, machines created
            self._check_loaded()
        self.vt_ms = 0.0                             # virtual time this object has run

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _connect(self, port: int, timeout: float = 120.0) -> socket.socket:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError(f"Renode exited early; see {self.log_path}")
            try:
                return socket.create_connection(("127.0.0.1", port), timeout=5)
            except OSError:
                time.sleep(0.2)
        raise RuntimeError(f"no Renode monitor on port {port}; see {self.log_path}")

    def _read_until(self, marker: str, timeout: float) -> str:
        """Read monitor output until `marker` appears on a line of its own; return the cleaned text."""
        self._mon.settimeout(timeout)
        data = b""
        pattern = re.compile(r"(^|\n)" + re.escape(marker) + r"\s*\n")
        while True:
            chunk = self._mon.recv(1 << 16)
            if not chunk:
                raise RuntimeError(f"Renode closed the monitor; see {self.log_path}")
            data += chunk
            text = _ANSI.sub(b"", data).decode(errors="replace")
            if pattern.search(text):
                return text

    def _sync(self, timeout: float = 180.0) -> None:
        """Wait until every command queued so far (including the startup script) has run."""
        self._seq += 1
        self._mon.sendall(f'echo "@@sync{self._seq}"\n'.encode())
        self._read_until(f"@@sync{self._seq}", timeout)

    def monitor(self, command: str, timeout: float = 600.0) -> str:
        """Run one monitor command; return what it printed (without the echo and the prompt).

        The command is followed by `echo "@@<n>"`; everything the monitor prints before that
        marker belongs to the command, so replies can never be attributed to the wrong command."""
        with self._lock:
            self._seq += 1
            marker = f"@@{self._seq}"
            self._mon.sendall(f'{command}\necho "{marker}"\n'.encode())
            text = self._read_until(marker, timeout)
        text = text[:text.rfind(f'echo "{marker}"')]          # drop the marker command's echo
        lines = text.split("\n")
        body = "\n".join(lines[1:-1]).strip() if len(lines) > 1 else ""   # drop command echo + prompt
        if ("There was an error executing command" in text or "No such command or device" in text
                or "Errors during compilation" in text):
            raise RuntimeError(f"monitor: {command!r} failed:\n{body}")
        return body

    def _check_loaded(self) -> None:
        """The startup script stops at its first error (e.g. a C# compile error), and that error goes
        to the monitor, not to the log. Make sure every machine exists before going on."""
        for what in [f'mach set "{m}"' for m in ["master"] + [s.name for s in self.slaves]] + ["hostlink"]:
            try:
                self.monitor(what)
            except RuntimeError:
                raise RuntimeError(f"robot.resc did not finish loading ({what!r} failed); run it by hand to see "
                                   f"why: renode --console -e 'include @{self.resc}'") from None

    def on(self, machine: str, command: str) -> str:
        """Run a monitor command in the context of one machine ("master", "slave0", ...)."""
        return self.monitor(f'mach set "{machine}"; {command}')

    # time
    def run_ms(self, ms: float) -> None:
        us = int(round(ms * 1000))
        self.monitor(f'emulation RunFor "{us // 3_600_000_000:02d}:{us // 60_000_000 % 60:02d}:'
                     f'{us // 1_000_000 % 60:02d}.{us % 1_000_000:06d}"')
        self.vt_ms += us / 1000.0

    def start(self) -> None:
        self.monitor("start")

    # host link (buffer mode)
    def send(self, data: bytes) -> None:
        for i in range(0, len(data), 128):           # keep monitor lines short
            chunk = data[i:i + 128]
            reply = self.monitor(f'hostlink Send "{chunk.hex()}"')
            got = re.findall(r"0x[0-9A-Fa-f]+|\d+", reply)
            if not got or int(got[-1], 0) != len(chunk):
                raise RuntimeError(f"hostlink accepted {reply!r}, expected {len(chunk)} bytes")

    def take(self) -> bytes:
        text = self.monitor("hostlink Take")
        return bytes.fromhex(re.sub(r"[^0-9a-fA-F]", "", text))

    # motors and wire
    def motor(self, slave: int, can_id: int) -> dict:
        """Ground truth of one emulated motor, from its own model (not from the firmware)."""
        text = self.on(self.slaves[slave].name, f"sysbus.motor{can_id} Status")
        out = {}
        for k, v in re.findall(r"(\w+)=(\S+)", text):
            with contextlib.suppress(ValueError):
                v = int(v, 16) if v.startswith("0x") else (float(v) if re.match(r"^[+-]?[\d.]+$", v) else v)
            out[k] = v
        return out

    def motor_do(self, slave: int, can_id: int, action: str) -> str:
        """Call a fault-injection hook, e.g. motor_do(0, 1, "InjectFault 2")."""
        return self.on(self.slaves[slave].name, f"sysbus.motor{can_id} {action}")

    def wire(self) -> list[dict]:
        text = self.on("master", "sysbus.spi1.spibridge Counters")
        out = []
        for part in text.replace("\n", " ").split(";"):
            d = {}
            for k, v in re.findall(r"(\w+)=(\S+)", part):
                d[k] = int(v) if v.isdigit() else v
            if d:
                out.append(d)
        return out

    def close(self) -> None:
        with contextlib.suppress(OSError):
            self._mon.sendall(b"quit\n")
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.proc.kill()


# ── serve: a live host link for a real host program ───────────────────────────────────────────────
def _pty_relay(tcp_port: int, link: str, stop: threading.Event, stats: dict) -> None:
    """Relay bytes between a pseudo-terminal (what a host program opens as its serial device) and
    Renode's host-link TCP socket. Bytes for a pty nobody reads are dropped, as cdc_acm drops data
    for a closed tty."""
    import pty
    import select
    import tty
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)                     # no echo, no line discipline, before anyone opens it
    os.set_blocking(master_fd, False)
    with contextlib.suppress(FileNotFoundError):
        os.remove(link)
    os.symlink(os.ttyname(slave_fd), link)
    sock = None
    while not stop.is_set() and sock is None:
        with contextlib.suppress(OSError):
            sock = socket.create_connection(("127.0.0.1", tcp_port), timeout=1)
        time.sleep(0.2)
    if sock is None:
        return
    sock.setblocking(False)
    try:
        while not stop.is_set():
            r, _, _ = select.select([sock, master_fd], [], [], 0.2)
            if sock in r:
                data = sock.recv(1 << 16)
                if not data:
                    break
                stats["to_host"] += len(data)
                try:
                    os.write(master_fd, data)
                except BlockingIOError:
                    stats["dropped"] += len(data)
            if master_fd in r:
                with contextlib.suppress(BlockingIOError, OSError):
                    data = os.read(master_fd, 1 << 16)
                    stats["to_master"] += len(data)
                    sock.sendall(data)
    finally:
        sock.close()
        os.close(master_fd)
        os.close(slave_fd)
        with contextlib.suppress(FileNotFoundError):
            os.remove(link)


def serve(setup: str | None, pty_link: str, tcp_port: int) -> None:
    robot = Robot(setup, hostlink="socket", tcp_port=tcp_port)
    stop, stats = threading.Event(), dict(to_host=0, to_master=0, dropped=0)
    relay = threading.Thread(target=_pty_relay, args=(tcp_port, pty_link, stop, stats), daemon=True)
    relay.start()
    robot.start()
    print(f"robot {robot.setup} running (virtual time, typically 3-5x slower than wall time)\n"
          f"  host link: serial device {pty_link}  (also raw TCP 127.0.0.1:{tcp_port})\n"
          f"  log:       {robot.log_path}\n"
          "Type Renode monitor commands (e.g. `mach set \"slave0\"; sysbus.motor1 InjectFault 2`),\n"
          "`stats`, or `quit`.")
    t0 = time.time()
    try:
        while True:
            try:
                line = input("robosim> ").strip()
            except EOFError:
                break
            if line in ("quit", "exit"):
                break
            if line == "stats":
                vt = robot.on("master", "machine LocalTimeSource ElapsedVirtualTime")
                print(f"  wall {time.time() - t0:.1f} s, virtual {vt.strip()}, bytes to host {stats['to_host']}, "
                      f"to master {stats['to_master']}, dropped (pty not read) {stats['dropped']}")
            elif line:
                try:
                    print(robot.monitor(line))
                except RuntimeError as e:
                    print(e)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        robot.close()


# ── CLI ───────────────────────────────────────────────────────────────────────────────────────────
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("command", choices=["build", "resc", "serve", "monitor"])
    ap.add_argument("--setup", help="configs/<setup> (default: $SOCCER_SETUP, else configs/active)")
    ap.add_argument("--pty", default="/tmp/robosoccer-master", help="serve: serial device path to create")
    ap.add_argument("--tcp", type=int, default=3456, help="serve: host-link TCP port")
    a = ap.parse_args()
    setup = active_setup(a.setup)
    if a.command == "build":
        build(setup)
    elif a.command == "resc":
        print(write_run_files(setup))
    elif a.command == "serve":
        serve(setup, a.pty, a.tcp)
    elif a.command == "monitor":
        resc = write_run_files(setup, hostlink="socket", tcp_port=a.tcp)
        os.execv(renode_binary(), [renode_binary(), "--disable-xwt", "--console", "-e", f"include @{resc}"])


if __name__ == "__main__":
    main()
