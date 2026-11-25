// RobStride RS-series actuator (RS00, RS02, RS03, RS05, RS06) on a CAN bus, emulated at the level of
// its private CAN protocol. One instance = one motor; the generated robot.resc declares one per
// motor of the setup and connects it to its slave's CAN hub.
//
// Source: the vendor CAN2.0 user manuals (github.com/RobStride/Product_Information). The protocol is
// identical on all five models: 29-bit extended id = type[28:24] | data area 2[23:8] | target[7:0],
// 1 Mbit/s. Only the MIT scaling ranges and the physical constants differ (ModelSpec below).
//
// Only what the slave firmware sends is implemented (firmware/slave/.../robostride.c,
// motor_runtime.c); anything else is logged and ignored, like a motor that does not answer:
//   Type 0  get device id   -> Type 0: id = (canId << 8) | 0xFE, payload = 8-byte UID
//   Type 1  MIT control     -> Type 2 feedback; the MIT law drives the rotor while in Run
//   Type 3  enable          -> Type 2; Run, refused while a fault is latched (*)
//   Type 4  stop            -> Type 2; Reset. Byte0 = 1 also clears latched faults
//   Type 6  set zero        -> Type 2; the shaft position becomes 0 (the MIT target too, so it holds)
//   Type 17 read parameter  -> Type 17 (0x3022 fault word, 0x7005 run mode; others: "read failed")
//   Type 18 write parameter -> Type 2 (only 0x7005, the run mode, is kept)
// (*) the manual does not say what Type 3 does with a latched fault; the firmware is written against
//     units that refuse (see can_clear_fault in robostride.c), so this model refuses.
//
// Rotor: J dw/dt = tau - b w - Fc sign(w) + LoadTorque, with tau = kp (p_des - p) + kd (v_des - v)
// + t_ff clamped to the model's peak torque while in Run with run_mode 0 (MIT), else 0. J is the
// vendor's output-side inertia. The kp/kd/b terms are integrated implicitly (backward Euler): with
// RS03/RS06 gains (Kd up to 100) explicit Euler at this step size is unstable. Friction b, Fc are not
// published: small placeholders. Good enough to show "the shaft follows the command"; not a model of
// the electrical, thermal or gearbox behaviour, and never timing evidence.
//
// Replies go out ReplyLatencyUs after the request (not published; default 300 us), serialised on
// the bus: Renode's CAN hub delivers a frame instantly, so five motors answering one burst would land
// in the same instant, while a real bus carries one frame at a time (~135 us for an 8-byte extended
// frame at 1 Mbit/s). Measured without this on setup 1s_5m: Renode's bxCAN model kept 3 of every 5
// simultaneous replies (its RX FIFO is 3 deep), the same two motors lost every reply and aged out to
// ABSENT -- an artifact no real bus produces. Requests from the slave still arrive instantly.
//
// Monitor hooks: Present (false = unplugged), RefuseEnable, IgnoreDisable (ADR-007 "failed joint
// isolation": acknowledges a stop but keeps driving), InjectFault, ClearFaults, LoadTorque,
// SetPosition, Status. Trace = true records every frame to and from this motor (candump-style,
// with virtual time); TakeTrace returns and clears the record (jetson.py `can on`).
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.CAN;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.CAN
{
    public class RobStrideMotor : ICAN, IDoubleWordPeripheral, IKnownSize
    {
        public RobStrideMotor(IMachine machine, int canId = 127, string model = "RS02",
                              double loadInertia = 0.0, double viscousFriction = 0.02, double coulombFriction = 0.02,
                              double initialPosition = 0.0, uint replyLatencyUs = 300)
        {
            if(!Models.TryGetValue(model, out spec))
            {
                throw new ArgumentException(string.Format("model '{0}' is not one of {1}", model, string.Join(", ", Models.Keys)));
            }
            if(canId < 1 || canId > 0x7F) throw new ArgumentException("canId must be 1..127");
            this.machine = machine;
            this.canId = (byte)canId;
            inertia = spec.Inertia + loadInertia;
            viscous = viscousFriction;
            coulomb = coulombFriction;
            this.initialPosition = initialPosition;
            ReplyLatencyUs = replyLatencyUs;
            Present = true;
            Reset();
        }

        public event Action<CANMessageFrame> FrameSent;

        // ── monitor-visible state and hooks ──────────────────────────────────────────────────────
        public string Model { get { return spec.Name; } }
        public bool Present { get; set; }
        public bool RefuseEnable { get; set; }
        public bool IgnoreDisable { get; set; }
        public uint ReplyLatencyUs { get; set; }
        public double LoadTorque { get; set; }     // constant external torque on the shaft (N*m)
        public double Position { get { lock(sync) { Advance(); return pos; } } }
        public double Velocity { get { lock(sync) { Advance(); return vel; } } }
        public double Torque { get { lock(sync) { Advance(); return tau; } } }
        public int Mode { get { return mode; } }   // 0 Reset, 1 Cali, 2 Run
        public uint FaultBits { get { return faultBits; } }
        public uint FramesHandled { get; private set; }
        public bool Trace { get; set; }

        public string TakeTrace()
        {
            lock(trace)
            {
                var s = string.Join("\n", trace);
                trace.Clear();
                return s;
            }
        }

        public string Status()
        {
            lock(sync)
            {
                Advance();
                return string.Format(System.Globalization.CultureInfo.InvariantCulture,
                    "id={0} model={1} mode={2} run_mode={3} pos={4:+0.0000;-0.0000;+0.0000} vel={5:+0.000;-0.000;+0.000} " +
                    "tau={6:+0.000;-0.000;+0.000} faults=0x{7:X2} word=0x{8:X8} frames={9} present={10}",
                    canId, spec.Name, mode == 0 ? "Reset" : mode == 2 ? "Run" : mode.ToString(), runMode,
                    pos, vel, tau, faultBits, faultWord, FramesHandled, Present ? 1 : 0);
            }
        }

        // Latch a fault by its Type-2 bit (0 undervoltage, 1 driver, 2 overtemperature, 3 encoder,
        // 4 stall/overload, 5 uncalibrated); the motor drops to Reset, like the real unit.
        public void InjectFault(int bit)
        {
            if(bit < 0 || bit > 5) throw new ArgumentException("fault bit must be 0..5");
            lock(sync)
            {
                Advance();
                faultBits |= 1u << bit;
                faultWord |= CompactToWord[bit];
                mode = 0;
            }
            this.Log(LogLevel.Info, "fault bit {0} injected; motor dropped to Reset", bit);
        }

        public void ClearFaults() { lock(sync) { faultBits = 0; faultWord = 0; } }

        public void SetPosition(double radians) { lock(sync) { Advance(); pos = radians; vel = 0; } }

        public void Reset()
        {
            lock(sync)
            {
                hostId = 0xFD; mode = 0; runMode = 0; faultBits = 0; faultWord = 0;
                pos = initialPosition; vel = 0; tau = 0;
                pDes = 0; vDes = 0; kp = 0; kd = 0; tff = 0;
                lastNs = NowNs();
                FramesHandled = 0;
            }
        }

        // A CAN device needs no bus address, but Renode only lets the monitor reach a peripheral by
        // name (`sysbus.motor1 Status`) if it is registered somewhere (`@ none` is not reachable), so
        // robot.resc maps each motor at an unused address. Read-only: 0x0 mode, 0x4 fault bits,
        // 0x8 frames handled, 0xC 0x3022 fault word.
        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case 0x0: return (uint)mode;
                case 0x4: return faultBits;
                case 0x8: return FramesHandled;
                case 0xC: return faultWord;
                default: return 0;
            }
        }
        public void WriteDoubleWord(long offset, uint value) { }
        public long Size { get { return 0x10; } }

        // ── frames from the bus ──────────────────────────────────────────────────────────────────
        public void OnFrameReceived(CANMessageFrame message)
        {
            if(!Present || !message.ExtendedFormat || message.Data == null) return;
            var id = message.Id;
            if((id & 0xFF) != canId) return;
            var type = (int)((id >> 24) & 0x1F);
            var field = (ushort)((id >> 8) & 0xFFFF);
            var d = new byte[8];
            Array.Copy(message.Data, d, Math.Min(8, message.Data.Length));

            if(Trace) Record(">", message);
            lock(sync)
            {
                FramesHandled++;
                Advance();
                if(type != 1) hostId = (byte)field;   // Type 1 carries torque in that field
                switch(type)
                {
                    case 0:
                        Send(Frame(0, canId, 0xFE, Uid()));
                        break;
                    case 1:
                        pDes = FromUint(d[0] << 8 | d[1], -PMax, PMax);
                        vDes = FromUint(d[2] << 8 | d[3], -spec.VMax, spec.VMax);
                        kp = FromUint(d[4] << 8 | d[5], 0.0, spec.KpMax);
                        kd = FromUint(d[6] << 8 | d[7], 0.0, spec.KdMax);
                        tff = FromUint(field, -spec.TMax, spec.TMax);
                        Send(Feedback());
                        break;
                    case 3:
                        if(faultBits == 0 && faultWord == 0 && !RefuseEnable) mode = 2;
                        Send(Feedback());
                        break;
                    case 4:
                        if(!IgnoreDisable) { mode = 0; tau = 0; }
                        if(d[0] == 1) { faultBits = 0; faultWord = 0; }
                        Send(Feedback());
                        break;
                    case 6:
                        if(d[0] == 1) { pos = 0; pDes = 0; }
                        Send(Feedback());
                        break;
                    case 17:
                    {
                        var index = d[0] | d[1] << 8;
                        var known = index == 0x3022 || index == 0x7005;
                        var value = index == 0x3022 ? faultWord : index == 0x7005 ? runMode : 0u;
                        var data = new byte[8];
                        data[0] = d[0]; data[1] = d[1];
                        data[4] = (byte)value; data[5] = (byte)(value >> 8); data[6] = (byte)(value >> 16); data[7] = (byte)(value >> 24);
                        Send(Frame(17, (ushort)((known ? 0 : 1) << 8 | canId), hostId, data));   // bits 23:16: 1 = read failed
                        break;
                    }
                    case 18:
                        if((d[0] | d[1] << 8) == 0x7005)
                        {
                            runMode = d[4];
                            if(runMode != 0) this.Log(LogLevel.Warning, "run_mode {0} is not modelled; only MIT (0) drives the rotor", runMode);
                        }
                        Send(Feedback());
                        break;
                    default:
                        this.Log(LogLevel.Debug, "ignoring communication type {0}", type);
                        break;
                }
            }
        }

        // ── plant ────────────────────────────────────────────────────────────────────────────────
        private void Advance()
        {
            var now = NowNs();
            var remaining = Math.Min((now - lastNs) / 1e9, 1.0);   // catch up at most 1 s per call
            lastNs = now;
            var driven = mode == 2 && runMode == 0;
            while(remaining > 0)
            {
                var h = Math.Min(Step, remaining);
                remaining -= h;
                var fc = Math.Abs(vel) > 1e-6 ? Math.Sign(vel) * coulomb : 0.0;
                double vNew, cmd = 0;
                if(driven)
                {
                    // Backward Euler on kp, kd and b: solve for the new velocity, then the torque.
                    vNew = (vel + h / inertia * (kp * (pDes - pos) + kd * vDes + tff + LoadTorque - fc))
                           / (1.0 + h / inertia * (kp * h + kd + viscous));
                    cmd = kp * (pDes - pos - h * vNew) + kd * (vDes - vNew) + tff;
                    if(Math.Abs(cmd) > spec.PeakTorque)
                    {
                        cmd = Math.Sign(cmd) * spec.PeakTorque;
                        vNew = (vel + h / inertia * (cmd + LoadTorque - fc)) / (1.0 + h * viscous / inertia);
                    }
                }
                else
                {
                    vNew = (vel + h / inertia * (LoadTorque - fc)) / (1.0 + h * viscous / inertia);
                }
                // Coulomb friction can stop the rotor but not reverse it: below breakaway it sticks.
                if((vel == 0 || vel * vNew < 0) && Math.Abs(cmd + LoadTorque) <= coulomb) vNew = 0;
                tau = cmd;
                vel = vNew;
                pos += h * vel;
            }
        }

        // ── frames out ───────────────────────────────────────────────────────────────────────────
        private CANMessageFrame Feedback()
        {
            var data = new byte[8];
            Put(data, 0, ToUint(Wrap(pos), -PMax, PMax));
            Put(data, 2, ToUint(vel, -spec.VMax, spec.VMax));
            Put(data, 4, ToUint(tau, -spec.TMax, spec.TMax));
            Put(data, 6, 300);                                         // 30.0 C
            var field = (ushort)(canId | (faultBits & 0x3Fu) << 8 | (uint)(mode & 3) << 14);
            return Frame(2, field, hostId, data);
        }

        private byte[] Uid()   // the manual gives no byte order; "EMU" + model digits + id, for logs
        {
            return new byte[] { (byte)'E', (byte)'M', (byte)'U', (byte)spec.Name[2], (byte)spec.Name[3], canId, 0, 0 };
        }

        private static CANMessageFrame Frame(int type, ushort dataField, uint target, byte[] data)
        {
            return new CANMessageFrame(((uint)(type & 0x1F) << 24) | ((uint)dataField << 8) | (target & 0xFF), data, true);
        }

        private void Send(CANMessageFrame frame)
        {
            var now = NowNs();
            long done;
            lock(BusFreeAt)   // one entry per machine = per slave = per CAN bus
            {
                long free;
                var start = now + ReplyLatencyUs * 1000L;
                if(BusFreeAt.TryGetValue(machine, out free) && free > start) start = free;
                done = start + FrameTimeNs;
                BusFreeAt[machine] = done;
            }
            machine.ScheduleAction(TimeInterval.FromMicroseconds((ulong)((done - now) / 1000)), _ =>
            {
                if(Trace) Record("<", frame);
                var handler = FrameSent;
                if(handler != null) handler(frame);
            });
        }

        // One line per frame: virtual time (s), direction (> to the motor, < from it), type, raw id,
        // data, and a decoded summary for MIT commands (Type 1) and feedback (Type 2).
        private void Record(string dir, CANMessageFrame f)
        {
            var type = (int)((f.Id >> 24) & 0x1F);
            var field = (int)((f.Id >> 8) & 0xFFFF);
            var d = f.Data;
            var what = "";
            if(type == 1)
            {
                what = string.Format(System.Globalization.CultureInfo.InvariantCulture, "MIT p={0:+0.000;-0.000;+0.000} v={1:+0.00;-0.00;+0.00} kp={2:0.0} kd={3:0.00} t_ff={4:+0.00;-0.00;+0.00}",
                    FromUint(d[0] << 8 | d[1], -PMax, PMax), FromUint(d[2] << 8 | d[3], -spec.VMax, spec.VMax),
                    FromUint(d[4] << 8 | d[5], 0, spec.KpMax), FromUint(d[6] << 8 | d[7], 0, spec.KdMax), FromUint(field, -spec.TMax, spec.TMax));
            }
            else if(type == 2)
            {
                var m = field >> 14;
                what = string.Format(System.Globalization.CultureInfo.InvariantCulture, "feedback mode={0} faults=0x{1:X2} p={2:+0.000;-0.000;+0.000} tau={3:+0.00;-0.00;+0.00}",
                    m == 0 ? "Reset" : m == 2 ? "Run" : m.ToString(), (field >> 8) & 0x3F,
                    FromUint(d[0] << 8 | d[1], -PMax, PMax), FromUint(d[4] << 8 | d[5], -spec.TMax, spec.TMax));
            }
            else
            {
                var names = new Dictionary<int, string> { { 0, "get id" }, { 3, "enable" }, { 4, "stop" }, { 6, "set zero" }, { 17, "read param" }, { 18, "write param" } };
                string n;
                what = names.TryGetValue(type, out n) ? n : "";
                if(type == 4 && d[0] == 1) what = "stop + clear fault";
                if(type == 17 || type == 18) what += string.Format(" 0x{0:X4}", d[0] | d[1] << 8);
            }
            var line = string.Format(System.Globalization.CultureInfo.InvariantCulture, "{0,11:0.000000} id{1,-3} {2} T{3,-2} {4:X8} {5}  {6}",
                NowNs() / 1e9, canId, dir, type, f.Id, BitConverter.ToString(d).Replace("-", " "), what);
            lock(trace) trace.Add(line);
        }

        // ── helpers ──────────────────────────────────────────────────────────────────────────────
        private long NowNs() { return (long)(machine.LocalTimeSource.ElapsedVirtualTime.TotalMilliseconds * 1e6); }
        private static double Wrap(double p)   // Type-2 position wraps within +-4pi (manual s4.2.3)
        {
            var x = (p + PMax) % (2 * PMax);
            return (x < 0 ? x + 2 * PMax : x) - PMax;
        }
        private static double FromUint(int raw, double lo, double hi) { return raw * (hi - lo) / 65535.0 + lo; }
        private static ushort ToUint(double x, double lo, double hi)
        {
            x = x < lo ? lo : (x > hi ? hi : x);
            return (ushort)((x - lo) * 65535.0 / (hi - lo));
        }
        private static void Put(byte[] b, int off, ushort v) { b[off] = (byte)(v >> 8); b[off + 1] = (byte)v; }

        // ── per-model data (vendor) ──────────────────────────────────────────────────────────────
        private class ModelSpec
        {
            public string Name;
            public double VMax, TMax, KpMax, KdMax;   // MIT scaling ranges: manual s4.1.2 / s4.4
            public double PeakTorque;                 // N*m, spec sheet
            public double Inertia;                    // kg*m^2, output-side equivalent, spec sheet
        }

        private static readonly Dictionary<string, ModelSpec> Models = new Dictionary<string, ModelSpec>
        {
            { "RS00", new ModelSpec { Name = "RS00", VMax = 33, TMax = 14,  KpMax = 500,  KdMax = 5,   PeakTorque = 14,  Inertia = 0.001 } },
            { "RS02", new ModelSpec { Name = "RS02", VMax = 44, TMax = 17,  KpMax = 500,  KdMax = 5,   PeakTorque = 17,  Inertia = 0.0042 } },
            { "RS03", new ModelSpec { Name = "RS03", VMax = 20, TMax = 60,  KpMax = 5000, KdMax = 100, PeakTorque = 60,  Inertia = 0.02 } },
            { "RS05", new ModelSpec { Name = "RS05", VMax = 50, TMax = 5.5, KpMax = 500,  KdMax = 5,   PeakTorque = 5.5, Inertia = 0.0007 } },
            { "RS06", new ModelSpec { Name = "RS06", VMax = 50, TMax = 36,  KpMax = 5000, KdMax = 100, PeakTorque = 36,  Inertia = 0.012 } },
        };

        // Type-2 fault bit -> 0x3022 fault-word bit. The manual documents both sets but no mapping;
        // this pairing is the emulator's choice: undervoltage->2, driver->1, overtemperature->0,
        // encoder->9 (position-init fault), stall->14, uncalibrated->7.
        private static readonly uint[] CompactToWord = { 1u << 2, 1u << 1, 1u << 0, 1u << 9, 1u << 14, 1u << 7 };

        private const double PMax = 12.57;       // +-4pi, as the firmware's P_MIN/P_MAX
        private const double Step = 0.0002;      // 200 us integration sub-step
        private const long FrameTimeNs = 135000; // 8-byte extended frame at 1 Mbit/s: 131 bits + stuffing

        private static readonly Dictionary<IMachine, long> BusFreeAt = new Dictionary<IMachine, long>();

        private readonly IMachine machine;
        private readonly ModelSpec spec;
        private readonly byte canId;
        private readonly double inertia, viscous, coulomb, initialPosition;
        private readonly object sync = new object();
        private readonly List<string> trace = new List<string>();
        private byte hostId, runMode;
        private int mode;
        private uint faultBits, faultWord;
        private double pos, vel, tau, pDes, vDes, kp, kd, tff;
        private long lastNs;
    }
}
