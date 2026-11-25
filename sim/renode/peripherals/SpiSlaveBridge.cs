// The SPI wire between the master MCU and up to four slave MCUs, each running in its own Renode
// machine. Renode's STM32SPI can only be a bus master and the stock platform wires no peripheral DMA,
// so the slaves' DMA-driven SPI cannot run as-is; this class stands in for "the wire + the slave's
// SPI peripheral + its DMA streams".
//
// Master side: the child ISPIPeripheral of the master's spi1. Its GPIO inputs 0..3 are the chip
// selects (master PC9, PC8, PC7, PC6 = SLAVE_CS_0..3; low = selected), in the order of `slaves`.
//
// Slave side: the slave firmware is built with shim/slave_spi_dma.c, which records each armed
// "DMA" transfer in plain globals (sim_spi_armed / _txp / _rxp / _len / _arm_count). The bridge finds
// them through the slave ELF's symbol table, reads the armed TX buffer out of slave RAM when CS
// falls, and, when CS rises, writes the master's bytes into the armed RX buffer, raises the slave's
// NSS pin (PA4) and pulses SPI3_IRQn, whose handler runs the real HAL_SPI_TxRxCpltCallback. Every
// slave-side effect is posted into the slave machine's time domain at the master's timestamp.
//
// Kept from real hardware:
//   * the slave answers with whatever TX buffer it had armed BEFORE the exchange began;
//   * bytes are consumed progressively, so a short exchange leaves the slave's DMA part-way (the
//     misalignment the firmware's resync logic exists for), and the next exchange continues there;
//   * an exchange while the slave has nothing armed returns 0x00 and is lost to the slave.
// Not modelled: byte-level timing, SPI overrun/underrun, MISO contention.
using System;
using System.Collections.Generic;
using System.Linq;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Time;
using Antmicro.Renode.Utilities;   // TimeDomainsManager.GetEffectiveVirtualTimeStamp()

namespace Antmicro.Renode.Peripherals.SPI
{
    public class SpiSlaveBridge : ISPIPeripheral, IGPIOReceiver
    {
        public SpiSlaveBridge(IMachine machine, string slaves = "slave0", int slaveIrq = 51, int nssPin = 4)
        {
            this.slaveIrq = slaveIrq;
            this.nssPin = nssPin;
            var names = slaves.Split(',').Select(s => s.Trim()).Where(s => s.Length > 0).ToArray();
            if(names.Length < 1 || names.Length > 4) throw new ArgumentException("slaves: 1..4 comma-separated machine names");
            ends = names.Select(n => new SlaveEnd(n)).ToArray();
            Reset();
        }

        public void Reset()
        {
            active = null;
            foreach(var e in ends) e.Clear();
        }

        // ── monitor hooks ────────────────────────────────────────────────────────────────────────
        // One line per slave: what the wire did, independent of any firmware counter.
        public string Counters()
        {
            return string.Join("; ", ends.Select((e, i) => string.Format(
                "slave{0}={1} exchanges={2} completed={3} not_armed={4} partial={5} present={6}",
                i, e.Name, e.Exchanges, e.Completed, e.NotArmed, e.Partial, e.Present ? 1 : 0)));
        }

        // "Unplug" a slave: its chip select still toggles but nothing answers (MISO reads 0x00).
        public void Unplug(int slave) { ends[slave].Present = false; }
        public void Plug(int slave) { ends[slave].Present = true; }

        // Call once after every machine exists: puts each slave's NSS input high (idle).
        public void SetupSlaves()
        {
            foreach(var e in ends) Post(e, high => SetNss(e, high), true);
        }

        // ── chip selects (input n = slave n; true = high = deselected) ───────────────────────────
        public void OnGPIO(int number, bool value)
        {
            if(number < 0 || number >= ends.Length) return;
            var e = ends[number];
            if(value == e.CsHigh) return;
            e.CsHigh = value;
            if(!value)
            {
                if(active != null)
                {
                    this.Log(LogLevel.Warning, "CS{0} asserted while CS for {1} is still low (bus contention); ignoring it", number, active.Name);
                    return;
                }
                Begin(e);
            }
            else if(active == e)
            {
                End(e);
            }
        }

        // ── one byte of the master's exchange ────────────────────────────────────────────────────
        public byte Transmit(byte data)
        {
            var e = active;
            if(e == null || !e.ArmedAtStart || e.TxSnap == null) return 0x00;
            if(e.Consumed >= e.TxSnap.Length) return 0x00;    // the slave's DMA is done: nothing to shift
            var reply = e.TxSnap[e.Consumed];
            e.RxBuf.Add(data);
            e.Consumed++;
            return reply;
        }

        public void FinishTransmission() { }

        // ──────────────────────────────────────────────────────────────────────────────────────────
        private void Begin(SlaveEnd e)
        {
            active = e;
            e.Exchanges++;
            e.ArmedAtStart = false;
            e.RxBuf.Clear();
            if(!e.Present || !Resolve(e))
            {
                e.NotArmed++;
                return;
            }
            var bus = e.Machine.SystemBus;
            Post(e, high => SetNss(e, high), false);
            if(bus.ReadDoubleWord(e.ArmedAddr) == 0)
            {
                e.NotArmed++;
                return;
            }
            var gen = bus.ReadDoubleWord(e.ArmCountAddr);
            if(gen != e.SnapGen)                          // (re)armed since last time: a fresh DMA transfer
            {
                e.SnapGen = gen;
                e.Consumed = 0;
                var txp = bus.ReadDoubleWord(e.TxpAddr);
                var len = bus.ReadByte(e.LenAddr) | (bus.ReadByte(e.LenAddr + 1) << 8);
                e.TxSnap = bus.ReadBytes(txp, len, true);
                e.Rxp = bus.ReadDoubleWord(e.RxpAddr);
            }
            e.RxStart = e.Consumed;                       // continue where a short exchange stopped
            e.ArmedAtStart = true;
        }

        private void End(SlaveEnd e)
        {
            active = null;
            if(!e.ArmedAtStart)
            {
                if(e.Machine != null) Post(e, high => SetNss(e, high), true);
                return;
            }
            var chunk = e.RxBuf.ToArray();
            var addr = e.Rxp + (ulong)e.RxStart;
            var complete = e.Consumed >= e.TxSnap.Length;
            if(complete)
            {
                e.Completed++;
                e.SnapGen = uint.MaxValue;                // that transfer is spent; wait for a re-arm
                e.Consumed = 0;
            }
            else
            {
                e.Partial++;
                this.Log(LogLevel.Warning, "{0}: exchange ended after {1} of {2} bytes; slave DMA left part-way (needs resync)",
                         e.Name, e.Consumed, e.TxSnap.Length);
            }
            Post(e, _ =>
            {
                if(chunk.Length > 0) e.Machine.SystemBus.WriteBytes(chunk, addr, true);
                SetNss(e, true);
                if(complete)
                {
                    var nvic = e.Machine["sysbus.nvic"] as IGPIOReceiver;
                    nvic.OnGPIO(slaveIrq, true);
                    nvic.OnGPIO(slaveIrq, false);
                }
            }, true);
        }

        private bool Resolve(SlaveEnd e)
        {
            if(e.Machine == null)
            {
                IMachine m;
                if(!EmulationManager.Instance.CurrentEmulation.TryGetMachineByName(e.Name, out m))
                {
                    this.Log(LogLevel.Error, "no machine named '{0}'", e.Name);
                    return false;
                }
                e.Machine = m;
            }
            if(e.ArmedAddr != 0) return true;
            try
            {
                e.ArmedAddr = Symbol(e, "sim_spi_armed");
                e.TxpAddr = Symbol(e, "sim_spi_txp");
                e.RxpAddr = Symbol(e, "sim_spi_rxp");
                e.LenAddr = Symbol(e, "sim_spi_len");
                e.ArmCountAddr = Symbol(e, "sim_spi_arm_count");
                return true;
            }
            catch(KeyNotFoundException ex)
            {
                this.Log(LogLevel.Error, "{0}: {1} (was the slave built with shim/slave_spi_dma.c?)", e.Name, ex.Message);
                e.ArmedAddr = 0;
                return false;
            }
        }

        private static ulong Symbol(SlaveEnd e, string name)
        {
            IEnumerable<ulong> addrs;
            if(!e.Machine.SystemBus.TryGetAllSymbolAddresses(name, out addrs) || !addrs.Any())
            {
                throw new KeyNotFoundException(string.Format("symbol '{0}' not found in the loaded ELF", name));
            }
            return addrs.First();
        }

        private static void Post(SlaveEnd e, Action<bool> action, bool arg)
        {
            if(e.Machine == null) return;
            var when = TimeDomainsManager.Instance.GetEffectiveVirtualTimeStamp();
            e.Machine.HandleTimeDomainEvent<bool>(action, arg, when);
        }

        private void SetNss(SlaveEnd e, bool high)
        {
            var port = e.Machine["sysbus.gpioPortA"] as IGPIOReceiver;
            port.OnGPIO(nssPin, high);
        }

        private class SlaveEnd
        {
            public SlaveEnd(string name) { Name = name; Present = true; }
            public void Clear()
            {
                CsHigh = true; ArmedAtStart = false; Consumed = 0; RxStart = 0; SnapGen = uint.MaxValue; TxSnap = null;
                RxBuf.Clear();
                Exchanges = Completed = NotArmed = Partial = 0;
            }
            public readonly string Name;
            public IMachine Machine;
            public ulong ArmedAddr, TxpAddr, RxpAddr, LenAddr, ArmCountAddr, Rxp;
            public bool CsHigh, ArmedAtStart, Present;
            public int Consumed, RxStart;
            public uint SnapGen;
            public byte[] TxSnap;
            public readonly List<byte> RxBuf = new List<byte>();
            public uint Exchanges, Completed, NotArmed, Partial;
        }

        private readonly SlaveEnd[] ends;
        private readonly int slaveIrq, nssPin;
        private SlaveEnd active;
    }
}
