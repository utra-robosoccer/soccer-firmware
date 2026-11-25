// Minimal Cortex-M DWT: CTRL (0x0) and CYCCNT (0x4). Renode's stock STM32F4 platform has no DWT,
// but the slave firmware uses DWT->CYCCNT as its microsecond timebase (slave_service gating), so
// without it the main loop never sees time advance.
//
// CYCCNT = virtual-time-since-enable * frequency. Counting is in virtual time, not retired
// instructions, so it is independent of Renode's emulated IPC.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class CortexMDwt : IDoubleWordPeripheral, IKnownSize
    {
        public CortexMDwt(IMachine machine, long frequency = 84000000)
        {
            this.machine = machine;
            this.frequency = frequency;
            Reset();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case 0x0: return enabled ? 1u : 0u;      // CTRL (only CYCCNTENA modelled)
                case 0x4: return Cycles();               // CYCCNT
                default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
                case 0x0:
                    var en = (value & 1u) != 0;
                    if(en && !enabled) { baseTicks = NowTicks() - (long)lastCount * 1000000000L / frequency; }
                    if(!en && enabled) { lastCount = Cycles(); }
                    enabled = en;
                    break;
                case 0x4:
                    lastCount = value;
                    baseTicks = NowTicks() - (long)value * 1000000000L / frequency;
                    break;
            }
        }

        public void Reset() { enabled = false; lastCount = 0; baseTicks = 0; }

        public long Size { get { return 0x1000; } }

        private long NowTicks()   // nanoseconds of virtual time
        {
            return (long)(machine.LocalTimeSource.ElapsedVirtualTime.TotalMilliseconds * 1000000.0);
        }

        private uint Cycles()
        {
            if(!enabled) return lastCount;
            var ns = NowTicks() - baseTicks;
            return (uint)((ns * (frequency / 1000000L)) / 1000L);   // ns * MHz / 1000
        }

        private readonly IMachine machine;
        private readonly long frequency;
        private bool enabled;
        private uint lastCount;
        private long baseTicks;
    }
}
