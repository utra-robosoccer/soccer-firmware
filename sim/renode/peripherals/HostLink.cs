// The Jetson end of the master's host link, as a byte buffer driven from the Renode monitor.
//
// On the robot the host link is USB CDC. Renode has no USB device model, so the sim master firmware
// carries the link on USART2 (shim/master_host_uart.c) and this external is plugged into it:
//
//     emulation CreateHostLink "hostlink"
//     connector Connect sysbus.usart2 hostlink
//
//     hostlink Send "a50901..."   hand bytes (hex) to the master's UART; returns how many
//     hostlink Take               every byte the master has sent since the last Take, as hex
//
// Because the monitor only runs while time is paused, bytes sent between two `emulation RunFor`
// commands always arrive at the same virtual instant: stepping is deterministic, with no socket
// and no sleeps. (For a live link to a real host program, robot-serve.resc uses Renode's server
// socket terminal instead; see robosim.py `serve`.)
//
// Threading: the bytes are handed to the UART inside the machine's time domain
// (HandleTimeDomainEvent), the way Renode's own terminals do it (BackendTerminal.WriteToUART), never
// by calling uart.WriteChar on the monitor thread. (USART2 itself is unpaced; see platforms/master.repl.)
using System;
using System.Collections.Generic;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.UART;
using Antmicro.Renode.Time;
using Antmicro.Renode.Utilities;

namespace Antmicro.Renode.Peripherals.UART
{
    public static class HostLinkExtensions
    {
        public static void CreateHostLink(this Emulation emulation, string name)
        {
            emulation.ExternalsManager.AddExternal(new HostLink(), name);
        }
    }

    public class HostLink : IExternal, IConnectable<IUART>
    {
        public void AttachTo(IUART uart)
        {
            this.uart = uart;
            machine = uart.GetMachine();
            uart.CharReceived += OnChar;
        }

        public void DetachFrom(IUART uart)
        {
            uart.CharReceived -= OnChar;
            this.uart = null;
            machine = null;
        }

        public int Send(string hex)
        {
            if(uart == null) throw new InvalidOperationException("hostlink is not connected to a UART");
            hex = hex.Trim();
            if(hex.Length % 2 != 0) throw new ArgumentException("hex string must have an even length");
            var bytes = new byte[hex.Length / 2];
            for(var i = 0; i < bytes.Length; i++) bytes[i] = Convert.ToByte(hex.Substring(2 * i, 2), 16);
            var target = uart;
            machine.HandleTimeDomainEvent<byte[]>(data => { foreach(var b in data) target.WriteChar(b); },
                                                  bytes, TimeDomainsManager.Instance.GetEffectiveVirtualTimeStamp());
            BytesSent += (ulong)bytes.Length;
            return bytes.Length;
        }

        public string Take()
        {
            lock(received)
            {
                var sb = new StringBuilder(received.Count * 2);
                foreach(var b in received) sb.Append(b.ToString("x2"));
                received.Clear();
                return sb.ToString();
            }
        }

        public ulong BytesSent { get; private set; }
        public ulong BytesReceived { get; private set; }

        private void OnChar(byte b)
        {
            lock(received)
            {
                received.Add(b);
                BytesReceived++;
            }
        }

        private readonly List<byte> received = new List<byte>();
        private IUART uart;
        private IMachine machine;
    }
}
