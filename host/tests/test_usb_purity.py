"""Master USB CDC purity guard (grep-based CI check).

The master's CDC stream must carry ONLY framed protocol messages: every
transmit goes proto_build → usb_tx_write → usb_tx.c's slot ring. These checks
make the invariant regress-proof:

  1. CDC_Transmit_FS is called nowhere outside usb_tx.c (its definition lives
     in USB_DEVICE/App/usbd_cdc_if.c and is allowed there).
  2. No printf-family call in the master's own sources (Core/Src) outside the
     gated debug_printf implementation — vsnprintf into a local buffer is fine,
     stdout-style printf/puts/putchar is not.
  3. usb_printf (the old raw-text-to-CDC helper) stays dead.
  4. The strong _write stdio guard is present in the master build.
"""
import os
import re
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
MASTER = os.path.join(ROOT, "firmware", "master")


def _sources(subdir):
    out = []
    for dirpath, dirs, names in os.walk(os.path.join(MASTER, subdir)):
        if "Debug" in dirpath.split(os.sep):
            continue
        for n in names:
            if n.endswith((".c", ".h")):
                out.append(os.path.join(dirpath, n))
    return out


def _strip_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


class UsbCdcPurity(unittest.TestCase):
    def test_cdc_transmit_only_from_usb_tx(self):
        offenders = []
        for p in _sources("Core") + _sources("USB_DEVICE") + _sources("Middlewares"):
            base = os.path.basename(p)
            if base in ("usb_tx.c", "usbd_cdc_if.c", "usbd_cdc_if.h"):
                continue
            code = _strip_comments(open(p, encoding="utf-8", errors="replace").read())
            for i, line in enumerate(code.splitlines(), 1):
                if "CDC_Transmit_FS" in line:
                    offenders.append(f"{os.path.relpath(p, ROOT)}:{i}")
        self.assertEqual(offenders, [],
                         "CDC_Transmit_FS outside usb_tx.c — the CDC stream "
                         "must stay framed-protocol-only")

    def test_no_stdio_prints_in_master_sources(self):
        # \b(printf|puts|putchar)\s*\( — vsnprintf/snprintf/usb_tx_* don't match.
        pat = re.compile(r"(?<![a-zA-Z_])(printf|puts|putchar)\s*\(")
        offenders = []
        for p in _sources("Core"):
            base = os.path.basename(p)
            if base == "syscalls.c":          # the libc shim itself
                continue
            code = _strip_comments(open(p, encoding="utf-8", errors="replace").read())
            in_debug_impl = "MASTER_DEBUG_UART" in code
            for i, line in enumerate(code.splitlines(), 1):
                if pat.search(line):
                    # the gated debug_printf implementation is the one allowed
                    # printf-shaped symbol (and it writes to UART4, not CDC)
                    if in_debug_impl and "debug_printf" in line:
                        continue
                    offenders.append(f"{os.path.relpath(p, ROOT)}:{i}: {line.strip()}")
        self.assertEqual(offenders, [],
                         "printf/puts/putchar in master sources — route "
                         "diagnostics through debug_printf (UART4)")

    def test_usb_printf_stays_dead(self):
        offenders = []
        for p in _sources("Core"):
            code = _strip_comments(open(p, encoding="utf-8", errors="replace").read())
            if "usb_printf" in code:
                offenders.append(os.path.relpath(p, ROOT))
        self.assertEqual(offenders, [], "usb_printf (raw text onto the CDC "
                                        "stream) must not come back")

    def test_write_guard_present(self):
        main_c = open(os.path.join(MASTER, "Core", "Src", "main.c"),
                      encoding="utf-8").read()
        self.assertIn("int _write(int file, char *ptr, int len)", main_c,
                      "the strong _write stdio guard left main.c")


if __name__ == "__main__":
    unittest.main()
