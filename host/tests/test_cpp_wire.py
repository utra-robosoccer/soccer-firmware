"""C++ wire mirror tests (generated/cpp/robot_wire_gen.hpp + fixtures).

Two guarantees:
  1. The generator's committed fixture byte files equal gen_fixture.c's output
     byte-for-byte — the C header and the C++ header describe the same bytes.
  2. The C++ header itself compiles clean (g++ -Wall -Wextra -Werror) and
     decode → check → re-encode round-trips every fixture (test_cpp_wire.cpp).

Skips (like the other cross-language tests) when no compiler is available.
"""
import os
import shutil
import subprocess
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
FIXTURE_DIR = os.path.join(ROOT, "generated", "cpp", "fixtures")


def _run_gen_fixture():
    """Compile + run gen_fixture.c; return {label: bytes}."""
    gcc = shutil.which("gcc") or shutil.which("cc")
    if gcc is None:
        return None
    src = os.path.join(ROOT, "firmware", "common", "test", "gen_fixture.c")
    inc = os.path.join(ROOT, "firmware", "common", "include")
    exe = os.path.join(HERE, "_gen_fixture_cpp_bin")
    subprocess.check_call([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                           "-I", inc, src, "-o", exe])
    try:
        out = subprocess.check_output([exe]).decode()
    finally:
        os.remove(exe)
    fields = {}
    for line in out.splitlines():
        parts = line.split()
        fields[parts[0]] = bytes(int(x, 16) for x in parts[1:])
    return fields


class FixtureFilesMatchC(unittest.TestCase):
    """Committed fixture .bin files == gen_fixture.c output, byte-for-byte."""

    def test_fixture_bytes_match_gen_fixture_c(self):
        got = _run_gen_fixture()
        if got is None:
            self.skipTest("no C compiler available")
        for fname, label in (("robot_cmd.bin", "ROBOTCMD"),
                             ("robot_tele.bin", "ROBOTTELE")):
            with open(os.path.join(FIXTURE_DIR, fname), "rb") as f:
                data = f.read()
            self.assertEqual(data, got[label],
                             f"{fname} drifted from gen_fixture.c ({label})")


class CppRoundTrip(unittest.TestCase):
    """robot_wire_gen.hpp compiles -Wall -Wextra -Werror and round-trips the
    fixtures (decode → value checks → re-encode → byte-compare)."""

    def test_cpp_decode_reencode(self):
        gxx = shutil.which("g++") or shutil.which("clang++")
        if gxx is None:
            self.skipTest("no C++ compiler available")
        src = os.path.join(ROOT, "firmware", "common", "test",
                           "test_cpp_wire.cpp")
        inc = os.path.join(ROOT, "generated", "cpp")
        exe = os.path.join(HERE, "_test_cpp_wire_bin")
        subprocess.check_call([gxx, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                               "-I", inc, src, "-o", exe])
        try:
            out = subprocess.check_output([exe, FIXTURE_DIR]).decode()
        finally:
            os.remove(exe)
        self.assertIn("test_cpp_wire: OK", out)


if __name__ == "__main__":
    unittest.main()
