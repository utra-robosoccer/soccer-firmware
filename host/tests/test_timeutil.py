"""Tests for U32Unwrapper — monotonic unwrapping of the 32-bit master_time_us
(the master's TIM2 µs clock, which wraps ~every 71.6 min)."""
import unittest

from master_link.timeutil import U32Unwrapper, u32_delta

MASK = 0xFFFFFFFF


class U32UnwrapTest(unittest.TestCase):
    def test_no_wrap_monotonic(self):
        u = U32Unwrapper()
        self.assertEqual(u.update(1000), 1000)          # seed
        self.assertEqual(u.update(6000), 6000)          # +5000 (one 200 Hz cycle)
        self.assertEqual(u.update(11000), 11000)

    def test_wrap_is_continuous(self):
        u = U32Unwrapper()
        u.update(MASK - 100)                            # seed near the top
        v = u.update(400)                               # wrapped past 2^32
        # delta across the wrap is 101 + 400 + ... = (400 - (MASK-100)) & MASK = 501
        self.assertEqual(v, (MASK - 100) + 501)
        self.assertEqual(u32_delta(MASK - 100, 400), 501)

    def test_many_wraps_stay_monotonic(self):
        u = U32Unwrapper()
        raw = 0
        prev_mono = -1
        for _ in range(100000):                         # 100k steps of 5000 µs → several wraps
            mono = u.update(raw & MASK)
            self.assertGreaterEqual(mono, prev_mono)    # never goes backwards
            prev_mono = mono
            raw += 5000
        # total monotonic span ≈ 100000 * 5000 µs, well past 2^32
        self.assertAlmostEqual(prev_mono, (100000 - 1) * 5000, delta=1)


if __name__ == "__main__":
    unittest.main()
