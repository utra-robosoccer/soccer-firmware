"""Monotonic unwrapping of wrapping fixed-width counters.

The master's `master_time_us` is a free-running 32-bit microsecond clock (TIM2),
so it wraps ~every 71.6 min. Any host code that takes deltas or plots it on a time
axis must unwrap it to a monotonic value first.
"""
from __future__ import annotations


def u32_delta(prev_raw: int, raw: int) -> int:
    """Forward delta between two 32-bit samples, wrap-safe (assumes < 2^31 apart)."""
    return (raw - prev_raw) & 0xFFFFFFFF


class U32Unwrapper:
    """Extends a wrapping unsigned counter into a monotonic value by accumulating
    wrap-safe forward deltas. First sample seeds the value; thereafter each sample
    adds `(raw - prev) & mask`. Safe as long as successive samples advance by less
    than half the range (trivially true for a µs clock sampled at ≥1 Hz)."""

    def __init__(self, bits: int = 32):
        self._mask = (1 << bits) - 1
        self._prev: int | None = None
        self.value = 0

    def update(self, raw: int) -> int:
        raw &= self._mask
        if self._prev is None:
            self.value = raw
        else:
            self.value += (raw - self._prev) & self._mask
        self._prev = raw
        return self.value
