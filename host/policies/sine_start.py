"""Smooth start for a centered-on-0 MIT sine.

From the motor's current position p0, do a minimum-jerk move (zero velocity at both ends)
to the NEAREST sine peak (+A or -A), then run the sine centered on 0 at that peak's phase
(±pi/2, where sine velocity is zero). Position and velocity are therefore continuous from
HOLD through the move into the sine. All timing comes from the caller's master-clock t_ns.

If ±A would exceed the soft limits the start is refused (raises ValueError) — never clamped
silently.
"""
import math


def fits_soft_limits(amp: float, lo: float, hi: float) -> bool:
    """True if a 0-centered sine of amplitude `amp` stays within [lo, hi]."""
    return (-amp) >= lo and amp <= hi


class SineStart:
    def __init__(self, amp: float, omega: float, v_max: float, min_move_s: float = 0.3):
        self.amp = float(amp)
        self.omega = float(omega)
        self.v_max = max(1e-6, float(v_max))
        self.min_move_s = float(min_move_s)
        self.p0 = 0.0
        self.t0_ns = 0
        self.peak = self.amp
        self.T = 0.0
        self.phase = math.pi / 2.0

    def begin(self, p0: float, t0_ns: int, lo: float | None = None, hi: float | None = None,
              who: str = "", peak_sign: int | None = None) -> None:
        """Capture the start (p0, t0) and plan the move. Raises ValueError if ±A exceeds
        [lo, hi] (when given).

        peak_sign forces which peak to start at (+1 → +A, −1 → −A); the resulting sines of two
        starters with opposite signs run in anti-phase (a half-cycle offset — used for the two
        legs of a walk). Default (None) picks the nearest peak to p0."""
        if lo is not None and hi is not None and not fits_soft_limits(self.amp, lo, hi):
            raise ValueError(
                f"sine start refused{(' for ' + who) if who else ''}: amplitude A={self.amp:.3f} "
                f"rad centered on 0 exceeds soft limits [{lo:.3f}, {hi:.3f}] "
                f"(need -A>={lo:.3f} and A<={hi:.3f}). Reduce the amplitude.")
        self.p0 = float(p0)
        self.t0_ns = t0_ns
        if peak_sign is not None:
            self.peak = self.amp if peak_sign >= 0 else -self.amp
        else:
            self.peak = self.amp if p0 >= 0.0 else -self.amp
        dist = abs(self.peak - self.p0)
        # Min-jerk peak speed = 1.875*dist/T; cap it at v_max → T >= 1.875*dist/v_max.
        self.T = 0.0 if dist < 1e-6 else max(self.min_move_s, 1.875 * dist / self.v_max)
        self.phase = math.pi / 2.0 if self.peak >= 0.0 else -math.pi / 2.0

    def sample(self, t_ns: int):
        """Return (pos, vel) at master time t_ns."""
        t = (t_ns - self.t0_ns) / 1e9
        if t < self.T:                           # minimum-jerk move p0 → peak
            tau = t / self.T
            s = tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau)
            sd = (30.0 * tau * tau - 60.0 * tau ** 3 + 30.0 * tau ** 4) / self.T
            d = self.peak - self.p0
            return self.p0 + d * s, d * sd
        ts = t - self.T                          # sine, centered on 0, from the peak phase
        return (self.amp * math.sin(self.omega * ts + self.phase),
                self.amp * self.omega * math.cos(self.omega * ts + self.phase))
