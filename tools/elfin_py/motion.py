"""Planned motions and the checks against confirmed.yaml (codebase plan §3.8 item 5).

Every motion's planned peak speed and acceleration are computed before anything is sent; a
motion above the confirmed values is refused, never clipped. Streamed trajectories are
checked numerically twice: the continuous profile at 1 ms, and the pushed samples by finite
differences at the servo period. Units here are the protocol's: degrees, mm, seconds.
Standard library only.
"""
import math
from dataclasses import dataclass


class LimitRefused(ValueError):
    pass


# ---------------------------------------------------------------------- WayPoint
def check_waypoint(kind, velocity, acc, confirmed):
    """kind 'joint' (deg/s, deg/s^2) or 'linear' (mm/s, mm/s^2). The controller plans the profile. That dVelocity
    and dAcc are its peaks is assumed, not stated: elfin_commands.md gives their units only (PROTO p.137-139)
    [UNTESTED]. A linear move's joint speeds are not planned here at all, so every move's joint speeds are
    measured while the tool waits for it and checked against the confirmed joint speed (link.Arm.wait_done;
    pre-hardware review S20)."""
    if kind == "joint":
        vmax, amax, unit = confirmed["joint_speed_deg_s"], confirmed["joint_acc_deg_s2"], "deg"
    elif kind == "linear":
        vmax, amax, unit = confirmed["linear_speed_mm_s"], confirmed["linear_acc_mm_s2"], "mm"
    else:
        raise ValueError(kind)
    if not (0 < velocity <= vmax) or not (0 < acc <= amax):
        raise LimitRefused(f"{kind} move at {velocity} {unit}/s, {acc} {unit}/s^2 exceeds the confirmed "
                           f"{vmax} {unit}/s, {amax} {unit}/s^2 (confirmed.yaml); refused")
    return {"kind": kind, "velocity": velocity, "acc": acc, "vmax": vmax, "amax": amax}


# ---------------------------------------------------------------------- the ladder's J6 profile
@dataclass(frozen=True)
class LadderProfile:
    """Example 3's J6 sine with the owner's changes (week 1): 5 deg amplitude, 10 s period,
    20 s of motion under a Tukey window with 2 s cosine tapers at each end, and 2 s of
    streamed holds at q0 before and after."""
    amplitude_deg: float = 5.0
    period_s: float = 10.0
    motion_s: float = 20.0
    taper_s: float = 2.0
    hold_s: float = 2.0

    @property
    def total_s(self):
        return 2 * self.hold_s + self.motion_s

    def _window(self, t):
        D, Tp = self.motion_s, self.taper_s
        if t <= 0 or t >= D:
            return 0.0, 0.0, 0.0
        k = math.pi / Tp
        if t < Tp:
            return 0.5 * (1 - math.cos(k * t)), 0.5 * k * math.sin(k * t), 0.5 * k * k * math.cos(k * t)
        if t > D - Tp:
            u = D - t
            return 0.5 * (1 - math.cos(k * u)), -0.5 * k * math.sin(k * u), 0.5 * k * k * math.cos(k * u)
        return 1.0, 0.0, 0.0

    def offset(self, t):
        """J6 offset from q0 at time t (from the start of the first hold): deg, deg/s, deg/s^2."""
        tm = t - self.hold_s
        w, wd, wdd = self._window(tm)
        om = 2 * math.pi / self.period_s
        s, c = math.sin(om * tm), math.cos(om * tm)
        A = self.amplitude_deg
        return (A * w * s,
                A * (wd * s + w * om * c),
                A * (wdd * s + 2 * wd * om * c - w * om * om * s))


def sample(profile, q0, period):
    """Joint targets (deg) at k * period, k = 0..N, ending exactly at q0."""
    n = int(math.ceil(profile.total_s / period - 1e-9))
    out = []
    for k in range(n + 1):
        t = min(k * period, profile.total_s)
        q = list(q0)
        q[5] += profile.offset(t)[0]
        out.append(q)
    return out


def peaks_continuous(profile, dt=0.001):
    n = int(round(profile.total_s / dt))
    v = a = 0.0
    for i in range(n + 1):
        _, vi, ai = profile.offset(i * dt)
        v, a = max(v, abs(vi)), max(a, abs(ai))
    return v, a


def peaks_samples(samples, period):
    v = a = 0.0
    for j in range(6):
        x = [q[j] for q in samples]
        for k in range(len(x) - 1):
            v = max(v, abs(x[k + 1] - x[k]) / period)
        for k in range(1, len(x) - 1):
            a = max(a, abs(x[k + 1] - 2 * x[k] + x[k - 1]) / period ** 2)
    return v, a


def check_stream(samples, period, confirmed, profile=None):
    """Refuse a stream whose period is shorter than confirmed or whose peaks exceed the confirmed
    streamed speed and acceleration."""
    if period < confirmed["servo_period_s"] - 1e-12:
        raise LimitRefused(f"servo period {period} s is shorter than the confirmed {confirmed['servo_period_s']} s; refused")
    vmax, amax = confirmed["stream_joint_speed_deg_s"], confirmed["stream_joint_acc_deg_s2"]
    vs, as_ = peaks_samples(samples, period)
    vc, ac = peaks_continuous(profile) if profile else (0.0, 0.0)
    report = {"peak_speed_deg_s": max(vs, vc), "peak_acc_deg_s2": max(as_, ac),
              "samples_peak_speed": vs, "samples_peak_acc": as_, "continuous_peak_speed": vc,
              "continuous_peak_acc": ac, "limit_speed": vmax, "limit_acc": amax}
    if report["peak_speed_deg_s"] > vmax or report["peak_acc_deg_s2"] > amax:
        raise LimitRefused(f"planned stream peaks {report['peak_speed_deg_s']:.3f} deg/s, "
                           f"{report['peak_acc_deg_s2']:.3f} deg/s^2 exceed the confirmed {vmax} deg/s, "
                           f"{amax} deg/s^2 (confirmed.yaml); refused")
    return report


def step_peaks(q_prev, q_last, q_next, period):
    """Speed and acceleration that one more push implies, by the finite differences peaks_samples uses:
    q_prev and q_last are the last two samples sent, q_next the one about to go."""
    v = max(abs(n - l) for n, l in zip(q_next, q_last)) / period
    a = max(abs(n - 2 * l + p) for n, l, p in zip(q_next, q_last, q_prev)) / period ** 2
    return v, a


def within(v, a, vmax, amax, rel_tol=1e-9):
    return v <= vmax * (1 + rel_tol) and a <= amax * (1 + rel_tol)


def decel_to_rest(q_last, q_prev, period, acc_max, hold_pushes=3):
    """Streamed deceleration to rest (codebase plan §3.8 item 6): from the velocity implied by the
    last two pushes, slow every joint linearly to zero at no more than acc_max, then hold."""
    v = [(a - b) / period for a, b in zip(q_last, q_prev)]
    vpeak = max(abs(x) for x in v)
    n = max(1, int(math.ceil(vpeak / (acc_max * period))))
    out, q = [], list(q_last)
    for i in range(1, n + 1):
        f = 1.0 - i / n
        q = [qj + vj * f * period for qj, vj in zip(q, v)]
        out.append(list(q))
    out += [list(q)] * hold_pushes
    return out
