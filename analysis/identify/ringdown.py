"""Ring-down fit for the per-pickup shake and kill test 2(a) (research plan §3.2). NumPy only.

    y(t) = A e^(-sigma t) cos(omega_d t + phi) + k0 + k1 t
    omega0 = sqrt(omega_d^2 + sigma^2),  zeta = sigma / omega0

Matrix-pencil initialization (Hua and Sarkar, IEEE Trans. ASSP 1990) on the first harmonic, then
Levenberg-Marquardt on the six parameters. The fit window may end where the envelope reaches the
contact-line dead band (§3.2). Also the dispersion relation of the first antisymmetric mode (§2.2) and
repeatability statistics over pickups. The same code serves recorded and synthetic data (R-C).
"""
import math

import numpy as np

XI1 = 1.8412          # first zero of J1' (research plan §2.1)
G_STD = 9.80665       # m/s^2; local g differs by less than 0.3 % [unverified for this lab]


def omega0_theory(radius_m, h_m, g=G_STD):
    """omega0 of the first antisymmetric mode of an upright cylinder (research plan §2.2)."""
    return math.sqrt(g * XI1 / radius_m * math.tanh(XI1 * h_m / radius_m))


def dispersion_mismatch(omega0_measured, radius_m, h_m, g=G_STD):
    """Relative mismatch omega0_measured / omega0_theory - 1 (§3.2 consistency row: > 3 % flags the trial)."""
    return omega0_measured / omega0_theory(radius_m, h_m, g) - 1.0


def matrix_pencil(y, dt, n_modes=4, pencil=None):
    """Complex exponents s_i (1/s) of the n_modes strongest components of y. A damped cosine is a conjugate
    pair; an offset plus a drift is a double root at s = 0, hence 4 by default."""
    y = np.asarray(y, dtype=float)
    n = len(y)
    L = pencil or n // 3
    if n < 2 * n_modes + 2 or L <= n_modes:
        raise ValueError("too few samples for the matrix pencil")
    Y = np.lib.stride_tricks.sliding_window_view(y, L + 1)          # (n - L) x (L + 1) Hankel
    _, _, vh = np.linalg.svd(Y, full_matrices=False)
    V = vh[:n_modes].conj().T                                       # (L + 1) x M
    z = np.linalg.eigvals(np.linalg.pinv(V[:-1]) @ V[1:])
    return np.log(z.astype(complex)) / dt


def _model(t, p):
    A, sig, wd, phi, k0, k1 = p
    e = np.exp(-sig * t)
    return A * e * np.cos(wd * t + phi) + k0 + k1 * t


def _jac(t, p):
    A, sig, wd, phi, _, _ = p
    e = np.exp(-sig * t)
    c, s = np.cos(wd * t + phi), np.sin(wd * t + phi)
    return np.column_stack([e * c, -A * t * e * c, -A * t * e * s, -A * e * s, np.ones_like(t), t])


def _linear_init(t, y, sig, wd):
    X = np.column_stack([np.exp(-sig * t) * np.cos(wd * t), np.exp(-sig * t) * np.sin(wd * t), np.ones_like(t), t])
    a, b, k0, k1 = np.linalg.lstsq(X, y, rcond=None)[0]
    return np.array([math.hypot(a, b), sig, wd, math.atan2(-b, a), k0, k1])


def _lm(t, y, p, iters=200, tol=1e-12):
    lam = 1e-3
    r = _model(t, p) - y
    cost = r @ r
    for _ in range(iters):
        J = _jac(t, p)
        H = J.T @ J
        g = J.T @ r
        step = np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-15), -g)
        p_new = p + step
        r_new = _model(t, p_new) - y
        c_new = r_new @ r_new
        if c_new < cost:
            p, r, lam = p_new, r_new, lam / 3
            done = cost - c_new <= tol * max(cost, 1e-300)
            cost = c_new
            if done:
                break
        else:
            lam *= 4
            if lam > 1e12:
                break
    return p, r


def fit(t, y, deadband=None, band_hz=(0.5, 20.0), min_cycles=2.0):
    """Fit one ring-down. t in s (any origin), y the first-harmonic amplitude (any unit). With deadband (the
    same unit as y), the window ends where the fitted envelope falls to it, and the fit is repeated once."""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(t) != len(y) or len(t) < 20:
        raise ValueError("need at least 20 samples")
    t = t - t[0]
    dt = float(np.median(np.diff(t)))
    s = matrix_pencil(y, dt)
    w = np.abs(s.imag)
    ok = (w > 2 * math.pi * band_hz[0]) & (w < 2 * math.pi * band_hz[1])
    if not ok.any():
        raise ValueError("no oscillation in the band")
    amp = np.abs(np.linalg.lstsq(np.exp(np.outer(t, s)), y.astype(complex), rcond=None)[0])
    pick = s[ok][np.argmax(amp[ok])]                  # the strongest in-band component
    sig0, wd0 = max(-pick.real, 0.0), abs(pick.imag)
    p, r = _lm(t, y, _linear_init(t, y, sig0, wd0))
    end = len(t)
    if deadband is not None and p[1] > 0 and abs(p[0]) > deadband:
        t_end = math.log(abs(p[0]) / deadband) / p[1]
        end = int(np.searchsorted(t, t_end))
        if end < len(t):
            min_n = int(min_cycles * 2 * math.pi / p[2] / dt)
            if end < max(min_n, 20):
                raise ValueError(f"the ring-down reaches the dead band after {t_end:.2f} s: too short to fit")
            p, r = _lm(t[:end], y[:end], p)
    A, sig, wd, phi, k0, k1 = (float(x) for x in p)
    if A < 0:
        A, phi = -A, phi + math.pi
    n = end
    dof = max(n - 6, 1)
    s2 = float(r @ r) / dof
    try:
        cov = s2 * np.linalg.inv(_jac(t[:n], p).T @ _jac(t[:n], p))
    except np.linalg.LinAlgError:
        cov = np.full((6, 6), np.nan)
    w0 = math.hypot(wd, sig)
    zeta = sig / w0
    # first-order propagation: omega0 and zeta from (sigma, omega_d)
    gw = np.array([0, sig / w0, wd / w0, 0, 0, 0])
    gz = np.array([0, wd ** 2 / w0 ** 3, -sig * wd / w0 ** 3, 0, 0, 0])
    return {"f0_hz": w0 / (2 * math.pi), "omega0": w0, "zeta": zeta, "omega_d": wd, "sigma": sig,
            "amplitude": A, "phase": phi, "k0": k0, "k1": k1, "rms": math.sqrt(float(r @ r) / n), "n": n,
            "window_s": float(t[n - 1]), "omega0_std": float(math.sqrt(max(gw @ cov @ gw, 0.0))),
            "zeta_std": float(math.sqrt(max(gz @ cov @ gz, 0.0))),
            "init": {"sigma": sig0, "omega_d": wd0}}


def spread(values):
    """Repeatability over pickups: relative standard deviation and the largest relative deviation from the
    median. Which of the two the MS1 criterion means ("within 1 % / 30 % of repeat runs") is my reading
    [proposal]: the workbench checks the largest deviation."""
    v = np.asarray(values, dtype=float)
    if len(v) < 2:
        raise ValueError("need at least two repeats")
    med = float(np.median(v))
    return {"n": len(v), "median": med, "rel_std": float(np.std(v, ddof=1) / abs(med)),
            "rel_max_dev": float(np.max(np.abs(v - med)) / abs(med))}


def isotropy(f0_x, f0_y):
    """Relative difference of the x and y kicks' f0 (§3.2: a circular cup agrees to < 1 %)."""
    return abs(f0_x - f0_y) / (0.5 * (f0_x + f0_y))
