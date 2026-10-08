"""Reeds-Shepp words for the Hybrid A* goal connection (priority-5 plan D7a).

Reeds and Shepp, "Optimal paths for a car that goes both forwards and
backwards", Pacific J. Math. 145(2), 1990; formulas 8.1-8.11 as implemented in
OMPL's ReedsSheppStateSpace (the paper's typos in 8.3/8.4 and 8.11 corrected
there). Every word of the five families (CSC, CCC, CCCC, CCSC, CCSCC) with its
timeflip, reflect and backwards variants is returned, not only the shortest, so
the caller can try them in cost order against obstacles. The caller verifies
each word by integrating it: a word whose end is not the goal is never used.

A word is a tuple of segments (turn, gear, length_m): turn +1 left, -1 right,
0 straight (steering curvature turn * k whichever the gear); gear +1 forward,
-1 reverse.
"""

from __future__ import annotations

from math import acos, asin, atan2, cos, fmod, hypot, pi, sin, sqrt

ZERO = 1e-9
L, R, S = 1, -1, 0
_TYPES = (
    (L, R, L), (R, L, R), (L, R, L, R), (R, L, R, L),
    (L, R, S, L), (R, L, S, R), (L, S, R, L), (R, S, L, R),
    (L, R, S, R), (R, L, S, L), (R, S, R, L), (L, S, L, R),
    (L, S, R), (R, S, L), (L, S, L), (R, S, R),
    (L, R, S, L, R), (R, L, S, R, L),
)


def _mod2pi(x: float) -> float:
    v = fmod(x, 2 * pi)
    if v < -pi:
        v += 2 * pi
    elif v > pi:
        v -= 2 * pi
    return v


def _polar(x: float, y: float) -> tuple[float, float]:
    return hypot(x, y), atan2(y, x)


def _tau_omega(u, v, xi, eta, phi):
    delta = _mod2pi(u - v)
    a = sin(u) - sin(delta)
    b = cos(u) - cos(delta) - 1.0
    t1 = atan2(eta * a - xi * b, xi * a + eta * b)
    t2 = 2.0 * (cos(delta) - cos(v) - cos(u)) + 3.0
    tau = _mod2pi(t1 + pi) if t2 < 0 else _mod2pi(t1)
    return tau, _mod2pi(tau - u + v - phi)


def _lp_sp_lp(x, y, phi):  # 8.1
    u, t = _polar(x - sin(phi), y - 1.0 + cos(phi))
    if t >= -ZERO:
        v = _mod2pi(phi - t)
        if v >= -ZERO:
            return t, u, v
    return None


def _lp_sp_rp(x, y, phi):  # 8.2
    u1, t1 = _polar(x + sin(phi), y - 1.0 - cos(phi))
    u1 = u1 * u1
    if u1 >= 4.0:
        u = sqrt(u1 - 4.0)
        t = _mod2pi(t1 + atan2(2.0, u))
        v = _mod2pi(t - phi)
        if t >= -ZERO and v >= -ZERO:
            return t, u, v
    return None


def _lp_rm_l(x, y, phi):  # 8.3 / 8.4
    u1, theta = _polar(x - sin(phi), y - 1.0 + cos(phi))
    if u1 <= 4.0:
        u = -2.0 * asin(0.25 * u1)
        t = _mod2pi(theta + 0.5 * u + pi)
        v = _mod2pi(phi - t + u)
        if t >= -ZERO and u <= ZERO:
            return t, u, v
    return None


def _lp_rup_lum_rm(x, y, phi):  # 8.7
    xi, eta = x + sin(phi), y - 1.0 - cos(phi)
    rho = 0.25 * (2.0 + sqrt(xi * xi + eta * eta))
    if rho <= 1.0:
        u = acos(rho)
        t, v = _tau_omega(u, -u, xi, eta, phi)
        if t >= -ZERO and v <= ZERO:
            return t, u, v
    return None


def _lp_rum_lum_rp(x, y, phi):  # 8.8
    xi, eta = x + sin(phi), y - 1.0 - cos(phi)
    rho = (20.0 - xi * xi - eta * eta) / 16.0
    if 0.0 <= rho <= 1.0:
        u = -acos(rho)
        if u >= -0.5 * pi:
            t, v = _tau_omega(u, u, xi, eta, phi)
            if t >= -ZERO and v >= -ZERO:
                return t, u, v
    return None


def _lp_rm_sm_lm(x, y, phi):  # 8.9
    rho, theta = _polar(x - sin(phi), y - 1.0 + cos(phi))
    if rho >= 2.0:
        r = sqrt(rho * rho - 4.0)
        u = 2.0 - r
        t = _mod2pi(theta + atan2(r, -2.0))
        v = _mod2pi(phi - 0.5 * pi - t)
        if t >= -ZERO and u <= ZERO and v <= ZERO:
            return t, u, v
    return None


def _lp_rm_sm_rm(x, y, phi):  # 8.10
    xi, eta = x + sin(phi), y - 1.0 - cos(phi)
    rho, theta = _polar(-eta, xi)
    if rho >= 2.0:
        t = theta
        u = 2.0 - rho
        v = _mod2pi(t + 0.5 * pi - phi)
        if t >= -ZERO and u <= ZERO and v <= ZERO:
            return t, u, v
    return None


def _lp_rm_slm_rp(x, y, phi):  # 8.11
    xi, eta = x + sin(phi), y - 1.0 - cos(phi)
    rho, _ = _polar(xi, eta)
    if rho >= 2.0:
        u = 4.0 - sqrt(rho * rho - 4.0)
        if u <= ZERO:
            t = _mod2pi(atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta))
            v = _mod2pi(t - phi)
            if t >= -ZERO and v >= -ZERO:
                return t, u, v
    return None


def _words_unit(x: float, y: float, phi: float) -> list[tuple[tuple[int, ...], tuple[float, ...]]]:
    """Every (turn types, signed unit lengths) word from the origin to (x, y, phi), radius 1."""
    out = []

    def add(kind, *lengths):
        out.append((_TYPES[kind], lengths))

    # CSC
    for fn, base in ((_lp_sp_lp, 14), (_lp_sp_rp, 12)):
        if (r := fn(x, y, phi)) is not None:
            add(base, *r)
        if (r := fn(-x, y, -phi)) is not None:
            add(base, *(-v for v in r))
        if (r := fn(x, -y, -phi)) is not None:
            add(base + 1, *r)
        if (r := fn(-x, -y, phi)) is not None:
            add(base + 1, *(-v for v in r))
    # CCC
    xb, yb = x * cos(phi) + y * sin(phi), x * sin(phi) - y * cos(phi)
    if (r := _lp_rm_l(x, y, phi)) is not None:
        add(0, *r)
    if (r := _lp_rm_l(-x, y, -phi)) is not None:
        add(0, *(-v for v in r))
    if (r := _lp_rm_l(x, -y, -phi)) is not None:
        add(1, *r)
    if (r := _lp_rm_l(-x, -y, phi)) is not None:
        add(1, *(-v for v in r))
    if (r := _lp_rm_l(xb, yb, phi)) is not None:
        add(0, r[2], r[1], r[0])
    if (r := _lp_rm_l(-xb, yb, -phi)) is not None:
        add(0, -r[2], -r[1], -r[0])
    if (r := _lp_rm_l(xb, -yb, -phi)) is not None:
        add(1, r[2], r[1], r[0])
    if (r := _lp_rm_l(-xb, -yb, phi)) is not None:
        add(1, -r[2], -r[1], -r[0])
    # CCCC
    if (r := _lp_rup_lum_rm(x, y, phi)) is not None:
        t, u, v = r
        add(2, t, u, -u, v)
    if (r := _lp_rup_lum_rm(-x, y, -phi)) is not None:
        t, u, v = r
        add(2, -t, -u, u, -v)
    if (r := _lp_rup_lum_rm(x, -y, -phi)) is not None:
        t, u, v = r
        add(3, t, u, -u, v)
    if (r := _lp_rup_lum_rm(-x, -y, phi)) is not None:
        t, u, v = r
        add(3, -t, -u, u, -v)
    if (r := _lp_rum_lum_rp(x, y, phi)) is not None:
        t, u, v = r
        add(2, t, u, u, v)
    if (r := _lp_rum_lum_rp(-x, y, -phi)) is not None:
        t, u, v = r
        add(2, -t, -u, -u, -v)
    if (r := _lp_rum_lum_rp(x, -y, -phi)) is not None:
        t, u, v = r
        add(3, t, u, u, v)
    if (r := _lp_rum_lum_rp(-x, -y, phi)) is not None:
        t, u, v = r
        add(3, -t, -u, -u, -v)
    # CCSC
    h = 0.5 * pi
    for fn, base in ((_lp_rm_sm_lm, 4), (_lp_rm_sm_rm, 8)):
        if (r := fn(x, y, phi)) is not None:
            t, u, v = r
            add(base, t, -h, u, v)
        if (r := fn(-x, y, -phi)) is not None:
            t, u, v = r
            add(base, -t, h, -u, -v)
        if (r := fn(x, -y, -phi)) is not None:
            t, u, v = r
            add(base + 1, t, -h, u, v)
        if (r := fn(-x, -y, phi)) is not None:
            t, u, v = r
            add(base + 1, -t, h, -u, -v)
    for fn, base in ((_lp_rm_sm_lm, 6), (_lp_rm_sm_rm, 10)):
        if (r := fn(xb, yb, phi)) is not None:
            t, u, v = r
            add(base, v, u, -h, t)
        if (r := fn(-xb, yb, -phi)) is not None:
            t, u, v = r
            add(base, -v, -u, h, -t)
        if (r := fn(xb, -yb, -phi)) is not None:
            t, u, v = r
            add(base + 1, v, u, -h, t)
        if (r := fn(-xb, -yb, phi)) is not None:
            t, u, v = r
            add(base + 1, -v, -u, h, -t)
    # CCSCC
    if (r := _lp_rm_slm_rp(x, y, phi)) is not None:
        t, u, v = r
        add(16, t, -h, u, -h, v)
    if (r := _lp_rm_slm_rp(-x, y, -phi)) is not None:
        t, u, v = r
        add(16, -t, h, -u, h, -v)
    if (r := _lp_rm_slm_rp(x, -y, -phi)) is not None:
        t, u, v = r
        add(17, t, -h, u, -h, v)
    if (r := _lp_rm_slm_rp(-x, -y, phi)) is not None:
        t, u, v = r
        add(17, -t, h, -u, h, -v)
    return out


def reeds_shepp_words(start, goal, curvature: float) -> list[tuple[tuple[int, int, float], ...]]:
    """Every Reeds-Shepp word from start to goal (x, y, yaw) at this curvature, as segments
    (turn, gear, length_m), shortest first. Unchecked: the caller integrates each."""
    radius = 1.0 / curvature
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    c, s = cos(start[2]), sin(start[2])
    x, y = (c * dx + s * dy) / radius, (-s * dx + c * dy) / radius
    phi = _mod2pi(goal[2] - start[2])
    words = []
    for types, lengths in _words_unit(x, y, phi):
        segments = tuple(
            (turn, 1 if length >= 0 else -1, abs(length) * radius)
            for turn, length in zip(types, lengths, strict=True)
        )
        words.append(segments)
    words.sort(key=lambda w: sum(seg[2] for seg in w))
    return words


__all__ = ["reeds_shepp_words"]
