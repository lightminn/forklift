# src/forklift_core/perception/recognition_stats.py
"""One-sided Clopper-Pearson bounds without scipy (design §3.3)."""

import math


def _binom_cdf(k: int, n: int, p: float) -> float:
    if p <= 0.0:
        return 1.0
    if p >= 1.0:
        return 0.0 if k < n else 1.0
    lp, lq = math.log(p), math.log1p(-p)
    return min(1.0, sum(math.exp(math.log(math.comb(n, i)) + i * lp + (n - i) * lq) for i in range(k + 1)))


def cp_upper(failures: int, n: int, confidence: float = 0.95) -> float | None:
    if n == 0:
        return None
    if not 0 <= failures <= n:
        raise ValueError("failures must lie in [0, n]")
    if failures == n:
        return 1.0
    alpha, low, high = 1.0 - confidence, 0.0, 1.0
    for _ in range(200):
        mid = (low + high) / 2
        if _binom_cdf(failures, n, mid) > alpha:
            low = mid
        else:
            high = mid
    return high


def allowed_failures(n: int, bound: float) -> int | None:
    best = None
    for k in range(n + 1):
        upper = cp_upper(k, n)
        if upper is None or upper > bound:
            break
        best = k
    return best


def min_scenes(failures: int, bound: float) -> int:
    n = failures + 1
    while cp_upper(failures, n) > bound:
        n += 1
    return n


def eval_count(total: int) -> int:
    return math.ceil(0.3 * total - 1e-12)
