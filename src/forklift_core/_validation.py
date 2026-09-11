"""Shared input validation for sensor geometry."""

from numbers import Real

import numpy as np
from numpy.typing import ArrayLike, NDArray


def _frame_id(value: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("A nonempty coordinate frame is required")
    return value


def _real_array(value: ArrayLike, name: str) -> NDArray[np.float64]:
    array = np.asarray(value)
    if array.dtype.kind not in "uif":
        raise ValueError(f"{name} must contain real numbers")
    return array.astype(np.float64, copy=True)


def _finite_scalar(value: float, name: str) -> float:
    if (
        isinstance(value, (bool, np.bool_))
        or not isinstance(value, Real)
        or not np.isfinite(value)
    ):
        raise ValueError(f"{name} must be a finite real number")
    return float(value)
