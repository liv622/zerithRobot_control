"""Reusable scalar trajectory time profiles."""

from __future__ import annotations

import numpy as np


def quintic(value: np.ndarray) -> np.ndarray:
    """Fifth-order smooth-step with zero endpoint velocity/acceleration."""

    return 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5
