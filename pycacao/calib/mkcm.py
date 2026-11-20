# SPDX-FileCopyrightText: 2026 Vincent Deo
#
# SPDX-License-Identifier: LGPL-3.0-or-later
"""
mkcm.py: control matrices from a response matrix pair, numpy only.

modes_dm (n, DMy, DMx) are the poke modes, modes_wfs (n, WFSy, WFSx) their WFS responses.
With W = V diag(sv) U^T (rows: poke modes), each function returns (cm_wfs, cm_dm, sv):
the loop computes a = cm_wfs . wfs and dm = cm_dm^T . a. sv holds all singular values.
nmodes keeps the singular modes k < nmodes; rcond keeps those with sv_k > rcond * sv_0.
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

Array = npt.NDArray[np.floating]
CM = tuple[Array, Array, Array]


def _svd(
    modes_dm: Array, modes_wfs: Array, nmodes: int | None, rcond: float | None
) -> tuple[Array, Array, Array, Array, Array, np.dtype]:
    """-> (D, V_K, sv_K, U_K^T, sv, dtype). Raises ValueError when K is empty or W is zero."""
    n = modes_dm.shape[0]
    D = modes_dm.reshape(n, -1).astype(np.float64)
    v, sv, ut = np.linalg.svd(
        modes_wfs.reshape(n, -1).astype(np.float64), full_matrices=False
    )
    keep = np.arange(sv.size) < (sv.size if nmodes is None else nmodes)
    if rcond is not None:
        keep &= sv > rcond * sv[0]
    if sv[0] == 0 or not keep.any():
        raise ValueError("no singular mode kept")
    dtype = np.result_type(modes_dm.dtype, modes_wfs.dtype, np.float32)
    return D, v[:, keep], sv[keep], ut[keep], sv, dtype


def _out(
    cm_wfs: Array,
    cm_dm: Array,
    sv: Array,
    modes_dm: Array,
    modes_wfs: Array,
    dtype: np.dtype,
) -> CM:
    return (
        cm_wfs.reshape(-1, *modes_wfs.shape[1:]).astype(dtype),
        cm_dm.reshape(-1, *modes_dm.shape[1:]).astype(dtype),
        sv,
    )


def cm_tsvd(
    modes_dm: Array, modes_wfs: Array, nmodes: int | None = None, rcond: float = 1e-6
) -> CM:
    """Truncated SVD: one slice per kept singular mode, u_k^T / sv_k and v_k^T . D."""
    D, vk, sk, utk, sv, dt = _svd(modes_dm, modes_wfs, nmodes, rcond)
    return _out(utk / sk[:, None], vk.T @ D, sv, modes_dm, modes_wfs, dt)


def cm_tikhonov(
    modes_dm: Array, modes_wfs: Array, reg: float, nmodes: int | None = None
) -> CM:
    """Tikhonov, lambda = reg * sv_0: u_k^T sv_k / (sv_k^2 + lambda^2) and v_k^T . D, k < nmodes."""
    D, vk, sk, utk, sv, dt = _svd(modes_dm, modes_wfs, nmodes, None)
    lam2 = (reg * sv[0]) ** 2
    return _out(
        utk * (sk / (sk**2 + lam2))[:, None], vk.T @ D, sv, modes_dm, modes_wfs, dt
    )


def cm_pinv(
    modes_dm: Array, modes_wfs: Array, nmodes: int | None = None, rcond: float = 1e-6
) -> CM:
    """Pseudo-inverse restricted to K, the poke modes as loop modes: n slices, cm_dm = D."""
    D, vk, sk, utk, sv, dt = _svd(modes_dm, modes_wfs, nmodes, rcond)
    return _out(vk @ (utk / sk[:, None]), D, sv, modes_dm, modes_wfs, dt)
