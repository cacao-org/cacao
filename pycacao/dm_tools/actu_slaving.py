# Grab the mask
# Grab the CM

# Define the full DM mask
# Prolong the CM by MMSE over the full.
# E.Z.

import typing as typ
import numpy as np


def make_prox_mmse_basis(total_mask: np.ndarray,
                         drive_mask: np.ndarray,
                         pos_maps: typ.Optional[typ.Tuple[np.ndarray, np.ndarray]] = None):
    """
        total_mask: 50x50 BOOL - all phys act on the DM
        drive_mask: 50x50 BOOL - actuators we're interested in
    """

    pow_mmse = 1.0

    # Mask, counts
    slaved_mask = total_mask ^ drive_mask
    n_drive = np.sum(drive_mask)
    n_slav = np.sum(slaved_mask)

    # Coordinates and distance matrices
    if pos_maps is None:
        xd, yd = np.where(drive_mask)
        xs, ys = np.where(slaved_mask)
    else:
        xd, yd = pos_maps[0][drive_mask], pos_maps[1][drive_mask]
        xs, ys = pos_maps[0][slaved_mask], pos_maps[1][slaved_mask]

    is_slaved = ~drive_mask[total_mask]
    drive_dmat = ((xd[None, :] - xd[:, None])**2 +
                  (yd[None, :] - yd[:, None])**2)**(5 / 6.0)
    # Compute the MMSE expander
    drive_slaved_dmat = ((xd[None, :] - xs[:, None])**2 +
                         (yd[None, :] - ys[:, None])**2)**(5 / 6.0)

    # OK now for the MMSE-ification
    # Actual Kolmo MMSE
    expander = drive_slaved_dmat**pow_mmse @ np.linalg.inv(drive_dmat**
                                                           pow_mmse)
    # Or a simpler, weighted average but it's bounded over the slaved zone
    # expander = drive_slaved_dmat / np.sum(drive_slaved_dmat, axis=1)[:, None] # Structure function weighted average

    return expander

def mk_dm_2k(rad = 25.6):
    x = np.arange(50) - 24.5
    r = (x[:, None]**2 + x[None, :]**2)**.5

    return r < rad