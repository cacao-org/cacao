# SPDX-FileCopyrightText: 2026 Vincent Deo
#
# SPDX-License-Identifier: LGPL-3.0-or-later

import numpy as np
from astropy.io import fits

"""
    Compute straight CM

    Copy functionality compute_straight_CM.c

    inputs:
        RMmodesDM.fits
        RMmodesWFS.fits
        svdlim
        GPU device (no need)

    outputs:
        CMmodesDM.fits
        CMmodesWFS.fits
"""
"""
# Summary of what the function needs to do
MDM = fits.getdata('./conf/RMmodesDM/RMmodesDM.fits')
MDM.shape

MWFS = fits.getdata('./conf/RMmodesWFS/RMmodesWFS.fits')
MWFS.shape

MDMf = MDM.reshape(2606, 2500)
MWFSf = MWFS.reshape(2606, 14400)

ATA = MWFSf @ MWFSf.T

u, v = np.linalg.eigh(ATA, 'U')

CMWFS = v @ MWFSf
CMWFSsq = CMWFS.reshape(2606, 120, 120)

CMDM = v.T @ MDMf
CMDMsq = CMDM.reshape(2606, 50, 50)
"""


def straight_CM_fitsio(
    file_in_rmmodesdm: str,  # conf/RMmodesDM/RMmodesDM.fits
    file_in_rmmodeswfs: str,  # conf/RMmodesDM/RMmodesWFS.fits
    file_out_cmmodesdm: str,  # conf/RMmodesDM/CMmodesDM.fits
    file_out_cmmodeswfs: str,  # conf/RMmodesDM/CMmodesWFS.fits
    file_in_modefilter: str | None,  # conf/RMmodeFilt.fits
    file_in_modeforce: str | None,  # conf/RMmodeForce.fits
    svd_lim: float,
):

    RMmodesDM = fits.getdata(file_in_rmmodesdm)
    RMmodesWFS = fits.getdata(file_in_rmmodeswfs)

    if file_in_modefilter is not None:
        RMmodesFilter = fits.getdata(file_in_modefilter)
    else:
        RMmodesFilter = None

    if file_in_modeforce is not None:
        RMmodesForce = fits.getdata(file_in_modefilter)
    else:
        RMmodesForce = None

    _, CMmodesWFS, CMmodesDM = straight_CM(RMmodesDM, RMmodesWFS)

    fits.writeto(file_out_cmmodesdm, CMmodesDM)
    fits.writeto(file_out_cmmodeswfs, CMmodesWFS)

    return CMmodesWFS, CMmodesDM


def straight_CM(
    modes_DM: np.ndarray,
    modes_WFS: np.ndarray,
    svd_lim: float,
    modes_filter: np.ndarray | None = None,
    modes_force: np.ndarray | None = None,
):
    """
    modes_DM: modal poke matrix, assuming 3D [n_modes * dm_i * dm_j]
    modes_DM: modal resp matrix, assuming 3D [n_modes * wfs_i * wfs_j]

    svd_lim: float

    TODO pretty incomplete
    """

    assert modes_DM.ndim == 3, "modes_DM, ndim = 3 [n_modes * dm_i * dm_j]"
    assert modes_WFS.ndim == 3, "modes_WFS, ndim = 3 [n_modes * wfs_i * wfs_j]"
    assert modes_DM.shape[0] == modes_WFS.shape[0]

    n_modes, dm_i, dm_j = modes_DM.shape
    _, wfs_i, wfs_j = modes_WFS.shape

    modes_DM_f = modes_DM.reshape(n_modes, dm_i * dm_j)
    modes_WFS_f = modes_WFS.reshape(n_modes, wfs_i * wfs_j)

    if modes_filter is not None:
        assert (
            modes_filter.ndim == 3
            and modes_filter.shape[1] == modes_DM.shape[1]
            and modes_filter.shape[2] == modes_DM.shape[2]
        ), "modes_filter, ndim = 3 [n_filter * dm_i * dm_j]"

        # We're gonna have to write a loop cuz my linear algebra is rusty
        modes_filter_normalized = (
            modes_filter / np.sum(modes_filter**2, axis=(1, 2)) ** 0.5
        )
        n_modes_filter = modes_filter.shape[0]
        modes_filter_normalized_flat = modes_filter_normalized.reshape(
            n_modes_filter, dm_i * dm_j
        )

        modes_filter_inv = np.linalg.pinv(
            modes_filter_normalized_flat
        )  # (dm_ij * n_mode_filter)
        # That's not correct still.
        modes_WFS_f -= (modes_DM_f @ modes_filter_inv).T @ modes_WFS_f

    mat_wfsTwfs = modes_WFS_f @ modes_WFS_f.T  # n_modes x n_modes

    u, v = np.linalg.eigh(mat_wfsTwfs, "U")

    # tr

    CMWFS = v.T @ modes_WFS_f
    CMDM = v.T @ modes_DM_f

    # TODO moveaxis and reshape here
    return u, CMWFS, CMDM


import click


@click.command("Cacao loop straight CM computation")
@click.argument(
    "svdlim", type=click.FloatRange(min=0.0, max=1.0, min_open=True, max_open=True)
)
@click.option(
    "-f",
    "--filter",
    is_flag=True,
    help="Toggle mode filtering from RM before inversion [uses RMmodeFilt.fits]",
)
def straight_CM_entrypoint(svdlim: float, filter: bool):
    print(f"    svdlim: {svdlim}")
    print(f"    filter:, {filter}")

    from ..arch.confutil import CacaoConf

    cacao_conf = CacaoConf.from_pwd_tree(".")
    cacao_conf.ensure_cwd()  # Redundant...

    filter_file = cacao_conf.PWD + "/conf/RMmodeFilt.fits" if filter else None

    straight_CM_fitsio(
        cacao_conf.PWD + "/conf/RMmodesDM/RMmodesDM.fits",
        cacao_conf.PWD + "/conf/RMmodesWFS/RMmodesWFS.fits",
        cacao_conf.PWD + "/conf/CMmodesDM/CMmodesDM.fits",
        cacao_conf.PWD + "/conf/CMmodesWFS/CMmodesWFS.fits",
        svdlim,
        filter_file,
    )


if __name__ == "__main__":

    import os

    straight_CM_entrypoint()
