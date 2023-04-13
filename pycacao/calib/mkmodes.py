'''
    mkmodes.py

    TODO: Move the main exec from the function lib...

    Usage:
        mkmodes.py (had|randhad) <outfile.fits> [--mask=<maskfile.fits>]
'''

# Random-permuted hadamard
# Zernikes
# Fourier
# Actuators
# DM_KL (= Gendron's ~= Btt)

# -> all maskable.
# MMSE-er

from astropy.io import fits
import numpy as np

from typing import Tuple

from ..arch.confutil import CacaoConf


def make_disk_from_conf(conf: CacaoConf, save_out: bool = True) -> np.ndarray:

    conf.ensure_cwd()

    disk_mask = make_disk(conf.get_dm_pup_params(), conf.get_dm_size())

    if save_out:
        pass

    return disk_mask


def make_disk(disk_parameters: Tuple[float, float, float],
              dm_size: Tuple[int, int]) -> np.ndarray:
    '''
        Make boolean ndarray disk
    '''
    cx, cy, r = disk_parameters
    dm_x, dm_y = dm_size

    x = np.arange(dm_x)
    y = np.arange(dm_y)

    mask = ((x[:, None] - cx)**2 + (y[None, :] - cy)**2)**.5 < r

    # FIXME
    # OK so the problem is that to match the CACAO convention
    # we're gonna need a transpose either here or at file write time

    return mask.T


def make_zonal(mask: np.ndarray) -> np.ndarray:
    '''
        Doesn't do much... cube with single poke slices from a mask.
    '''

    n_actu = np.sum(mask)
    dm_x, dm_y = mask.shape

    modal_cube = np.zeros((n_actu, dm_x, dm_y), dtype=np.float32)

    mask_wh = np.where(mask)

    for ii in range(n_actu):
        modal_cube[ii, mask_wh[0][ii], mask_wh[1][ii]] = 1.0

    return modal_cube


def make_hadamard(mask: np.ndarray, permuter: np.ndarray = None,
                  permute_random: bool = False
                  ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    '''
        Make permuted Hadamard basis.

        If permute_random, generate a random permutation of Hadamard modes.
    '''

    n_actu = np.sum(mask)

    if permuter is not None:
        assert len(permuter) == n_actu
        assert len(np.unique(permuter)) == n_actu
    else:
        if not permute_random:
            permuter = np.arange(n_actu)
        else:
            permuter = np.argsort(np.random.rand(n_actu))
            # WARNING: having 0 in the permuter is an actuator that doens't move at all!
            # Cuz row / col 0 of Hadamard is piston.

    dm_x, dm_y = mask.shape

    next_pow2 = 1
    while next_pow2 < n_actu:
        next_pow2 *= 2

    from scipy import linalg as sclinalg

    hadamard_matrix = sclinalg.hadamard(next_pow2)

    modal_cube = np.zeros((next_pow2, dm_x, dm_y), dtype=np.float32)
    for ii in range(next_pow2):
        modal_cube[ii, mask] = hadamard_matrix[ii][permuter]

    return modal_cube, hadamard_matrix, permuter


def make_fourier(dm_size: Tuple[int, int], dm_radius: float, cpa_max: int = 8,
                 delta_cpa: float = 0.8,
                 radius_factor_limit: float = 1.5) -> np.ndarray:
    pass


def make_zernike(n: int):
    pass


def make_dmkl(mask: np.ndarray, remove_piston: bool = True) -> np.ndarray:
    '''
    DM KL, or proximity-based MMSE basis, or radial-Fourier basis.
    No real official name. But essentially KLs.

    Extension to positional, non-matrix DMs will be easy.
    '''

    n_actu = np.sum(mask)
    dm_x, dm_y = mask.shape

    xd, yd = np.where(mask)

    geom_covariance = ((xd[None, :] - xd[:, None])**2 +
                       (yd[None, :] - yd[:, None])**2)**(5 / 6.0)
    # Diagonalize - Columns of sv_modes are our basis
    sv_modes, _sv_values, _ = np.linalg.svd(geom_covariance)

    # Note the 1 smaller and offset-by-one if removing piston.
    modal_cube = np.zeros((n_actu - remove_piston, dm_x, dm_y),
                          dtype=np.float32)
    for ii in range(n_actu - remove_piston):
        modal_cube[ii, mask] = sv_modes[:, ii + remove_piston]

    return modal_cube


'''
    Proposed DMKL pipeline:

    - Make randomized Hadamards
    - Acquire RM
    - Zonal-ify
    - Make masks (dmdrive, dmslaved, wfsmask)

    Pipeline one:
        - SV-TSVD on unmasked RM --- ACTUALLY SHOULD mask in WFS space.
        - Erase [dmslaved] and perform extension of DMmodes from [dmdrive] onto [dmslaved]

    Pipeline two
        - Make DMKL on dmdrive
        - Extension of DMmodes from [dmdrive] onto [dmslaved]
        - Synth RM onto this basis
        - Rank-trunc-TSVD on this basis. Inversion from masked WFS.
'''

if __name__ == "__main__":

    import os

    from docopt import docopt
    args = docopt(__doc__)

    cacao_conf = CacaoConf.from_pwd_tree('.')
    cacao_conf.ensure_cwd()  # Redundant...

    outfile = args['<outfile.fits>']
    randomize_had = args['randhad']
    maskfile = args['--mask']

    assert not '/' in outfile
    assert outfile.endswith('.fits')

    if maskfile is not None:
        assert not '/' in maskfile
        assert maskfile.endswith('.fits')

        mask = fits.getdata(cacao_conf.PWD + '/conf/RMmodesDM/' + maskfile)

    else:
        mask = make_disk_from_conf(cacao_conf)

    modal_cube, _, _ = make_hadamard(mask, permute_random=randomize_had)

    fits.writeto(cacao_conf.PWD + '/conf/RMmodesDM/' + outfile, modal_cube,
                 overwrite=True)
