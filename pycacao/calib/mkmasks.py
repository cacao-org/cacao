from astropy.io import fits

import numpy as np

import logging

from typing import Tuple
from collections import namedtuple

PercentileParam = namedtuple('PercentileParam', ['p0', 'c0', 'p1', 'c1'])


def make_masks_fitsio(
        file_in_resp: str,  # conf/RMmodesWFS/zrespM-H.fits
        dm_perc: PercentileParam = PercentileParam(p0=0.2, c0=0.5, p1=0.5,
                                                   c1=0.5),
        wfs_perc: PercentileParam = PercentileParam(p0=0.2, c0=0.5, p1=0.5,
                                                    c1=0.5),
        *,
        dm_size: Tuple[int, int] = (50, 50)) -> None:
    '''
        Replacement of RMmkmask cacao bash script

        file_in_resp:
            Response obtained through measlinresp - decoded - corresponds to zrespM-H.fits

        dm_perc:
            PercentileParam - percentiles for DMmask truncation
        wfs_perc:
            PercentileParam - percentiles for WFSmask truncation

    '''

    resp_matrix = fits.getdata(file_in_resp)

    dm_n, wfs_size_i, wfs_size_j = resp_matrix.shape

    assert dm_n == dm_size[0] * dm_size[1]

    # Re-arrange into 4D tensor
    resp_4D = resp_matrix.reshape(*dm_size, wfs_size_i, wfs_size_j)

    dm_map, dm_mask, wfs_map, wfs_mask = make_masks(resp_4D, dm_perc, wfs_perc)

    fits.writeto('./conf/dmmap.fits', dm_map, overwrite=True)
    fits.writeto('./conf/dmmask.fits', dm_mask.astype(np.float32),
                 overwrite=True)
    fits.writeto('./conf/wfsmap.fits', wfs_map, overwrite=True)
    fits.writeto('./conf/wfsmask.fits', wfs_mask.astype(np.float32),
                 overwrite=True)


def make_masks(
        resp_4D: np.ndarray,
        dm_perc: PercentileParam,
        wfs_perc: PercentileParam,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    '''

    '''

    # Dimension checks
    assert resp_4D.ndim == 4, "resp_matrix not 4D"

    dm_map = np.sum(resp_4D**2, axis=(2, 3))
    wfs_map = np.sum(resp_4D**2, axis=(0, 1))

    dm_mask = apply_percentile_masker(dm_map, dm_perc)
    wfs_mask = apply_percentile_masker(wfs_map, wfs_perc)

    return dm_map, dm_mask, wfs_map, wfs_mask


def apply_percentile_masker(map: np.ndarray, perc: PercentileParam):

    p0_val, p1_val = np.percentile(map, (perc.p0 * 100, perc.p1 * 100))

    return map > (p0_val * perc.c0 + p1_val * perc.c1)


if __name__ == "__main__":

    pass
