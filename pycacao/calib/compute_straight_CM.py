import numpy as np
from astropy.io import fits
'''
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
'''

'''
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
'''


def straight_CM_fitsio():
    pass


def straight_CM(modes_DM: np.ndarray, modes_WFS: np.ndarray, svd_lim: float):
    '''
        modes_DM: modal poke matrix, assuming 3D [n_modes * dm_i * dm_j]
        modes_DM: modal resp matrix, assuming 3D [n_modes * wfs_i * wfs_j]

        svd_lim: float
        
        TODO pretty incomplete
    '''

    assert modes_DM.ndim == 3, "modes_DM, ndim = 3 [n_modes * dm_i * dm_j]"
    assert modes_WFS.ndim == 3, "modes_WFS, ndim = 3 [n_modes * wfs_i * wfs_j]"
    assert modes_DM.shape[0] == modes_WFS.shape[0]

    n_modes, dm_i, dm_j = modes_DM.shape
    _, wfs_i, wfs_j = modes_WFS.shape

    modes_DM_f = modes_WFS.reshape(n_modes, wfs_i * wfs_j)
    modes_WFS_f = modes_WFS.reshape(n_modes, wfs_i * wfs_j)

    mat_wfsTwfs = modes_WFS_f @ modes_WFS_f.T  # n_modes x n_modes

    u, v = np.linalg.eigh(mat_wfsTwfs, 'U')

    # tr

    CMWFS = v.T @ modes_WFS_f
    CMDM = v.T @ modes_DM_f

    return u, CMWFS, CMDM
