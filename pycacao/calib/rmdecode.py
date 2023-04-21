from astropy.io import fits

import numpy as np

import logging


def decode_pokes_to_zonal_fitsio(
        file_in_poke_modes: str,  # conf/RMmodesDM/HpokeC.fits
        file_in_resp: str,  # conf/RMmodesWFS/HpokeC.WFSresp.fits
        file_out_zpokes: str,  # conf/RMmodesDM/zpokeC-H.fits
        file_out_zresp: str,  # conf/RMmodesWFS/zrespM-H.fits
        *,
        trim_to_mask: bool = True,
) -> None:
    '''
        Extension of cacaocc.aolHaddec
        Reconstructs zonal RM from arbitrary set of modes

        This file-ified version resembles (and englobes)
        cacao-aorun-031-RMHdecode

        file_in_poke_modes:
            Modes (not pokes) - corresponds to HpokeC.fits
        file_in_resp:
            Response obtained through measlinresp - corresponds to HpokeC.WFSresp.fits


        file_out_zpokes:
            Zonal poked actuators (<shape>) - corresponds to zpokeC.fits
        file_out_zreps:
            Zonal poke response - corresponds to zrespM.fits

    '''
    poke_modes = fits.getdata(file_in_poke_modes)
    resp_matrix = fits.getdata(file_in_resp)

    dm_i, dm_j = poke_modes.shape[1], poke_modes.shape[2]

    mask_can_be_controlled, zonal_resp_matrix = decode_pokes_to_zonal(
            poke_modes, resp_matrix)

    # For compat reasons... we'll see where that gets us.
    if trim_to_mask:
        n_actu_ctrl = np.sum(mask_can_be_controlled)
        poke_matrix = np.zeros((n_actu_ctrl, dm_i * dm_j))
        _wh = np.where(mask_can_be_controlled > 0)[0]
        for kk, ww in enumerate(_wh):
            poke_matrix[kk, ww] = 1.0
        poke_matrix = poke_matrix.reshape(n_actu_ctrl, dm_i, dm_j)

        zonal_resp_matrix = zonal_resp_matrix[mask_can_be_controlled]
    else:
        poke_matrix = np.diag(mask_can_be_controlled)
        poke_matrix = poke_matrix.reshape(dm_i * dm_j, dm_i, dm_j)

    # TODO
    fits.writeto(file_out_zpokes, poke_matrix, overwrite=True)
    fits.writeto(file_out_zresp, zonal_resp_matrix, overwrite=True)


def project_zonal_on_modal(
    modes: np.ndarray,
    resp_matrix: np.ndarray,
):
    # Either ndim == 2 and it's actuator maps
    # Or ndim = 2 and it's modal coefficients
    # resp_matrix should have flattened actuators as first axis and pixels as second and third.

    assert modes.ndim in [2,3]
    assert resp_matrix.ndim == 3
    
    if modes.ndim == 3:
        modes_flat = modes.reshape(modes.shape[0], modes.shape[1]*modes.shape[2])
    else:
        modes_flat = modes
        
    
    n_act, n_px1, n_px2 = resp_matrix.shape
    respm_flat = resp_matrix.reshape(n_act, n_px1 * n_px2)
                               
    n_modes = modes_flat.shape[0]
    
    assert resp_matrix.shape[0] == modes_flat.shape[1]
    
    modal_respm_flat = modes_flat @ respm_flat
    
    # De flatten the pixel axis:
    modal_respm = modal_respm_flat.reshape(n_modes, n_px1, n_px2)
    
    return modal_respm
    
        
    

def decode_pokes_to_zonal(
        poke_modes: np.ndarray,
        resp_matrix: np.ndarray,
) -> np.ndarray:
    '''

    '''

    # Dimension checks
    assert poke_modes.ndim == 3, "poke_modes not 3D"
    assert resp_matrix.ndim == 3, "resp_matrix not 3D"
    assert poke_modes.shape[0] == resp_matrix.shape[
            0], "poke_modes / resp_matrix: incompatible shapes."

    n_modes = poke_modes.shape[0]
    dm_i, dm_j = poke_modes.shape[1:]
    wfs_i, wfs_j = resp_matrix.shape[1:]

    # 2D arrays
    poke_modes_2D = poke_modes.reshape(n_modes, dm_i * dm_j)
    resp_matrix_2D = resp_matrix.reshape(n_modes, wfs_i * wfs_j)

    # How many independent actuators can we control?
    _q, _r = np.linalg.qr(poke_modes_2D, 'raw')
    actu_q = np.sum(_q**2, axis=1)
    QR_TOLERANCING = 1e-6
    mask_actu_ctrl = actu_q > (actu_q.max() * QR_TOLERANCING)
    n_actu_ctrl = np.sum(mask_actu_ctrl)

    assert n_actu_ctrl <= n_modes, "More actuators in span than modes to control them."

    logging.info(
            f"n_actu_ctrl: {n_actu_ctrl} actuators can be controlled [From basis QR dec.]"
    )

    # Perform poke inversions to controllable actuators
    inverse_of_pokes = np.zeros_like(poke_modes_2D.T)
    inverse_of_pokes[mask_actu_ctrl, :] = np.linalg.pinv(
            poke_modes_2D[:, mask_actu_ctrl])

    # Decode the matrix
    zonal_resp_matrix_2D = inverse_of_pokes @ resp_matrix_2D
    zonal_resp_matrix = zonal_resp_matrix_2D.reshape(dm_i * dm_j, wfs_i, wfs_j)

    return mask_actu_ctrl, zonal_resp_matrix


if __name__ == "__main__":  # quick devdebug test

    FILE_HpokeC = "conf/RMmodesDM/HpokeC.fits"
    FILE_HrespC = "conf/RMmodesWFS/HpokeC.WFSresp.fits"

    FILEOUT_zpokeC = "conf/RMmodesDM/zpokeC-H.fits"
    FILEOUT_zrespC = "conf/RMmodesWFS/zrespM-H.fits"

    decode_pokes_to_zonal_fitsio(FILE_HpokeC, FILE_HrespC, FILEOUT_zpokeC,
                                 FILEOUT_zrespC)
