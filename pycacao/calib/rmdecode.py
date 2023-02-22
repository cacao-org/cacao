from astropy.io import fits

import numpy as np

import logging


def decode_pokes_to_zonal_fitsio(
        file_in_poke_modes: str,
        file_in_resp: str,
        file_out_zpokes: str,
        file_out_zresp: str,
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
            Zonal poked actuators (<shape>) - corresponds to zpokeC-H.fits
        file_out_zreps:
            Zonal poke response - corresponds to zrespM.fits

    '''
    poke_modes = fits.getdata(file_in_poke_modes)
    resp_matrix = fits.getdata(file_in_resp)

    mask_can_be_controlled, zonal_resp_matrix = decode_pokes_to_zonal(
            poke_modes, resp_matrix)

    # TODO
    fits.writeto('tmp_pycacao_mask.fits', zonal_resp_matrix)
    fits.writeto('tmp_pycacao_zrespM.fits', mask_can_be_controlled)
    #fits.writeto(file_out_zresp, resp)


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

    return mask_actu_ctrl, zonal_resp_matrix, _q, _r


if __name__ == "__main__":  # quick devdebug test
    pfx = '/home/vdeo/sshfs/RM_2023_smartmodes'
    A, B, _q, _r = decode_pokes_to_zonal(
            fits.getdata(f'{pfx}/RMmodesDM/HpokeC.fits'),
            fits.getdata(f'{pfx}/RMmodesWFS/HpokeC.WFSresp.fits'),
    )

    if False:
        A, B, _q, _r = decode_pokes_to_zonal(
                fits.getdata(f'{pfx}/RMmodesDM/synRMmodesDM.fits'),
                fits.getdata(f'{pfx}/RMmodesWFS/synRMmodesWFS.fits'),
        )
