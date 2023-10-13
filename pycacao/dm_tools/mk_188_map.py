import typing as t

import numpy as np
from astropy.io import fits

from pycacao.calib.rmdecode import project_zonal_on_modal


def mk_coords_bim188():
    R = 60  # reference pup radius (a/r)
    # Relative radii of actuator rings

    ring_radii = np.array([17.0, 24, 30, 37, 43, 50], np.float32) / R
    ring_count = [18, 24, 30, 36, 40, 40]  # Number actuator per ring.
    ring_offsets = [5, 7, 8, 10, 10.5,
                    11]  # Angular offset of ring origin in unit of actuators.

    actuator_position_ringlist = []

    for ring in range(6):
        n_r = ring_count[ring]
        offset_r = ring_offsets[ring]
        rad_r = ring_radii[ring]

        argument = 2 * np.pi / n_r * (np.arange(n_r) + offset_r)
        pos = rad_r * np.c_[np.cos(argument), np.sin(argument)]

        actuator_position_ringlist += [pos]

    actuator_pos = np.vstack(actuator_position_ringlist)

    return actuator_pos


def write_file(actuator_pos):
    with open('tmp_actpos.txt', 'w') as f:
        for nn, (xpos, ypos) in enumerate(p):
            f.write(f'{nn:4d} {xpos:+1.3f} {ypos:+1.3f}\n')
            
def write_neighbor_file(neighbors_all: t.List[t.List[int]]):
    with open('act_neighbors.txt', 'w') as f:
        for act, neighbors in enumerate(neighbors_all):
            f.write(f'{act:4d}')
            for nn in neighbors:
                f.write(f' {nn:4d}')
            f.write('\n')
            
def write_pair_file(pairs: t.List[t.Tuple[int, int]]):
    with open('act_neighbor_pairs.txt', 'w') as f:
        for ii, jj in pairs:
            f.write(f'{ii:d} {jj:d}\n')


def influ_functions_of_188_onto_50x50(actuator_pos: np.ndarray,
                                      size_buff: int = 50,
                                      influ_radius: float = 1 / 12.):
    # assume actuator pos are given in a coordinate system where pup center = (0,0) and pup_radius is 1.

    # Make a 50x50 coordinate grid, the usual
    row_map = np.arange(size_buff)[:, None] + np.zeros((1, size_buff))
    col_map = row_map.T.copy()

    n_actu = len(actuator_pos)
    '''
    # Scale actuator pos into pupil footprint on 50x50
    # -1,1 -> -0.5, 49.5 but actually it's smaller.
    from scexao vispy bin2 cacaovars:
    '''
    CACAO_DM_beam_xcent = 24.2
    CACAO_DM_beam_ycent = 23.5
    CACAO_DM_beam_rad = 24.5

    actu_pos_n = np.zeros_like(actuator_pos)
    actu_pos_n[:, 0] = actuator_pos[:, 0] * CACAO_DM_beam_rad + CACAO_DM_beam_xcent
    actu_pos_n[:, 1] = actuator_pos[:, 1] * CACAO_DM_beam_rad + CACAO_DM_beam_ycent

    influ_radius_n = influ_radius * size_buff / 2.

    actu_influ_nxn = np.zeros((n_actu, size_buff, size_buff))

    for ii in range(n_actu):
        r_pos, c_pos = actu_pos_n[ii]

        actu_influ_nxn[ii] = np.exp(-((row_map - r_pos)**2 +
                                      (col_map - c_pos)**2) / 2 /
                                    influ_radius_n**2)

    return actu_influ_nxn

_typethis = t.Tuple[t.List[t.List[int]], t.List[t.Tuple[int, int]]]
def make_neighbors_from_actupos(pos: np.ndarray) -> _typethis:
    
    n_actu = len(pos)
    
    pos_x = pos[:, 0]
    pos_y = pos[:, 1]
    
    rel_matrix = ((pos_x[None,:] - pos_x[:, None])**2 + (pos_y[None,:] - pos_y[:, None])**2)**.5
    # Remove diagonal
    rel_matrix += 10*np.max(rel_matrix)*np.eye(n_actu)
    
    neighblists: t.List[t.List[int]] = []
    pairs: t.List[t.Tuple[int, int]] = []
    
    for ii in range(n_actu):
        sorted = np.argsort(rel_matrix[ii])
        how_many = (3, 6)[ii >= 18 and ii < 188 - 40]
        neighblists += [list(sorted[:how_many])]
        pairs += [(min(ii,jj), max(ii,jj)) for jj in neighblists[ii]]
        
    pairs = list(set(pairs)) # remove potential duplicates
        

    return neighblists, pairs
    
    
    


if __name__ == "__main__":
    p = mk_coords_bim188()
    write_file(p)
    
    neighblists, pairs = make_neighbors_from_actupos(p)
    write_neighbor_file(neighblists)
    write_pair_file(pairs)
    

    influs = influ_functions_of_188_onto_50x50(p)
    fits.writeto('influ_sim.fits', influs, overwrite=True)

    sim_wd = '/home/aorts/src/milk/plugins/cacao-src/AOloopControl/examples/ao3k-nirpyr188-conf/simLHS/'

    sim_respM_2500 = fits.getdata(sim_wd + '/respM160_2500.fits')

    bim_respM = project_zonal_on_modal(
            modes=influs, resp_matrix=sim_respM_2500).astype(np.float32)

    fits.writeto('respM_160x160_toBIM188.fits', bim_respM, overwrite=True)
    
    
    
    1/0
    import matplotlib.pyplot as plt; plt.ion()
    
    plt.scatter(p[:,0], p[:,1], c=[len(l) for l in neighblists])
    for p1, p2 in pairs:
        plt.plot([p[p1, 0], p[p2, 0]], [p[p1, 1], p[p2, 1]], 'k-')