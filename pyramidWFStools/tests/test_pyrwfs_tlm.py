# SPDX-FileCopyrightText: 2026 Olivier Guyon et al
#
# SPDX-License-Identifier: LGPL-3.0-or-later

"""End-to-end verification tests for cacao/pyramidWFStools (C core, @calc ext, .calc units, CLI)."""

import math
import os
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest
from astropy.io import fits

INST = Path("/home/guyon/src/milk/_install/milk-1.03.00")
BIN = INST / "bin"


def make_env(tmp_path: Path) -> dict:
    shm = tmp_path / "shm"
    cache = tmp_path / "calc_cache"
    work = tmp_path / "work"
    shm.mkdir(exist_ok=True)
    cache.mkdir(exist_ok=True)
    work.mkdir(exist_ok=True)
    env = os.environ.copy()
    env["MILK_INSTALLDIR"] = str(INST)
    env["MILK_SHM_DIR"] = str(shm)
    env["MILK_CALC_CACHE"] = str(cache)
    env["TMPDIR"] = str(tmp_path)
    env["PATH"] = f"{BIN}:{env.get('PATH', '')}"
    env["MILK_QUIET"] = "1"
    return env


def run_cli(cmd: str, env: dict, cwd: Path, timeout: int = 60) -> subprocess.CompletedProcess:
    proc = subprocess.run(
        [str(BIN / "milk-cli"), "-q", "-e", "-o", "-c", cmd],
        env=env,
        cwd=str(cwd),
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    if proc.returncode != 0:
        raise AssertionError(
            f"milk-cli failed (rc={proc.returncode})\ncmd: {cmd}\n"
            f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )
    return proc


def sim_pywfs_frame(
    phase: np.ndarray,
    amp: np.ndarray | None = None,
    N: int = 128,
    R: float = 18.0,
    s: float = 24.0,
    shift_xy: tuple[float, float] = (0.0, 0.0),
    sep_dxy: tuple[float, float] = (0.0, 0.0),
    flux: float = 1e6,
    bg: float = 10.0,
) -> np.ndarray:
    """2D Fourier-optics simulation of a non-modulated 4-sided PyWFS (annular pupil + 4 spiders)."""
    y, x = np.mgrid[-N // 2 : N // 2, -N // 2 : N // 2].astype(np.float64)
    r = np.hypot(x, y)
    P = ((r <= R) & (r >= 0.30 * R)).astype(np.float64)
    # 45-deg spiders
    ca = math.cos(math.pi / 4.0)
    xr = np.abs((x + y) * ca)
    yr = np.abs((-x + y) * ca)
    P[(xr < 1.0) | (yr < 1.0)] = 0.0
    if amp is not None:
        P = P * amp

    psi = P * np.exp(1j * phase)
    F = np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(psi)))
    fy, fx = np.mgrid[-N // 2 : N // 2, -N // 2 : N // 2] / float(N)

    sx = s + sep_dxy[0]
    sy = s + sep_dxy[1]
    dx, dy = shift_xy
    # Pyramid phase ramp + optional global pupil shift (forward fft2 shifts by +dx, +dy with +1j)
    M = np.exp(-1j * 2.0 * np.pi * (sx * np.abs(fx) + sy * np.abs(fy)) + 1j * 2.0 * np.pi * (dx * fx + dy * fy))
    E = np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(F * M)))
    I = np.abs(E) ** 2
    I = I / np.sum(I) * flux + bg
    return I.astype(np.float32)


def make_kolm_phase(N: int, R: float, rms: float, seed: int, fmin: float = 1.0, fmax: float = 18.0):
    rng = np.random.default_rng(seed)
    fy, fx = np.mgrid[-N // 2 : N // 2, -N // 2 : N // 2].astype(np.float64)
    fr = np.hypot(fx, fy)
    mask_f = (fr >= fmin) & (fr <= fmax)
    amp = np.zeros_like(fr)
    amp[mask_f] = fr[mask_f] ** (-11.0 / 6.0)
    F = amp * np.exp(1j * rng.uniform(0, 2 * np.pi, (N, N)))
    ph = np.real(np.fft.ifft2(np.fft.ifftshift(F)))
    y, x = np.mgrid[-N // 2 : N // 2, -N // 2 : N // 2]
    pup = (x**2 + y**2 <= R**2) & (x**2 + y**2 >= (0.3 * R) ** 2)
    ph -= ph[pup].mean()
    ph /= ph[pup].std() + 1e-12
    return ph * rms


def test_calc_extensions_calib_and_physics(tmp_path: Path):
    """Verify @calc Tier 2 ops, pyrwfs.calib, Strehl flux physics, anisotropy, and 6-DOF pupreg."""
    env = make_env(tmp_path)
    work = tmp_path / "work"
    N, R, s = 128, 18.0, 24.0

    I_flat = sim_pywfs_frame(np.zeros((N, N)), N=N, R=R, s=s, bg=15.0)
    ph_ab = make_kolm_phase(N, R, rms=1.2, seed=7)
    I_ab = sim_pywfs_frame(ph_ab, N=N, R=R, s=s, bg=15.0)

    # Shifted + aberrated + strong tip-tilt illumination imbalance!
    y, x = np.mgrid[-N // 2 : N // 2, -N // 2 : N // 2].astype(np.float64)
    ph_tt = ph_ab + 0.9 * (x / R) - 0.6 * (y / R)
    I_shift = sim_pywfs_frame(
        ph_tt, N=N, R=R, s=s, shift_xy=(1.35, -0.75), sep_dxy=(0.0, 0.0), bg=15.0
    )

    # Radial expansion (defocus / axial focus) and astigmatic separation
    I_sep = sim_pywfs_frame(
        ph_ab, N=N, R=R, s=s, shift_xy=(0.0, 0.0), sep_dxy=(1.10, -0.80), bg=15.0
    )

    # Anisotropic 1D horizontal vs vertical jitter average
    I_jit_x = np.mean(
        [sim_pywfs_frame(2.0 * np.pi * u * (x / (2 * R)), N=N, R=R, s=s, bg=15.0)
         for u in np.linspace(-1.5, 1.5, 7)],
        axis=0,
    ).astype(np.float32)
    I_jit_y = np.mean(
        [sim_pywfs_frame(2.0 * np.pi * u * (y / (2 * R)), N=N, R=R, s=s, bg=15.0)
         for u in np.linspace(-1.5, 1.5, 7)],
        axis=0,
    ).astype(np.float32)

    for name, arr in [
        ("flat.fits", I_flat),
        ("ab.fits", I_ab),
        ("shift.fits", I_shift),
        ("sep.fits", I_sep),
        ("jitx.fits", I_jit_x),
        ("jity.fits", I_jit_y),
    ]:
        fits.writeto(work / name, arr, overwrite=True)

    caldir = work / "cal"
    cmd = f"""
iofits.loadfits flat.fits @S:ref 1
iofits.loadfits ab.fits @S:im_ab 1
iofits.loadfits shift.fits @S:im_sh 1
iofits.loadfits sep.fits @S:im_sep 1
iofits.loadfits jitx.fits @S:im_jx 1
iofits.loadfits jity.fits @S:im_jy 1
pyrwfs.calib ref pyr .outdir={caldir}
@calc '
    m0 = pyrwfs_metrics(ref, pyr_zones, ref)
    ma = pyrwfs_metrics(im_ab, pyr_zones, ref)
    mx = pyrwfs_metrics(im_jx, pyr_zones, ref)
    my = pyrwfs_metrics(im_jy, pyr_zones, ref)
    g_sh, d_sh = pyrwfs_pupreg(im_sh, ref, pyr_nomgeom, maxshift=5, edge_mode=1)
    g_sp, d_sp = pyrwfs_pupreg(im_sep, ref, pyr_nomgeom, maxshift=5, edge_mode=1)
    isum, sx, sy = pyrwfs_extract4(im_ab, g_sh, npix=40, norm_mode=0)
    S:o_m0 = m0; S:o_ma = ma; S:o_mx = mx; S:o_my = my
    S:o_dsh = d_sh; S:o_dsp = d_sp; S:o_isum = isum; S:o_sx = sx; S:o_sy = sy
'
iofits.saveFITS o_m0 m0.fits
iofits.saveFITS o_ma ma.fits
iofits.saveFITS o_mx mx.fits
iofits.saveFITS o_my my.fits
iofits.saveFITS o_dsh dsh.fits
iofits.saveFITS o_dsp dsp.fits
iofits.saveFITS o_isum isum.fits
"""
    run_cli(cmd, env, work)

    nomgeom = fits.getdata(caldir / "nomgeom.fits")
    m0 = fits.getdata(work / "m0.fits")
    ma = fits.getdata(work / "ma.fits")
    mx = fits.getdata(work / "mx.fits")
    my = fits.getdata(work / "my.fits")
    dsh = fits.getdata(work / "dsh.fits")
    dsp = fits.getdata(work / "dsp.fits")
    isum = fits.getdata(work / "isum.fits")

    # 1. Auto-detected pupil centers within 1 px of truth (64 +/- 24)
    exp_x = np.array([64 - s, 64 - s, 64 + s, 64 + s])
    exp_y = np.array([64 - s, 64 + s, 64 - s, 64 + s])
    assert np.max(np.abs(nomgeom[:, 0] - exp_x)) < 1.2
    assert np.max(np.abs(nomgeom[:, 1] - exp_y)) < 1.2
    assert np.all(np.abs(nomgeom[:, 2] - R) < 1.5)

    # 2. Strehl-drop flux transfer physics: positive inside pupils, negative between!
    assert 0.35 < m0[5] < 0.72, f"Unexpected flat f_pup0={m0[5]}"
    assert ma[10] > 0.10, f"Expected df_pup > +0.10 under 1.2 rad RMS, got {ma[10]}"
    assert ma[6] < m0[6] - 0.03, f"Expected apex diamond drop: flat={m0[6]}, ab={ma[6]}"
    assert ma[12] > 0.10, f"Expected nonzero nl_ratio under 1.2 rad RMS, got {ma[12]}"
    assert 15.0 <= m0[1] < 26.0, f"Corner background should track 15 ADU + far diffraction tail, got {m0[1]}"

    # 3. Anisotropy flips sign cleanly between X-jitter and Y-jitter
    assert mx[11] < -0.05 and my[11] > 0.05 or (mx[11] * my[11] < -0.002), (
        f"Expected opposite anisotropy signs for X vs Y jitter: mx={mx[11]}, my={my[11]}"
    )

    # 4. Edge-based 6-DOF pupil registration recovers shift under Strehl drop + TT imbalance!
    assert abs(dsh[0] - 1.35) < 0.15, f"Expected dx=1.35, got {dsh[0]}"
    assert abs(dsh[1] - (-0.75)) < 0.15, f"Expected dy=-0.75, got {dsh[1]}"
    assert dsh[7] < 0.15, f"Expected tiny non-rigid residual, got {dsh[7]}"

    # 5. Differential pupil separation (dsx, dsy) recovered accurately!
    assert abs(dsp[2] - 1.10) < 0.15, f"Expected dsx=+1.10, got {dsp[2]}"
    assert abs(dsp[3] - (-0.80)) < 0.15, f"Expected dsy=-0.80, got {dsp[3]}"
    assert isum.shape == (40, 40) and np.max(isum) > 0


def test_pywfsgridmatch_exact_affine_non_square_dm(tmp_path: Path):
    """Verify upgraded pyWFSgridmatch on a non-square (12x10) DM with rotation and unequal pitch."""
    env = make_env(tmp_path)
    work = tmp_path / "work"

    nx, ny = 128, 128
    dmx, dmy = 12, 10
    cx0, cy0 = 64.5, 63.2
    sx0, sy0 = 28.0, 27.0
    pitch_x, pitch_y = 2.35, 2.15
    ang = 0.14  # rad
    Axx = pitch_x * math.cos(ang)
    Ayx = pitch_x * math.sin(ang)
    Axy = -pitch_y * math.sin(ang)
    Ayy = pitch_y * math.cos(ang)

    centers = [
        (cx0 - sx0, cy0 - sy0),
        (cx0 - sx0, cy0 + sy0),
        (cx0 + sx0, cy0 - sy0),
        (cx0 + sx0, cy0 + sy0),
    ]

    yy, xx = np.mgrid[0:ny, 0:nx].astype(np.float32)
    zrm = np.zeros((dmx * dmy, ny, nx), dtype=np.float32)
    dm_cx = 0.5 * (dmx - 1)
    dm_cy = 0.5 * (dmy - 1)

    for j in range(dmy):
        for i in range(dmx):
            u = i - dm_cx
            v = j - dm_cy
            if math.hypot(u / (0.5 * dmx), v / (0.5 * dmy)) > 0.92:
                continue  # unilluminated corner actuator
            sl = j * dmx + i
            img = np.zeros((ny, nx), dtype=np.float32)
            for q, (px0, py0) in enumerate(centers):
                px = px0 + Axx * u + Axy * v
                py = py0 + Ayx * u + Ayy * v
                sgn = 1.0 if (q in (0, 3)) else -1.0
                img += sgn * np.exp(-0.5 * ((xx - px) ** 2 + (yy - py) ** 2) / (1.3**2))
            zrm[sl] = img

    fits.writeto(work / "zrm.fits", zrm, overwrite=True)
    cmd = """
iofits.loadfits zrm.fits @S:zrm 1
cacao-fpsexec-pywfsgridmatch -n gmtest exec .in_name=zrm .spotsize=2.0 .dmxsize=12 .dmysize=10 .outgeom=gm_geom
iofits.saveFITS gm_geom gm_geom.fits
@calc -v cx,cy,sx,sy,px,py,ang,rms '
    f.gmtest.out.center_x; f.gmtest.out.center_y;
    f.gmtest.out.sep_x; f.gmtest.out.sep_y;
    f.gmtest.out.actpitch_x; f.gmtest.out.actpitch_y;
    f.gmtest.out.act_angle; f.gmtest.out.fit_rms
'
printf '%s %s %s %s %s %s %s %s\\n' "$cx" "$cy" "$sx" "$sy" "$px" "$py" "$ang" "$rms" > gm_out.txt
"""
    run_cli(cmd, env, work)

    vals = [float(v) for v in (work / "gm_out.txt").read_text().strip().split()]
    cx, cy, sx, sy, px, py, ang_fit, rms = vals
    assert abs(cx - cx0) < 0.08
    assert abs(cy - cy0) < 0.08
    assert abs(sx - sx0) < 0.08
    assert abs(sy - sy0) < 0.08
    assert abs(px - pitch_x) < 0.05
    assert abs(py - pitch_y) < 0.05
    assert abs(ang_fit - ang) < 0.01
    assert rms < 0.08
    geom = fits.getdata(work / "gm_geom.fits")
    assert geom.shape == (4, 4)


def test_pywfstlm_cube_inspect_and_live_calc_units(tmp_path: Path):
    """Verify pyWFStlm 3D cube mode, pyrwfs.inspect, live streaming, pupsteer.calc, and vibmon.calc."""
    env = make_env(tmp_path)
    work = tmp_path / "work"
    N, R, s = 128, 18.0, 24.0
    y, x = np.mgrid[-N // 2 : N // 2, -N // 2 : N // 2].astype(np.float64)

    I_ref = sim_pywfs_frame(np.zeros((N, N)), N=N, R=R, s=s, bg=10.0)
    fits.writeto(work / "ref.fits", I_ref, overwrite=True)

    # Build 90-frame cube:
    #   0..29 : near-flat, centered
    #  30..59 : aberrated (1.1 rad) + TT jitter + pupil shift (+1.5, -0.8) + scintillation
    #  60..88 : aberrated + spider LWE phase step + pupil shift (+1.5, -0.8)
    #  89     : dark/shutter dropout frame (flux_ok == 0)
    nfr = 90
    cube = np.zeros((nfr, N, N), dtype=np.float32)
    rng = np.random.default_rng(123)
    for k in range(nfr):
        if k < 30:
            ph = make_kolm_phase(N, R, rms=0.10, seed=100 + k)
            cube[k] = sim_pywfs_frame(ph, N=N, R=R, s=s, bg=10.0)
        elif k < 60:
            ph = make_kolm_phase(N, R, rms=1.1, seed=200 + k)
            ph += 0.6 * math.sin(2 * math.pi * k / 8.0) * (x / R)
            scint = 1.0 + 0.18 * make_kolm_phase(N, R, rms=1.0, seed=500 + k, fmin=4, fmax=14)
            scint = np.clip(scint, 0.3, 2.0)
            cube[k] = sim_pywfs_frame(ph, amp=scint, N=N, R=R, s=s, shift_xy=(1.5, -0.8), bg=10.0)
        elif k < 89:
            ph = make_kolm_phase(N, R, rms=0.9, seed=300 + k)
            # Petal piston jump across the 45-deg spiders (LWE)
            ph += 1.4 * (((x + y) > 0) ^ ((-x + y) > 0)).astype(np.float64)
            cube[k] = sim_pywfs_frame(ph, N=N, R=R, s=s, shift_xy=(1.5, -0.8), bg=10.0)
        else:
            cube[k] = np.full((N, N), 10.0, dtype=np.float32)  # shutter closed

    fits.writeto(work / "cube.fits", cube, overwrite=True)

    # 1. Run cacao-pyrwfs-inspect CLI end-to-end on the 3D FITS cube
    repdir = work / "inspect_out"
    t0 = time.perf_counter()
    proc = subprocess.run(
        [str(BIN / "cacao-pyrwfs-inspect"), str(work / "cube.fits"), str(work / "ref.fits"), str(repdir), "10", "1"],
        env=env,
        cwd=str(work),
        capture_output=True,
        text=True,
        timeout=60,
    )
    elapsed = time.perf_counter() - t0
    assert proc.returncode == 0, f"inspect failed:\n{proc.stdout}\n{proc.stderr}"
    assert (repdir / "REPORT.md").is_file()
    tlm = fits.getdata(repdir / "tlm.fits")  # shape (nfr, 29) in numpy (row=frame, col=metric)
    assert tlm.shape == (nfr, 29)

    # Check flux transfer: frames 30..88 must have much higher df_pup than frames 0..29
    df_flat = float(np.mean(tlm[5:28, 11]))
    df_turb = float(np.mean(tlm[35:58, 11]))
    assert df_turb > df_flat + 0.08, f"Expected Strehl-drop df_pup jump: flat={df_flat}, turb={df_turb}"

    # Check TT jitter RMS rises in 35..58 vs 5..28
    assert np.mean(tlm[40:58, 16]) > 2.0 * np.mean(tlm[10:28, 16])

    # Check scintillation index rises in 40..58 vs 10..28
    assert np.mean(tlm[45:58, 19]) > 2.0 * np.mean(tlm[10:28, 19])

    # Check pupil shift tracks (+1.5, -0.8) after registration updates (frames 50..88)
    assert abs(float(tlm[80, 22]) - 1.5) < 0.25, f"Expected pup_dx~1.5, got {tlm[80, 22]}"
    assert abs(float(tlm[80, 23]) - (-0.8)) < 0.25, f"Expected pup_dy~-0.8, got {tlm[80, 23]}"

    # Check flux_ok == 1 on good frames and 0 on shutter-closed frame 89
    assert float(tlm[10, 28]) == 1.0
    assert float(tlm[89, 28]) == 0.0

    # 2. Live streaming smoke test: pyrwfs.tlm_start + pyrwfs.pupsteer_start + pyrwfs.vibmon_start
    live_cmd = """
iofits.loadfits ref.fits @S:aol0_wfsref 1
@calc 'S:aol0_imWFS0 = aol0_wfsref * 1.0'
pyrwfs.calib aol0_wfsref aol0_pyr
pyrwfs.tlm_start 0 .inwfs=aol0_imWFS0 .inref=aol0_wfsref .reg_stride=5
pyrwfs.pupsteer_start 0 .gain=0.25 .deadband=0.01
pyrwfs.vibmon_start 0 .fs_hz=100.0 .upd_stride=32
iofits.loadfits cube.fits @L:cb 1
# Feed 20 shifted frames to drive live pyWFStlm + pupsteer_cmd
for k in $(seq 35 54); do
    @calc "S:aol0_imWFS0 = cb[*, *, ${k}]"
    sleep 0.015
done
pyrwfs.tlm_status 0
@calc -v cx,cy,lk 'aol0_pyr_pupsteer_cmd[0]; aol0_pyr_pupsteer_cmd[1]; f.pyrpupsteer-0.locked'
printf '%s %s %s\\n' "$cx" "$cy" "$lk" > live_steer.txt
milk-fpsexec-milk-calc -n pyrvibmon-0 runstop
milk-fpsexec-milk-calc -n pyrpupsteer-0 runstop
pyrwfs.tlm_stop 0
"""
    try:
        run_cli(live_cmd, env, work, timeout=60)
    finally:
        for exe, name in [
            ("milk-fpsexec-milk-calc", "pyrvibmon-0"),
            ("milk-fpsexec-milk-calc", "pyrpupsteer-0"),
            ("cacao-fpsexec-pyrwfstlm", "pyrwfstlm-0"),
        ]:
            subprocess.run([str(BIN / exe), "-n", name, "runstop"], env=env, cwd=str(work), capture_output=True)
    cx, cy, lk = [float(v) for v in (work / "live_steer.txt").read_text().strip().split()]
    assert lk == 1.0, f"Expected pupsteer locked=1, got {lk}"
    assert cx > 0.2 and cy < -0.1, f"Expected pupsteer integrating toward (+1.5,-0.8), got ({cx},{cy})"


def _sample_2d_bilinear(im: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    ny, nx = im.shape
    x0 = np.floor(x).astype(np.int32)
    y0 = np.floor(y).astype(np.int32)
    fx = x - x0
    fy = y - y0
    ok = (x0 >= 0) & (x0 < nx - 1) & (y0 >= 0) & (y0 < ny - 1)
    x0c = np.clip(x0, 0, nx - 2)
    y0c = np.clip(y0, 0, ny - 2)
    out = (1.0 - fy) * ((1.0 - fx) * im[y0c, x0c] + fx * im[y0c, x0c + 1]) + fy * (
        (1.0 - fx) * im[y0c + 1, x0c] + fx * im[y0c + 1, x0c + 1]
    )
    return np.where(ok, out, 0.0)


def test_temporal_variance_pupil_detection_cube_and_live_autocal(tmp_path: Path):
    """Verify temporal-variance 4-pupil detection via @calc pyrwfs_detect(cube), pyrwfs.calib, and live autocal."""
    env = make_env(tmp_path)
    work = tmp_path / "work"

    # 1. Build challenging non-modulated cube:
    #    global shift (+5.2, -4.3), rotation 10 deg, unequal sep (24.5, 21.0),
    #    non-rigid facet wedge shift (+1.7, -1.3) on pupil 3, strong TT illumination bias,
    #    readout noise, and 2 dark dropout frames.
    N, R, T = 128, 17.0, 48
    rng = np.random.default_rng(42)
    ang = math.radians(10.0)
    ca, sa = math.cos(ang), math.sin(ang)
    cx_t, cy_t = 64.0 + 5.2, 64.0 - 4.3
    sx_t, sy_t = 24.5, 21.0
    s0 = 28.0

    y, x = np.mgrid[-N // 2 : N // 2, -N // 2 : N // 2].astype(np.float64)
    r = np.hypot(x, y)
    P = ((r <= R) & (r >= 0.30 * R)).astype(np.float64)
    P[(np.abs((x + y) * 0.7071) < 0.8) | (np.abs((-x + y) * 0.7071) < 0.8)] = 0.0
    fy, fx = np.mgrid[-N // 2 : N // 2, -N // 2 : N // 2] / float(N)
    M = np.exp(-1j * 2.0 * np.pi * s0 * (np.abs(fx) + np.abs(fy)))

    base = [(-sx_t, -sy_t), (-sx_t, +sy_t), (+sx_t, -sy_t), (+sx_t, +sy_t)]
    true_c = np.array(
        [
            [
                cx_t + ux * ca - uy * sa + (1.7 if q == 3 else 0.0),
                cy_t + ux * sa + uy * ca + (-1.3 if q == 3 else 0.0),
            ]
            for q, (ux, uy) in enumerate(base)
        ],
        dtype=np.float64,
    )
    src_c = np.array([[64 - s0, 64 - s0], [64 - s0, 64 + s0], [64 + s0, 64 - s0], [64 + s0, 64 + s0]])
    yy_i, xx_i = np.mgrid[0:N, 0:N].astype(np.float64)

    def warp_frame(I0: np.ndarray, noise_std: float) -> np.ndarray:
        frame = np.full((N, N), 50.0, dtype=np.float64)
        for q in range(4):
            u = xx_i - true_c[q, 0]
            v = yy_i - true_c[q, 1]
            frame += np.where(
                (u * u + v * v) <= ((s0 - 2.0) ** 2),
                _sample_2d_bilinear(I0, src_c[q, 0] + u, src_c[q, 1] + v),
                0.0,
            )
        if noise_std > 0.0:
            frame += rng.normal(0.0, noise_std, (N, N))
        return frame.astype(np.float32)

    # Flat-WF reference at the same warped pupil locations
    F_flat = np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(P)))
    I0_flat = np.abs(np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(F_flat * M)))) ** 2
    I0_flat = I0_flat / I0_flat.sum() * 2e5
    ref_warp = warp_frame(I0_flat, noise_std=0.0)

    cube = np.zeros((T + 2, N, N), dtype=np.float32)
    for k in range(T):
        ph = make_kolm_phase(N, R, rms=1.1, seed=700 + k)
        ph += (0.5 + 0.4 * rng.normal()) * (x / R) + (-0.7 + 0.4 * rng.normal()) * (y / R)
        F_p = np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(P * np.exp(1j * ph))))
        I0 = np.abs(np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(F_p * M)))) ** 2
        I0 = I0 / I0.sum() * 2e5
        cube[k] = warp_frame(I0, noise_std=3.0)
    # Append 2 dark shutter-closed frames to verify dropout rejection
    cube[T] = np.full((N, N), 50.0, dtype=np.float32)
    cube[T + 1] = np.full((N, N), 50.0, dtype=np.float32)

    # Static cube (identical copies of ref_warp) to verify clean fallback
    static_cube = np.repeat(ref_warp[None, :, :], 8, axis=0)

    fits.writeto(work / "warped_ref.fits", ref_warp, overwrite=True)
    fits.writeto(work / "warped_cube.fits", cube, overwrite=True)
    fits.writeto(work / "static_cube.fits", static_cube, overwrite=True)

    # 2. Test @calc pyrwfs_detect(cube), pyrwfs.calib .cube=..., static fallback, and pyWFStlm 3D cube
    caldir = work / "cal_cube"
    cmd = f"""
iofits.loadfits warped_ref.fits @S:wref 1
iofits.loadfits warped_cube.fits @S:wcube 1
iofits.loadfits static_cube.fits @S:scube 1
@calc '
    S:g_cube = pyrwfs_detect(wcube)
    S:g_stat = pyrwfs_detect(scube)
'
pyrwfs.calib wref calp .cube=wcube .outdir={caldir}
cacao-fpsexec-pyrwfstlm -n tvartest exec .inwfs=wcube .inref=wref .outprefix=tvar .reg_stride=0 .pub_maps=ON
iofits.saveFITS g_cube g_cube.fits
iofits.saveFITS g_stat g_stat.fits
iofits.saveFITS tvar_pupgeom tvar_pupgeom.fits
"""
    run_cli(cmd, env, work)

    g_cube = fits.getdata(work / "g_cube.fits")
    g_stat = fits.getdata(work / "g_stat.fits")
    g_cal = fits.getdata(caldir / "nomgeom.fits")
    g_tlm = fits.getdata(work / "tvar_pupgeom.fits")

    for tag, geom in [("calc_cube", g_cube), ("calib_cube", g_cal), ("tlm_cube", g_tlm)]:
        assert geom.shape == (4, 4), f"{tag}: unexpected shape {geom.shape}"
        err_x = geom[:, 0] - true_c[:, 0]
        err_y = geom[:, 1] - true_c[:, 1]
        rel_x = err_x - np.mean(err_x)
        rel_y = err_y - np.mean(err_y)
        assert np.max(np.abs(err_x)) < 0.35, f"{tag}: abs X err too large: {err_x}"
        assert np.max(np.abs(err_y)) < 0.35, f"{tag}: abs Y err too large: {err_y}"
        assert np.max(np.abs(rel_x)) < 0.20, f"{tag}: rel X err too large: {rel_x}"
        assert np.max(np.abs(rel_y)) < 0.20, f"{tag}: rel Y err too large: {rel_y}"
        assert np.all(np.abs(geom[:, 2] - R) < 1.2), f"{tag}: Rout err: {geom[:, 2]} vs {R}"

    assert g_stat.shape == (4, 4) and np.all(np.isfinite(g_stat)) and np.all(g_stat[:, 2] > 5.0)

    # 3. Test LIVE 2D stream auto-calibration (no innomgeom supplied, autocal_n=24)
    live_cmd = """
iofits.loadfits warped_ref.fits @S:lref 1
iofits.loadfits warped_cube.fits @L:lcb 1
@calc 'S:lwfs = lref * 1.0'
cacao-fpsexec-pyrwfstlm -n liveautocal -loops fpsinit \
    .inwfs=lwfs .inref=lref .outprefix=lac .autocal_n=24 .reg_stride=0 .pub_maps=ON
cacao-fpsexec-pyrwfstlm -n liveautocal runstart
for k in $(seq 0 32); do
    @calc "S:lwfs = lcb[*, *, ${k}]"
    sleep 0.015
done
cacao-fpsexec-pyrwfstlm -n liveautocal runstop
iofits.saveFITS lac_pupgeom lac_pupgeom.fits
"""
    try:
        run_cli(live_cmd, env, work, timeout=60)
    finally:
        subprocess.run(
            [str(BIN / "cacao-fpsexec-pyrwfstlm"), "-n", "liveautocal", "runstop"],
            env=env,
            cwd=str(work),
            capture_output=True,
        )

    g_live = fits.getdata(work / "lac_pupgeom.fits")
    assert g_live.shape == (4, 4)
    err_x = g_live[:, 0] - true_c[:, 0]
    err_y = g_live[:, 1] - true_c[:, 1]
    rel_x = err_x - np.mean(err_x)
    rel_y = err_y - np.mean(err_y)
    assert np.max(np.abs(err_x)) < 0.45, f"live_autocal abs X err: {err_x}"
    assert np.max(np.abs(err_y)) < 0.45, f"live_autocal abs Y err: {err_y}"
    assert np.max(np.abs(rel_x)) < 0.25, f"live_autocal rel X err: {rel_x}"
    assert np.max(np.abs(rel_y)) < 0.25, f"live_autocal rel Y err: {rel_y}"


def test_scintillation_noise_debias(tmp_path: Path):
    """Verify lag-1 EMA-corrected scintillation cancels white noise while preserving flying shadows."""
    env = make_env(tmp_path)
    work = tmp_path / "work"
    N, R, s, T = 128, 18.0, 24.0, 120
    rng = np.random.default_rng(77)

    I_flat = sim_pywfs_frame(np.zeros((N, N)), N=N, R=R, s=s, flux=5e5, bg=100.0)
    fits.writeto(work / "ref.fits", I_flat, overwrite=True)

    # Estimate pupil pixel signal level so we can inject strong white photon/RON noise (~25% RMS)
    sig_lvl = float(np.percentile(I_flat - 100.0, 92.0))
    noise_sigma = 0.28 * sig_lvl

    # Cube 1: Pure white detector noise on flat WF (true scintillation = 0)
    cube_noise = np.empty((T, N, N), dtype=np.float32)
    for k in range(T):
        cube_noise[k] = (I_flat + rng.normal(0.0, noise_sigma, (N, N))).astype(np.float32)

    # Cube 2: Temporally correlated flying-shadow scintillation (drifting smooth amplitude screen)
    #         PLUS the same strong white detector noise
    scr = make_kolm_phase(N, R, rms=1.0, seed=901, fmin=2.0, fmax=10.0)
    scr = scr / (np.std(scr) + 1e-12)
    cube_scint = np.empty((T, N, N), dtype=np.float32)
    for k in range(T):
        # Slow drift (0.5 px/fr) -> high lag-1 temporal correlation
        sh_x = int(round(0.5 * k))
        sh_y = int(round(0.25 * k))
        sc = np.clip(1.0 + 0.16 * np.roll(np.roll(scr, sh_x, axis=1), sh_y, axis=0), 0.35, 1.9)
        frm = sim_pywfs_frame(np.zeros((N, N)), amp=sc, N=N, R=R, s=s, flux=5e5, bg=100.0)
        cube_scint[k] = (frm + rng.normal(0.0, noise_sigma, (N, N))).astype(np.float32)

    fits.writeto(work / "cube_noise.fits", cube_noise, overwrite=True)
    fits.writeto(work / "cube_scint.fits", cube_scint, overwrite=True)

    cmd = """
iofits.loadfits ref.fits @S:sref 1
iofits.loadfits cube_noise.fits @S:cnoise 1
iofits.loadfits cube_scint.fits @S:cscint 1
cacao-fpsexec-pyrwfstlm --no-procinfo -n sn_noise exec \
    .inwfs=cnoise .inref=sref .outprefix=snn .outcsv=noise.csv \
    .sub_bg=OFF .bg_const=100.0 .autocal_n=0 .reg_stride=0 .a_slow=0.04 .pub_maps=ON
cacao-fpsexec-pyrwfstlm --no-procinfo -n sn_scint exec \
    .inwfs=cscint .inref=sref .outprefix=sns .outcsv=scint.csv \
    .sub_bg=OFF .bg_const=100.0 .autocal_n=0 .reg_stride=0 .a_slow=0.04 .pub_maps=ON
iofits.saveFITS snn_scint_debias snn_deb.fits
iofits.saveFITS sns_scint_debias sns_deb.fits
"""
    run_cli(cmd, env, work)

    def _tail_means(csv_path: Path):
        import csv

        raw_v, deb_v = [], []
        with csv_path.open("r", encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                raw_v.append(float(row["scint_idx"]))
                deb_v.append(float(row["scint_debias"]))
        return float(np.mean(raw_v[50:])), float(np.mean(deb_v[50:]))

    raw_n, deb_n = _tail_means(work / "noise.csv")
    raw_s, deb_s = _tail_means(work / "scint.csv")

    # Pure white noise: raw scint_idx is strongly inflated (>0.015), debiased cancels near 0
    assert raw_n > 0.012, f"Expected noticeable raw noise variance, got {raw_n}"
    assert abs(deb_n) < 0.003, f"Expected near-zero debiased scint on white noise, got {deb_n} (raw={raw_n})"

    # Flying-shadow scintillation + white noise: debiased recovers true correlated variance (~4*0.16^2 ~ 0.05..0.10)
    # while staying clearly below the noise-inflated raw scint_idx!
    assert deb_s > 0.020, f"Expected strong positive debiased scint on flying shadows, got {deb_s}"
    assert raw_s > deb_s + 0.5 * raw_n, f"Expected raw ({raw_s}) > deb ({deb_s}) + noise floor ({raw_n})"
    assert fits.getdata(work / "sns_deb.fits").ndim == 2


def test_high_strehl_closed_loop_120x120_scintillation_pupil_detection(tmp_path: Path):
    """Regression test for Stage-2 closed-loop 120x120 PyWFS with high-SF residuals + scintillation + X/Y anisotropy."""
    env = make_env(tmp_path)
    work = tmp_path / "work"
    N, R, s, T = 120, 27.5, 30.0, 64
    rng = np.random.default_rng(2026)

    y, x = np.mgrid[-N // 2 : N // 2, -N // 2 : N // 2].astype(np.float64)
    r = np.hypot(x, y)
    P = ((r <= R) & (r >= 0.30 * R)).astype(np.float64)
    P[(np.abs((x + y) * 0.7071) < 0.8) | (np.abs((-x + y) * 0.7071) < 0.8)] = 0.0
    fy, fx = np.mgrid[-N // 2 : N // 2, -N // 2 : N // 2] / float(N)
    fr = np.hypot(fx * N, fy * N)
    # Shift by (-0.5, -0.5) so pupils sit symmetrically at (29.5, 29.5) .. (89.5, 89.5) like SCExAO 120x120
    M = np.exp(-1j * 2.0 * np.pi * s * (np.abs(fx) + np.abs(fy)) + 1j * 2.0 * np.pi * (-0.5 * fx - 0.5 * fy))

    def make_high_sf_phase(seed: int) -> np.ndarray:
        r2 = np.random.default_rng(seed)
        # Closed-loop residual: low orders suppressed (< 10 cyc/frame), high-SF speckles (10..26 cyc)
        # plus anisotropic X/Y weighting so Var(Wx) != Var(Wy)
        mask_f = (fr >= 10.0) & (fr <= 26.0)
        aniso = 1.0 + 0.8 * (fx**2 / (fx**2 + fy**2 + 1e-12))
        amp_f = np.zeros_like(fr)
        amp_f[mask_f] = aniso[mask_f] * (fr[mask_f] ** (-0.5))
        ph = np.real(np.fft.ifft2(np.fft.ifftshift(amp_f * np.exp(1j * r2.uniform(0, 2 * np.pi, (N, N))))))
        pup = P > 0.5
        ph = (ph - ph[pup].mean()) / (ph[pup].std() + 1e-12) * 0.35
        return ph

    def make_scint(seed: int) -> np.ndarray:
        r2 = np.random.default_rng(seed)
        mask_f = (fr >= 4.0) & (fr <= 14.0)
        amp_f = np.zeros_like(fr)
        amp_f[mask_f] = fr[mask_f] ** (-1.0)
        sc = np.real(np.fft.ifft2(np.fft.ifftshift(amp_f * np.exp(1j * r2.uniform(0, 2 * np.pi, (N, N))))))
        pup = P > 0.5
        sc = (sc - sc[pup].mean()) / (sc[pup].std() + 1e-12)
        return np.clip(1.0 + 0.22 * sc, 0.35, 1.9)

    F_flat = np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(P)))
    I_ref = np.abs(np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(F_flat * M)))) ** 2
    I_ref = (I_ref / I_ref.sum() * 1.1e5 + 100.0).astype(np.float32)

    cube = np.empty((T, N, N), dtype=np.float32)
    for k in range(T):
        ph = make_high_sf_phase(4000 + k)
        amp = make_scint(5000 + k)
        F_p = np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(P * amp * np.exp(1j * ph))))
        I0 = np.abs(np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(F_p * M)))) ** 2
        I0 = I0 / I0.sum() * 1.1e5 + 100.0 + rng.normal(0.0, 0.35, (N, N))
        cube[k] = I0.astype(np.float32)

    fits.writeto(work / "ref120.fits", I_ref, overwrite=True)
    fits.writeto(work / "cube120.fits", cube, overwrite=True)

    cmd = """
iofits.loadfits ref120.fits @S:r120 1
iofits.loadfits cube120.fits @S:c120 1
@calc 'S:g120 = pyrwfs_detect(c120)'
cacao-fpsexec-pyrwfstlm --no-procinfo -n cl120 exec .inwfs=c120 .inref=r120 .outprefix=p120 .reg_stride=0 .pub_maps=OFF
iofits.saveFITS g120 g120.fits
iofits.saveFITS p120_pupgeom p120_pupgeom.fits
"""
    run_cli(cmd, env, work)

    exp_x = np.array([29.5, 29.5, 89.5, 89.5])
    exp_y = np.array([29.5, 89.5, 29.5, 89.5])
    for tag, fname in [("calc_120", "g120.fits"), ("tlm_120", "p120_pupgeom.fits")]:
        geom = fits.getdata(work / fname)
        err_x = geom[:, 0] - exp_x
        err_y = geom[:, 1] - exp_y
        rel_x = err_x - np.mean(err_x)
        rel_y = err_y - np.mean(err_y)
        assert np.max(np.abs(err_x)) < 0.30, f"{tag}: abs X err {err_x}"
        assert np.max(np.abs(err_y)) < 0.30, f"{tag}: abs Y err {err_y}"
        assert np.max(np.abs(rel_x)) < 0.15, f"{tag}: rel X err {rel_x}"
        assert np.max(np.abs(rel_y)) < 0.15, f"{tag}: rel Y err {rel_y}"
        assert np.all(np.abs(geom[:, 2] - R) < 1.0), f"{tag}: Rout {geom[:, 2]} vs {R}"


