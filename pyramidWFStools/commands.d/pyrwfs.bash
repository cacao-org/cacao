# SPDX-FileCopyrightText: 2026 Olivier Guyon et al
#
# SPDX-License-Identifier: LGPL-3.0-or-later
#
# pyrwfs.bash - Pyramid WFS calibration, @calc zone builder, live telemetry & inspection
# Loaded automatically from $MILK_INSTALLDIR/share/milk/commands.d/pyrwfs.bash

declare -gi __PYRWFS_SEQ=0

__pyrwfs_install_root() {
    if [[ -n "${MILK_INSTALLDIR:-}" && -d "${MILK_INSTALLDIR}" ]]; then
        printf '%s' "${MILK_INSTALLDIR}"
        return 0
    fi
    local _src="${BASH_SOURCE[0]}"
    # .../share/milk/commands.d/pyrwfs.bash -> 4 levels up
    realpath -m "$(dirname "${_src}")/../../.."
}

__pyrwfs_short_path() {
    local -n __sp_out="$1"
    local __sp_raw="$2"
    local -n __sp_links="$3"
    if [[ -z "${__sp_raw}" || "${__sp_raw}" == "none" ]]; then
        __sp_out="${__sp_raw}"
        return 0
    fi
    if (( ${#__sp_raw} <= 54 )); then
        __sp_out="${__sp_raw}"
        return 0
    fi
    local __abs __lnk
    __abs="$(realpath -m "${__sp_raw}")" || return 1
    __lnk="/tmp/pyr_p_${BASHPID}_$((++__PYRWFS_SEQ))"
    ln -snf "${__abs}" "${__lnk}" || return 1
    __sp_links+=("${__lnk}")
    __sp_out="${__lnk}"
}

__pyrwfs_cleanup_links() {
    local -n __cl_links="$1"
    local __l
    for __l in "${__cl_links[@]}"; do
        rm -f "${__l}"
    done
}

# Build packed uint8 zone-label image + defocus-orthogonalized even-mode Strehl basis via @calc JIT.
# Zone encoding matches pyrwfs_tlm_core.h:
#   Low nibble: 1..4=pupils (BL,TL,BR,TR), 5=diamond, 6=bridge_H, 7=bridge_V, 8=outer, 9=bg
#   High nibble flags: 16=inner_pup, 32=spider, 64=inner_rim
pyrwfs.mkzones() {
    @args 'ref:str:reference 2D WFS stream (defines nx x ny and flat diffraction weight)' \
          'nomgeom:str:input [4x4] nominal pupil geometry stream [xc, yc, rout, rin]' \
          'outzones:str=pyr_zones:output packed uint8 zone-label SHM stream' \
          'outeven:str=pyr_even:output unit-norm defocus-orthogonalized even-mode basis SHM' \
          'sp_ang:float=45.0:spider arm angle [deg]' \
          'sp_hw:float=1.5:spider-flanking band half-width [pix] (0=disable)' \
          'halo_frac:float=0.45:outer diffraction halo margin [fraction of Rout]' -- "$@"

    @calc "
        _x = ix(${ref}, 0, 0); _y = iy(${ref}, 0, 0)
        _x0 = ${nomgeom}[0,0]; _y0 = ${nomgeom}[1,0]; _r0 = ${nomgeom}[2,0]; _ri0 = ${nomgeom}[3,0]
        _x1 = ${nomgeom}[0,1]; _y1 = ${nomgeom}[1,1]; _r1 = ${nomgeom}[2,1]; _ri1 = ${nomgeom}[3,1]
        _x2 = ${nomgeom}[0,2]; _y2 = ${nomgeom}[1,2]; _r2 = ${nomgeom}[2,2]; _ri2 = ${nomgeom}[3,2]
        _x3 = ${nomgeom}[0,3]; _y3 = ${nomgeom}[1,3]; _r3 = ${nomgeom}[2,3]; _ri3 = ${nomgeom}[3,3]

        _cx = 0.25 * (_x0 + _x1 + _x2 + _x3)
        _cy = 0.25 * (_y0 + _y1 + _y2 + _y3)
        _sx = 0.25 * ((_x2 + _x3) - (_x0 + _x1))
        _sy = 0.25 * ((_y1 + _y3) - (_y0 + _y2))
        _ravg = 0.25 * (_r0 + _r1 + _r2 + _r3)
        _erode = (_ravg > 8.0) ? 1.5 : 1.0

        _d0 = hypot(_x - _x0, _y - _y0)
        _d1 = hypot(_x - _x1, _y - _y1)
        _d2 = hypot(_x - _x2, _y - _y2)
        _d3 = hypot(_x - _x3, _y - _y3)

        _p0 = (_d0 <= _r0) && (_d0 >= _ri0)
        _p1 = (_d1 <= _r1) && (_d1 >= _ri1)
        _p2 = (_d2 <= _r2) && (_d2 >= _ri2)
        _p3 = (_d3 <= _r3) && (_d3 >= _ri3)
        _obs = (((_ri0 > 0.5) && (_d0 < _ri0)) || ((_ri1 > 0.5) && (_d1 < _ri1)) || ((_ri2 > 0.5) && (_d2 < _ri2)) || ((_ri3 > 0.5) && (_d3 < _ri3)))
        _in_any = _p0 || _p1 || _p2 || _p3

        _ax = abs(_x - _cx); _ay = abs(_y - _cy)
        _gx = max(_sx - 0.55 * _ravg, 2.0)
        _gy = max(_sy - 0.55 * _ravg, 2.0)
        _is_bet  = (!_in_any) && (!_obs) && (_ax < _sx) && (_ay < _sy)
        _is_diam = _is_bet && (_ax <= _gx) && (_ay <= _gy)
        _is_bH   = _is_bet && (!_is_diam) && (_ax > _ay)
        _is_bV   = _is_bet && (!_is_diam) && (_ax <= _ay)

        _dmin = min(min(_d0 - _r0, _d1 - _r1), min(_d2 - _r2, _d3 - _r3))
        _nx = s.${ref}.size0; _ny = s.${ref}.size1
        _bgm = max(0.14 * min(_nx, _ny), 3.0)
        _corner = ((_x < _bgm) || (_x >= _nx - _bgm)) && ((_y < _bgm) || (_y >= _ny - _bgm))
        _is_out = (!_in_any) && (_obs || ((!_is_bet) && (!_corner || _dmin <= 2.0) && (_dmin <= ${halo_frac} * _ravg + 3.0)))
        _is_bg  = (!_in_any) && (!_is_bet) && (!_is_out)

        _code = where(_p0, 1, where(_p1, 2, where(_p2, 3, where(_p3, 4, where(_is_diam, 5, where(_is_bH, 6, where(_is_bV, 7, where(_is_out, 8, where(_is_bg, 9, 0)))))))))

        _in0 = _p0 && (_d0 <= _r0 - _erode) && ((_ri0 <= 0.5) || (_d0 >= _ri0 + _erode))
        _in1 = _p1 && (_d1 <= _r1 - _erode) && ((_ri1 <= 0.5) || (_d1 >= _ri1 + _erode))
        _in2 = _p2 && (_d2 <= _r2 - _erode) && ((_ri2 <= 0.5) || (_d2 >= _ri2 + _erode))
        _in3 = _p3 && (_d3 <= _r3 - _erode) && ((_ri3 <= 0.5) || (_d3 >= _ri3 + _erode))
        _f_in = where(_in0 || _in1 || _in2 || _in3, 16, 0)

        _ang = ${sp_ang} * pi / 180.0
        _ca = cos(_ang); _sa = sin(_ang); _hw = ${sp_hw}
        _dxp = where(_p0, _x - _x0, where(_p1, _x - _x1, where(_p2, _x - _x2, _x - _x3)))
        _dyp = where(_p0, _y - _y0, where(_p1, _y - _y1, where(_p2, _y - _y2, _y - _y3)))
        _sp  = (_hw > 0.0) && _in_any && ((abs(_dxp * _ca + _dyp * _sa) <= _hw) || (abs(-_dxp * _sa + _dyp * _ca) <= _hw))
        _f_sp = where(_sp, 32, 0)

        _irm = ((_p0 && (_x > _x0) && (_y > _y0) && (_d0 > 0.72 * _r0)) || (_p1 && (_x > _x1) && (_y < _y1) && (_d1 > 0.72 * _r1)) || (_p2 && (_x < _x2) && (_y > _y2) && (_d2 > 0.72 * _r2)) || (_p3 && (_x < _x3) && (_y < _y3) && (_d3 > 0.72 * _r3)))
        _f_irm = where(_irm, 64, 0)

        S:${outzones} = u8(_code + _f_in + _f_sp + _f_irm)

        # Build defocus-orthogonalized unit-norm even-mode Strehl basis:
        # positive inside pupils, negative weighted by ref outside/between pupils,
        # orthogonalized against radial pupil shift (inner - outer rim)
        _act = (_code >= 1) && (_code <= 8)
        _bg0 = imean(${ref}, _is_bg)
        _rn  = max(${ref} - _bg0, 0.0)
        _rn  = _rn / max(itot(_rn, _act), 1e-12)
        _e0  = where(_in_any, 1.0, where(_act, -_rn / max(imean(_rn, _act && (!_in_any)), 1e-12), 0.0))
        _e0  = where(_act, _e0 - imean(_e0, _act), 0.0)

        # Radial outward-shift proxy (positive outer half of each pupil, negative inner half)
        _rp  = where(_p0, (_x0 - _x) + (_y0 - _y), where(_p1, (_x1 - _x) + (_y - _y1), where(_p2, (_x - _x2) + (_y0 - _y), where(_p3, (_x - _x3) + (_y - _y3), 0.0))))
        _rp  = where(_act, _rp - imean(_rp, _act), 0.0)
        _rpn = dot(_rp, _rp)
        _e1  = (_rpn > 1e-12) ? (_e0 - (dot(_e0, _rp) / _rpn) * _rp) : _e0
        _e1  = where(_act, _e1 - imean(_e1, _act), 0.0)
        _en  = sqrt(max(dot(_e1, _e1), 1e-24))
        S:${outeven} = f32(_e1 / _en)
    " || return 1

    # Clean up session-local temporaries
    local _v
    for _v in _x _y _d0 _d1 _d2 _d3 _p0 _p1 _p2 _p3 _obs _in_any _ax _ay \
              _is_bet _is_diam _is_bH _is_bV _dmin _corner _is_out _is_bg \
              _code _in0 _in1 _in2 _in3 _f_in _dxp _dyp _sp _f_sp _irm _f_irm \
              _act _rn _e0 _rp _e1; do
        mem.rm "${_v}" 0 >/dev/null 2>&1 || true
    done
}

# Calibrate pyramid geometry, zone masks, and flat-WF baseline flux fractions.
# Uses pyWFSgridmatch when zrespM is supplied, temporal-variance 4-pupil sum minimization
# when a 3D turbulent cube is supplied, else single-frame pyrwfs_detect on ref.
pyrwfs.calib() {
    @args 'ref:str:flat-WF reference 2D stream (e.g. aol0_wfsref or mean imWFS0)' \
          'prefix:str=aol0_pyr:output SHM prefix (<prefix>_nomgeom, _zones, _even)' \
          'zrespM:str=:optional zonal RM cube for DM-anchored pyWFSgridmatch' \
          'dmx:int=0:DM x size (required if zrespM is given)' \
          'dmy:int=0:DM y size (required if zrespM is given)' \
          'spotsize:float=2.0:spot radius for pyWFSgridmatch [pix]' \
          'thr:float=0.25:footprint threshold for pyrwfs_detect (when zrespM/cube empty)' \
          'sp_ang:float=45.0:spider arm angle [deg]' \
          'sp_hw:float=1.5:spider band half-width [pix]' \
          'outdir:str=:optional output directory to save FITS + baseline.kv' \
          'cube:str=:optional 3D turbulent WFS cube for temporal-variance pupil detection' -- "$@"

    local _g="${prefix}_nomgeom"
    local _z="${prefix}_zones"
    local _e="${prefix}_even"

    if [[ -n "${zrespM}" ]]; then
        if (( dmx <= 0 || dmy <= 0 )); then
            echo "pyrwfs.calib: dmx and dmy must be > 0 when zrespM is provided" >&2
            return 1
        fi
        cacaopyrwfs.pyWFSgridmatch ".in_name=${zrespM}" ".spotsize=${spotsize}" \
            ".dmxsize=${dmx}" ".dmysize=${dmy}" ".outgeom=${_g}" || return 1
    elif [[ -n "${cube}" ]]; then
        @calc "S:${_g} = pyrwfs_detect(${cube}, thr=${thr})" || return 1
    else
        @calc "S:${_g} = pyrwfs_detect(${ref}, thr=${thr})" || return 1
    fi

    pyrwfs.mkzones "${ref}" "${_g}" "${_z}" "${_e}" "${sp_ang}" "${sp_hw}" || return 1

    local _fpup0 _fdiam0 _fbH0 _fbV0 _fout0 _ftot0
    @calc -v _ftot0,_fpup0,_fdiam0,_fbH0,_fbV0,_fout0 "
        _m = pyrwfs_metrics(${ref}, ${_z}, ${ref})
        _m[0]; _m[5]; _m[6]; _m[7]; _m[8]; _m[9]
    " || return 1
    mem.rm _m 0 >/dev/null 2>&1 || true

    printf 'pyrwfs.calib [%s]: ftot=%.1f  f_pup0=%.4f  f_diam0=%.4f  f_bH0=%.4f  f_bV0=%.4f  f_out0=%.4f\n' \
        "${prefix}" "${_ftot0}" "${_fpup0}" "${_fdiam0}" "${_fbH0}" "${_fbV0}" "${_fout0}"

    if [[ -n "${outdir}" ]]; then
        mkdir -p "${outdir}" || return 1
        local -a _links=()
        local _so_g _so_z _so_e
        local _od
        __pyrwfs_short_path _od "${outdir}" _links || return 1
        iofits.saveFITS "${_g}" "${_od}/nomgeom.fits" >/dev/null
        iofits.saveFITS "${_z}" "${_od}/zones.fits" >/dev/null
        iofits.saveFITS "${_e}" "${_od}/even.fits" >/dev/null
        cat > "${outdir}/baseline.kv" <<EOF
prefix=${prefix}
ftot0=${_ftot0}
f_pup0=${_fpup0}
f_diam0=${_fdiam0}
f_bH0=${_fbH0}
f_bV0=${_fbV0}
f_out0=${_fout0}
EOF
        __pyrwfs_cleanup_links _links
    fi
}

# Start live C-core pyramid telemetry engine (cacao-fpsexec-pyrwfstlm).
pyrwfs.tlm_start() {
    @args 'loop:int=0:AO loop index' \
          'inwfs:str=:input WFS 2D stream (default aol<loop>_imWFS0)' \
          'inref:str=:optional flat-WF reference stream' \
          'prefix:str=:output stream prefix (default aol<loop>_pyr)' \
          'stride:int=1:process every Nth frame' \
          'reg_stride:int=20:pupil registration cadence [processed frames]' \
          'pub_maps:bool=1:publish 2D isum/sx/sy/scint/lvar maps' \
          'fpsname:str=:FPS unit name (default pyrwfstlm-<loop>)' -- "$@"

    local _wfs="${inwfs:-aol${loop}_imWFS0}"
    local _pfx="${prefix:-aol${loop}_pyr}"
    local _fps="${fpsname:-pyrwfstlm-${loop}}"

    local -a _args=(
        ".AOloopindex=${loop}"
        ".inwfs=${_wfs}"
        ".outprefix=${_pfx}"
        ".stride=${stride}"
        ".reg_stride=${reg_stride}"
        ".pub_maps=${pub_maps}"
    )
    [[ -n "${inref}" ]] && _args+=(".inref=${inref}")

    # Auto-attach calibrated masks/geometry if already present in SHM
    local _shmdir="${MILK_SHM_DIR:-/milk/shm}"
    [[ -d "${_shmdir}" ]] || _shmdir="/dev/shm"
    [[ -e "${_shmdir}/${_pfx}_zones.im.shm" ]]   && _args+=(".inzones=${_pfx}_zones")
    [[ -e "${_shmdir}/${_pfx}_nomgeom.im.shm" ]] && _args+=(".innomgeom=${_pfx}_nomgeom")
    [[ -e "${_shmdir}/${_pfx}_even.im.shm" ]]    && _args+=(".ineven=${_pfx}_even")

    cacao-fpsexec-pyrwfstlm -n "${_fps}" -loops fpsinit "${_args[@]}" || return 1
    cacao-fpsexec-pyrwfstlm -n "${_fps}" runstart || return 1
    declare -F __fpspipe_track_unit >/dev/null 2>&1 && __fpspipe_track_unit "${_fps}"
    printf 'pyrwfs.tlm_start: running %s on %s -> %s_{tlm,pupgeom,pupdof}\n' \
        "${_fps}" "${_wfs}" "${_pfx}"
}

pyrwfs.tlm_stop() {
    @args 'loop:int=0:AO loop index' \
          'fpsname:str=:FPS unit name (default pyrwfstlm-<loop>)' -- "$@"

    local _fps="${fpsname:-pyrwfstlm-${loop}}"
    cacao-fpsexec-pyrwfstlm -n "${_fps}" runstop || return 1
}

pyrwfs.tlm_status() {
    @args 'loop:int=0:AO loop index' \
          'fpsname:str=:FPS unit name (default pyrwfstlm-<loop>)' -- "$@"

    local _fps="${fpsname:-pyrwfstlm-${loop}}"
    local _n _ok _fp _df _an _nl _qx _qy _sc _cr _lw _dx _dy _ds _cc
    @calc -v _n,_ok,_fp,_df,_an,_nl,_qx,_qy,_sc,_cr,_lw,_dx,_dy,_ds,_cc "
        f.${_fps}.health.nframes; f.${_fps}.health.flux_ok;
        f.${_fps}.flux.pup; f.${_fps}.flux.df_pup;
        f.${_fps}.flux.aniso; f.${_fps}.flux.nl_ratio;
        f.${_fps}.jitter.qx_rms; f.${_fps}.jitter.qy_rms;
        f.${_fps}.wf.scint_idx; f.${_fps}.wf.curl_ratio; f.${_fps}.wf.lwe_metric;
        f.${_fps}.align.dx; f.${_fps}.align.dy; f.${_fps}.align.ds; f.${_fps}.align.corr
    " || return 1
    printf '[%s] n=%s ok=%s | f_pup=%.3f df=%+.3f aniso=%+.2f nl=%.3f | qrms=(%.3f,%.3f) | scint=%.4f curl=%.3f lwe=%.3f | align=(%+.2f,%+.2f) ds=%+.2f corr=%.2f\n' \
        "${_fps}" "${_n}" "${_ok}" "${_fp}" "${_df}" "${_an}" "${_nl}" \
        "${_qx}" "${_qy}" "${_sc}" "${_cr}" "${_lw}" "${_dx}" "${_dy}" "${_ds}" "${_cc}"
}

# Connect mlt.windlayers to live pyWFStlm outputs (zero duplicated wind profiling).
pyrwfs.windattach() {
    @args 'loop:int=0:AO loop index' \
          'prefix:str=:pyWFStlm stream prefix (default aol<loop>_pyr)' \
          'wfs:str=:raw WFS stream (used when mode=0; default aol<loop>_imWFS0)' \
          'pha:str=:optional wavefront OPD stream (e.g. aol<loop>_dmdisp)' \
          'frate:float=1000.0:WFS frame rate [Hz]' \
          'diam:float=8.0:telescope pupil diameter [m]' \
          'mode:int=0:0=pass live pupx/pupy/flux_in to raw PyWFS, 1=feed registered pyr_isum directly' \
          'wlname:str=:windlayers FPS name (default wl-<loop>)' -- "$@"

    local _pfx="${prefix:-aol${loop}_pyr}"
    local _wfs="${wfs:-aol${loop}_imWFS0}"
    local _wl="${wlname:-wl-${loop}}"
    local _pg="${_pfx}_pupgeom"

    local _x0 _x1 _x2 _x3 _y0 _y1 _y2 _y3 _diampix
    @calc -v _x0,_x1,_x2,_x3,_y0,_y1,_y2,_y3,_diampix "
        ${_pg}[0,0]; ${_pg}[0,1]; ${_pg}[0,2]; ${_pg}[0,3];
        ${_pg}[1,0]; ${_pg}[1,1]; ${_pg}[1,2]; ${_pg}[1,3];
        0.5 * (${_pg}[2,0] + ${_pg}[2,1] + ${_pg}[2,2] + ${_pg}[2,3])
    " || return 1

    local _pupx _pupy
    printf -v _pupx '%.2f,%.2f,%.2f,%.2f' "${_x0}" "${_x1}" "${_x2}" "${_x3}"
    printf -v _pupy '%.2f,%.2f,%.2f,%.2f' "${_y0}" "${_y1}" "${_y2}" "${_y3}"

    local -a _args=(
        ".outname=${_wl}"
        ".frate=${frate}"
        ".diam=${diam}"
        ".diampix=${_diampix}"
    )
    if (( mode == 0 )); then
        _args+=(".wfs=${_wfs}" ".wfsmode=0" ".pupx=${_pupx}" ".pupy=${_pupy}")
    else
        _args+=(".wfs=${_pfx}_isum" ".wfsmode=1")
    fi
    [[ -n "${pha}" ]] && _args+=(".pha=${pha}")

    milk-fpsexec-mlt-windlayers -n "${_wl}" -loops fpsinit "${_args[@]}" || return 1
    milk-fpsexec-mlt-windlayers -n "${_wl}" runstart || return 1
    declare -F __fpspipe_track_unit >/dev/null 2>&1 && __fpspipe_track_unit "${_wl}"
    printf 'pyrwfs.windattach: started %s (mode=%d diampix=%.1f pupx=%s pupy=%s)\n' \
        "${_wl}" "${mode}" "${_diampix}" "${_pupx}" "${_pupy}"
}

# Start Tier 0 milk.calc slow pupil-steering offload unit on <prefix>_pupdof / _pupgeom.
pyrwfs.pupsteer_start() {
    @args 'loop:int=0:AO loop index' \
          'prefix:str=:pyWFStlm stream prefix (default aol<loop>_pyr)' \
          'gain:float=0.15:integrator gain' \
          'leak:float=0.999:integrator leak' \
          'deadband:float=0.03:deadband [pix]' \
          'maxcmd:float=5.0:command clamp [pix]' \
          'corr_min:float=0.4:min edge xcorr peak to integrate' \
          'fpsname:str=:FPS unit name (default pyrpupsteer-<loop>)' -- "$@"

    local _pfx="${prefix:-aol${loop}_pyr}"
    local _fps="${fpsname:-pyrpupsteer-${loop}}"
    local _root _ufile _tmpcalc
    _root="$(__pyrwfs_install_root)"
    _ufile="${_root}/share/cacao/pyrwfs/pyrwfs_pupsteer.calc"
    [[ -r "${_ufile}" ]] || { echo "pyrwfs.pupsteer_start: missing ${_ufile}" >&2; return 1; }

    _tmpcalc="/tmp/pyrpupsteer_${loop}.calc"
    sed -e "s/\bpupdof\b/${_pfx}_pupdof/g" \
        -e "s/\bpupgeom\b/${_pfx}_pupgeom/g" \
        -e "s/\bpupsteer_cmd\b/${_pfx}_pupsteer_cmd/g" \
        "${_ufile}" > "${_tmpcalc}" || return 1

    milk-fpsexec-milk-calc -n "${_fps}" -t "${_pfx}_pupdof" \
        -p "gain:float=${gain} leak:float=${leak} deadband:float=${deadband} maxcmd:float=${maxcmd} corr_min:float=${corr_min} max_resid:float=0.5 z_gain:float=0.05 cmd_x:float=0 cmd_y:float=0 cmd_z:float=0 locked:int=0" \
        -i "${_pfx}_pupsteer_cmd = zeros(3)" \
        -f "${_tmpcalc}" fpsinit || return 1
    milk-fpsexec-milk-calc -n "${_fps}" runstart || return 1
    declare -F __fpspipe_track_unit >/dev/null 2>&1 && __fpspipe_track_unit "${_fps}"
}

# Start Tier 0 milk.calc quad-cell vibration & modulator-beat FFT monitor on <prefix>_tlm.
pyrwfs.vibmon_start() {
    @args 'loop:int=0:AO loop index' \
          'prefix:str=:pyWFStlm stream prefix (default aol<loop>_pyr)' \
          'fs_hz:float=100.0:effective pyWFStlm update rate [Hz]' \
          'upd_stride:int=64:FFT update cadence [ticks]' \
          'fpsname:str=:FPS unit name (default pyrvibmon-<loop>)' -- "$@"

    local _pfx="${prefix:-aol${loop}_pyr}"
    local _fps="${fpsname:-pyrvibmon-${loop}}"
    local _root _ufile _tmpcalc
    _root="$(__pyrwfs_install_root)"
    _ufile="${_root}/share/cacao/pyrwfs/pyrwfs_vibmon.calc"
    [[ -r "${_ufile}" ]] || { echo "pyrwfs.vibmon_start: missing ${_ufile}" >&2; return 1; }

    _tmpcalc="/tmp/pyrvibmon_${loop}.calc"
    sed -e "s/\bpyr_tlm\b/${_pfx}_tlm/g" \
        -e "s/\bpyr_vibpsd\b/${_pfx}_vibpsd/g" \
        "${_ufile}" > "${_tmpcalc}" || return 1

    milk-fpsexec-milk-calc -n "${_fps}" -t "${_pfx}_tlm" \
        -p "upd_stride:int=${upd_stride} fs_hz:float=${fs_hz} peak_hz:float=0 peak_ratio:float=0 df_mean:float=0 df_std:float=0" \
        -i "${_pfx}_vibpsd = zeros(129, 3)" \
        -f "${_tmpcalc}" fpsinit || return 1
    milk-fpsexec-milk-calc -n "${_fps}" runstart || return 1
    declare -F __fpspipe_track_unit >/dev/null 2>&1 && __fpspipe_track_unit "${_fps}"
}

# Offline 3D FITS/.fits.fz cube diagnostic using C-core pyWFStlm + @calc summary report.
pyrwfs.inspect() {
    @args 'cube:str:input 3D FITS cube path or existing SHM stream name' \
          'ref:str=:optional flat-WF reference FITS path or SHM stream' \
          'outdir:str=:optional directory to save tlm.fits and REPORT.md' \
          'reg_stride:int=20:pupil registration cadence [frames]' \
          'sub_bg:bool=1:subtract corner background before flux budget' -- "$@"

    local -a _links=()
    local _seq=$((++__PYRWFS_SEQ))
    local _cname="pyrinsp_c_${_seq}"
    local _rname=""
    local _pfx="pyrinsp_${_seq}"
    local _loaded_c=0
    local _loaded_r=0

    if [[ -f "${cube}" ]]; then
        local _sc
        __pyrwfs_short_path _sc "${cube}" _links || return 1
        iofits.loadfits "${_sc}" "@S:${_cname}" 1 >/dev/null || { __pyrwfs_cleanup_links _links; return 1; }
        _loaded_c=1
    else
        _cname="${cube}"
    fi

    if [[ -n "${ref}" ]]; then
        if [[ -f "${ref}" ]]; then
            local _sr
            _rname="pyrinsp_r_${_seq}"
            __pyrwfs_short_path _sr "${ref}" _links || { __pyrwfs_cleanup_links _links; return 1; }
            iofits.loadfits "${_sr}" "@S:${_rname}" 1 >/dev/null || { __pyrwfs_cleanup_links _links; return 1; }
            _loaded_r=1
        else
            _rname="${ref}"
        fi
    fi

    local -a _cmd=(
        cacao-fpsexec-pyrwfstlm -n "pyrinsp_${_seq}" exec
        ".inwfs=${_cname}" ".outprefix=${_pfx}"
        ".reg_stride=${reg_stride}" ".sub_bg=${sub_bg}" ".pub_maps=1"
    )
    [[ -n "${_rname}" ]] && _cmd+=(".inref=${_rname}")
    "${_cmd[@]}" || { __pyrwfs_cleanup_links _links; return 1; }

    local _t="${_pfx}_tlm"
    local _n _ok _fp_m _fp_s _df_m _df_p10 _df_p90 _an_m _an_s _nl_m \
          _qxr _qyr _sc_m _cr_m _lw_m _dx_m _dy_m _ds_m _da_m _rt_m _cc_min
    @calc -v _n,_ok,_fp_m,_fp_s,_df_m,_df_p10,_df_p90,_an_m,_an_s,_nl_m,_qxr,_qyr,_sc_m,_cr_m,_lw_m,_dx_m,_dy_m,_ds_m,_da_m,_rt_m,_cc_min "
        _c_ok = ${_t}[28, *]; _c_fp = ${_t}[6, *];  _c_df = ${_t}[11, *]
        _c_an = ${_t}[12, *]; _c_nl = ${_t}[13, *]; _c_qx = ${_t}[16, *]
        _c_qy = ${_t}[17, *]; _c_sc = ${_t}[19, *]; _c_cr = ${_t}[20, *]
        _c_lw = ${_t}[21, *]; _c_dx = ${_t}[22, *]; _c_dy = ${_t}[23, *]
        _c_ds = ${_t}[24, *]; _c_da = ${_t}[25, *]; _c_rt = ${_t}[26, *]
        _c_cc = ${_t}[27, *]
        s.${_t}.size1; imean(_c_ok);
        imean(_c_fp); istd(_c_fp);
        imean(_c_df); perc(_c_df, 0.10); perc(_c_df, 0.90);
        imean(_c_an); istd(_c_an); imean(_c_nl);
        imean(_c_qx); imean(_c_qy);
        imean(_c_sc); imean(_c_cr); imean(_c_lw);
        imean(_c_dx); imean(_c_dy); imean(_c_ds); imean(_c_da); imean(_c_rt); imin(_c_cc)
    " || { __pyrwfs_cleanup_links _links; return 1; }

    local _rep
    _rep=$(cat <<EOF
# Pyramid WFS Telemetry Inspection Report

- Input: \`${cube}\` (${_n} frames, \`flux_ok\` duty = $(printf '%.1f%%' "$(awk "BEGIN{print 100*${_ok}}")"))
- Flux budget: \`f_pup\` = ${_fp_m} +/- ${_fp_s}, \`df_pup\` = ${_df_m} (p10=${_df_p10}, p90=${_df_p90})
- Anisotropy & NL: \`aniso\` = ${_an_m} +/- ${_an_s}, \`nl_ratio\` = ${_nl_m}
- Quad-cell jitter RMS: \`qx_rms\` = ${_qxr}, \`qy_rms\` = ${_qyr}
- Wavefront / scintillation: \`scint_idx\` = ${_sc_m}, \`curl_ratio\` = ${_cr_m}, \`lwe_metric\` = ${_lw_m}
- Pupil alignment (mean): \`dx\` = ${_dx_m} px, \`dy\` = ${_dy_m} px, \`ds\` = ${_ds_m} px, \`dast\` = ${_da_m} px, \`rot\` = ${_rt_m} mrad, \`min_corr\` = ${_cc_min}
EOF
)
    printf '%s\n' "${_rep}"

    if [[ -n "${outdir}" ]]; then
        mkdir -p "${outdir}" || return 1
        local _od
        __pyrwfs_short_path _od "${outdir}" _links || return 1
        iofits.saveFITS "${_t}" "${_od}/tlm.fits" >/dev/null
        iofits.saveFITS "${_pfx}_isum" "${_od}/isum_avg.fits" >/dev/null
        iofits.saveFITS "${_pfx}_scint" "${_od}/scint_map.fits" >/dev/null
        iofits.saveFITS "${_pfx}_lvar" "${_od}/lvar_map.fits" >/dev/null
        printf '%s\n' "${_rep}" > "${outdir}/REPORT.md"
    fi

    # Cleanup temporary streams and FPS
    local _v
    for _v in _c_ok _c_fp _c_df _c_an _c_nl _c_qx _c_qy _c_sc _c_cr _c_lw \
              _c_dx _c_dy _c_ds _c_da _c_rt _c_cc; do
        mem.rm "${_v}" 0 >/dev/null 2>&1 || true
    done
    milk-fps-rm -f -q --ignore-missing "pyrinsp_${_seq}" >/dev/null 2>&1 || true
    milk-stream-rm -q --ignore-missing \
        "${_pfx}_tlm" "${_pfx}_pupgeom" "${_pfx}_pupdof" \
        "${_pfx}_isum" "${_pfx}_sx" "${_pfx}_sy" "${_pfx}_scint" "${_pfx}_lvar" \
        >/dev/null 2>&1 || true
    (( _loaded_c )) && milk-stream-rm -q --ignore-missing "${_cname}" >/dev/null 2>&1 || true
    (( _loaded_r )) && milk-stream-rm -q --ignore-missing "${_rname}" >/dev/null 2>&1 || true
    __pyrwfs_cleanup_links _links
}
