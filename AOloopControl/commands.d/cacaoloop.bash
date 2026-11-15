# SPDX-FileCopyrightText: 2026 Olivier Guyon et al
#
# SPDX-License-Identifier: LGPL-3.0-or-later
#
# cacaoloop.bash - High-speed CACAO AO loop workspace, DM channel linking, and teardown
# Loaded automatically from $MILK_INSTALLDIR/share/milk/commands.d/cacaoloop.bash

# Initialize CACAO AO loop workspace, SHM loop record, and cacaovars file
cacaoloop.workdir() {
    @args 'name:str:AO loop name' \
          'num:int:AO loop number' \
          'rootdir:str:AO loop root directory' \
          'dm:str=00:active DM index (2 digits)' \
          'dmx:int=50:DM actuator x-dimension' \
          'dmy:int=50:DM actuator y-dimension' \
          'wfs:str=wfsim:input WFS stream name' \
          'restart:bool=0:restart active units during deployment' -- "$@"

    fpspipe.workdir "${name}" "${rootdir}" .prefix="aol${num}_" .app="cacao" .restart="${restart}" || return 1

    export CACAO_LOOPNAME="${name}"
    export CACAO_LOOPNUMBER="${num}"
    export CACAO_DMINDEX="${dm}"
    export CACAO_DMxsize="${dmx}"
    export CACAO_DMysize="${dmy}"
    export CACAO_DMSPATIAL=1
    export CACAO_LOOPROOTDIR="${FPSPIPE_ROOTDIR}"
    export CACAO_LOOPRUNDIR="${name}-rundir"
    export CACAO_WFSSTREAM="${wfs}"

    # Write loop metadata in both rootdir and rundir for cacao-aorun / cacao-setDMnolink compatibility
    local dir
    for dir in "${FPSPIPE_ROOTDIR}" "${FPSPIPE_RUNDIR}"; do
        printf '%s\n' "${name}" > "${dir}/LOOPNAME"
        printf '%s\n' "${num}"  > "${dir}/LOOPNUMBER"
        printf '%s\n' "${dm}"   > "${dir}/DMINDEX"
    done

    mkdir -p "${FPSPIPE_RUNDIR}/measlinrespm" "${FPSPIPE_RUNDIR}/mkmodestmp" || return 1

    # Write cacaovars.<name>.bash for cacao-check-cacaovars and legacy utility scripts
    local cvfile="${FPSPIPE_ROOTDIR}/cacaovars.${name}.bash"
    cat > "${cvfile}" <<EOF
#!/usr/bin/env bash
export CACAO_LOOPNAME="${name}"
export CACAO_LOOPNUMBER="${num}"
export CACAO_DMINDEX="${dm}"
export CACAO_DMxsize="${dmx}"
export CACAO_DMysize="${dmy}"
export CACAO_DMSPATIAL="1"
export CACAO_LOOPROOTDIR="${FPSPIPE_ROOTDIR}"
export CACAO_LOOPRUNDIR="${name}-rundir"
export CACAO_WFSSTREAM="${wfs}"
EOF

    # Populate $MILK_SHM_DIR/cacaoloop-<num> discovery record for cacao-loops
    local shmdir="${MILK_SHM_DIR:-/milk/shm}"
    [[ -d "${shmdir}" ]] || shmdir="/dev/shm"
    local looprec="${shmdir}/cacaoloop-${num}"
    mkdir -p "${looprec}" || return 1
    printf '%s\n' "${name}"             > "${looprec}/loopname"
    printf '%s\n' "${FPSPIPE_ROOTDIR}"  > "${looprec}/looprootdir"
    printf '%s\n' "${dm}"               > "${looprec}/dmindex"
    printf '%s\n' "${dmx}"              > "${looprec}/dmxsize"
    printf '%s\n' "${dmy}"              > "${looprec}/dmysize"
    printf '%s\n' "${0}"                > "${looprec}/access-script"
}

# Atomically link all 14 standard CACAO DM channels in a single C transaction
cacaoloop.link_dm() {
    @args 'dm:str:DM index (e.g. 00, 01, 10, 11)' \
          'dmx:int:expected DM x-dimension' \
          'dmy:int:expected DM y-dimension' -- "$@"

    local num="${CACAO_LOOPNUMBER:?cacaoloop.link_dm: CACAO_LOOPNUMBER is not set}"
    local pfx="aol${num}_"

    local -a aliases=(
        dmO dmRM dmC dmoffloadC
        dmZP0 dmZP1 dmZP2 dmZP3 dmZP4 dmZP5 dmZP6 dmZP7
        dmturb dmdisp
    )

    milk-stream-link -q --no-conf --expect-size "${dmx}x${dmy}" -p "${pfx}" \
        "dm${dm}disp00=dmO" \
        "dm${dm}disp02=dmRM" \
        "dm${dm}disp03=dmC" \
        "dm${dm}disp07=dmoffloadC" \
        "dm${dm}disp04=dmZP0" \
        "dm${dm}disp05=dmZP1" \
        "dm${dm}disp06=dmZP2" \
        "dm${dm}disp07=dmZP3" \
        "dm${dm}disp08=dmZP4" \
        "dm${dm}disp09=dmZP5" \
        "dm${dm}disp10=dmZP6" \
        "dm${dm}disp11=dmZP7" \
        "dm${dm}disp10=dmturb" \
        "dm${dm}disp=dmdisp" || return 1

    export CACAO_DMINDEX="${dm}"
    local dir
    for dir in "${FPSPIPE_ROOTDIR:-}" "${FPSPIPE_RUNDIR:-}"; do
        if [[ -n "${dir}" && -d "${dir}" ]]; then
            printf '%s\n' "${dm}" > "${dir}/DMINDEX"
        fi
    done

    local shmdir="${MILK_SHM_DIR:-/milk/shm}"
    [[ -d "${shmdir}" ]] || shmdir="/dev/shm"
    if [[ -d "${shmdir}/cacaoloop-${num}" ]]; then
        printf '%s\n' "${dm}"  > "${shmdir}/cacaoloop-${num}/dmindex"
        printf '%s\n' "${dmx}" > "${shmdir}/cacaoloop-${num}/dmxsize"
        printf '%s\n' "${dmy}" > "${shmdir}/cacaoloop-${num}/dmysize"
    fi

    local sfx
    for sfx in "${aliases[@]}"; do
        __fpspipe_track_stream "${pfx}${sfx}"
    done
    return 0
}

# Stop and remove CACAO AO loop units, streams, optional sim DM channels, and loop record
cacaoloop.teardown() {
    @args 'purge_dm:str=:optional simulation DM index whose dm<XX>disp* streams should be removed' \
          'clean_rundir:bool=1:remove fps.*.{confdir,datadir} in rundir' -- "$@"

    local num="${CACAO_LOOPNUMBER:-}"
    fpspipe.teardown .clean_rundir="${clean_rundir}" || return 1

    if [[ -n "${num}" ]]; then
        milk-stream-rm -r -f -q --ignore-missing "^aol${num}_" || true
        local shmdir="${MILK_SHM_DIR:-/milk/shm}"
        [[ -d "${shmdir}" ]] || shmdir="/dev/shm"
        rm -rf "${shmdir}/cacaoloop-${num}"
    fi

    if [[ -n "${purge_dm}" ]]; then
        milk-stream-rm -r -f -q --ignore-missing "^dm${purge_dm}disp" || true
    fi
}
