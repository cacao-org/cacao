set(LIBNAME "cacaoAOloopControl")
set(SRCNAME "AOloopControl")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")





project(lib_${LIBNAME}_project)

add_library(${LIBNAME} SHARED ${SRCNAME}.c
	${SRCNAME}.h
	AOloopControl_AOcompute.c
	AOloopControl_AOcompute.h
	AOloopControl_aorun.c
	AOloopControl_aorun.h
	AOloopControl_arpf_onoff.c
	AOloopControl_autotune.c
	AOloopControl_autotune.h
	AOloopControl_AutoTuneGains.c
	AOloopControl_CompModes_loop.c
	AOloopControl_ProcessModeCoefficients.c
	AOloopControl_ProcessModeCoefficients.h
	AOloopControl_dm.c
	AOloopControl_dmwrite.c
	AOloopControl_fpspeckle_mod.c
	AOloopControl_initmem.c
	AOloopControl_loadconfigure.c
	AOloopControl_loop_ctr.c
	AOloopControl_loop_onoff.c
	AOloopControl_loop_param.c
	AOloopControl_process_files.c
	AOloopControl_read_param.c
	AOloopControl_RTstreamLOG.c
	AOloopControl_sig2Modecoeff.c
	AOloopControl_time_param.c
	AOloopControl_wfs.c
	modalfilter.c
	modalfilter.h
	modalCTRL_stats.c
	modalCTRL_stats.h
	modalstatsTUI.c
	modalstatsTUI.h
)


#if(USE_CUDA)
#find_package(CUDA 9.0 REQUIRED)
#message(STATUS "Found CUDA ${CUDA_VERSION_STRING} at ${CUDA_TOOLKIT_ROOT_DIR}")
#message("---- CUDA_INCLUDE_DIRS = ${CUDA_INCLUDE_DIRS}")
#message("---- CUDA_LIBRARIES    = ${CUDA_LIBRARIES}" )
#target_compile_options(${LIBNAME} PUBLIC -DHAVE_CUDA)
#target_link_libraries(${LIBNAME} PUBLIC ${CUDA_LIBRARIES})

#if(USE_MAGMA)
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(MAGMA REQUIRED magma)
#message("---- MAGMA_LIBRARY_DIRS =  ${MAGMA_LIBRARY_DIRS}")
#message("---- MAGMA_LIBRARIES    =  ${MAGMA_LIBRARIES}" )
#message("---- MAGMA_CFLAGS_OTHER =  ${MAGMA_CFLAGS_OTHER}" )
#target_include_directories(${LIBNAME} PUBLIC ${MAGMA_INCLUDE_DIRS})
#target_link_directories(${LIBNAME} PUBLIC ${MAGMA_LIBRARY_DIRS})
#target_link_libraries(${LIBNAME} PUBLIC ${MAGMA_LIBRARIES})
#target_compile_options(${LIBNAME} PUBLIC -DHAVE_MAGMA ${MAGMA_CFLAGS_OTHER})
#endif(USE_MAGMA)

#endif(USE_CUDA)

target_include_directories(${LIBNAME} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/..
	${CMAKE_CURRENT_SOURCE_DIR}
	${CUDA_INCLUDE_DIRS}
	${MAGMA_INCLUDE_DIRS}
)

target_link_libraries(${LIBNAME} PUBLIC CLIcore milklinoptimtools)

install(TARGETS ${LIBNAME} DESTINATION lib)

install(PROGRAMS scripts/cacao-setup
	scripts/cacao
	scripts/cacao-check-cacaovars
	scripts/cacao-fpsctrl
	scripts/cacao-fpsctrl-log
	scripts/cacao-fpsctrl-logprocess
	scripts/cacao-fpsctrl-TUI
	scripts/cacao-RMdecode
	scripts/cacao-mkDMWFSmasks
	scripts/cacao-mkDMslaveActprox
	scripts/cacao-mkLODMmodes
	scripts/cacao-NULL
	scripts/cacao-calib-archive
	scripts/cacao-calib-apply
	scripts/cacao-calib-archivecurrent
	scripts/cacao-exec
	scripts/cacao.tmux
	scripts/cacao-fpsarchive
	scripts/cacao-fpslistadd
	scripts/cacao-fpslistadd-MODALCTRL_STATS
	scripts/cacao-fpslistadd-MODALFILTERING
	scripts/cacao-fpslistadd-MVMGPU_ZPO
	scripts/cacao-modalstatsTUI
	scripts/cacao-setDMnolink
	scripts/cacao_simLHS
	scripts/cacao-takedark
	scripts/cacao-task-manager
	scripts/cacao-loop-deploy
	scripts/cacaotask-check
	scripts/cacaotask-INITSETUP
	scripts/cacaotask-GETSIMCONFFILES
	scripts/cacaotask-TESTCONFIG
	scripts/cacaotask-CACAOSETUP
	scripts/cacaotask-STARTDMCOMB
	scripts/cacaotask-STARTSTREAMDELAY
	scripts/aorun/cacao-aorun-000-dm
	scripts/aorun/cacao-aorun-001-dmsim
	scripts/aorun/cacao-aorun-002-simwfs
	scripts/aorun/cacao-aorun-003-wfsmapping
	scripts/aorun/cacao-aorun-005-takedark
	scripts/aorun/cacao-aorun-020-mlat
	scripts/aorun/cacao-aorun-025-acqWFS
	scripts/aorun/cacao-aorun-026-takeref
	scripts/aorun/cacao-aorun-030-acqlinResp
	scripts/aorun/cacao-aorun-031-RMHdecode
	scripts/aorun/cacao-aorun-032-RMmkmask
	scripts/aorun/cacao-aorun-033-RM-mksynthetic
	scripts/aorun/cacao-aorun-035-acqloRM
	scripts/aorun/cacao-aorun-039-compstrCM
	scripts/aorun/cacao-aorun-040-compfCM
	scripts/aorun/cacao-aorun-041-loadCM
	scripts/aorun/cacao-aorun-041-saveCM
	scripts/aorun/cacao-aorun-050-wfs2cmval
	scripts/aorun/cacao-aorun-060-mfilt
	scripts/aorun/cacao-aorun-061-setmgains
	scripts/aorun/cacao-aorun-070-cmval2dm
	scripts/aorun/cacao-aorun-071-zpo
	scripts/aorun/cacao-aorun-080-testOL
	scripts/aorun/cacao-aorun-100-DMturb
	scripts/aorun/cacao-aorun-120-mstat
	scripts/aorun/cacao-aorun-130-mkPF
	scripts/aorun/cacao-aorun-140-applyPF
	DESTINATION bin)

install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})
