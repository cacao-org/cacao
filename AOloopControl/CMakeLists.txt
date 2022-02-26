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
	scripts/cacao-fpslistadd-APPLYPF
	scripts/cacao-setDMnolink
	scripts/cacao_simLHS
	scripts/cacao-task-manager
	scripts/cacaotask-check
	scripts/cacaotask-INITSETUP
	scripts/cacaotask-GETSIMCONFFILES
	scripts/cacaotask-TESTCONFIG
	scripts/cacaotask-CACAOSETUP
	scripts/cacaotask-STARTDMCOMB
	scripts/cacaotask-STARTSTREAMDELAY
	DESTINATION bin)

install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})
