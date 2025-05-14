set(LIBNAME "cacaoAOloopControl")
set(SRCNAME "AOloopControl")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")





project(lib_${LIBNAME}_project)

add_library(${LIBNAME} SHARED ${SRCNAME}.c
	${SRCNAME}.h
	modalfilter.c
	modalfilter.h
	modalfilter_test.c
	modalfilter_test.h
	modalCTRL_stats.c
	modalCTRL_stats.h
	modalstatsTUI.c
	modalstatsTUI.h
	zonalfilter.c
	zonalfilter.h
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
	scripts/cacaofuncs-log
	scripts/cacao-check-cacaovars
	scripts/cacao-commands
	scripts/cacao-fpsctrl
	scripts/cacao-fpsctrl-log
	scripts/cacao-fpsctrl-logprocess
	scripts/cacao-fpsctrl-TUI
	scripts/cacao-loops
	scripts/cacao-NULL
	scripts/cacao-calib-archive
	scripts/cacao-calib-apply
	scripts/cacao-calib-archivecurrent
	scripts/cacao-calib-streamload
	scripts/cacao-exec
	scripts/cacao.tmux
	scripts/cacao-fpsconfstart
	scripts/cacao-fpsconfstop
	scripts/cacao-fpslistadd
	scripts/cacao-fpslistadd-ACQWFS
	scripts/cacao-fpslistadd-MAPWFS
	scripts/cacao-fpslistadd-MFILTTEST
	scripts/cacao-fpslistadd-MVMGPU_CMODEVAL2DM
	scripts/cacao-fpslistadd-MVMGPU_CMODEVALOFFLOAD2DM
	scripts/cacao-fpslistadd-MVMGPU_DM2MVAL
	scripts/cacao-fpslistadd-MVMGPU_WFS2CMODEVAL
	scripts/cacao-fpslistadd-MVMGPU_ZPO
	scripts/cacao-fpslistadd-MODALCTRL_STATS
	scripts/cacao-fpslistadd-MODALFILTERING
	scripts/cacao-fpslistadd-MVMGPU_ZPO
	scripts/cacao-fpslistadd-OLMODEVAL2DM
	scripts/cacao-fpslistadd-SIMMVMGPU
	scripts/cacao-fpslistadd-WFSMODEVAL2DM
	scripts/cacao-fpslistadd-WFSCAMSIM
	scripts/cacao-fpslistadd-ZONALFILTERING
	scripts/cacao-fpsrunstart
	scripts/cacao-fpsrunstop
	scripts/cacao-modalstatsTUI
	scripts/cacao-takedark
	scripts/cacao-task-manager
	scripts/cacao-msglogCLI
	scripts/cacao-msglogCTRL
	scripts/cacao-logstreamsFITS
	scripts/cacao-loop-deploy
	scripts/cacao-setDMnolink
	scripts/cacao-wfsref-setflat
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
	scripts/aorun/cacao-aorun-020-mlatset
	scripts/aorun/cacao-aorun-021-mlatshow
	scripts/aorun/cacao-aorun-025-acqWFS
	scripts/aorun/cacao-aorun-026-takeref
	scripts/aorun/cacao-aorun-028-mkZFmodes
	scripts/aorun/cacao-aorun-030-acqlinResp
	scripts/aorun/cacao-aorun-031-RMHdecode
	scripts/aorun/cacao-aorun-032-RMmkmask
	scripts/aorun/cacao-aorun-033-RM-mksynthetic
	scripts/aorun/cacao-aorun-034-RMzonal2modal
	scripts/aorun/cacao-aorun-039-compstrCM
	scripts/aorun/cacao-aorun-040-compfCM
	scripts/aorun/cacao-aorun-041-loadCM
	scripts/aorun/cacao-aorun-042-loadCM
	scripts/aorun/cacao-aorun-041-saveCM
	scripts/aorun/cacao-aorun-045-compCM-byblocks
	scripts/aorun/cacao-aorun-050-wfs2cmval
	scripts/aorun/cacao-aorun-060-mfilt
	scripts/aorun/cacao-aorun-061-setmgains
	scripts/aorun/cacao-aorun-062-setmmults
	scripts/aorun/cacao-aorun-063-setmlimits
	scripts/aorun/cacao-aorun-065-zonalfiltering
	scripts/aorun/cacao-aorun-070-cmval2dm
	scripts/aorun/cacao-aorun-071-zpo
	scripts/aorun/cacao-aorun-080-testOL
	scripts/aorun/cacao-aorun-100-DMturb
	scripts/aorun/cacao-aorun-120-mstat
	scripts/aorun/cacao-aorun-130-mkPF
	scripts/aorun/cacao-aorun-140-applyPF
	scripts/aorun/cacao-aorun-200-mfiltturbrec
	DESTINATION bin)

install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})
