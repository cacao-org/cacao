set(LIBNAME "cacaoAOloopControl")
set(SRCNAME "AOloopControl")

project(lib_${LIBNAME}_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")

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
)



if(USE_CUDA)
find_package(CUDA 9.0 REQUIRED)
message(STATUS "Found CUDA ${CUDA_VERSION_STRING} at ${CUDA_TOOLKIT_ROOT_DIR}")
message("---- CUDA_INCLUDE_DIRS = ${CUDA_INCLUDE_DIRS}")
message("---- CUDA_LIBRARIES    = ${CUDA_LIBRARIES}" )
target_include_directories(${LIBNAME} PUBLIC ${CUDA_INCLUDE_DIRS})
target_link_libraries(${LIBNAME} PRIVATE ${CUDA_LIBRARIES})
target_compile_options(${LIBNAME} PUBLIC -DHAVE_CUDA)
endif(USE_CUDA)




if(USE_MAGMA)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MAGMA REQUIRED magma)
message("---- MAGMA_LIBRARY_DIRS =  ${MAGMA_LIBRARY_DIRS}") 
message("---- MAGMA_LIBRARIES    =  ${MAGMA_LIBRARIES}" )   
message("---- MAGMA_CFLAGS_OTHER =  ${MAGMA_CFLAGS_OTHER}" ) 
target_include_directories(${LIBNAME} PUBLIC ${MAGMA_INCLUDE_DIRS})
target_link_libraries(${LIBNAME} PRIVATE ${MAGMA_LIBRARIES})
target_compile_options(${LIBNAME} PUBLIC -DHAVE_CUDA -DHAVE_MAGMA -DUSE_MAGMA ${MAGMA_CFLAGS_OTHER})
endif(USE_MAGMA)


install(TARGETS ${LIBNAME} DESTINATION lib)

install(PROGRAMS scripts/cacao-setup
	scripts/cacao.tmux
	scripts/cacao-fpslistadd.bash
	scripts/cacao_simLHS
	DESTINATION bin)

install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})

