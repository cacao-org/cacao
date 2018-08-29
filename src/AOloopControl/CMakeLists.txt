project(lib_AOloopControl_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")

add_library(AOloopControl SHARED AOloopControl.c 
AOloopControl.h
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


if(USE_MAGMA)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MAGMA REQUIRED magma)
message("---- MAGMA_LIBRARY_DIRS =  ${MAGMA_LIBRARY_DIRS}") 
message("---- MAGMA_LIBRARIES    =  ${MAGMA_LIBRARIES}" )   
message("---- MAGMA_CFLAGS_OTHER =  ${MAGMA_CFLAGS_OTHER}" ) 
target_include_directories(AOloopControl PUBLIC ${MAGMA_INCLUDE_DIRS})
target_link_libraries(AOloopControl PRIVATE ${MAGMA_LIBRARIES}) 
target_link_libraries(AOloopControl PRIVATE cusolver)
target_compile_options(AOloopControl PUBLIC -DHAVE_CUDA -DHAVE_MAGMA -DUSE_MAGMA ${MAGMA_CFLAGS_OTHER})
endif(USE_MAGMA)


install(TARGETS AOloopControl DESTINATION lib)
install(FILES AOloopControl.h DESTINATION include)

