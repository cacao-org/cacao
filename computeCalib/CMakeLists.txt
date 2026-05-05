# ═══════════════════════════════════════════
#  cacaocomputeCalib — Calibration computation
# ═══════════════════════════════════════════

set(LIBNAME "cacaocomputeCalib")
project(lib_${LIBNAME}_project)

# ── Source files ─────────────────────────────────
# Dual-mode files: compiled into ${LIBNAME} for CLI
# AND as standalone executables (add_cacao_standalone).
set(SOURCEFILES
	actmap_sample2D.c                          # → cacaocc-actsamp2d
	AOloopControl_computeCalib_dm.c
	computeHadamard.c                          # → cacaocc-comphadamard
	AOloopControl_computeCalib_loDMmodes.c
	AOloopControl_computeCalib_processRM.c
	compute_straight_CM.c                      # → cacaocc-compstrcm
	compute_control_modes.c                    # → cacaocc-compctrlmodes
	compute_masksWFSDM.c                       # → cacaocc-compmaskwfsdm
	generateRMWFS.c                            # → cacaocc-genrmwfs
	maskextrapolate.c                          # → cacaocc-maskextrapolate
	modes_spatial_extrapolate.c
	RM2zonal.c                                 # → cacaocc-rm2zonal
)


# SRCNAME is current directory (last subdir)
set(NAME0 "${CMAKE_CURRENT_SOURCE_DIR}")
string(REPLACE "/" " " NAME1 ${NAME0})
string(REGEX MATCH "[a-zA-Z0-9_]+$" SRCNAME "${NAME0}")
message(" SRCNAME = ${SRCNAME}")

# Library can be compiled from multiple source files
# Convention: the main souce file is named <libname>.c
#
add_library(${LIBNAME} SHARED ${SRCNAME}.c ${SOURCEFILES})



# list include files (.h) that should be installed on system
set(INCLUDEFILES
)

# list scripts that should be installed on system
set(SCRIPTFILES
	scripts/cacao-fpslistadd-COMPSTRCM
)

set(LINKLIBS
	CLIcore
	milkinfo
	cacaoAOloopControl
)

#If not using MKL, check if OpenBLAS has lapacke symbols.  Include lapacke if not.
if(NOT MKL_FOUND)

    check_library_exists(openblas LAPACKE_sgemlq "" DONT_NEED_LAPACKE)

    if(DONT_NEED_LAPACKE)
       message("-- MKL NOT FOUND")
       message("   LINKING OpenBLAS")
    else()
       message("-- MKL NOT FOUND")
       message("   LINKING LAPACKE")
       list(APPEND LINKLIBS lapacke)
    endif()

endif()





if(USE_CUDA)
  find_package(CUDAToolkit 9.0 REQUIRED)
	message(STATUS "Found CUDA ${CUDAToolkit_VERSION} at ${CUDAToolkit_LIBRARY_DIR}")
	message("---- CUDA_INCLUDE_DIRS = ${CUDAToolkit_INCLUDE_DIRS}")
  target_compile_options(${LIBNAME} PUBLIC -DHAVE_CUDA)
  target_link_libraries(${LIBNAME} PRIVATE CUDA::cublas)

  if(USE_MAGMA)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(MAGMA REQUIRED magma)
    message("---- MAGMA_LIBRARY_DIRS =  ${MAGMA_LIBRARY_DIRS}")
    message("---- MAGMA_LIBRARIES    =  ${MAGMA_LIBRARIES}" )
    message("---- MAGMA_CFLAGS_OTHER =  ${MAGMA_CFLAGS_OTHER}" )
    target_include_directories(${LIBNAME} PUBLIC ${MAGMA_INCLUDE_DIRS})
    target_link_directories(${LIBNAME} PUBLIC ${MAGMA_LIBRARY_DIRS})
    target_link_libraries(${LIBNAME} PUBLIC ${MAGMA_LIBRARIES})
    target_compile_options(${LIBNAME} PUBLIC -DHAVE_MAGMA ${MAGMA_CFLAGS_OTHER})
  endif(USE_MAGMA)

endif(USE_CUDA)



# OpenBLAS detected in upsteam CMakeList with pkg_check_modules(OPENBLAS openblas)
if(OPENBLAS_FOUND)
message("---- OPENBLAS_LIBRARY_DIRS =  ${OPENBLAS_LIBRARY_DIRS}")
message("---- OPENBLAS_LIBRARIES    =  ${OPENBLAS_LIBRARIES}" )
message("---- OPENBLAS_CFLAGS_OTHER =  ${OPENBLAS_CFLAGS_OTHER}" )
target_include_directories(${LIBNAME} PUBLIC ${OPENBLAS_INCLUDE_DIRS})
target_link_directories(${LIBNAME} PUBLIC ${OPENBLAS_LIBRARY_DIRS})
target_link_libraries(${LIBNAME} PUBLIC ${OPENBLAS_LIBRARIES})
target_compile_options(${LIBNAME} PUBLIC -DHAVE_OPENBLAS ${OPENBLAS_CFLAGS_OTHER})
endif()



# MKL detected in upstream CMakeList with pkg_check_modules(MKL mkl-sdl)
if(MKL_FOUND)
message("---- MKL_LIBRARY_DIRS =  ${MKL_LIBRARY_DIRS}")
message("---- MKL_LIBRARIES    =  ${MKL_LIBRARIES}" )
message("---- MKL_CFLAGS_OTHER =  ${MKL_CFLAGS_OTHER}" )
target_include_directories(${LIBNAME} PUBLIC ${MKL_INCLUDE_DIRS})
target_link_directories(${LIBNAME} PUBLIC ${MKL_LIBRARY_DIRS})
target_link_libraries(${LIBNAME} PUBLIC ${MKL_LIBRARIES})
target_compile_options(${LIBNAME} PUBLIC -DHAVE_MKL ${MKL_CFLAGS_OTHER})
endif()



# DEFAULT SETTINGS
# Do not change unless needed
# =====================================================================



# LIBNAME constructed by removing spaces
#string(REPLACE "_" "" LIBNAME "${SRCNAME}")
#message(" LIBNAME = ${LIBNAME}")


# note: set to PUBLIC if other modules link to this one
target_include_directories(${LIBNAME} PRIVATE ${PROJECT_SOURCE_DIR}/.. ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${LIBNAME} PRIVATE ${LINKLIBS})

install(TARGETS ${LIBNAME} DESTINATION lib)
add_cacao_standalone(cacaocc-maskextrapolate maskextrapolate.c)
target_link_libraries(cacao-fpsexec-cacaocc-maskextrapolate PUBLIC)

add_cacao_standalone(cacaocc-actsamp2d actmap_sample2D.c)
target_link_libraries(cacao-fpsexec-cacaocc-actsamp2d PUBLIC)

add_cacao_standalone(cacaocc-rm2zonal RM2zonal.c)
target_include_directories(cacao-fpsexec-cacaocc-rm2zonal
    PRIVATE ${PROJECT_SOURCE_DIR}/plugins/milk-extra-src)
target_link_libraries(cacao-fpsexec-cacaocc-rm2zonal PUBLIC milklinoptimtools_compute)
if(OPENBLAS_FOUND)
    target_include_directories(cacao-fpsexec-cacaocc-rm2zonal
        PRIVATE ${OPENBLAS_INCLUDE_DIRS})
    target_link_directories(cacao-fpsexec-cacaocc-rm2zonal
        PUBLIC ${OPENBLAS_LIBRARY_DIRS})
    target_link_libraries(cacao-fpsexec-cacaocc-rm2zonal
        PUBLIC ${OPENBLAS_LIBRARIES})
    target_compile_definitions(cacao-fpsexec-cacaocc-rm2zonal
        PRIVATE HAVE_OPENBLAS ${OPENBLAS_CFLAGS_OTHER})
endif()
if(MKL_FOUND)
    target_include_directories(cacao-fpsexec-cacaocc-rm2zonal
        PRIVATE ${MKL_INCLUDE_DIRS})
    target_link_directories(cacao-fpsexec-cacaocc-rm2zonal
        PUBLIC ${MKL_LIBRARY_DIRS})
    target_link_libraries(cacao-fpsexec-cacaocc-rm2zonal
        PUBLIC ${MKL_LIBRARIES})
endif()
if(NOT MKL_FOUND AND NOT DONT_NEED_LAPACKE)
    target_link_libraries(cacao-fpsexec-cacaocc-rm2zonal PUBLIC lapacke)
endif()
if(USE_CUDA)
    target_link_libraries(cacao-fpsexec-cacaocc-rm2zonal PRIVATE CUDA::cudart CUDA::cublas)
endif()

add_cacao_standalone(cacaocc-comphadamard computeHadamard.c)
target_link_libraries(cacao-fpsexec-cacaocc-comphadamard PUBLIC)

add_cacao_standalone_plugins(cacaocc-compstrcm compute_straight_CM.c)
if(OPENBLAS_FOUND)
    target_include_directories(cacao-fpsexec-cacaocc-compstrcm
        PRIVATE ${OPENBLAS_INCLUDE_DIRS})
    target_link_directories(cacao-fpsexec-cacaocc-compstrcm
        PUBLIC ${OPENBLAS_LIBRARY_DIRS})
    target_link_libraries(cacao-fpsexec-cacaocc-compstrcm
        PUBLIC ${OPENBLAS_LIBRARIES})
    target_compile_definitions(cacao-fpsexec-cacaocc-compstrcm
        PRIVATE HAVE_OPENBLAS ${OPENBLAS_CFLAGS_OTHER})
endif()
if(MKL_FOUND)
    target_include_directories(cacao-fpsexec-cacaocc-compstrcm
        PRIVATE ${MKL_INCLUDE_DIRS})
    target_link_directories(cacao-fpsexec-cacaocc-compstrcm
        PUBLIC ${MKL_LIBRARY_DIRS})
    target_link_libraries(cacao-fpsexec-cacaocc-compstrcm
        PUBLIC ${MKL_LIBRARIES})
endif()
if(NOT MKL_FOUND AND NOT DONT_NEED_LAPACKE)
    target_link_libraries(cacao-fpsexec-cacaocc-compstrcm PUBLIC lapacke)
endif()
if(USE_CUDA)
    target_link_libraries(cacao-fpsexec-cacaocc-compstrcm PRIVATE CUDA::cudart CUDA::cublas)
endif()

add_cacao_standalone(cacaocc-genrmwfs generateRMWFS.c)
target_link_libraries(cacao-fpsexec-cacaocc-genrmwfs PUBLIC)

add_cacao_standalone(cacaocc-compmaskwfsdm compute_masksWFSDM.c)
target_link_libraries(cacao-fpsexec-cacaocc-compmaskwfsdm PUBLIC milkstatistic_compute milkinfo_compute)

add_cacao_standalone_plugins(cacaocc-compctrlmodes compute_control_modes.c)
target_sources(cacao-fpsexec-cacaocc-compctrlmodes PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/modes_spatial_extrapolate.c)
target_link_libraries(cacao-fpsexec-cacaocc-compctrlmodes
    PUBLIC milklinoptimtools_compute milkZernikePolyn_compute)
