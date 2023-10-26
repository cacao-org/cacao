set(LIBNAME "cacaocomputeCalib")
project(lib_${LIBNAME}_project)

# list source files (.c) other than modulename.c
set(SOURCEFILES
	actmap_sample2D.c
	AOloopControl_computeCalib_dm.c
	computeHadamard.c
	AOloopControl_computeCalib_loDMmodes.c
	AOloopControl_computeCalib_processRM.c
	compute_straight_CM.c
	compute_control_modes.c
	compute_masksWFSDM.c
	generateRMWFS.c
	maskextrapolate.c
	modes_spatial_extrapolate.c
	RM2zonal.c
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

if((NOT MKL_FOUND) AND (NOT OPENBLAS_FOUND))
  list(APPEND LINKLIBS lapacke)
endif()






if(USE_CUDA)
find_package(CUDA 9.0 REQUIRED)
message(STATUS "Found CUDA ${CUDA_VERSION_STRING} at ${CUDA_TOOLKIT_ROOT_DIR}")
message("---- CUDA_INCLUDE_DIRS = ${CUDA_INCLUDE_DIRS}")
message("---- CUDA_LIBRARIES    = ${CUDA_LIBRARIES}" )
target_compile_options(${LIBNAME} PUBLIC -DHAVE_CUDA)
target_link_libraries(${LIBNAME} PUBLIC ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES})

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
install(FILES ${SRCNAME}.h ${INCLUDEFILES} DESTINATION include/${SRCNAME})
install(PROGRAMS ${SCRIPTFILES} DESTINATION bin)
