# ═══════════════════════════════════════════
#  cacaocomputeCalib — Calibration computation
# ═══════════════════════════════════════════

set(LIBNAME "cacaocomputeCalib")
set(SRCNAME "computeCalib")

message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")

# CUDA/MAGMA/BLAS/LAPACKE needs are declared via MILK_CMAKE_REQUEST_*
# comments in the .c files themselves and wired automatically by
# milk_apply_extensions() (both for this library and for each standalone,
# via add_cacao_standalone()'s internal call).
file(GLOB SOURCEFILES "*.c" "*.cpp")
file(GLOB INCLUDEFILES "*.h" "*.hpp")
file(GLOB SCRIPTS "scripts/*")

set(LINKLIBS
	CLIcore
	milkinfo
	cacaoAOloopControl
)

# DEFAULT SETTINGS
# Do not change unless needed
# =====================================================================

project(lib_${LIBNAME}_project)

add_library(${LIBNAME} SHARED ${SOURCEFILES})

target_include_directories(${LIBNAME} PRIVATE $<TARGET_PROPERTY:CLIcore,INTERFACE_INCLUDE_DIRECTORIES>)

# sources #include cross-plugin headers (e.g. statistic/statistic.h) directly
target_include_directories(${LIBNAME} PRIVATE ${CMAKE_SOURCE_DIR}/plugins/milk-extra-src)
target_include_directories(${LIBNAME} PRIVATE ${CMAKE_SOURCE_DIR}/plugins/cacao-src)

target_link_libraries(${LIBNAME} PRIVATE ${LINKLIBS})

milk_apply_extensions(${LIBNAME})

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})
install(PROGRAMS ${SCRIPTS} DESTINATION bin)

add_cacao_standalone(cacaocc-maskextrapolate maskextrapolate.c)
add_cacao_standalone(cacaocc-actsamp2d actmap_sample2D.c)
add_cacao_standalone(cacaocc-rm2zonal RM2zonal.c)
add_cacao_standalone(cacaocc-comphadamard computeHadamard.c)
add_cacao_standalone_plugins(cacaocc-compstrcm compute_straight_CM.c)
add_cacao_standalone(cacaocc-genrmwfs generateRMWFS.c)
add_cacao_standalone(cacaocc-compmaskwfsdm compute_masksWFSDM.c)

add_cacao_standalone_plugins(cacaocc-compctrlmodes compute_control_modes.c)
target_sources(cacao-fpsexec-cacaocc-compctrlmodes PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/modes_spatial_extrapolate.c)

# ---- specialized standalone overrides (TODO: automate) ----
# "_compute" library variants no longer exist; link the regular libs.
# rm2zonal's/compstrcm's CUDA/BLAS/LAPACKE linkage now comes from the
# MILK_CMAKE_REQUEST_* directives in their .c files (see above).
target_link_libraries(cacao-fpsexec-cacaocc-rm2zonal PUBLIC milklinoptimtools)
target_link_libraries(cacao-fpsexec-cacaocc-compmaskwfsdm PUBLIC milkstatistic milkinfo)
target_link_libraries(cacao-fpsexec-cacaocc-compctrlmodes PUBLIC milklinoptimtools milkZernikePolyn)
