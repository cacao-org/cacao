# ═══════════════════════════════════════════
#  cacaocomputeCalib — Calibration computation
# ═══════════════════════════════════════════

set(LIBNAME "cacaocomputeCalib")
set(SRCNAME "computeCalib")

message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")

# CUDA/MAGMA/BLAS/LAPACKE needs are declared via MILK_CMAKE_REQUEST_* and
# MILK_CMAKE_MANDATE_* comments in the .c files and wired by milk_add_runnable_module().
file(GLOB SOURCEFILES "*.c" "*.cpp")
file(GLOB INCLUDEFILES "*.h" "*.hpp")
file(GLOB SCRIPTS "scripts/*")

project(lib_${LIBNAME}_project)

include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("${PROJECT_SOURCE_DIR}/..")

milk_add_runnable_module(${LIBNAME} SOURCES ${SOURCEFILES} LINK_LIBS
        milkinfo_static
        milklinoptimtools_static
        milkstatistic_static
        milkZernikePolyn_static
        cacaoAOloopControl_static
)
target_include_directories(${LIBNAME}_objs PRIVATE ${CMAKE_SOURCE_DIR}/plugins/milk-extra-src)
milk_add_fpsexec_stubs(${LIBNAME})

# CUSTOM ===========
target_link_libraries(${LIBNAME} PUBLIC
        milkinfo
        milklinoptimtools
        milkstatistic
        milkZernikePolyn
        cacaoAOloopControl
)
# ==================

install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})
install(PROGRAMS ${SCRIPTS} DESTINATION bin)
