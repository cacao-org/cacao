set(LIBNAME "cacaopyramidWFStools")
set(SRCNAME "pyramidWFStools")

message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")

set(SOURCEFILES
    pyramidWFStools.c
    pyWFSgridmatch.c
    pyWFStlm.c)
set(INCLUDEFILES
    pyramidWFStools.h
    pyrwfs_tlm_core.h)

file(GLOB_RECURSE SCRIPTS LIST_DIRECTORIES false "scripts/*")
list(FILTER SCRIPTS EXCLUDE REGEX "/\\..+")

file(GLOB UNITFILES LIST_DIRECTORIES false "units/*.calc")

# DEFAULT SETTINGS
# Do not change unless needed
# =====================================================================

project(lib_${LIBNAME}_project)

include_directories("${CMAKE_SOURCE_DIR}/src")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/..")

milk_add_runnable_module(${LIBNAME} SOURCES ${SOURCEFILES} LINK_LIBS milkinfo_static cacaoAOloopControl_static)
target_include_directories(${LIBNAME}_objs PRIVATE ${CMAKE_SOURCE_DIR}/plugins/milk-extra-src)
milk_add_fpsexec_stubs(${LIBNAME})

# CUSTOM ===========
target_link_libraries(${LIBNAME} PUBLIC milkinfo cacaoAOloopControl)
# ==================

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})
install(PROGRAMS ${SCRIPTS} DESTINATION bin)
install(FILES commands.d/pyrwfs.bash DESTINATION share/milk/commands.d)
install(FILES ${UNITFILES} DESTINATION share/cacao/pyrwfs)

# Tier 2 @calc extension (auto-loaded from lib/milk/calc.d/)
if(USE_CALC)
    find_package(OpenMP REQUIRED)
    add_library(cacaopyrwfs_calc SHARED calc/pyrwfs_calc_ext.c)
    target_include_directories(
        cacaopyrwfs_calc PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}"
                                 "${CMAKE_SOURCE_DIR}/src/calc"
                                 "${CMAKE_SOURCE_DIR}/src/engine/libfps"
                                 "${CMAKE_SOURCE_DIR}/src/engine")
    target_compile_options(
        cacaopyrwfs_calc PRIVATE -O3 -march=native -fopenmp -fopenmp-simd
                                 -fno-finite-math-only -Wall -Wextra -Werror
                                 -Wno-unused-parameter)
    target_link_libraries(cacaopyrwfs_calc PRIVATE OpenMP::OpenMP_C m)
    install(TARGETS cacaopyrwfs_calc LIBRARY DESTINATION lib/milk/calc.d)
endif()
