set(LIBNAME "cacaoAOloopControlDM")
set(SRCNAME "AOloopControl_DM")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")


set(SOURCEFILES
	${SRCNAME}.c
    ${SRCNAME}.h
    AOloopControl_DM_comb.c
    AOloopControl_DM_config.c
    AOloopControl_DM_init.c
    AOloopControl_DM_runtimecomp.c
    AOloopControl_DM_toolbox.c
    AOloopControl_DM_TTcircle_astrogrid.c
    AOloopControl_DM_turbusimul.c
    DMturbulence.c
    mk3Ddmgrid.c
    pokerndmodes.c
)

set(INCLUDEFILES
	${SRCNAME}.h
    AOloopControl_DM_comb.h
)


set(SCRIPTS
    scripts/cacao-astrogrid-mkgrid
    scripts/cacao-fpslistadd-DMATMTURB
)

set(LINKLIBS
	CLIcore
    milkfft
    milkimagegen
    cacaoAOloopControl
)




# DEFAULT SETTINGS
# Do not change unless needed
# =====================================================================

project(lib_${LIBNAME}_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")

# Library can be compiled from multiple source files
# Convention: the main souce file is named <libname>.c
#
add_library(${LIBNAME} SHARED ${SOURCEFILES})

target_include_directories(${LIBNAME} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/..
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${LIBNAME} PRIVATE ${LINKLIBS})

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})

install(PROGRAMS ${SCRIPTS} DESTINATION bin)
