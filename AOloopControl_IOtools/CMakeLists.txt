set(LIBNAME "cacaoAOloopControlIOtools")
set(SRCNAME "AOloopControl_IOtools")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")


set(SOURCEFILES
	${SRCNAME}.c
    ${SRCNAME}.h
    acquireWFSim.c
    ao188_preprocessor.c
    findspots.c
    spotpos.c
    WFScamsim.c
    WFSmap.c
    acquireWFSspec.c
)

set(INCLUDEFILES
	${SRCNAME}.h
    AOloopControl_IOtools.h
    acquireWFSim.h
    ao188_preprocessor.h
    findspots.h
    spotpos.h
    WFScamsim.h
    WFSmap.h
    acquireWFSspec.h
)


set(SCRIPTS
)

set(LINKLIBS
	CLIcore
    milkinfo
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

install(PROGRAMS
	scripts/cacao-fpslistadd-SPOTPOS
	DESTINATION bin)


install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})

install(PROGRAMS ${SCRIPTS} DESTINATION bin)

add_cacao_standalone(cacaoiot-acquireWFS acquireWFSim.c)
