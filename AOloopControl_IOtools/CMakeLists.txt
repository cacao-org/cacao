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
    WFScamsim.c
    WFSmap.c
    extract_spectra.c
    norm_spectra.c
)

set(INCLUDEFILES
	${SRCNAME}.h
    AOloopControl_IOtools.h
    acquireWFSim.h
    ao188_preprocessor.h
    findspots.h
    WFScamsim.h
    WFSmap.h
    extract_spectra.h
    norm_spectra.h
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
install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})

install(PROGRAMS ${SCRIPTS} DESTINATION bin)
