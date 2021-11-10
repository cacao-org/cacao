set(LIBNAME "cacaoAOloopControlIOtools")
set(SRCNAME "AOloopControl_IOtools")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")


set(SOURCEFILES
	${SRCNAME}.c
    ${SRCNAME}.h
    AOloopControl_IOtools_camerainput.c
    AOloopControl_IOtools_datastream_processing.c
    AOloopControl_IOtools_load_image_sharedmem.c
    AOloopControl_IOtools_RTLOGsave.c
    acquireWFS.c
)

set(INCLUDEFILES
	${SRCNAME}.h
    AOloopControl_IOtools.h
    acquireWFS.h
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









#project(lib_${LIBNAME}_project)

#add_library(${LIBNAME} SHARED ${SRCNAME}.c
#${SRCNAME}.h
#AOloopControl_IOtools_camerainput.c
#AOloopControl_IOtools_datastream_processing.c
#AOloopControl_IOtools_load_image_sharedmem.c
#AOloopControl_IOtools_RTLOGsave.c
#)

#target_link_libraries(${LIBNAME} PUBLIC CLIcore milkinfo cacaoAOloopControl)

#install(TARGETS ${LIBNAME} DESTINATION lib)
#install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})

