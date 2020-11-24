# list source files (.c) other than modulename.c
set(SOURCEFILES
	AOloopControl_perfTest_DM.c  
	AOloopControl_perfTest_status.c 
	AOloopControl_perfTest_LinSim.c
)

# list include files (.h) that should be installed on system
set(INCLUDEFILES
	)

# list scripts that should be installed on system
set(SCRIPTFILES
	scripts/cacao-selectWFSframes
	scripts/cacao-syncStreams
	scripts/cacao-WFSrefTunePSFloop
)


# DEFAULT SETTINGS 
# Do not change unless needed
# =====================================================================


# SRCNAME is current directory (last subdir)
set(NAME0 "${CMAKE_CURRENT_SOURCE_DIR}")
string(REPLACE "/" " " NAME1 ${NAME0})
string(REGEX MATCH "[a-zA-Z0-9_]+$" SRCNAME "${NAME0}")
message(" SRCNAME = ${SRCNAME}")

# LIBNAME constructed by removing spaces
string(REPLACE "_" "" LIBNAME "${PROJECT_NAME}${SRCNAME}")
message(" LIBNAME = ${LIBNAME}")



project(lib_${LIBNAME}_project)

include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/..")


# Library can be compiled from multiple source files
# Convention: the main souce file is named <libname>.c
# 
add_library(${LIBNAME} SHARED ${SRCNAME}.c ${SOURCEFILES})

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${SRCNAME}.h ${INCLUDEFILES} DESTINATION include/${SRCNAME})

install(PROGRAMS
	${SCRIPTFILES}
	DESTINATION bin)





#project(lib_${LIBNAME}_project)

#include_directories ("${PROJECT_SOURCE_DIR}/src")
#include_directories ("${PROJECT_SOURCE_DIR}/..")
#add_library(${LIBNAME} SHARED 
#${SRCNAME}.c 
#${SRCNAME}.h 
#AOloopControl_perfTest_DM.c  
#AOloopControl_perfTest_status.c 
#AOloopControl_perfTest_LinSim.c
#)

#install(TARGETS ${LIBNAME} DESTINATION lib)
#install(FILES ${SRCNAME}.h DESTINATION include/${SRCNAME})


#install(PROGRAMS
#	${SCRIPTFILES}
#	DESTINATION bin)

#set(LIBNAME "cacaoAOloopControlperfTest")
#set(SRCNAME "AOloopControl_perfTest")
