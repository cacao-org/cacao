set(PROJECTNAME "cacao")

# list source files (.c) other than modulename.c
set(SOURCEFILES
	compRMsensitivity.c
	mlat.c
	mlat_decode.c
	streamlogtimesample.c
	wfsrefoptimselect.c
	zoptsearch.c
)

# list include files (.h) that should be installed on system
set(INCLUDEFILES
	mlat.h
	mlat_decode.h
)

# list scripts that should be installed on system
set(SCRIPTFILES
	scripts/cacao-compRMsens
	scripts/cacao-selectWFSframes
	scripts/cacao-syncStreams
	scripts/cacao-WFSrefTunePSFloop
	scripts/cacao-fpslistadd-MLAT
)


# DEFAULT SETTINGS
# Do not change unless needed
# =====================================================================


# SRCNAME is current directory (last subdir)
set(NAME0 "${CMAKE_CURRENT_SOURCE_DIR}")
string(REPLACE "/" " " NAME1 ${NAME0})
string(REGEX MATCH "[a-zA-Z0-9_]+$" SRCNAME "${NAME0}")


# LIBNAME constructed by removing spaces
string(REPLACE "_" "" LIBNAME "${PROJECTNAME}${SRCNAME}")


message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")


project(lib_${LIBNAME}_project)

# Library can be compiled from multiple source files
# Convention: the main souce file is named <libname>.c
#
add_library(${LIBNAME} SHARED ${SRCNAME}.c ${SOURCEFILES})

target_link_libraries(${LIBNAME} PUBLIC CLIcore milkstatistic cacaoAOloopControl)

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${SRCNAME}.h ${INCLUDEFILES} DESTINATION include/${SRCNAME})

install(PROGRAMS
	${SCRIPTFILES}
	DESTINATION bin)

add_cacao_standalone(cacaopt-compRMsens compRMsensitivity.c)
add_cacao_standalone(cacaopt-mlat mlat.c)
target_link_libraries(cacao-fpsexec-cacaopt-mlat PUBLIC ${LIBNAME} milkstatistic_compute)
add_cacao_standalone(cacaopt-zoptsearch zoptsearch.c)
target_link_libraries(cacao-fpsexec-cacaopt-zoptsearch PUBLIC ${LIBNAME} milkstatistic_compute)
add_cacao_standalone(cacaopt-wfsroptsel wfsrefoptimselect.c)
add_cacao_standalone_plugins(cacaopt-slogtsample streamlogtimesample.c)
target_link_libraries(cacao-fpsexec-cacaopt-slogtsample PUBLIC milkstatistic_compute)
add_cacao_standalone(cacaopt-mlatdecode mlat_decode.c)
target_link_libraries(cacao-fpsexec-cacaopt-mlatdecode PUBLIC ${LIBNAME} milkstatistic_compute)
