set(LIBNAME "cacaoAOloopControlIOtools")
set(SRCNAME "AOloopControl_IOtools")

message("")
message(" SRCNAME = ${SRCNAME} -> LIBNAME = ${LIBNAME}")


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
# sources #include cross-plugin headers (e.g. statistic/statistic.h) directly
target_include_directories(${LIBNAME} PRIVATE ${CMAKE_SOURCE_DIR}/plugins/milk-extra-src)

target_link_libraries(${LIBNAME} PRIVATE ${LINKLIBS})

milk_apply_extensions(${LIBNAME})

install(TARGETS ${LIBNAME} DESTINATION lib)
install(FILES ${INCLUDEFILES} DESTINATION include/${SRCNAME})
install(PROGRAMS ${SCRIPTS} DESTINATION bin)

add_cacao_standalone(cacaoiot-acquireWFS acquireWFSim.c)
